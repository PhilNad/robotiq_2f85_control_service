#include <string>
#include <math.h>
#include <vector>
#include <array>
#include "ros/ros.h"
#include <ros/callback_queue.h>
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "sensor_msgs/JointState.h"
#include <gazebo_msgs/ApplyJointEffort.h>
#include <gazebo_msgs/JointRequest.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_input.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>
#include "gripper_2f85/SetPosition.h"

//If argv[1] == 'realworld', set this to true.
bool SIMULATION = false;

using namespace std;

//Used to record the last N positions errors and better control the gripper
const unsigned int errorArraySize = 5;
vector<double> lPositionErrors (errorArraySize);
vector<double> rPositionErrors (errorArraySize);
map<string, double> positionOfJoint;
map<string, double> velocityOfJoint;
map<string, double> effortOfJoint;

//Goal position we should always aim to reach
uint8_t currentGoalPosition = 0;

//Resolution as per Robotiq 2f-85 specs.
const double positionResolution = 0.004;
const double positionUpperLimit = 180;
const double positionLowerLimit = 0;

//This amplitude must be below the max_effort parameter specified in the SDF
//See generateSDF.sh script.
//If the amplitude gets above the specified max., it breaks the simulation.
//So it is better to specify a max which is a bit too high.
const float effortAmplitudeRight = 4;
const float effortAmplitudeLeft  = 8;

//These tolerance values were experimentally found to be OK
const double positionTolerance = 0.005;
const double velocityTolerance = 0.1;

//Names of actuated joints
const string rJointName = "gripper_right_driver_joint";
const string lJointName = "gripper_left_driver_joint";

//The node handle needs to be global so its accessible in the callback
//Needs to be instantiated in the main once ros::init is done
ros::NodeHandle *thisNode;

//This is the handle to the publishers
ros::Publisher current_position_pub;
ros::Publisher gripper_command_pub;

//This is the handles to the subscribers
ros::Subscriber gripper_state_sub;
ros::Subscriber joint_states_sub;

//This function gets called whenever a node request our service
bool setPosition(   gripper_2f85::SetPosition::Request  &req,
                    gripper_2f85::SetPosition::Response &res)
{
    currentGoalPosition = req.GoalPosition;
    ROS_INFO("Setting gripper goal to: %d",currentGoalPosition);

    res.Success = true;
    return true;
}

//Apply a given torque on specified joint
bool applyJointEffort(float torque, string jointName){
    //A negative effort closes the gripper
    gazebo_msgs::ApplyJointEffort effort;
    //This torque should be computer from a given end-effector force
    //to better reproduce Robotiq specs.
    effort.request.effort = torque;
    //TODO: This duration should be set to the updateEfforts period
    effort.request.duration = ros::Duration(0.05);

    effort.request.joint_name  = jointName;

    ros::ServiceClient effortService = thisNode->serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");

    bool success = effortService.call(effort);

    return success;
}

//Remove any joint efforts previously applied
bool clearEffort(string jointName){
    gazebo_msgs::JointRequest effort;
    effort.request.joint_name = jointName;

    ros::ServiceClient effortService = thisNode->serviceClient<gazebo_msgs::JointRequest>("/gazebo/clear_joint_forces");

    bool success = effortService.call(effort);

    if(success){
        ROS_DEBUG("Clearing efforts on %s: SUCCESS",jointName.c_str());
    }else{
        ROS_WARN("Clearing efforts on %s: FAILURE",jointName.c_str());
    }

    return success;
}

//Output the appropriate effort given a position error
double effortController(float effortAmplitude, double position, double targetPosition, vector<double>& lastErrors, double velocity){
    double positionError  = position-targetPosition;

    //A positive effort closes the gripper and decreases the position
    int polarity = 1;
    if(positionError > 0){
        polarity = -1;
    }

    //Shhh, dont ask why.
    double effort = 0;
    if(fabs(positionError) >= 0.005){
        effort = 0.7686*effortAmplitude*log10(200*fabs(positionError));

        //Visquous damping using a sigmoid function
        //double damping = velocity/sqrt(1+pow(velocity,2));
        //effort = effort * (1 - damping);
    }

    return polarity*effort;

}

//Take into account the new positions values and update the efforts
//imposed onto the gripper's actuated joints
bool updateEfforts(){

    //Compute the position from the controller's 8-bits code
    double targetPosition = currentGoalPosition*positionResolution;

    double leftGripperEffort    = effortOfJoint[lJointName];
    double rightGripperEffort   = effortOfJoint[rJointName];
    double leftGripperPosition  = positionOfJoint[lJointName];
    double rightGripperPosition = positionOfJoint[rJointName];
    double leftGripperVelocity  = velocityOfJoint[lJointName];
    double rightGripperVelocity = velocityOfJoint[rJointName];

    //Enforce the limits
    if(targetPosition > positionUpperLimit*positionResolution){
        targetPosition = positionUpperLimit*positionResolution;
        ROS_WARN("Specified position %d=%f was greater than the upper limit of %f, target position set to %f.",currentGoalPosition,targetPosition,positionUpperLimit*positionResolution,positionUpperLimit*positionResolution);
    }
    if(targetPosition < positionLowerLimit*positionResolution){
        targetPosition = positionLowerLimit*positionResolution;
        ROS_WARN("Specified position %d=%f was smaller than the lower limit of %f, target position set to %f.",currentGoalPosition,targetPosition,positionUpperLimit*positionResolution,positionUpperLimit*positionResolution);
    }

    //If a huge effort was imposed on the links, they can break
    if(fabs(leftGripperPosition) > 1 || fabs(rightGripperPosition) > 1 ){
        ROS_WARN("Simulation has crashed, gripper position faulty.");
        return false;
    }

    //Compute the position error, polynomial controller anyone?
    double leftPositionError  = leftGripperPosition-targetPosition;
    double rightPositionError = rightGripperPosition-targetPosition;
    //Update left errors vector
    vector<double>::iterator it = lPositionErrors.begin();
    lPositionErrors.insert ( it , leftPositionError );
    lPositionErrors.pop_back();
    //Update right errors vector
    it = rPositionErrors.begin();
    rPositionErrors.insert ( it , rightPositionError );
    rPositionErrors.pop_back();

    ROS_DEBUG("Current error: %f on %s.",leftPositionError, lJointName.c_str());
    ROS_DEBUG("Current error: %f on %s.",rightPositionError, rJointName.c_str());

    //To mimic the effects of Robotiq 2f-85 internal mechanism (wormgear)
    //the errors should be kept similar. A big difference between these
    //position errors should indicate an issue possibly caused by external forces
    double errorDiff = fabs(leftPositionError-rightPositionError);
    if(errorDiff > 0.3)
        ROS_DEBUG("Big position difference detected between gripper's fingers.");

    //Compute the efforts, they should be very similar one to the other
    double leftCalcEffort  = effortController(effortAmplitudeLeft, leftGripperPosition,  
            targetPosition, lPositionErrors, leftGripperVelocity);
    double rightCalcEffort = effortController(effortAmplitudeRight, rightGripperPosition, 
            targetPosition, rPositionErrors, rightGripperVelocity);

    //Efforts are cumulative, so we clear them first.
    //TODO: I suspect that this clearing doesnt work...
    clearEffort(lJointName);
    clearEffort(rJointName);

    //Apply the computed efforts
    bool success = applyJointEffort(rightCalcEffort, rJointName);
    if(!success)
        ROS_WARN("Failed to apply effort on %s.", rJointName.c_str());
    else
        ROS_DEBUG("Applied effort %f on %s.",rightCalcEffort, rJointName.c_str());

    success = applyJointEffort(leftCalcEffort, lJointName);
    if(!success)
        ROS_WARN("Failed to apply effort on %s.",lJointName.c_str());
    else
        ROS_DEBUG("Applied effort %f on %s.",leftCalcEffort,lJointName.c_str());

    //For debugging
    //printf("%f,%f,%f,%f,%f,",targetPosition,leftGripperPosition,leftPositionError,leftCalcEffort,leftGripperVelocity);
    //printf("%f,%f,%f,%f\n",rightGripperPosition,rightPositionError,rightCalcEffort,rightGripperVelocity);

    return true;
}

//The message type is : http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html
void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg){

    //Iterates over every name in the message
    //Note that it is not guaranteed that a given joint will be present
    //in every messages.
    for(size_t i = 0; i < msg->name.size(); i++){
        string strName(msg->name[i]);
        //If the name corresponds to an entry in our map, update its value
        //if its not, create a new entry
        positionOfJoint[strName]    = msg->position[i];
        velocityOfJoint[strName]    = msg->velocity[i];
        effortOfJoint[strName]      = msg->effort[i];
    }

}

//Publish the position of the gripper as given by the JointState message
void publishGripperPosition(){
    double leftGripperPosition  = positionOfJoint[lJointName];
    double rightGripperPosition = positionOfJoint[rJointName];

    //Compute the position from the controller's 8-bits code
    uint8_t lPosScaled = round(leftGripperPosition/positionResolution);
    uint8_t rPosScaled = round(rightGripperPosition/positionResolution);

    //We assume that both fingers are at the same position
    std_msgs::UInt8 msg;
    msg.data = lPosScaled;
    current_position_pub.publish(msg);
}

//Callback called when the gripper sends its state.
void gripper_input_callback(const robotiq_2f_gripper_control::Robotiq2FGripper_robot_input::ConstPtr &msg){
    //gACT is the activation status of the gripper where 1 means that the gripper is enabled.
    uint8_t gACT  = msg->gACT;
    //gGTO is the position request status where 1 means that it is moving toward a requested position.
    uint8_t gGTO  = msg->gGTO;
    //gCU is the electrical current being fed to the motor where a single count is ~0.1mA
    uint8_t gCU   = msg->gCU;
    //gFLT is the fault flag:
    //  0 => No fault.
    //  5 => Reactivation must be performed before any further movement.
    //  7 => Activation bit must be set prior to action.
    //  8 => Gripper's temperature has risen too high and it needs to cool down.
    // 10 => Undervoltage.
    // 11 => Automatic release in progress (see rATR).
    // 12 => Internal fault.
    // 13 => Activation fault.
    // 14 => Overcurrent.
    // 15 => Automatic release completed.
    uint8_t gFLT  = msg->gFLT;
    //gOBJ is the object detection status. If this feature is used (see rFR),
    //  0 => No object detected during current motion.
    //  1 => Object detected while opening gripper.
    //  2 => Object detected while closing gripper.
    //  3 => Requested position reached without any detection.
    uint8_t gOBJ  = msg->gOBJ;
    //gPO is the current position obtained via encoders.
    uint8_t gPO   = msg->gPO;
    //gPR is the current goal position to reach (see rPR).
    uint8_t gPR   = msg->gPR;
    //gSTA is the current status of the gripper where:
    //  0 => Gripper is in reset state.
    //  1 => Activation is in progress.
    //  2 => This status is not defined.
    //  3 => Activation is completed.
    uint8_t gSTA  = msg->gSTA;

    //Print some important fault messages.
    switch(gFLT){
        case 5 :
            ROS_WARN("FAULT: Reactivation must be performed before any further movement.");
            break;
        case 7 :
            ROS_WARN("FAULT: Activation bit must be set prior to this action.");
            break;
        case 8 :
            ROS_ERROR("FAULT: Gripper's temperature has risen too high and it needs to cool down.");
            break;
        case 10:
            ROS_ERROR("FAULT: Undervoltage.");
            break;
        case 12:
            ROS_WARN("FAULT: Internal fault.");
            break;
        case 13:
            ROS_WARN("FAULT: Activation fault.");
            break;
        case 14:
            ROS_ERROR("FAULT: Overcurrent.");
            break;
    }

    //Publish current gripper position
    std_msgs::UInt8 positionMessage;
    positionMessage.data = gPO;
    current_position_pub.publish(positionMessage);
}

//Uses robotiq_2f_gripper_control's interface to send a position command to the gripper 
void commandPosition(uint8_t goalPosition){
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output gripper_command;
    //rACT:First action to be made prior to any other actions, rACT bit will activate the Gripper. 
    //Clear rACT to reset the Gripper and clear fault status.
    //rACT must stay on after activation. Power loss will set rACT; rACT bit must then be cleared,
    //then set to allow operation of the Gripper.
    gripper_command.rACT    = 1;
    //The rATR flag enables the automatic release of the gripper after an emergency stop.
    //If this flag is enabled and such condition happens, this behavior overrides any other command.
    //After all motion is completed, the Gripper sends a fault signal and needs to be reactivated 
    //before any other motion is performed.
    gripper_command.rATR    = 0;
    //The rSP 0-255 register specifies the speed at which the gripper can move where 0 is the minimum speed and
    //255 is the maximum speed.
    gripper_command.rSP     = 0;
    //The rFR 0-255 register specifies the maximum final force that the gripper can generate. Internally, this
    //limits the amount of current sent to the motor. If the current limit is exceeded, an object detection notification
    //is generated. For the 2f-85, this force is roughly linear between 25N and 220N.
    gripper_command.rFR     = 0;
    //Requested position of the gripper between 0-255 where 0 means fully open and 255 is fully closed.
    //Every count from 0 is a position increment of 0.4 mm for 2f-85 and 0.65 mm for 2f-140.
    gripper_command.rPR     = goalPosition;
    //rGTO: The "GoTo" action moves the Gripper fingers to the requested position using the configuration defined
    // by the other registers.
    gripper_command.rGTO    = 1;

    //Send the command defined by the aforementioned values.
    gripper_command_pub.publish(gripper_command);
}

int main(int argc, char **argv){

    //The first argument defines the context in which this service is to be used.
    if(strcmp(argv[1],"realworld")==0){
        SIMULATION = false;
    }else{
        SIMULATION = true;
    }
    
    //Initialize the ROS node
    ros::init(argc, argv, "gripper_2f85_set_position_server");

    //Instantiate the handle, needs to be done after ros::init
    thisNode = new ros::NodeHandle();

    //Publish the current fingers position to this topic
    current_position_pub = thisNode->advertise<std_msgs::UInt8>("gripper/current_position", 1);

    if( SIMULATION == false){
        //Subscribe to this topic that the Robotiq ROS communication interface publishes to.
        gripper_state_sub = thisNode->subscribe("/Robotiq2FGripperRobotInput", 1, gripper_input_callback);

        //Messages sent to this topic are interpreted as gripper commands and sent to the hardware.
        gripper_command_pub = thisNode->advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 1);
    }
    
    if(SIMULATION == true){
        //Subscribe to this topic that Gazebo publishes to.
        joint_states_sub = thisNode->subscribe("joint_states", 1, jointStatesCallback);

        //Cancel any leftover efforts
        clearEffort(lJointName);
        clearEffort(rJointName);
    }

    //Tell everyone that you exist
    ros::ServiceServer service = thisNode->advertiseService("gripper/set_position", setPosition);
    ROS_INFO("Ready to control the Robotiq 2f-85 gripper.");
    
    //Default callback queue
    ros::CallbackQueue* queue = ros::getGlobalCallbackQueue();


    while(ros::ok()){
        if(SIMULATION == true){
            updateEfforts();
            publishGripperPosition();
        }

        if(SIMULATION == false){
            commandPosition(currentGoalPosition);
        }
        
        //Run callbacks at a rate of 10 Hz
        int frequency = 10;
        double period = 1/frequency;

        //Going too fast screws the simulation
        usleep(period*1000*1000);
        //callAvailable() can take in an optional timeout, which is the amount 
        //of time it will wait for a callback to become available before 
        //returning. If this is zero and there are no callbacks in the queue 
        //the method will return immediately. 
        queue->callAvailable(ros::WallDuration(0));
    }

    return 0;
}
