#include <math.h>

#include <geometry_msgs/Twist.h>

#include "aubo_driver/aubo_driver.h"
#include "aubo_driver.cpp"

#define MAX_END_ACC     0.5                 // unit m/s^2
#define MAX_END_VEL     0.2                 // unit m/s
#define MAX_JOINT_ACC   100.0/180.0*M_PI    // unit rad/s^2
#define MAX_JOINT_VEL   100.0/180.0*M_PI    // unit rad/s

#define JOTSTICK_THRESHHOLD 0.02

class AuboJogControl {
  public:
    int msg_timeout_;

    aubo_driver::AuboDriver *robot_driver_;
    geometry_msgs::Twist twist_;

  public:
    AuboJogControl(aubo_driver::AuboDriver *driver) {
      robot_driver_ = driver;

      // initialize move properties
      robot_driver_->robot_send_service_.robotServiceInitGlobalMoveProfile();

      // set max joint acceleration and velocity
      aubo_robot_namespace::JointVelcAccParam joint_max_acc;
      aubo_robot_namespace::JointVelcAccParam joint_max_velc;
      for(int i = 0; i < 6; i++) {
        joint_max_acc.jointPara[i] = MAX_JOINT_ACC;
        joint_max_velc.jointPara[i] = MAX_JOINT_VEL;
      }
      robot_driver_->robot_send_service_.robotServiceSetGlobalMoveJointMaxAcc(joint_max_acc);
      robot_driver_->robot_send_service_.robotServiceSetGlobalMoveJointMaxVelc(joint_max_velc);

      // set max end-tool linear acceleration and velocity
      robot_driver_->robot_send_service_.robotServiceSetGlobalMoveEndMaxLineAcc(MAX_END_ACC);
      robot_driver_->robot_send_service_.robotServiceSetGlobalMoveEndMaxLineVelc(MAX_END_VEL);

      // set max end-tool angular acceleration and velocity
      robot_driver_->robot_send_service_.robotServiceSetGlobalMoveEndMaxAngleAcc(MAX_END_ACC);
      robot_driver_->robot_send_service_.robotServiceSetGlobalMoveEndMaxAngleVelc(MAX_END_VEL);

      // set end tool
      aubo_robot_namespace::ToolKinematicsParam tool_kinematics_param;
      tool_kinematics_param.toolInEndPosition = {x:0, y:0, z:0.489};
      tool_kinematics_param.toolInEndOrientation = {w:1, x:0, y:0, z:0};
      int result = robot_driver_->robot_send_service_.robotServiceSetToolKinematicsParam(tool_kinematics_param);
      ROS_INFO("set tool kinematics params result %d", result);

      // set jog control end-tool coordinate
      aubo_robot_namespace::CoordCalibrateByJointAngleAndTool tool_coordinate;
      tool_coordinate.coordType = aubo_robot_namespace::EndCoordinate;
      result = robot_driver_->robot_send_service_.robotServiceSetTeachCoordinateSystem(tool_coordinate);
      ROS_INFO("set teach coordinate system result %d", result);
    }

    ~AuboJogControl() {
      // stop arm before node close
      robot_driver_->robot_send_service_.robotServiceTeachStop();
    }

    void UpdateVelocityCommand() {
      int result;

      if(msg_timeout_ > 20) {
        // if there is no new data for 20 loops, stop arm and reset linear & angular max velocity
        robot_driver_->robot_send_service_.robotServiceSetGlobalMoveEndMaxLineVelc(MAX_END_VEL);
        robot_driver_->robot_send_service_.robotServiceSetGlobalMoveEndMaxAngleVelc(MAX_END_VEL);
        robot_driver_->robot_send_service_.robotServiceTeachStop();
      } else if(fabs(twist_.linear.x) > JOTSTICK_THRESHHOLD && fabs(twist_.linear.x)>fabs(twist_.linear.y)) {
        robot_driver_->robot_send_service_.robotServiceSetGlobalMoveEndMaxLineVelc(MAX_END_VEL*fabs(twist_.linear.x));
        robot_driver_->robot_send_service_.robotServiceTeachStart(aubo_robot_namespace::MOV_X, twist_.linear.x>0?1:0);
      } else if(fabs(twist_.linear.y) > JOTSTICK_THRESHHOLD && fabs(twist_.linear.y)>fabs(twist_.linear.x)) {
        robot_driver_->robot_send_service_.robotServiceSetGlobalMoveEndMaxLineVelc(MAX_END_VEL*fabs(twist_.linear.y));
        robot_driver_->robot_send_service_.robotServiceTeachStart(aubo_robot_namespace::MOV_Y, twist_.linear.y>0?1:0);
      } else if(fabs(twist_.linear.z) > JOTSTICK_THRESHHOLD){
        robot_driver_->robot_send_service_.robotServiceSetGlobalMoveEndMaxLineVelc(MAX_END_VEL*fabs(twist_.linear.z));
        robot_driver_->robot_send_service_.robotServiceTeachStart(aubo_robot_namespace::MOV_Z, twist_.linear.z>0?1:0);
      } else if(fabs(twist_.angular.x) > JOTSTICK_THRESHHOLD && fabs(twist_.angular.x)>fabs(twist_.angular.y)) {
        robot_driver_->robot_send_service_.robotServiceSetGlobalMoveEndMaxAngleVelc(MAX_END_VEL*fabs(twist_.angular.x));
        robot_driver_->robot_send_service_.robotServiceTeachStart(aubo_robot_namespace::ROT_X, twist_.angular.x>0?1:0);
      } else if(fabs(twist_.angular.y) > JOTSTICK_THRESHHOLD && fabs(twist_.angular.y)>fabs(twist_.angular.x)) {
        robot_driver_->robot_send_service_.robotServiceSetGlobalMoveEndMaxAngleVelc(MAX_END_VEL*fabs(twist_.angular.y));
        robot_driver_->robot_send_service_.robotServiceTeachStart(aubo_robot_namespace::ROT_Y, twist_.angular.y>0?1:0);
      } else if(fabs(twist_.angular.z) > JOTSTICK_THRESHHOLD) {
        robot_driver_->robot_send_service_.robotServiceSetGlobalMoveEndMaxAngleVelc(MAX_END_VEL*fabs(twist_.angular.z));
        robot_driver_->robot_send_service_.robotServiceTeachStart(aubo_robot_namespace::ROT_Z, twist_.angular.z>0?1:0);
      } else {
        // no vaild data come in, stop arm and reset linear & angular max velocity
        robot_driver_->robot_send_service_.robotServiceSetGlobalMoveEndMaxLineVelc(MAX_END_VEL);
        robot_driver_->robot_send_service_.robotServiceSetGlobalMoveEndMaxAngleVelc(MAX_END_VEL);
        robot_driver_->robot_send_service_.robotServiceTeachStop();
      }

      // update msg timeout
      msg_timeout_ += 1;

      // ROS_INFO("XYZ %.3f, %.3f, %.3f; RPY %.3f, %.3f, %.3f",
      //           twist_.linear.x, twist_.linear.y, twist_.linear.z,
      //           twist_.angular.x, twist_.angular.y, twist_.angular.z);
    }

    void VelocityControlCallback(const geometry_msgs::Twist::ConstPtr &msg) {
      // set timeout to 0
      msg_timeout_ = 0;

      twist_ = *msg;

      // ROS_INFO("get twist message");
      // ROS_INFO("XYZ %.3f, %.3f, %.3f; RPY %.3f, %.3f, %.3f",
      //     twist_.linear.x, twist_.linear.y, twist_.linear.z,
      //     twist_.angular.x, twist_.angular.y, twist_.angular.z);
    }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "velocity_control_node");
  ros::NodeHandle nh("~");

  aubo_driver::AuboDriver robot_driver;
  int ret = robot_driver.connectToRobotController();

  // if connect to a real robot, then you need initialize the dynamics parameters
  aubo_robot_namespace::ROBOT_SERVICE_STATE result;

  // set tool parameters
  aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
  memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));
  toolDynamicsParam.positionZ = 0.45;
  toolDynamicsParam.payload = 1.0;
  robot_driver.robot_send_service_.rootServiceRobotStartup( toolDynamicsParam,  // tool dynamics paramters
                                                            6,                  // collision class
                                                            true,               // is allowed to read robot pose
                                                            true,               // default
                                                            1000,               // default
                                                            result);            // initialize

  int res = robot_driver.robot_send_service_.robotServiceEnterTcp2CanbusMode();
  ROS_INFO("test %d", res);

  AuboJogControl aubo_jog_control(&robot_driver);

  ros::Subscriber velocity_control_sub = nh.subscribe("velocity_control", 10, &AuboJogControl::VelocityControlCallback, &aubo_jog_control);

  while(ros::ok()) {
    // update velocity command every time
    aubo_jog_control.UpdateVelocityCommand();

    ros::spinOnce();
  }

  ROS_INFO("all finish");

  return 0;
}
