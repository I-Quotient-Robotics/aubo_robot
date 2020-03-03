#include "aubo_driver/aubo_driver.h"
#include "aubo_driver.cpp"

#include <geometry_msgs/Twist.h>

#include <string>
#include <cstdlib>
#include <unistd.h>
#include <math.h>
#include <stdio.h>
#include <sstream>
#include <fstream>

using namespace aubo_driver;

#define MAX_END_ACC     1.0                 // unit m/s^2
#define MAX_END_VEL     0.1                // unit m/s
#define MAX_JOINT_ACC   100.0/180.0*M_PI    // unit rad/s^2
#define MAX_JOINT_VEL   60.0/180.0*M_PI     // unit rad/s

const double THRESHHOLD = 0.000001;

class AuboJogControl {
  public:
    int msg_timeout_;

    AuboDriver *robot_driver_;
    geometry_msgs::Twist twist_;

  public:
    AuboJogControl(AuboDriver *driver) {
      robot_driver_ = driver;

      /** Set Max joint acc and vel***/
      aubo_robot_namespace::JointVelcAccParam joint_max_acc_;
      aubo_robot_namespace::JointVelcAccParam joint_max_velc_;
      for(int i = 0; i < 6; i++) {
        joint_max_acc_.jointPara[i] = MAX_JOINT_ACC;
        joint_max_velc_.jointPara[i] = MAX_JOINT_VEL;
      }

      /** Initialize move properties ***/
      robot_driver_->robot_send_service_.robotServiceInitGlobalMoveProfile();

      /** Set Max END acc and vel**/
      robot_driver_->robot_send_service_.robotServiceSetGlobalMoveJointMaxAcc(joint_max_acc_);
      robot_driver_->robot_send_service_.robotServiceSetGlobalMoveJointMaxVelc(joint_max_velc_);

      robot_driver_->robot_send_service_.robotServiceSetGlobalMoveEndMaxLineAcc(MAX_END_ACC);
      robot_driver_->robot_send_service_.robotServiceSetGlobalMoveEndMaxLineVelc(MAX_END_VEL);

      robot_driver_->robot_send_service_.robotServiceSetGlobalMoveEndMaxAngleAcc(MAX_END_ACC);
      robot_driver_->robot_send_service_.robotServiceSetGlobalMoveEndMaxAngleVelc(MAX_END_VEL);
    }

    ~AuboJogControl() {
      robot_driver_->robot_send_service_.robotServiceTeachStop();
    }

    bool RoadPointCompare(double *point1, double *point2) {
      /** If there is a enough difference, then it will return true. **/
      bool ret = false;
      for(int i = 0; i < 6;i++) {
        if(fabs(point1[i] - point2[i]) >= THRESHHOLD) {
          ret = true;
          break;
        }
      }
      return ret;
    }

    void UpdateVelocityCommand() {
      int result;

      if(msg_timeout_ > 20) {
        robot_driver_->robot_send_service_.robotServiceSetGlobalMoveEndMaxLineVelc(MAX_END_VEL);
        robot_driver_->robot_send_service_.robotServiceSetGlobalMoveEndMaxAngleVelc(MAX_END_VEL);
        robot_driver_->robot_send_service_.robotServiceTeachStop();
      } else if(fabs(twist_.linear.x) > 0.02 && fabs(twist_.linear.x)>fabs(twist_.linear.y)) {
        robot_driver_->robot_send_service_.robotServiceSetGlobalMoveEndMaxLineVelc(MAX_END_VEL*fabs(twist_.linear.x));
        robot_driver_->robot_send_service_.robotServiceTeachStart(aubo_robot_namespace::MOV_X, twist_.linear.x>0?1:0);
      } else if(fabs(twist_.linear.y) > 0.02 && fabs(twist_.linear.y)>fabs(twist_.linear.x)) {
        robot_driver_->robot_send_service_.robotServiceSetGlobalMoveEndMaxLineVelc(MAX_END_VEL*fabs(twist_.linear.y));
        robot_driver_->robot_send_service_.robotServiceTeachStart(aubo_robot_namespace::MOV_Y, twist_.linear.y>0?1:0);
      } else if(fabs(twist_.linear.z) > 0.02){
        robot_driver_->robot_send_service_.robotServiceSetGlobalMoveEndMaxLineVelc(MAX_END_VEL*fabs(twist_.linear.z));
        robot_driver_->robot_send_service_.robotServiceTeachStart(aubo_robot_namespace::MOV_Z, twist_.linear.z>0?1:0);
      } else if(fabs(twist_.angular.x) > 0.02 && fabs(twist_.angular.x)>fabs(twist_.angular.y)) {
        robot_driver_->robot_send_service_.robotServiceSetGlobalMoveEndMaxAngleVelc(MAX_END_VEL*fabs(twist_.angular.x));
        robot_driver_->robot_send_service_.robotServiceTeachStart(aubo_robot_namespace::ROT_X, twist_.angular.x>0?1:0);
      } else if(fabs(twist_.angular.y) > 0.02 && fabs(twist_.angular.y)>fabs(twist_.angular.x)) {
        robot_driver_->robot_send_service_.robotServiceSetGlobalMoveEndMaxAngleVelc(MAX_END_VEL*fabs(twist_.angular.y));
        robot_driver_->robot_send_service_.robotServiceTeachStart(aubo_robot_namespace::ROT_Y, twist_.angular.y>0?1:0);
      } else if(fabs(twist_.angular.z) > 0.02) {
        robot_driver_->robot_send_service_.robotServiceSetGlobalMoveEndMaxAngleVelc(MAX_END_VEL*fabs(twist_.angular.z));
        robot_driver_->robot_send_service_.robotServiceTeachStart(aubo_robot_namespace::ROT_Z, twist_.angular.z>0?1:0);
      } else {
        robot_driver_->robot_send_service_.robotServiceSetGlobalMoveEndMaxLineVelc(MAX_END_VEL);
        robot_driver_->robot_send_service_.robotServiceSetGlobalMoveEndMaxAngleVelc(MAX_END_VEL);
        robot_driver_->robot_send_service_.robotServiceTeachStop();
      }

      msg_timeout_ += 1;

      // ROS_INFO("XYZ %.3f, %.3f, %.3f; RPY %.3f, %.3f, %.3f",
      //           twist_.linear.x, twist_.linear.y, twist_.linear.z,
      //           twist_.angular.x, twist_.angular.y, twist_.angular.z);
    }

    void VelocityControlCallback(const geometry_msgs::Twist::ConstPtr &msg) {
      // ROS_INFO("get info");

      msg_timeout_ = 0;
      twist_ = *msg;
    }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "velocity_control");
  ros::NodeHandle nh("~");

  AuboDriver robot_driver;
  int ret = robot_driver.connectToRobotController();

  /** If connect to a real robot, then you need initialize the dynamics parameters　**/
  aubo_robot_namespace::ROBOT_SERVICE_STATE result;
  //tool parameters
  aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
  memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));

  robot_driver.robot_send_service_.rootServiceRobotStartup( toolDynamicsParam/**tool dynamics paramters**/,
                                                            6        /*collision class*/,
                                                            true     /* Is allowed to read robot pose*/,
                                                            true,    /*default */
                                                            1000,    /*default */
                                                            result); /*initialize*/

  AuboJogControl aubo_jog_control(&robot_driver);

  ros::Subscriber velocity_control_sub = nh.subscribe("velocity_control", 10, &AuboJogControl::VelocityControlCallback, &aubo_jog_control);

  // aubo_robot_namespace::wayPoint_S wayPoint;
  // int result_1 = robot_driver_->robot_send_service_.robotServiceGetCurrentWaypointInfo(wayPoint);
  while(ros::ok()) {
    // if(abs(twist.linear.x) > 0.2 && abs(twist.linear.x)>abs(twist.linear.y)) {
    //   robot_driver_->robot_send_service_.robotServiceTeachStart(aubo_robot_namespace::MOV_X, twist.linear.x>0?1:0);
    // } else if(abs(twist.linear.y) > 0.2 && abs(twist.linear.y)>abs(twist.linear.x)) {
    //   robot_driver_->robot_send_service_.robotServiceTeachStart(aubo_robot_namespace::MOV_Y, twist.linear.y>0?1:0);
    // } else if(abs(twist.linear.z) > 0.2){
    //   robot_driver_->robot_send_service_.robotServiceTeachStart(aubo_robot_namespace::MOV_Z, twist.linear.z>0?1:0);
    // } else {
    //   robot_driver_->robot_send_service_.robotServiceTeachStop();
    // }
    aubo_jog_control.UpdateVelocityCommand();
    ros::spinOnce();
  }

  // robot_driver_->robot_send_service_.rootServiceRobotMoveControl(aubo_robot_namespace::RobotMoveStop);
  ROS_INFO("all finish");

  return 0;
}
