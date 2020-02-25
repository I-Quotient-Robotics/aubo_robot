/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017-2018, AUBO Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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

#define MAX_JOINT_ACC 100.0/180.0*M_PI  //unit rad/s^2
#define MAX_JOINT_VEL 50.0/180.0*M_PI   //unit rad/s
#define MAX_END_ACC    4                // unit m/s^2
#define MAX_END_VEL    2                // unit m/s

geometry_msgs::Twist twist;


const double THRESHHOLD = 0.000001;
bool roadPointCompare(double *point1, double *point2)
{
    /** If there is a enough difference, then it will return true. **/
    bool ret = false;
    for(int i = 0; i < 6;i++)
    {
        if(fabs(point1[i] - point2[i]) >= THRESHHOLD)
        {
            ret = true;
            break;
        }
    }
    return ret;
}


void VelocityControl(AuboDriver &robot_driver) {
  /** Initialize move properties ***/
  robot_driver.robot_send_service_.robotServiceInitGlobalMoveProfile();

  /** Set Max joint acc and vel***/
  aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
  aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
  for(int i = 0; i < ARM_DOF; i++) {
    jointMaxAcc.jointPara[i] = MAX_JOINT_ACC;
    jointMaxVelc.jointPara[i] = MAX_JOINT_VEL;
  }
  robot_driver.robot_send_service_.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
  robot_driver.robot_send_service_.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

   /** move to inital position **/
  // int ret = robot_driver.robot_send_service_.robotServiceJointMove(initial_poeition, true);
  // if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
  //   ROS_ERROR("Failed to move to initial postion, error code:%d", ret);


  /** Initialize move properties ***/
  robot_driver.robot_send_service_.robotServiceInitGlobalMoveProfile();

  /** Set Max END acc and vel**/
  robot_driver.robot_send_service_.robotServiceSetGlobalMoveEndMaxLineAcc(MAX_END_ACC);
  robot_driver.robot_send_service_.robotServiceSetGlobalMoveEndMaxAngleAcc(MAX_END_ACC);
  robot_driver.robot_send_service_.robotServiceSetGlobalMoveEndMaxLineVelc(MAX_END_VEL);
  robot_driver.robot_send_service_.robotServiceSetGlobalMoveEndMaxAngleVelc(MAX_END_VEL);

  int result;
  aubo_robot_namespace::wayPoint_S wayPoint;
  result = robot_driver.robot_send_service_.robotServiceGetCurrentWaypointInfo(wayPoint);

  if(result == aubo_robot_namespace::ErrnoSucc) {
    int i = 100;
    ros::Rate r(5);
    while(ros::ok() && i>0) {
      result = robot_driver.robot_send_service_.robotServiceGetCurrentWaypointInfo(wayPoint);
      // ROS_INFO("Get pos %lf %lf %lf, orient %lf %lf %lf %lf",
      // wayPoint.cartPos.position.x, wayPoint.cartPos.position.y, wayPoint.cartPos.position.z,
      // wayPoint.orientation.w, wayPoint.orientation.x, wayPoint.orientation.y, wayPoint.orientation.z);

      if(i % 2 == 0) {
        wayPoint.cartPos.position.y += 0.1;
      } else {
        wayPoint.cartPos.position.x += 0.1;
      }

      aubo_robot_namespace::wayPoint_S new_wayPoint;
      std::vector<aubo_robot_namespace::wayPoint_S> wayPointVector;

      result = robot_driver.robot_send_service_.robotServiceRobotIk(wayPoint.jointpos, wayPoint.cartPos.position, wayPoint.orientation, new_wayPoint);
      // ROS_INFO("Get pos %lf %lf %lf, orient %lf %lf %lf %lf",
      // new_wayPoint.cartPos.position.x, new_wayPoint.cartPos.position.y, new_wayPoint.cartPos.position.z,
      // new_wayPoint.orientation.w, new_wayPoint.orientation.x, new_wayPoint.orientation.y, new_wayPoint.orientation.z);

      // result = robot_driver.robot_send_service_.robotServiceRobotIk(wayPoint.cartPos.position, wayPoint.orientation, wayPointVector);
      // ROS_INFO("vector length: %d", wayPointVector.size());
      // result = robot_driver.robot_send_service_.robotServiceLineMove(new_wayPoint, true);
      result = robot_driver.robot_send_service_.robotServiceFollowModeJointMove(new_wayPoint.jointpos);
      ROS_INFO("result %d", result);
      i -= 1;
      r.sleep();
    }
  }
}

void VelocityControlCallback(const geometry_msgs::Twist::ConstPtr &msg) {
  ROS_INFO("get info");
  twist = *msg;
}

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

  /** Initialize move properties ***/
  robot_driver.robot_send_service_.robotServiceInitGlobalMoveProfile();

  /** Set Max joint acc and vel***/
  aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
  aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
  for(int i = 0; i < ARM_DOF; i++) {
    jointMaxAcc.jointPara[i] = MAX_JOINT_ACC;
    jointMaxVelc.jointPara[i] = MAX_JOINT_VEL;
  }
  robot_driver.robot_send_service_.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
  robot_driver.robot_send_service_.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

   /** move to inital position **/
  // ret = robot_driver.robot_send_service_.robotServiceJointMove(initial_poeition, true);
  // if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
  //   ROS_ERROR("Failed to move to initial postion, error code:%d", ret);


  /** Initialize move properties ***/
  robot_driver.robot_send_service_.robotServiceInitGlobalMoveProfile();

  /** Set Max END acc and vel**/
  robot_driver.robot_send_service_.robotServiceSetGlobalMoveEndMaxLineAcc(MAX_END_ACC);
  robot_driver.robot_send_service_.robotServiceSetGlobalMoveEndMaxAngleAcc(MAX_END_ACC);
  robot_driver.robot_send_service_.robotServiceSetGlobalMoveEndMaxLineVelc(MAX_END_VEL);
  robot_driver.robot_send_service_.robotServiceSetGlobalMoveEndMaxAngleVelc(MAX_END_VEL);

  ros::Subscriber velocity_control_sub = nh.subscribe("velocity_control", 10, VelocityControlCallback);

  aubo_robot_namespace::wayPoint_S wayPoint;
  int result_1 = robot_driver.robot_send_service_.robotServiceGetCurrentWaypointInfo(wayPoint);
  while(ros::ok()) {
    result_1 = robot_driver.robot_send_service_.robotServiceGetCurrentWaypointInfo(wayPoint);
    ROS_INFO( "Get pos %lf %lf %lf, orient %lf %lf %lf %lf",
              wayPoint.cartPos.position.x, wayPoint.cartPos.position.y, wayPoint.cartPos.position.z,
              wayPoint.orientation.w, wayPoint.orientation.x, wayPoint.orientation.y, wayPoint.orientation.z);

    wayPoint.cartPos.position.x += twist.linear.x;

    aubo_robot_namespace::wayPoint_S new_wayPoint;
    std::vector<aubo_robot_namespace::wayPoint_S> wayPointVector;

    result_1 = robot_driver.robot_send_service_.robotServiceRobotIk(wayPoint.jointpos, wayPoint.cartPos.position, wayPoint.orientation, new_wayPoint);
    ROS_INFO("New pos %lf %lf %lf, orient %lf %lf %lf %lf",
    new_wayPoint.cartPos.position.x, new_wayPoint.cartPos.position.y, new_wayPoint.cartPos.position.z,
    new_wayPoint.orientation.w, new_wayPoint.orientation.x, new_wayPoint.orientation.y, new_wayPoint.orientation.z);

    if(roadPointCompare(wayPoint.jointpos, new_wayPoint.jointpos)) {
      result_1 = robot_driver.robot_send_service_.robotServiceLineMove(new_wayPoint, true);
      // result_1 = robot_driver.robot_send_service_.robotServiceFollowModeJointMove(new_wayPoint.jointpos);
      ROS_INFO("go result %d", result_1);
    }

    ros::spinOnce();
  }

  // robot_driver.robot_send_service_.rootServiceRobotMoveControl(aubo_robot_namespace::RobotMoveStop);
  ROS_INFO("all finish");

  return 0;
}
