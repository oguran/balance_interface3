/*
 * Copyright (C) 2013-2014 Sony Corporation
 */

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>

#include <stdio.h>
//#include <stdlib.h>   //abs() for integer
#include <cmath>
#include <algorithm>

#include "balance_interface/balance_interface.h"

BalanceIF::BalanceIF()
{
}

BalanceIF::~BalanceIF()
{
}


int BalanceIF::Init()
{
  m_targetPos.x = 0.00;
  m_targetPos.y = 0.00;
  m_controlMode = BlancerCtrlMode_Initialize;

  m_currentPos.x = 0.00;
  m_currentPos.y = 0.00;

  m_savePos.x = 0.00;
  m_savePos.y = 0.00;

  m_saveLeftEnc = 0;
  m_saveRightEnc = 0;

  m_shortPrevLeftEnc = 0;
  m_shortPrevRightEnc = 0;
  m_multiTurnCntLeft = 0;
  m_multiTurnCntRight = 0;

  m_obstainPos.x = 0.00;
  m_obstainPos.y = 0.00;

  m_waitForStability.clear();

  return 0;
}

void BalanceIF::Start()
{
  ros::NodeHandle nh;

  // m_p0 = ros::Time::now();
  // m_cnt = 0;

//for balance
  balance_sub = nh.subscribe(
              "/bl/balance_odm", 10,
              &BalanceIF::BalanceCallback, this);
  balance_pub = nh.advertise<balance_msg::BalanceCmd>("/bl/balance_cmd", 10);

//for ui
  ui_sub = nh.subscribe(
            "move_base_simple/goal", 10,
            &BalanceIF::SetGoalCallback, this);
  ui_pub = nh.advertise<geometry_msgs::PoseStamped>("/notify_pos", 10);

  //for ui (obstacle pos)
  ui_obstacle_sub = nh.subscribe(
            "clicked_point", 10,
            &BalanceIF::SetObstacleCallback, this);

  ros::Duration(0.5).sleep();

  //start balancer @note speed:0
  SetBalancerSpeed(0, 0);
}

void BalanceIF::Stop()
{
}

void BalanceIF::BalanceCallback(const balance_msg::BalanceOdm& odm_msg)
{
  int left_speed, right_speed;
  int intLeftEnc, intRightEnc;

  //from balancer (notify odm)
  // printf("BalanceCB: le=%d re%d s1=%d s2=%d ls=%d rs=%d \n",
    // odm_msg.left_enc, odm_msg.right_enc, odm_msg.status_1, odm_msg.status_2, odm_msg.left_speed, odm_msg.right_speed);
  
  // Calc for over flow short encoder count.
  intLeftEnc = (int)odm_msg.left_enc - (int)m_shortPrevLeftEnc;
  intRightEnc = (int)odm_msg.right_enc - (int)m_shortPrevRightEnc;
  if (std::abs(intLeftEnc) > 0x7FFF) {
    if (intLeftEnc > 0) m_multiTurnCntLeft--;
    else m_multiTurnCntLeft++;
  }
  if (std::abs(intRightEnc) > 0x7FFF) {
    if (intRightEnc > 0) m_multiTurnCntRight--;
    else m_multiTurnCntRight++;
  }
  intLeftEnc  = m_multiTurnCntLeft  * 0x10000 + odm_msg.left_enc;
  intRightEnc = m_multiTurnCntRight * 0x10000 + odm_msg.right_enc;
  m_shortPrevLeftEnc = odm_msg.left_enc;
  m_shortPrevRightEnc = odm_msg.right_enc;
  //printf("intLeftEnc: %d, intRightEnc: %d, odm_msg.left_enc: %d, odm_msg.right_enc: %d\n", intLeftEnc, intRightEnc, odm_msg.left_enc, odm_msg.right_enc);

  //Calc balance Odometry to ui postion
  MotionControl(odm_msg.status_1, intLeftEnc, intRightEnc, &left_speed, &right_speed);

  if (odm_msg.status_1) {
    //set balancer speed To Arduino
    SetBalancerSpeed(left_speed, right_speed);
  }

  //Notify Current Pos To Rviz
  NotifyCurrentPos(m_currentPos.x, m_currentPos.y, m_currentPos.angle);

  // printf("\n");
}

const double PI = 3.141592;
const double TIRE_RADIUS = 0.040; // m
const double MOTOR_RATIO_IN = 51.45;
const double MOTOR_RATIO_OUT = 1.0;
const double GEAR_RATIO_IN = 45.0;
const double GEAR_RATIO_OUT = 21.0;
const double ENCODER_CNT = 12.0;
const double TIRE_AROUNT_ENCODER_CNT = (MOTOR_RATIO_IN / MOTOR_RATIO_OUT) * (GEAR_RATIO_IN / GEAR_RATIO_OUT) * ENCODER_CNT;
const double TIRE_CIRCUMFERENCE =  2 * PI * TIRE_RADIUS; // m
const double WHEEL_DISTANCE = 0.100; // m
const double BALANCER_CIRCUMFERENCE = WHEEL_DISTANCE * PI; // m
const int MOVE_SPEED_GETTING_OVER = 22;
const int MOVE_SPEED_STRAIGHT_HIGH = 12;
const int MOVE_SPEED_STRAIGHT_MIDDLE = 5;
const int MOVE_SPEED_STRAIGHT_SLOW = 3;
const int MOVE_SPEED_ROTATE = 1;
const int MOVE_SPEED_MAX = 300;
const double COLD_ZONE = 0.010; // m
const double SLOW_ZONE = 0.100; // m
const double MIDDLE_ZONE = 0.200; // m
const double WAIT_SEC = 2.0;
const double COLD_ANGLE = PI / (180 * 2); // radian
const double OBSTAIN_ZONE = 0.3;

namespace {
  inline double angle_normalize(double angle) {
    if (PI < angle) {
      return angle_normalize(angle - 2 * PI);
    }
    if (-PI > angle) {
      return angle_normalize(angle + 2 * PI);
    }
    return angle;
  }
}

void BalanceIF::MotionControl(char start, int left_enc, int right_enc, int *left_speed, int *right_speed)
{
  int aveEncCnt, speed;
  double distance, distanceDiff, absDiistanceDiff, angleDiff, targetAngle, targetDirection;

  // m_cnt++;
  // ros::Time p1 = ros::Time::now();
  // ros::Duration duration = p1 - m_p0;
  // if (duration > ros::Duration(1.0)) {
  //   printf("%f Hz (%d/%f)\n", (m_cnt / duration.toSec()), m_cnt, duration.toSec());
  //   m_p0 = p1;
  //   m_cnt = 0;
  // }

  *left_speed = *right_speed = 0;
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0;  // +:forward, -:backword. m/s.
  cmd_vel.linear.y = 0;
  cmd_vel.linear.z = 0;
  cmd_vel.angular.x = 0;
  cmd_vel.angular.y = 0;
  cmd_vel.angular.z = 0;  // -pi to pi. +:left, -:right. rad/s.
  if (m_waitForStability.isWaiting()) {
    BuffPrint("Waiting\n");
    if (m_waitForStability.isFinish()) {
      m_waitForStability.clear();
    }
  } else if (m_controlMode == BlancerCtrlMode_Initialize) {
    m_saveLeftEnc = left_enc;
    m_saveRightEnc = right_enc;
    m_currentPos.clear();
    m_targetPos.clear();
    m_savePos.clear();
    m_waitForStability.start(WAIT_SEC); // wait for stabiliry
    m_controlMode = BlancerCtrlMode_Standby;
    m_speed = MOVE_SPEED_MAX;
    BuffPrint("Initialized\n");
  } else if (m_controlMode == BlancerCtrlMode_Standby) {
    //Calc Distance Diff between current and target.
    if (std::abs(m_targetPos.x - m_currentPos.x) >= COLD_ZONE) {
      m_controlMode = BlancerCtrlMode_Rotation_On_X;
    } else if (std::abs(m_targetPos.y - m_currentPos.y) >= COLD_ZONE) {
      m_controlMode = BlancerCtrlMode_Rotation_On_Y;
    }

    if (m_controlMode != BlancerCtrlMode_Standby) {
      m_saveLeftEnc = left_enc;
      m_saveRightEnc = right_enc;
      m_savePos = m_currentPos;
      BuffPrint("Go to Target\n");
    }
    m_speed = MOVE_SPEED_MAX;
  } else if ((m_controlMode == BlancerCtrlMode_Rotation_On_X) ||
      (m_controlMode == BlancerCtrlMode_Rotation_On_Y)) {
    if (m_controlMode == BlancerCtrlMode_Rotation_On_X) {
      targetAngle = (m_targetPos.x - m_savePos.x >= 0.0) ? 0.0 : PI;
    } else {
      targetAngle = (m_targetPos.y - m_savePos.y >= 0.0) ? PI / 2 : -PI / 2;
    }

    //Encoder Ave Calc @fix left Only
    // aveEncCnt = (std::abs(left_enc - m_saveLeftEnc) + std::abs(right_enc - m_saveRightEnc)) / 2;
    //distance calc
    // distance = aveEncCnt * (TIRE_CIRCUMFERENCE / TIRE_AROUNT_ENCODER_CNT);

    // distanceDiff = (BALANCER_CIRCUMFERENCE / 4.0) - distance; // rotate 45 digrees
    // angleDiff = (PI / 2.0) * distance / (BALANCER_CIRCUMFERENCE / 4.0);

    // previous encode data
    static EncodeData prv_enc;
    if(!prv_enc.enable) {
      prv_enc.enable = true;
      prv_enc.left = m_saveLeftEnc;
      prv_enc.right = m_saveRightEnc;
    }

    aveEncCnt = ((prv_enc.left - left_enc) - (prv_enc.right - right_enc)) / 2;
    m_currentPos.angle = m_currentPos.angle + (double)aveEncCnt / TIRE_AROUNT_ENCODER_CNT * 2.0 * PI;
    m_currentPos.angle = angle_normalize(m_currentPos.angle);
    angleDiff = angle_normalize(targetAngle - m_currentPos.angle);

    prv_enc.left = left_enc;
    prv_enc.right = right_enc;

    // std::cout << "left=" << m_saveLeftEnc << " -> " << left_enc << " right=" << m_saveRightEnc << " -> " << right_enc << " ave=" << aveEncCnt << std::endl;
    // std::cout << "angle cur=" << m_currentPos.angle << " tgt=" << targetAngle << " diff=" << angleDiff << std::endl;

    //Cold Zone
    if (std::abs(angleDiff) < COLD_ANGLE) {
      angleDiff = 0;
      m_currentPos.angle = targetAngle;
    }

    if (angleDiff < 0) {
      //rotation right
      cmd_vel.angular.z = 0.1 * MOVE_SPEED_ROTATE;  // fix me
      BuffPrint("Rotation Right\n");
     } else if (angleDiff > 0) {
      //rotation left
      cmd_vel.angular.z = -0.1 * MOVE_SPEED_ROTATE;  // fix me
      BuffPrint("Rotation Left\n");
    } else {
      //Stop
      BuffPrint("Rotation Stop\n");

      //Mode Change
      if (m_controlMode == BlancerCtrlMode_Rotation_On_X) {
        m_controlMode = BlancerCtrlMode_Move_X_Direction;
      } else if (m_controlMode == BlancerCtrlMode_Rotation_On_Y){
        m_controlMode = BlancerCtrlMode_Move_Y_Direction;
      }

      m_waitForStability.start(WAIT_SEC); // wait for stabiliry

      prv_enc.enable = false;

      m_saveLeftEnc = left_enc;
      m_saveRightEnc = right_enc;
      m_savePos = m_currentPos;
      m_speed = MOVE_SPEED_MAX;
    }
  } else if ((m_controlMode == BlancerCtrlMode_Move_X_Direction) ||
      (m_controlMode == BlancerCtrlMode_Move_Y_Direction)) {
    if (m_controlMode == BlancerCtrlMode_Move_X_Direction) {
      targetDirection = m_targetPos.x - m_savePos.x;
    } else {
      targetDirection = m_targetPos.y - m_savePos.y;
    }
    targetDirection = targetDirection >= 0 ? 1 : -1;

    //Encoder Ave Calc
    aveEncCnt = ((left_enc - m_saveLeftEnc) + (right_enc - m_saveRightEnc)) / 2;
    //distance calc
    distance = aveEncCnt * (TIRE_CIRCUMFERENCE / TIRE_AROUNT_ENCODER_CNT);

    // std::cout << "left=" << m_saveLeftEnc << " -> " << left_enc << " right=" << m_saveRightEnc << " -> " << right_enc << " ave=" << aveEncCnt << std::endl;

    if (m_controlMode == BlancerCtrlMode_Move_X_Direction) {
      //X
      m_currentPos.x = m_savePos.x + distance * targetDirection;
      distanceDiff = (m_targetPos.x - m_currentPos.x) * targetDirection;
      // std::cout << "pos x cur=" << m_currentPos.x << " tgt=" << m_targetPos.x << " diff=" << distanceDiff << std::endl;
    } else {
      //Y
      m_currentPos.y = m_savePos.y + distance * targetDirection;
      distanceDiff = (m_targetPos.y - m_currentPos.y) * targetDirection;
      // std::cout << "pos y cur=" << m_currentPos.y << " tgt=" << m_targetPos.y << " diff=" << distanceDiff << std::endl;
    }

    //Cold Zone
    if (std::abs(distanceDiff) < COLD_ZONE) {
      distanceDiff = 0;
      speed = 0;
    } else if (std::abs(distanceDiff) < SLOW_ZONE) {
      if (m_speed > MOVE_SPEED_STRAIGHT_SLOW) {
        speed = MOVE_SPEED_STRAIGHT_SLOW;
      }
    } else if (std::abs(distanceDiff) < MIDDLE_ZONE) {
      if (m_speed > MOVE_SPEED_STRAIGHT_MIDDLE) {
        speed = MOVE_SPEED_STRAIGHT_MIDDLE;
      }
    } else {
		speed = MOVE_SPEED_STRAIGHT_HIGH;
		if(m_controlMode == BlancerCtrlMode_Move_X_Direction) {
			if(std::abs(m_obstainPos.x - m_currentPos.x) < OBSTAIN_ZONE) {
				speed = MOVE_SPEED_GETTING_OVER;
			}
		}
		else if(m_controlMode == BlancerCtrlMode_Move_Y_Direction) {
			if(std::abs(m_obstainPos.y - m_currentPos.y) < OBSTAIN_ZONE) {
				speed = MOVE_SPEED_GETTING_OVER;
			}
		}
    }

    if (distanceDiff > 0) {
      //Advance
      cmd_vel.linear.x = 0.1 * speed; // fix me
      BuffPrint("Advance\n");
    } else if (distanceDiff < 0) {
      //Back
      cmd_vel.linear.x = -0.1 * speed; // fix me
      BuffPrint("Back\n");
    } else {
      //Stop
      cmd_vel.linear.x = 0.1 * speed; // fix me
      BuffPrint("Stop\n");

      m_waitForStability.start(WAIT_SEC); // wait for stabiliry

      //Mode Change
      if (m_controlMode == BlancerCtrlMode_Move_X_Direction) {
        if (std::abs(m_targetPos.y - m_currentPos.y) < COLD_ZONE) {
          m_controlMode = BlancerCtrlMode_Goal;
        } else {
          m_controlMode = BlancerCtrlMode_Rotation_On_Y;
        }
      } else {
        m_controlMode = BlancerCtrlMode_Goal;
      }
      m_saveLeftEnc = left_enc;
      m_saveRightEnc = right_enc;
      m_savePos = m_currentPos;
      m_speed = MOVE_SPEED_MAX; 
    }
  } else {
    //Goal
    BuffPrint("Goal\n");

    //Mode Change
    m_controlMode = BlancerCtrlMode_Standby;
  }

  *left_speed = 0;
  *right_speed = 0;
  if (cmd_vel.linear.x != 0) {
    *left_speed = *right_speed = cmd_vel.linear.x * 10;  // fix me : m/s -> speed.
  } else if (cmd_vel.angular.z != 0) {
    double speed = cmd_vel.angular.z * 10;  // fix me,  +:right, -:left : rad/s -> speed.
    *right_speed = -speed;
    *left_speed = speed;
  }

  return;
}

void BalanceIF::SetBalancerSpeed(int left_speed, int right_speed)
{
  balance_msg::BalanceCmd balance_cmd_msg;

  balance_cmd_msg.left_speed = left_speed;
  balance_cmd_msg.right_speed = right_speed;
  balance_cmd_msg.flag_1 = 1; //echo cmd
  balance_cmd_msg.flag_2 = 0;

  balance_pub.publish(balance_cmd_msg);
  // printf("balance_cmd publish, ls: %d, rs: %d\n", balance_cmd_msg.left_speed, balance_cmd_msg.right_speed);

  return;
}

void BalanceIF::SetGoalCallback(const geometry_msgs::PoseStamped& msg)
{
  if (m_controlMode != BlancerCtrlMode_Standby) {
    printf("SetGoalCB: ignored. x=%f, y=%f\n", msg.pose.position.x, msg.pose.position.y);
    return;
  }
  //from rvis
  printf("SetGoalCB: x=%f, y=%f\n", msg.pose.position.x, msg.pose.position.y);

  m_targetPos.x = msg.pose.position.x;
  m_targetPos.y = msg.pose.position.y;

  //start balancer @note speed:0 = Start Ctrl
  //SetBalancerSpeed(0, 0);
}

void BalanceIF::SetObstacleCallback(const geometry_msgs::PointStamped& msg)
{
  //from rvis
  printf("SetObstacleCB: x=%f, y=%f\n", msg.point.x, msg.point.y);

  m_obstainPos.x = msg.point.x;
  m_obstainPos.y = msg.point.y;
}

void BalanceIF::NotifyCurrentPos(double x_pos, double y_pos, double angle)
{
  geometry_msgs::PoseStamped msg;
  static int seqNo = 0;

  memset(&msg, 0, sizeof(msg));
  msg.header.seq = seqNo++;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "base";
  
  msg.pose.position.x = x_pos;
  msg.pose.position.y = y_pos;
  msg.pose.position.z = 0.0;

//  msg.pose.orientation = tf::transformations::quaternion_from_euler(0.0, 0.0, angle);
  msg.pose.orientation = tf::createQuaternionMsgFromYaw(angle);

  ui_pub.publish(msg);
  // printf("notify_pos publish, x: %f, y: %f\n", msg.pose.position.x,  msg.pose.position.y);

  return;
}


////////////////////////// END /////////////////////////
