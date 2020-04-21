/*
 * Copyright (C) 2013-2014 Sony Corporation
 */

#include <ros/ros.h>

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
  //m_targetPos.x = 100.00;
  //m_targetPos.y = 100.00;
  m_targetPos.x = 0.00;
  m_targetPos.y = 0.00;
  m_controlMode = BlancerCtrlMode_Initialize;

  m_currentPos.x = 0.00;
  m_currentPos.y = 0.00;

  m_saveLeftEnc = 0;
  m_saveRightEnc = 0;

  return 0;
}

void BalanceIF::Start()
{
  ros::NodeHandle nh;

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

  //from balancer (notify odm)
  printf("BalanceCB: le=%d re%d s1=%d s2=%d ls=%d rs=%d \n",
    odm_msg.left_enc, odm_msg.right_enc, odm_msg.status_1, odm_msg.status_2, odm_msg.left_speed, odm_msg.right_speed);
  
  //Calc balance Odometry to ui postion
  MotionControl(odm_msg.status_1, odm_msg.left_enc, odm_msg.right_enc, &left_speed, &right_speed);

  if (odm_msg.status_1) {
    //set balancer speed To Arduino
    SetBalancerSpeed(left_speed, right_speed);
  }

  //Notify Current Pos To Rviz
  NotifyCurrentPos(m_currentPos.x, m_currentPos.y);

  printf("\n");
}

const float PI = 3.14;
const float TIRE_RADIUS = 4.0; // cm
const float MOTOR_RATIO_IN = 51.45;
const float MOTOR_RATIO_OUT = 1.0;
const float GEAR_RATIO_IN = 45.0;
const float GEAR_RATIO_OUT = 21.0;
const float ENCODER_CNT = 12.0;
const float TIRE_AROUNT_ENCODER_CNT = (MOTOR_RATIO_IN / MOTOR_RATIO_OUT) * (GEAR_RATIO_IN / GEAR_RATIO_OUT) * ENCODER_CNT;
const float TIRE_CIRCUMFERENCE =  2 * PI * TIRE_RADIUS;
const float WHEEL_DISTANCE = 10.0;
const float BALANCER_CIRCUMFERENCE = WHEEL_DISTANCE * PI; // cm
const int MOVE_SPEED = 2;
//#define TIRE_AROUNT_ENCODER_CNT  (1332)
//#define TIRE_CIRCUMFERENCE       (25)    //cm
//#define BALANCER_CIRCUMFERENCE   (36)    //cm
//#define MOVE_SPEED               (1)


void BalanceIF::MotionControl(char start, int left_enc, int right_enc, int *left_speed, int *right_speed)
{
  int aveEncCnt;
  float distance, distanceDiff;

  *left_speed = *right_speed = 0;
  if (m_controlMode == BlancerCtrlMode_Initialize) {
    m_saveLeftEnc = left_enc;
    m_saveRightEnc = right_enc;
    m_currentPos.x = 0.00;
    m_currentPos.y = 0.00;
    m_targetPos.x = 0.00;
    m_targetPos.y = 0.00;
    m_controlMode = BlancerCtrlMode_Check;
    printf("Initialized\n");
  } else if (m_controlMode == BlancerCtrlMode_Check) {
    //Calc Distance Diff between current and target.
    distanceDiff = std::max(std::abs(m_targetPos.x - m_currentPos.x), std::abs(m_targetPos.y - m_currentPos.y));
    //Cold Zone 3cm
    if (distanceDiff >= 1) {
      m_controlMode = BlancerCtrlMode_Move_X_Direction;
      m_saveLeftEnc = left_enc;
      m_saveRightEnc = right_enc;
      printf("Go to Target\n");
    }
  } else if ((m_controlMode == BlancerCtrlMode_Move_X_Direction) ||
      (m_controlMode == BlancerCtrlMode_Move_Y_Direction)) {
    //Move

    //Encoder Ave Calc
    aveEncCnt = ((left_enc - m_saveLeftEnc) + (right_enc - m_saveRightEnc)) / 2;
    //distance calc
    distance = aveEncCnt * (TIRE_CIRCUMFERENCE / TIRE_AROUNT_ENCODER_CNT);

    if (m_controlMode == BlancerCtrlMode_Move_X_Direction) {
      //X
      m_currentPos.x = distance;
      distanceDiff = m_targetPos.x - m_currentPos.x;
    } else {
      //Y
      m_currentPos.y = distance;
      distanceDiff = m_targetPos.y - m_currentPos.y;
    }
    //Cold Zone 3cm
    if (std::abs(distanceDiff) < 1) distanceDiff = 0;

    if (distanceDiff > 0) {
      //Advance
      *left_speed = MOVE_SPEED;
      *right_speed = MOVE_SPEED;
      printf("Advance\n");
    } else if (distanceDiff < 0) {
      //Back
      *left_speed = -MOVE_SPEED;
      *right_speed = -MOVE_SPEED;
      printf("Back\n");
    } else {
      //Stop
      *left_speed = 0;
      *right_speed = 0;
      printf("Stop\n");

      //Mode Change
      if (m_controlMode == BlancerCtrlMode_Move_X_Direction) {
        m_controlMode = BlancerCtrlMode_Rotation;
      } else {
        m_controlMode = BlancerCtrlMode_Goal;
      }
      m_saveLeftEnc = left_enc;
      m_saveRightEnc = right_enc;
    }

  } else if (m_controlMode == BlancerCtrlMode_Rotation) {
    //rotation
    //Encoder Ave Calc @fix left Only
    aveEncCnt = (std::abs(left_enc - m_saveLeftEnc) + std::abs(right_enc - m_saveRightEnc)) / 2;
    // aveEncCnt = (std::abs(left_enc - m_saveLeftEnc));
    //distance calc
    distance = aveEncCnt * (TIRE_CIRCUMFERENCE / TIRE_AROUNT_ENCODER_CNT);

    distanceDiff = (BALANCER_CIRCUMFERENCE / 4) - distance; // rotate 45 digrees
    //Cold Zone 1mc
    if (std::abs(distanceDiff) < 1) distanceDiff = 0;

    // if (m_currentPos.x >= 0) {
    //   if (m_targetPos.y > m_currentPos.y) {
    //     distanceDiff *= -1; // rotation left
    //   } else {
    //     distanceDiff *= 1; // rotation right
    //   }
    // } else {
    //   if (m_targetPos.y > m_currentPos.y) {
    //     distanceDiff *= 1; // rotation right
    //   } else {
    //     distanceDiff *= -1; // rogation left
    //   }
    // }

    distanceDiff *= -1;

    if (distanceDiff > 0) {
      //rotation right
      *left_speed = MOVE_SPEED;
      *right_speed = -MOVE_SPEED;
      printf("Rotation Right\n");
     } else if (distanceDiff < 0) {
      //rotation left
      *left_speed = -MOVE_SPEED;
      *right_speed = MOVE_SPEED;
      printf("Rotation Left\n");
    } else {
      //Stop
      *left_speed = 0;
      *right_speed = 0;
      printf("Rotation Stop\n");

      //Mode Change
      m_controlMode = BlancerCtrlMode_Move_Y_Direction;
      m_saveLeftEnc = left_enc;
      m_saveRightEnc = right_enc;
    }

  } else if (m_controlMode == BlancerCtrlMode_Goal) {
    //Goal
    printf("Goal\n");
    int lEncCnt = left_enc - m_saveLeftEnc;
    int rEncCnt = right_enc - m_saveRightEnc;
    float lDistance = lEncCnt * (TIRE_CIRCUMFERENCE / TIRE_AROUNT_ENCODER_CNT);
    float rDistance = rEncCnt * (TIRE_CIRCUMFERENCE / TIRE_AROUNT_ENCODER_CNT);
  } else {
    //Goal
    *left_speed = 0;
    *right_speed = 0;
    printf("Wait\n");
  }

  // for Debug
  static const char * m_controlModeStrings[BlancerCtrlMode_Max] = {
    "Init",
    "Check",
    "Move_X",
    "Rotation",
    "Move_Y",
    "Goal",
  };

  printf("mode: %s, distanceDiff: %f\n", m_controlModeStrings[m_controlMode], distanceDiff);
  printf("targetPos: %f, %f, currentPos: %f, %f\n", m_targetPos.x, m_targetPos.y, m_currentPos.x, m_currentPos.y);

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
  printf("balance_cmd publish, ls: %d, rs: %d\n", balance_cmd_msg.left_speed, balance_cmd_msg.right_speed);

  return;
}


void BalanceIF::SetGoalCallback(const geometry_msgs::PoseStamped& msg)
{
  if (m_controlMode != BlancerCtrlMode_Check) {
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

void BalanceIF::NotifyCurrentPos(float x_pos, float y_pos)
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

  //@fix orientation
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;
  msg.pose.orientation.w = 1.0;

  ui_pub.publish(msg);
  printf("notify_pos publish, x: %f, y: %f\n", msg.pose.position.x,  msg.pose.position.y);

  return;
}

////////////////////////// END /////////////////////////
