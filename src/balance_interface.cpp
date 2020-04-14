/*
 * Copyright (C) 2013-2014 Sony Corporation
 */

#include <ros/ros.h>

#include <stdio.h>
#include <stdlib.h>   //abs() for integer

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
  m_controlMode = BlancerCtrlMode_Move_X_Direction;

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

  //100ms sleep
  //ros::Rate r(10);
  //r.sleep();
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
  MotionControl(odm_msg.left_enc, odm_msg.right_enc, &left_speed, &right_speed);

  if (odm_msg.status_1) {
    //set balancer speed To Arduino
    SetBalancerSpeed(left_speed, right_speed);
  }

  //Notify Current Pos To Rviz
  NotifyCurrentPos(m_currentPos.x, m_currentPos.y);

  printf("\n");
}


#define TIRE_AROUNT_ENCODER_CNT  (1332)
#define TIRE_CIRCUMFERENCE       (25)    //cm
#define BALANCER_CIRCUMFERENCE   (36)    //cm
#define MOVE_SPEED               (1)

void BalanceIF::MotionControl(int left_enc, int right_enc, int *left_speed, int *right_speed)
{
  int aveEncCnt;
  float distance, distanceDiff;

  if ((m_controlMode == BlancerCtrlMode_Move_X_Direction) ||
      (m_controlMode == BlancerCtrlMode_Move_Y_Direction)) {
    //Move

    //Encoder Ave Calc
    aveEncCnt = ((left_enc - m_saveLeftEnc) + (right_enc - m_saveRightEnc)) / 2;
    //distance calc
    distance = (float)aveEncCnt * ((float)TIRE_CIRCUMFERENCE / (float)TIRE_AROUNT_ENCODER_CNT);

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
    if (abs(distanceDiff) < 3) distanceDiff = 0;

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
    aveEncCnt = (abs(left_enc - m_saveLeftEnc) + abs(right_enc - m_saveRightEnc)) / 2;
    //distance calc
    distance = (float)aveEncCnt * ((float)TIRE_CIRCUMFERENCE / TIRE_AROUNT_ENCODER_CNT);

    distanceDiff = (BALANCER_CIRCUMFERENCE / 4) - distance;
    //Cold Zone 1mc
    if (abs(distanceDiff) < 1) distanceDiff = 0;

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

  } else {
    //Goal
    *left_speed = 0;
    *right_speed = 0;
    printf("Goal\n");
  }

  printf("mode: %d, distanceDiff: %f\n", m_controlMode, distanceDiff);
  printf("targetPos.x: %f, currentPos.x: %f\n", m_targetPos.x, m_currentPos.x);
  printf("targetPos.y: %f, currentPos.y: %f\n", m_targetPos.y, m_currentPos.y);

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
