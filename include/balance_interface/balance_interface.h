/*
 * Copyright (C) 2019 SEG
 */

#ifndef BALANCEIO_HPP_
#define BALANCEIO_HPP_

#include <ros/ros.h>
#include "balance_msg/BalanceCmd.h"
#include "balance_msg/BalanceOdm.h"
//#include "balance_interface/PosOdm.h"
//#include "balance_interface/PosCmd.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#define BL_STATUS_ERR  -1
#define BL_STATUS_OK    0

class Wait {
public:
  Wait() : waiting(false) {};
  bool start(float duration_sec) {
    if (waiting) {
      return false;
    }
    waiting = true;
    end = ros::Time::now() + ros::Duration(duration_sec);
    return true;
  }
  bool isFinish() {
    if (waiting) {
      return end <= ros::Time::now();
    }
    return true;
  }
  bool isWaiting() {
    return waiting;
  }
  void clear() {
    waiting = false;
  }
private:
  bool waiting;
  ros::Time end;
};

struct EncodeData {
  EncodeData() : enable(false), left(0), right(0) { };
  bool enable;
  int left;
  int right;
};

class BalanceIF
{
public:
    BalanceIF();
    virtual ~BalanceIF();
    int Init();
    void Start();
    void Stop();
    void BalanceCallback(const balance_msg::BalanceOdm& odm_msg); //from balancer (notify odm)

    void MotionControl(char start, int left_enc, int right_enc, int *left_speed, int *right_speed);
    void SetBalancerSpeed(int left_speed, int right_speed);

    void SetGoalCallback(const geometry_msgs::PoseStamped& msg);
    void NotifyCurrentPos(double x_pos, double y_pos, double angle);

    void SetObstacleCallback(const geometry_msgs::PointStamped& msg);

//dfu I/F

private:
//	sub modules
    void BuffPrint(const char * message) {
      if (message == message_buffer) {
        return;
      }
      message_buffer = message;
      printf("%s", message);
    }

//value
    //ros::NodeHandle nh;
    //for balance
    ros::Publisher balance_pub;
    ros::Subscriber balance_sub;

    //for UI
    ros::Publisher ui_pub;
    ros::Subscriber ui_sub;

    ros::Subscriber ui_obstacle_sub;

    struct BalancerPos {
      double x;  // pos x on xy plane. unit:meter.
      double y;  // pos y on xy plane. unit:meter.
      double angle;  // rotation z axis. unit:radian.
      BalancerPos() : x(0), y(0), angle(0) { }
      void clear() { x = y = angle = 0; }
    };

    BalancerPos m_targetPos;
    BalancerPos m_currentPos;
    BalancerPos m_savePos;
    BalancerPos m_obstainPos;

    int m_saveLeftEnc;
    int m_saveRightEnc;

    enum BalancerCtrlMode {
      BlancerCtrlMode_Initialize = 0,
      BlancerCtrlMode_Standby,
      BlancerCtrlMode_Move_X_Direction,
      BlancerCtrlMode_Rotation_Left,
      BlancerCtrlMode_Rotation_On_X,
      BlancerCtrlMode_Move_Y_Direction,
      BlancerCtrlMode_Rotation_Right,
      BlancerCtrlMode_Rotation_On_Y,
      BlancerCtrlMode_Goal,
      BlancerCtrlMode_Max,
    };
    BalancerCtrlMode m_controlMode;
    //std::String m_test;		//

    Wait m_waitForStability;

    int m_speed;

    double m_targetAngle;

    // ros::Time m_p0;
    // int m_cnt;

    std::string message_buffer;
};

#endif /* BALANCEIO_HPP_ */
