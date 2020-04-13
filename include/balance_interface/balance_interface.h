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

#define BL_STATUS_ERR		-1
#define BL_STATUS_OK    0

class BalanceIF
{
public:
    BalanceIF();
    virtual ~BalanceIF();
    int Init();
    void Start();
    void Stop();
    void BalanceCallback(const balance_msg::BalanceOdm& odm_msg); //from balancer (notify odm)

    void MotionControl(int left_enc, int right_enc, int *left_speed, int *right_speed);
    void SetBalancerSpeed(int left_speed, int right_speed);

    void SetGoalCallback(const geometry_msgs::PoseStamped& msg);
    void NotifyCurrentPos(float x_pos, float y_pos);

//dfu I/F

private:
//	sub modules


//value
    //ros::NodeHandle nh;
    //for balance
    ros::Publisher balance_pub;
    ros::Subscriber balance_sub;

    //for UI
    ros::Publisher ui_pub;
    ros::Subscriber ui_sub;

    struct BalancerPos {
      float x;
      float y;
    };

    BalancerPos m_targetPos;
    BalancerPos m_currentPos;

    int m_saveLeftEnc;
    int m_saveRightEnc;

    enum BalancerCtrlMode {
      BlancerCtrlMode_Move_X_Direction = 0,
      BlancerCtrlMode_Rotation,
      BlancerCtrlMode_Move_Y_Direction,
      BlancerCtrlMode_Goal,
      BlancerCtrlMode_Max,
    };
    BalancerCtrlMode m_controlMode;

    //std::String m_test;		//

};

#endif /* BALANCEIO_HPP_ */
