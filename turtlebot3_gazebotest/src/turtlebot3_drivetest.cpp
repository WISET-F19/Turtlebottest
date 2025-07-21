// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Taehun Lim (Darby), Ryan Shim

#include "turtlebot3_gazebotest/turtlebot3_drivetest.hpp"

#include <memory>

using namespace std::chrono_literals;

Turtlebot3Drive::Turtlebot3Drive()
    : Node("turtlebot3_drivetest_node")
{
  /************************************************************
  ** Initialise variables
  ************************************************************/
  scan_data_[0] = 0.0;
  scan_data_[1] = 0.0;
  scan_data_[2] = 0.0;

  robot_pose_ = 0.0;
  prev_robot_pose_ = 0.0;

  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise publishers
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

  // Initialise subscribers
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan",
      rclcpp::SensorDataQoS(),
      std::bind(
          &Turtlebot3Drive::scan_callback,
          this,
          std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", qos, std::bind(&Turtlebot3Drive::odom_callback, this, std::placeholders::_1));

  /************************************************************
  ** Initialise ROS timers
  ************************************************************/
  update_timer_ = this->create_wall_timer(10ms, std::bind(&Turtlebot3Drive::update_callback, this));

  RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node has been initialised");
}

Turtlebot3Drive::~Turtlebot3Drive()
{
  RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/
void Turtlebot3Drive::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  robot_pose_ = yaw;
  RCLCPP_INFO(this->get_logger(), "현재 robot_pose_: %.2f rad", robot_pose_);
}

// void Turtlebot3Drive::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
// {
//   uint16_t scan_angle[3] = {0, 30, 330}; //확인할 3개의 방향

//   for (int num = 0; num < 3; num++)
//   {
//     if (std::isinf(msg->ranges.at(scan_angle[num]))) //해당 각도에서 측정한 거리 + 거리 값이 무한대인지 확인 -> 무한이면 장애물이 없다고 판단
//     {
//       scan_data_[num] = msg->range_max; //라이다가 감자할 수 있는 최대 거리 값 -> range max 값 대체
//     }
//     else
//     {
//       scan_data_[num] = msg->ranges.at(scan_angle[num]); //무한 아님 -> 실제 해당 각도에서 측정한 거리로 대체
//     }
//   }
// }

int get_index_from_angle(float angle_deg, const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  float angle_rad = angle_deg * DEG2RAD;
  int index = static_cast<int>((angle_rad - msg->angle_min) / msg->angle_increment);
  index = std::clamp(index, 0, static_cast<int>(msg->ranges.size()) - 1);
  return index;
}


void Turtlebot3Drive::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  float angle_deg_list[3] = {0, 30, -30}; // 기준: 정면을 0도로

  for (int i = 0; i < 3; i++)
  {
    int index = get_index_from_angle(angle_deg_list[i], msg);
    if (std::isinf(msg->ranges[index]))
      scan_data_[i] = msg->range_max;
    else
      scan_data_[i] = msg->ranges[index]; //최대 거리 값
  }
}

// void Turtlebot3Drive::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
// {
//   size_t range_size = msg->ranges.size();

//   float min_range = msg->range_max; // 가장 먼 값으로 초기화
//   size_t min_index = 0;             // 가장 가까운 인덱스 초기화

//   // 가장 가까운 거리 계산
//   for (size_t i = 0; i < range_size; ++i)
//   {
//     float r = msg->ranges[i];
//     if (!std::isinf(r) && !std::isnan(r) && r < min_range)
//     {
//       min_range = r;
//       min_index = i;
//     }
//   }

//   uint16_t right_index = 0;
//   uint16_t center_index = range_size / 2;
//   uint16_t left_index = range_size - 1;

//   uint16_t scan_indices[3] = {right_index, center_index, left_index};

//   RCLCPP_INFO(this->get_logger(), "가장 가까운 장애물: %.2f m (인덱스: %zu)", min_range, min_index);

//   for (int i = 0; i < 3; i++)
//   {
//     if (scan_indices[i] >= range_size)
//     {
//       scan_data_[i] = msg->range_max;
//     }
//     else if (std::isinf(msg->ranges.at(scan_indices[i])))
//     {
//       scan_data_[i] = msg->range_max;
//     }
//     else
//     {
//       scan_data_[i] = msg->ranges.at(scan_indices[i]);
//     }
//   }

//   RCLCPP_INFO(this->get_logger(), "scan_data_[CENTER]: %.2f", scan_data_[CENTER]);
// }

void Turtlebot3Drive::update_cmd_vel(double linear, double angular)
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_->publish(cmd_vel);
}

/********************************************************************************
** Update functions
********************************************************************************/
void Turtlebot3Drive::update_callback()
{
  static uint8_t turtlebot3_state_num = 0;
  double escape_range = 30.0 * DEG2RAD; //현재로선 여기가 의심됨 (30라디안)
  double check_forward_dist = 0.13; //정면에서 장애물 감지 임계값
  double check_side_dist = 0.13; //좌우 측면에서의 장애물 감지 임계값

  // #define CENTER 0 -> hpp에서 이미 정의해놓음
  // #define LEFT 1
  // #define RIGHT 2
  // #define DEG2RAD (M_PI / 180.0) <- 1 라디안 값
  // #define GET_TB3_DIRECTION 0
  // #define TB3_DRIVE_FORWARD 1
  // #define TB3_RIGHT_TURN 2
  // #define TB3_LEFT_TURN 3

  switch (turtlebot3_state_num)
  {
  case GET_TB3_DIRECTION: //방향 판단 (현재 상타개 0일때)
    if (scan_data_[CENTER] > check_forward_dist) //정면 최대 거리 값이 임계값 보다 큼
    {
      if (scan_data_[LEFT] < check_side_dist) //왼쪽 최대 거리 값이 좌우 임계값 보다 작음
      {
        prev_robot_pose_ = robot_pose_; //현재 로봇의 각도를 이전 로봇의 각도로 대체
        turtlebot3_state_num = TB3_RIGHT_TURN; //터틀봇의 현재 동작 상태 오른쪽 회전 (2)
      }
      else if (scan_data_[RIGHT] < check_side_dist) //오른쪽 최대 거리 값이 좌우 임계값 보다 작음
      {
        prev_robot_pose_ = robot_pose_; //현재 로봇 각도를 우선 이전 로봇 각도에 대입
        turtlebot3_state_num = TB3_LEFT_TURN; //동작 같애를 왼쪽 회전(3)
      }
      else //왼쪽 오른쪽 둘다 임계값보다 큼
      {
        turtlebot3_state_num = TB3_DRIVE_FORWARD; //앞으로 전진 (1)
      }
    }

    if (scan_data_[CENTER] < check_forward_dist) //정면 최대 거리 값이 임계값 보다 작음
    {
      prev_robot_pose_ = robot_pose_; //일단 저장
      turtlebot3_state_num = TB3_RIGHT_TURN; //오른쪽 회전 시켜(2)
    }
    break;

  case TB3_DRIVE_FORWARD: //상태 값이 1임
    update_cmd_vel(LINEAR_VELOCITY, 0.0); //선 속도 값 업뎃 해주기 (0.3 고정)
    turtlebot3_state_num = GET_TB3_DIRECTION; //0으로 다시 초기화
    break;

  case TB3_RIGHT_TURN: //상태 값 2(오른쪽 회전)
    RCLCPP_INFO(this->get_logger(), "차이 값: %.2f", fabs(prev_robot_pose_ - robot_pose_));
    if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range) //절댓값(이전-현재) 이 30 라디안 보다 크거나 같을 때
    {
      turtlebot3_state_num = GET_TB3_DIRECTION; //0으로 초기화
    }
    else
    {
      update_cmd_vel(0.0, -1 * ANGULAR_VELOCITY); //선 속도는 0, 방향만 업뎃 ->움직이지 않고 회전만 (이게 제자리에서 돌게 만든 원인일 듯)
    }
    break;

  case TB3_LEFT_TURN: //상태 값 3(왼쪽 회전)
    if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range) //절댓값(이전 - 현재)값이 30 라디안보다 크거나 같을때
    {
      turtlebot3_state_num = GET_TB3_DIRECTION; //0으로 초기화
    }
    else
    {
      update_cmd_vel(0.0, ANGULAR_VELOCITY); //선 속도는 0 고정 -> 회전 각도만 주어짐
    }
    break;

  default:
    turtlebot3_state_num = GET_TB3_DIRECTION; //0으로 초기화
    break;
  }
}

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Turtlebot3Drive>());
  rclcpp::shutdown();

  return 0;
}
