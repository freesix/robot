/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Samsung Research Russia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the <ORGANIZATION> nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Alexey Merzlyakov
 *********************************************************************/

#include "nav2_costmap_2d/costmap_filters/speed_filter.hpp"

#include <cmath>
#include <utility>
#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

namespace nav2_costmap_2d
{

SpeedFilter::SpeedFilter()
: filter_info_sub_(nullptr), mask_sub_(nullptr),
  speed_limit_pub_(nullptr), filter_mask_(nullptr), mask_frame_(""), global_frame_(""),
  speed_limit_(NO_SPEED_LIMIT), speed_limit_prev_(NO_SPEED_LIMIT)
{
}

void SpeedFilter::initializeFilter(
  const std::string & filter_info_topic)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  // 声明速度限制的话题，并从参数中获取
  // Declare "speed_limit_topic" parameter specific to SpeedFilter only
  std::string speed_limit_topic;
  declareParameter("speed_limit_topic", rclcpp::ParameterValue("speed_limit"));
  node->get_parameter(name_ + "." + "speed_limit_topic", speed_limit_topic);

  filter_info_topic_ = filter_info_topic;
  // Setting new costmap filter info subscriber
  // 订阅这个话题，并绑定相应的回调函数
  RCLCPP_INFO(
    logger_,
    "SpeedFilter: Subscribing to \"%s\" topic for filter info...",
    filter_info_topic_.c_str());
  filter_info_sub_ = node->create_subscription<nav2_msgs::msg::CostmapFilterInfo>(
    filter_info_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&SpeedFilter::filterInfoCallback, this, std::placeholders::_1));

  // Get global frame required for speed limit publisher
  global_frame_ = layered_costmap_->getGlobalFrameID();

  // Create new speed limit publisher 创建一个速度限制的发布者
  speed_limit_pub_ = node->create_publisher<nav2_msgs::msg::SpeedLimit>(
    speed_limit_topic, rclcpp::QoS(10));
  speed_limit_pub_->on_activate();

  // Reset speed conversion states
  base_ = BASE_DEFAULT;
  multiplier_ = MULTIPLIER_DEFAULT;
  percentage_ = false;
}
// 限速回调函数
void SpeedFilter::filterInfoCallback(
  const nav2_msgs::msg::CostmapFilterInfo::SharedPtr msg)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  // 检查是否从话题中收到限速的mask信息
  if (!mask_sub_) {
    RCLCPP_INFO(
      logger_,
      "SpeedFilter: Received filter info from %s topic.", filter_info_topic_.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "SpeedFilter: New costmap filter info arrived from %s topic. Updating old filter info.",
      filter_info_topic_.c_str());
    // Resetting previous subscriber each time when new costmap filter information arrives
    mask_sub_.reset();
  }

  // Set base_/multiplier_ or use speed limit in % of maximum speed
  base_ = msg->base;
  multiplier_ = msg->multiplier;
  if (msg->type == SPEED_FILTER_PERCENT) { // 以百分比的形式在限速区进行限速
    // Using speed limit in % of maximum speed
    percentage_ = true;
    RCLCPP_INFO(
      logger_,
      "SpeedFilter: Using expressed in a percent from maximum speed"
      "speed_limit = %f + filter_mask_data * %f",
      base_, multiplier_);
  } else if (msg->type == SPEED_FILTER_ABSOLUTE) { // 以绝对值形式在限速区限速
    // Using speed limit in m/s
    percentage_ = false;
    RCLCPP_INFO(
      logger_,
      "SpeedFilter: Using absolute speed_limit = %f + filter_mask_data * %f",
      base_, multiplier_);
  } else { // 不支持的限速模式
    RCLCPP_ERROR(logger_, "SpeedFilter: Mode is not supported");
    return;
  }
  // 获取mask的topic
  mask_topic_ = msg->filter_mask_topic;

  // Setting new filter mask subscriber 创建关于限速mask的回调订阅
  RCLCPP_INFO(
    logger_,
    "SpeedFilter: Subscribing to \"%s\" topic for filter mask...",
    mask_topic_.c_str());
  mask_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    mask_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&SpeedFilter::maskCallback, this, std::placeholders::_1));
}
// 限速mask的回调
void SpeedFilter::maskCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (!filter_mask_) {
    RCLCPP_INFO(
      logger_,
      "SpeedFilter: Received filter mask from %s topic.", mask_topic_.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "SpeedFilter: New filter mask arrived from %s topic. Updating old filter mask.",
      mask_topic_.c_str());
    filter_mask_.reset();
  }
  // 保存mask(msg中包含的就是一个限速区域栅格地图，其中栅格值表示限速比值)
  filter_mask_ = msg;
  mask_frame_ = msg->header.frame_id;
}
// 发布速度限制的处理函数，主要获取机器人当前位置，查找mask中速度限制值并发布
// 由父类在updateCost中调用
void SpeedFilter::process(
  nav2_costmap_2d::Costmap2D & /*master_grid*/,
  int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/,
  const geometry_msgs::msg::Pose2D & pose)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (!filter_mask_) { // 是否收到mask
    // Show warning message every 2 seconds to not litter an output
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 2000,
      "SpeedFilter: Filter mask was not received");
    return;
  }

  geometry_msgs::msg::Pose2D mask_pose;  // robot coordinates in mask frame
  // 将机器人坐标从当前层转换到mask坐标下
  // Transforming robot pose from current layer frame to mask frame
  if (!transformPose(global_frame_, pose, mask_frame_, mask_pose)) {
    return;
  }
  // 将机器人在mask坐标系下坐标转换到mask的图像坐标系下
  // Converting mask_pose robot position to filter_mask_ indexes (mask_robot_i, mask_robot_j)
  unsigned int mask_robot_i, mask_robot_j;
  if (!worldToMask(filter_mask_, mask_pose.x, mask_pose.y, mask_robot_i, mask_robot_j)) {
    return;
  }

  // Getting filter_mask data from cell where the robot placed and
  // calculating speed limit value
  int8_t speed_mask_data = getMaskData(filter_mask_, mask_robot_i, mask_robot_j); // 提取mask对应位置的速度限制值
  if (speed_mask_data == SPEED_MASK_NO_LIMIT) { // 限制值为没有速度限制
    // Corresponding filter mask cell is free.
    // Setting no speed limit there.
    speed_limit_ = NO_SPEED_LIMIT;
  } else if (speed_mask_data == SPEED_MASK_UNKNOWN) { // 没有限制值具体数值
    // Corresponding filter mask cell is unknown.
    // Do nothing.
    RCLCPP_ERROR(
      logger_,
      "SpeedFilter: Found unknown cell in filter_mask[%i, %i], "
      "which is invalid for this kind of filter",
      mask_robot_i, mask_robot_j);
    return;
  } else {
    // Normal case: speed_mask_data in range of [1..100]
    speed_limit_ = speed_mask_data * multiplier_ + base_; // 计算根据mask值得到的速度限制值
    if (percentage_) {
      if (speed_limit_ < 0.0 || speed_limit_ > 100.0) { // 计算出来的速度值不满足要求
        RCLCPP_WARN(
          logger_,
          "SpeedFilter: Speed limit in filter_mask[%i, %i] is %f%%, "
          "out of bounds of [0, 100]. Setting it to no-limit value.",
          mask_robot_i, mask_robot_j, speed_limit_);
        speed_limit_ = NO_SPEED_LIMIT;
      }
    } else {
      if (speed_limit_ < 0.0) {
        RCLCPP_WARN(
          logger_,
          "SpeedFilter: Speed limit in filter_mask[%i, %i] is less than 0 m/s, "
          "which can not be true. Setting it to no-limit value.",
          mask_robot_i, mask_robot_j);
        speed_limit_ = NO_SPEED_LIMIT;
      }
    }
  }
  // 如果计算出来的速度不等于上一次的速度限制值
  if (speed_limit_ != speed_limit_prev_) {
    if (speed_limit_ != NO_SPEED_LIMIT) {
      RCLCPP_DEBUG(logger_, "SpeedFilter: Speed limit is set to %f", speed_limit_);
    } else {
      RCLCPP_DEBUG(logger_, "SpeedFilter: Speed limit is set to its default value");
    }

    // Forming and publishing new SpeedLimit message 将这个速度限制值转换为ros消息
    std::unique_ptr<nav2_msgs::msg::SpeedLimit> msg =
      std::make_unique<nav2_msgs::msg::SpeedLimit>();
    msg->header.frame_id = global_frame_;
    msg->header.stamp = clock_->now();
    msg->percentage = percentage_;
    msg->speed_limit = speed_limit_;
    speed_limit_pub_->publish(std::move(msg)); // 发布速度限制话题

    speed_limit_prev_ = speed_limit_;
  }
}

void SpeedFilter::resetFilter()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  filter_info_sub_.reset();
  mask_sub_.reset();
  if (speed_limit_pub_) {
    speed_limit_pub_->on_deactivate();
    speed_limit_pub_.reset();
  }
}

bool SpeedFilter::isActive()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (filter_mask_) {
    return true;
  }
  return false;
}

}  // namespace nav2_costmap_2d

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::SpeedFilter, nav2_costmap_2d::Layer)
