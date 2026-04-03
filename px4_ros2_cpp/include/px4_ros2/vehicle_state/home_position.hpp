/****************************************************************************
 * Copyright (c) 2023-2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <Eigen/Core>
#include <px4_msgs/msg/home_position.hpp>
#include <px4_ros2/common/context.hpp>
#include <px4_ros2/utils/message_version.hpp>
#include <rclcpp/rclcpp.hpp>

namespace px4_ros2
{
/** \ingroup vehicle_state
 *  @{
 */

/**
 * @brief Provides access to the vehicle's home position
 *
 * @ingroup vehicle_state
 */
class HomePosition
{
public:
  explicit HomePosition(Context & context)
  : _node(context.node())
  {
    std::string topic = context.topicNamespacePrefix() + "fmu/out/home_position" + px4_ros2::getMessageNameVersion<px4_msgs::msg::HomePosition>();
    // Transient Local is required because home_position is published very infrequently (usually once at takeoff)
    _sub = _node.create_subscription<px4_msgs::msg::HomePosition>(
      topic,
      rclcpp::QoS(1).best_effort().transient_local(),
      [this](const px4_msgs::msg::HomePosition::SharedPtr msg) {
        _last = *msg;
        _received = true;
      });
  }

  const px4_msgs::msg::HomePosition & last() const { return _last; }
  
  bool hasReceivedMessages() const { return _received; }

  Eigen::Vector3f localPosition() const
  {
    return Eigen::Vector3f{_last.x, _last.y, _last.z};
  }

  Eigen::Vector3d globalPosition() const
  {
    return Eigen::Vector3d{_last.lat, _last.lon, _last.alt};
  }

  float yaw() const
  {
    return _last.yaw;
  }

  bool localPositionValid() const
  {
    return hasReceivedMessages() && _last.valid_lpos;
  }

  bool globaHorizontalPositionValid() const
  {
    return hasReceivedMessages() && _last.valid_hpos;
  }

  bool altitudeValid() const
  {
    return hasReceivedMessages() && _last.valid_alt;
  }

  bool manualHome() const
  {
    return _last.manual_home;
  }

private:
  rclcpp::Node & _node;
  typename rclcpp::Subscription<px4_msgs::msg::HomePosition>::SharedPtr _sub;
  px4_msgs::msg::HomePosition _last;
  bool _received{false};
};

/** @}*/
} /* namespace px4_ros2 */
