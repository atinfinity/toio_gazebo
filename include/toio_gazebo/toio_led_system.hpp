// Copyright (C) 2026 atinfinity
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
// The way the visual material is driven through components::VisualCmd follows
// https://github.com/jasmeet0915/gz_sim_led_plugin (Apache License 2.0).

#ifndef TOIO_GAZEBO__TOIO_LED_SYSTEM_HPP_
#define TOIO_GAZEBO__TOIO_LED_SYSTEM_HPP_

#include <memory>

#include <gz/sim/System.hh>
#include <gz/sim/config.hh>

namespace toio_gazebo
{
class ToioLedSystemPrivate;

/// \brief Drives the indicator LED of the toio cube from a Gazebo Transport
/// topic carrying gz.msgs.Color, so that ros_gz_bridge can expose it as the
/// std_msgs/msg/ColorRGBA topic that the real cube uses (toio/led).
///
/// Colors are taken as 0.0-1.0 per channel. An all zero color turns the LED
/// off, which restores the material the visual was loaded with.
///
/// ## System Parameters
///
/// <plugin filename="ToioLedSystem" name="toio_gazebo::ToioLedSystem">
///   <!-- Topic to subscribe to. Defaults to /model/<model_name>/led -->
///   <topic>/model/toio/led</topic>
///
///   <!-- Substring identifying the visual to drive. A fixed joint is lumped
///        into its parent link when a URDF is converted to SDF, so the visual
///        is matched by substring instead of by its exact generated name. -->
///   <led_visual>led</led_visual>
///
///   <!-- Lighting time in milliseconds. 0 keeps the LED lit until the next
///        command, 10-2550 turns it off once the time has elapsed. -->
///   <led_duration_ms>0</led_duration_ms>
/// </plugin>
class ToioLedSystem
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  ToioLedSystem();
  ~ToioLedSystem() override;

  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & _eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) override;

private:
  std::unique_ptr<ToioLedSystemPrivate> dataPtr;
};
}  // namespace toio_gazebo

#endif  // TOIO_GAZEBO__TOIO_LED_SYSTEM_HPP_
