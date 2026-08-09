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

#include "toio_gazebo/toio_led_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <optional>
#include <string>

// cpplint mistakes the generated protobuf headers for C system headers.
#include <gz/msgs/color.pb.h>  // NOLINT(build/include_order)
#include <gz/msgs/light.pb.h>  // NOLINT(build/include_order)
#include <gz/msgs/visual.pb.h>  // NOLINT(build/include_order)

#include <gz/math/Color.hh>
#include <gz/math/Helpers.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/TopicUtils.hh>

#include <gz/sim/Conversions.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Light.hh>
#include <gz/sim/components/LightCmd.hh>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/VisualCmd.hh>

using namespace std::chrono_literals;

namespace toio_gazebo
{
namespace
{
/// \brief Lighting time is sent to the real cube as a single byte in units of
/// 10 ms, so only multiples of 10 up to 2550 can be expressed.
constexpr int kLedDurationStepMs = 10;
constexpr int kLedDurationMaxMs = 2550;

/// \brief Clamp a channel of an incoming color into 0.0-1.0, mapping the
/// non-finite values that ColorRGBA allows onto "off".
float SanitizeChannel(float _value)
{
  if (!std::isfinite(_value)) {
    return 0.0f;
  }
  return std::clamp(_value, 0.0f, 1.0f);
}
}  // namespace

/// \brief The four material colors that make up the look of the LED visual.
struct LedMaterial
{
  gz::math::Color ambient{gz::math::Color::Black};
  gz::math::Color diffuse{gz::math::Color::Black};
  gz::math::Color specular{gz::math::Color::Black};
  gz::math::Color emissive{gz::math::Color::Black};
};

class ToioLedSystemPrivate
{
public:
  /// \brief Callback for the LED color topic.
  void OnLedColor(const gz::msgs::Color & _msg);

  /// \brief Look up the LED visual and light below the model of this plugin.
  /// Neither exists yet while Configure runs, so this is retried until the
  /// visual is found.
  void ResolveLedEntities(const gz::sim::EntityComponentManager & _ecm);

  /// \brief Find the entity below the model whose name contains _name.
  gz::sim::Entity FindNamed(
    const gz::sim::EntityComponentManager & _ecm, const std::string & _name,
    bool _light) const;

  /// \brief Whether _entity is the model of this plugin or below it.
  bool BelongsToModel(
    const gz::sim::EntityComponentManager & _ecm, gz::sim::Entity _entity) const;

  /// \brief Request a material for the LED visual through components::VisualCmd.
  void SetVisualMaterial(
    gz::sim::EntityComponentManager & _ecm, const LedMaterial & _material);

  /// \brief Request a color and intensity for the LED light through
  /// components::LightCmd, so that the lamp also lights up its surroundings.
  void SetLight(
    gz::sim::EntityComponentManager & _ecm, const gz::math::Color & _color,
    double _intensity);

  /// \brief Model interface.
  gz::sim::Model model{gz::sim::kNullEntity};

  /// \brief Gazebo communication node.
  gz::transport::Node node;

  /// \brief Substring identifying the visual of the LED.
  std::string ledVisualName{"led"};

  /// \brief Substring identifying the light of the LED, if there is one.
  std::string ledLightName{"led_light"};

  /// \brief Intensity the light is driven at while the LED is lit.
  double litIntensity{1.0};

  /// \brief Visual entity of the LED, once resolved.
  gz::sim::Entity ledVisualEntity{gz::sim::kNullEntity};

  /// \brief Light entity of the LED, if the model has one.
  gz::sim::Entity ledLightEntity{gz::sim::kNullEntity};

  /// \brief Material the LED visual was loaded with, restored when the LED is
  /// turned off.
  LedMaterial offMaterial;

  /// \brief Lighting time. Zero keeps the LED lit until the next command.
  std::chrono::duration<double> ledDuration{0s};

  /// \brief Color requested since the last update, if any.
  std::optional<gz::math::Color> pendingColor;

  /// \brief Whether the LED is currently lit.
  bool lit{false};

  /// \brief Simulation time at which the LED was last turned on.
  std::chrono::duration<double> litSince{0s};

  /// \brief Mutex protecting pendingColor.
  std::mutex mutex;
};

//////////////////////////////////////////////////
ToioLedSystem::ToioLedSystem()
: System(), dataPtr(std::make_unique<ToioLedSystemPrivate>())
{
}

//////////////////////////////////////////////////
ToioLedSystem::~ToioLedSystem() = default;

//////////////////////////////////////////////////
void ToioLedSystem::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager &)
{
  this->dataPtr->model = gz::sim::Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm)) {
    gzerr << "[ToioLedSystem] The plugin should be attached to a model entity. "
          << "Failed to initialize." << std::endl;
    return;
  }

  if (_sdf->HasElement("led_visual")) {
    this->dataPtr->ledVisualName = _sdf->Get<std::string>("led_visual");
  }

  if (_sdf->HasElement("led_light")) {
    this->dataPtr->ledLightName = _sdf->Get<std::string>("led_light");
  }

  if (_sdf->HasElement("led_light_intensity")) {
    this->dataPtr->litIntensity =
      std::max(0.0, _sdf->Get<double>("led_light_intensity"));
  }

  if (_sdf->HasElement("led_duration_ms")) {
    int durationMs = _sdf->Get<int>("led_duration_ms");
    if (durationMs < 0) {
      gzwarn << "[ToioLedSystem] Negative led_duration_ms [" << durationMs
             << "], keeping the LED lit until the next command." << std::endl;
      durationMs = 0;
    } else if (durationMs > kLedDurationMaxMs) {
      gzwarn << "[ToioLedSystem] led_duration_ms [" << durationMs
             << "] exceeds the maximum of the toio specification, using ["
             << kLedDurationMaxMs << "]." << std::endl;
      durationMs = kLedDurationMaxMs;
    } else if (durationMs > 0 && durationMs < kLedDurationStepMs) {
      gzwarn << "[ToioLedSystem] led_duration_ms [" << durationMs
             << "] is shorter than the resolution of the toio specification, "
             << "using [" << kLedDurationStepMs << "]." << std::endl;
      durationMs = kLedDurationStepMs;
    }
    this->dataPtr->ledDuration = std::chrono::milliseconds(durationMs);
  }

  std::string topic = _sdf->HasElement("topic") ?
    _sdf->Get<std::string>("topic") :
    "/model/" + this->dataPtr->model.Name(_ecm) + "/led";

  const std::string validTopic = gz::transport::TopicUtils::AsValidTopic(topic);
  if (validTopic.empty()) {
    gzerr << "[ToioLedSystem] Failed to create a valid topic from [" << topic
          << "]. Failed to initialize." << std::endl;
    return;
  }

  this->dataPtr->node.Subscribe(
    validTopic, &ToioLedSystemPrivate::OnLedColor, this->dataPtr.get());

  gzmsg << "[ToioLedSystem] Driving the visual matching ["
        << this->dataPtr->ledVisualName << "] of model ["
        << this->dataPtr->model.Name(_ecm) << "] from [" << validTopic << "]"
        << std::endl;
}

//////////////////////////////////////////////////
void ToioLedSystem::PreUpdate(
  const gz::sim::UpdateInfo & _info,
  gz::sim::EntityComponentManager & _ecm)
{
  if (this->dataPtr->model.Entity() == gz::sim::kNullEntity) {
    return;
  }

  if (this->dataPtr->ledVisualEntity == gz::sim::kNullEntity) {
    this->dataPtr->ResolveLedEntities(_ecm);
    if (this->dataPtr->ledVisualEntity == gz::sim::kNullEntity) {
      return;
    }
  }

  const auto simTime =
    std::chrono::duration_cast<std::chrono::duration<double>>(_info.simTime);

  // The world may have been reset, which moves simulation time backwards.
  if (simTime < this->dataPtr->litSince) {
    this->dataPtr->litSince = simTime;
  }

  std::optional<gz::math::Color> requested;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    requested.swap(this->dataPtr->pendingColor);
  }

  if (requested.has_value()) {
    const gz::math::Color & color = requested.value();
    // An all zero color means "turn the indicator off" on the real cube.
    const bool off = color.R() <= 0.0f && color.G() <= 0.0f && color.B() <= 0.0f;

    if (off) {
      this->dataPtr->SetVisualMaterial(_ecm, this->dataPtr->offMaterial);
      this->dataPtr->SetLight(_ecm, gz::math::Color::Black, 0.0);
    } else {
      LedMaterial material;
      material.ambient = color;
      material.diffuse = color;
      material.specular = color;
      material.emissive = color;
      this->dataPtr->SetVisualMaterial(_ecm, material);
      this->dataPtr->SetLight(_ecm, color, this->dataPtr->litIntensity);
    }

    this->dataPtr->lit = !off;
    this->dataPtr->litSince = simTime;
    return;
  }

  const bool expires = this->dataPtr->ledDuration > 0s;
  if (this->dataPtr->lit && expires &&
    (simTime - this->dataPtr->litSince) >= this->dataPtr->ledDuration)
  {
    this->dataPtr->SetVisualMaterial(_ecm, this->dataPtr->offMaterial);
    this->dataPtr->SetLight(_ecm, gz::math::Color::Black, 0.0);
    this->dataPtr->lit = false;
  }
}

//////////////////////////////////////////////////
void ToioLedSystemPrivate::OnLedColor(const gz::msgs::Color & _msg)
{
  gz::math::Color color(
    SanitizeChannel(_msg.r()), SanitizeChannel(_msg.g()),
    SanitizeChannel(_msg.b()), 1.0f);

  std::lock_guard<std::mutex> lock(this->mutex);
  this->pendingColor = color;
}

//////////////////////////////////////////////////
bool ToioLedSystemPrivate::BelongsToModel(
  const gz::sim::EntityComponentManager & _ecm, gz::sim::Entity _entity) const
{
  const gz::sim::Entity modelEntity = this->model.Entity();
  for (gz::sim::Entity entity = _entity; entity != gz::sim::kNullEntity; ) {
    if (entity == modelEntity) {
      return true;
    }
    const auto * parent = _ecm.Component<gz::sim::components::ParentEntity>(entity);
    if (parent == nullptr) {
      return false;
    }
    entity = parent->Data();
  }
  return false;
}

//////////////////////////////////////////////////
gz::sim::Entity ToioLedSystemPrivate::FindNamed(
  const gz::sim::EntityComponentManager & _ecm, const std::string & _name,
  bool _light) const
{
  gz::sim::Entity found = gz::sim::kNullEntity;
  std::size_t matches = 0;

  const auto consider =
    [&](const gz::sim::Entity & _entity, const std::string & _entityName)
    {
      if (_entityName.find(_name) == std::string::npos) {
        return;
      }
      if (!this->BelongsToModel(_ecm, _entity)) {
        return;
      }
      ++matches;
      if (found == gz::sim::kNullEntity) {
        found = _entity;
      }
    };

  if (_light) {
    _ecm.Each<gz::sim::components::Light, gz::sim::components::Name>(
      [&](const gz::sim::Entity & _entity,
      const gz::sim::components::Light *,
      const gz::sim::components::Name * _n) -> bool
      {
        consider(_entity, _n->Data());
        return true;
      });
  } else {
    _ecm.Each<gz::sim::components::Visual, gz::sim::components::Name>(
      [&](const gz::sim::Entity & _entity,
      const gz::sim::components::Visual *,
      const gz::sim::components::Name * _n) -> bool
      {
        consider(_entity, _n->Data());
        return true;
      });
  }

  if (matches > 1) {
    gzwarn << "[ToioLedSystem] Found " << matches << " entities matching ["
           << _name << "], using the first one." << std::endl;
  }
  return found;
}

//////////////////////////////////////////////////
void ToioLedSystemPrivate::ResolveLedEntities(
  const gz::sim::EntityComponentManager & _ecm)
{
  const gz::sim::Entity visual = this->FindNamed(_ecm, this->ledVisualName, false);
  if (visual == gz::sim::kNullEntity) {
    return;
  }
  this->ledVisualEntity = visual;

  // Remember the material of the visual so that it can be restored when the
  // LED is turned off.
  const auto * material =
    _ecm.Component<gz::sim::components::Material>(this->ledVisualEntity);
  if (material != nullptr) {
    this->offMaterial.ambient = material->Data().Ambient();
    this->offMaterial.diffuse = material->Data().Diffuse();
    this->offMaterial.specular = material->Data().Specular();
    this->offMaterial.emissive = material->Data().Emissive();
  } else {
    gzwarn << "[ToioLedSystem] The LED visual has no material, the LED will "
           << "turn black when it is switched off." << std::endl;
  }

  // The light is optional, and it is created together with the visual, so it
  // is only looked for once.
  this->ledLightEntity = this->FindNamed(_ecm, this->ledLightName, true);
  if (this->ledLightEntity == gz::sim::kNullEntity) {
    gzmsg << "[ToioLedSystem] No light matching [" << this->ledLightName
          << "], the LED will light up itself but not its surroundings."
          << std::endl;
  }
}

//////////////////////////////////////////////////
void ToioLedSystemPrivate::SetVisualMaterial(
  gz::sim::EntityComponentManager & _ecm, const LedMaterial & _material)
{
  gz::msgs::Visual visualMsg;
  visualMsg.set_id(this->ledVisualEntity);

  auto * msgMaterial = visualMsg.mutable_material();
  const auto setColor =
    [](gz::msgs::Color * _dst, const gz::math::Color & _src)
    {
      _dst->set_r(_src.R());
      _dst->set_g(_src.G());
      _dst->set_b(_src.B());
      _dst->set_a(_src.A());
    };
  setColor(msgMaterial->mutable_ambient(), _material.ambient);
  setColor(msgMaterial->mutable_diffuse(), _material.diffuse);
  setColor(msgMaterial->mutable_specular(), _material.specular);
  setColor(msgMaterial->mutable_emissive(), _material.emissive);

  const auto colorEq =
    [](const gz::msgs::Color & _a, const gz::msgs::Color & _b)
    {
      return gz::math::equal(_a.r(), _b.r(), 1e-6f) &&
             gz::math::equal(_a.g(), _b.g(), 1e-6f) &&
             gz::math::equal(_a.b(), _b.b(), 1e-6f) &&
             gz::math::equal(_a.a(), _b.a(), 1e-6f);
    };
  std::function<bool(const gz::msgs::Visual &, const gz::msgs::Visual &)> visualEq =
    [&colorEq](const gz::msgs::Visual & _a, const gz::msgs::Visual & _b)
    {
      return _a.id() == _b.id() &&
             colorEq(_a.material().ambient(), _b.material().ambient()) &&
             colorEq(_a.material().diffuse(), _b.material().diffuse()) &&
             colorEq(_a.material().specular(), _b.material().specular()) &&
             colorEq(_a.material().emissive(), _b.material().emissive());
    };

  auto * visualCmd =
    _ecm.Component<gz::sim::components::VisualCmd>(this->ledVisualEntity);
  if (visualCmd == nullptr) {
    _ecm.CreateComponent(
      this->ledVisualEntity, gz::sim::components::VisualCmd(visualMsg));
    return;
  }

  const auto state = visualCmd->SetData(visualMsg, visualEq) ?
    gz::sim::ComponentState::OneTimeChange :
    gz::sim::ComponentState::NoChange;
  _ecm.SetChanged(
    this->ledVisualEntity, gz::sim::components::VisualCmd::typeId, state);
}

//////////////////////////////////////////////////
void ToioLedSystemPrivate::SetLight(
  gz::sim::EntityComponentManager & _ecm, const gz::math::Color & _color,
  double _intensity)
{
  if (this->ledLightEntity == gz::sim::kNullEntity) {
    return;
  }

  const auto * light =
    _ecm.Component<gz::sim::components::Light>(this->ledLightEntity);
  if (light == nullptr) {
    return;
  }

  // Carry the shape of the light over from the model and only drive how it
  // looks, so that the range and falloff stay where the model put them.
  const gz::msgs::Light shape = gz::sim::convert<gz::msgs::Light>(light->Data());
  gz::msgs::Light lightMsg;
  lightMsg.set_id(this->ledLightEntity);
  lightMsg.set_type(shape.type());
  lightMsg.set_range(shape.range());
  lightMsg.set_attenuation_constant(shape.attenuation_constant());
  lightMsg.set_attenuation_linear(shape.attenuation_linear());
  lightMsg.set_attenuation_quadratic(shape.attenuation_quadratic());
  lightMsg.set_cast_shadows(shape.cast_shadows());
  // A spot loses its cone if these are left at their defaults.
  lightMsg.mutable_direction()->CopyFrom(shape.direction());
  lightMsg.set_spot_inner_angle(shape.spot_inner_angle());
  lightMsg.set_spot_outer_angle(shape.spot_outer_angle());
  lightMsg.set_spot_falloff(shape.spot_falloff());
  lightMsg.set_intensity(_intensity);

  const auto setColor =
    [](gz::msgs::Color * _dst, const gz::math::Color & _src)
    {
      _dst->set_r(_src.R());
      _dst->set_g(_src.G());
      _dst->set_b(_src.B());
      _dst->set_a(_src.A());
    };
  setColor(lightMsg.mutable_diffuse(), _color);
  setColor(lightMsg.mutable_specular(), _color);

  std::function<bool(const gz::msgs::Light &, const gz::msgs::Light &)> lightEq =
    [](const gz::msgs::Light & _a, const gz::msgs::Light & _b)
    {
      const auto & da = _a.diffuse();
      const auto & db = _b.diffuse();
      return _a.id() == _b.id() &&
             gz::math::equal(_a.intensity(), _b.intensity(), 1e-6f) &&
             gz::math::equal(da.r(), db.r(), 1e-6f) &&
             gz::math::equal(da.g(), db.g(), 1e-6f) &&
             gz::math::equal(da.b(), db.b(), 1e-6f);
    };

  auto * lightCmd =
    _ecm.Component<gz::sim::components::LightCmd>(this->ledLightEntity);
  if (lightCmd == nullptr) {
    _ecm.CreateComponent(
      this->ledLightEntity, gz::sim::components::LightCmd(lightMsg));
    return;
  }

  const auto state = lightCmd->SetData(lightMsg, lightEq) ?
    gz::sim::ComponentState::OneTimeChange :
    gz::sim::ComponentState::NoChange;
  _ecm.SetChanged(
    this->ledLightEntity, gz::sim::components::LightCmd::typeId, state);
}
}  // namespace toio_gazebo

GZ_ADD_PLUGIN(
  toio_gazebo::ToioLedSystem,
  gz::sim::System,
  toio_gazebo::ToioLedSystem::ISystemConfigure,
  toio_gazebo::ToioLedSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(toio_gazebo::ToioLedSystem, "toio_gazebo::ToioLedSystem")
