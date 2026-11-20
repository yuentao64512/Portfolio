#include "NavigationBeacons.cc"
#include <iostream>
#include <stdexcept>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components/ParentEntity.hh>

namespace sim = gz::sim;
namespace cmp = gz::sim::components;

void Require(bool ok, const char *message) {
  if (!ok) throw std::runtime_error(message);
}

int main() {
  sim::EntityComponentManager ecm;
  usv::NavigationBeacons beacons;
  auto visual = [&](const std::string &modelName, const std::string &name) {
    auto model = ecm.CreateEntity();
    ecm.CreateComponent(model, cmp::Model());
    ecm.CreateComponent(model, cmp::Name(modelName));
    auto id = ecm.CreateEntity();
    ecm.SetParentEntity(id, model);
    ecm.CreateComponent(id, cmp::ParentEntity(model));
    ecm.CreateComponent(id, cmp::Visual());
    ecm.CreateComponent(id, cmp::Name(name));
    sdf::Material mat;
    mat.SetAmbient(gz::math::Color(0, 0.2, 0.7, 1));
    mat.SetDiffuse(gz::math::Color(0, 0.3, 1, 1));
    mat.SetEmissive(gz::math::Color(0, 0.2, 1, 1));
    ecm.CreateComponent(id, cmp::Material(mat));
    return id;
  };
  auto light = [&](const std::string &name) {
    auto id = ecm.CreateEntity();
    sdf::Light source;
    source.SetName(name);
    source.SetType(sdf::LightType::POINT);
    source.SetIntensity(2.5);
    source.SetDiffuse(gz::math::Color(0, 0.3, 1, 1));
    ecm.CreateComponent(id, cmp::Light(source));
    ecm.CreateComponent(id, cmp::Name(name));
    return id;
  };
  const auto entrance = visual("entry_blue", "top_beacon");
  const auto body = visual("entry_blue", "body");
  const auto exit = visual("exit_blue", "top_beacon");
  const auto green = visual("green_1", "top_beacon");
  const auto sun = light("sun");
  const auto entryLight = light("entry_blue_light");
  const auto exitLight = light("exit_blue_light");
  auto tick = [&](double time, bool paused = false) {
    sim::UpdateInfo info;
    info.simTime = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(time));
    info.paused = paused;
    beacons.PreUpdate(info, ecm);
  };
  auto bright = [&](sim::Entity id) {
    auto cmd = ecm.Component<cmp::VisualCmd>(id);
    Require(cmd != nullptr, "visual command missing");
    return cmd->Data().material().emissive().b() > 0.5;
  };
  for (double t : {0.0, 0.49, 0.5, 0.99, 1.0, 1.49, 1.5, 1.99, 2.0, 2.5, 3.0}) {
    tick(t);
    const bool entranceOn = std::fmod(t, 2.0) < 1.0;
    const bool greenOn = std::fmod(t, 1.0) < 0.5;
    Require(bright(entrance) == entranceOn, "entrance not 0.5 Hz");
    Require(bright(green) == greenOn, "other beacon not 1 Hz");
    Require(bright(exit), "exit must stay solid");
    const auto cmd = ecm.Component<cmp::LightCmd>(entryLight)->Data();
    Require(cmd.is_light_off() == !entranceOn, "illumination phase mismatch");
    Require(std::abs(cmd.intensity() - (entranceOn ? 2.5 : 0)) < 1e-6,
            "original light intensity not preserved");
    Require(!ecm.Component<cmp::LightCmd>(exitLight)->Data().is_light_off(),
            "exit illumination must stay on");
    Require(ecm.Component<cmp::VisualCmd>(body)->Data().material().diffuse().b() == 1,
            "buoy body color changed");
    Require(!ecm.Component<cmp::LightCmd>(sun), "sun must not blink");
  }
  tick(4.0, true);
  Require(!bright(entrance), "pause changed lights");
  tick(0.0);
  Require(bright(entrance), "reset did not reset blink phase");
  const auto dock = visual("boat_dock_green", "green_beacon_left");
  const auto dockLight = light("boat_dock_green_beacon_left");
  tick(0.5);
  Require(!bright(dock), "late-spawned dock not discovered/blinking");
  Require(ecm.Component<cmp::LightCmd>(dockLight)->Data().is_light_off(),
          "dock illumination not synchronized");
  tick(1.0);
  Require(bright(dock), "dock did not blink on");
  std::cout << "PASS: frequencies, steady exit, colors, illumination, pause, reset, late docks\n";
}
