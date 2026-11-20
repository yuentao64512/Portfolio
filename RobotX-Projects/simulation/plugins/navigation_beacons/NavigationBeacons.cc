// Navigation lamps run on simulation time, including pause and reset.
#include <chrono>
#include <cmath>
#include <map>
#include <regex>
#include <string>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Conversions.hh>
#include <gz/sim/components/Light.hh>
#include <gz/sim/components/LightCmd.hh>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/VisualCmd.hh>

namespace usv {
namespace sim = gz::sim;
namespace cmp = gz::sim::components;

class NavigationBeacons : public sim::System,
    public sim::ISystemConfigure, public sim::ISystemPreUpdate {
  struct Lamp {
    double hz = 1.0;
    int previous = -1;
    bool bulb = false;
    sdf::Material material;
    gz::msgs::Light light;
  };
  std::map<sim::Entity, Lamp> visuals, lights;
  double entranceHz = 0.5, defaultHz = 1.0, exitHz = 0.0;
  double nextDiscovery = -1.0, lastTime = -1.0;

  static bool IsBuoy(const std::string &name) {
    static const std::regex pattern(
        "entry_blue|exit_blue|(red|green|black)_[0-9]+[ab]?");
    return std::regex_match(name, pattern);
  }
  static bool IsDock(const std::string &name) {
    return name == "boat_dock_red" || name == "boat_dock_green" ||
           name == "boat_dock_blue";
  }
  double Rate(const std::string &name) const {
    if (name == "entry_blue") return entranceHz;
    if (name == "exit_blue") return exitHz;
    return defaultHz;
  }
  static bool On(double t, double hz) {
    return hz == 0.0 || std::fmod(std::max(0.0, t) * hz, 1.0) < 0.5;
  }
  void Discover(sim::EntityComponentManager &ecm) {
    ecm.Each<cmp::Visual, cmp::Name, cmp::Material>(
      [&](const sim::Entity &id, const cmp::Visual *, const cmp::Name *name,
          const cmp::Material *material) {
        if (visuals.count(id)) return true;
        auto parent = ecm.ParentEntity(id);
        while (parent != sim::kNullEntity && !ecm.Component<cmp::Model>(parent))
          parent = ecm.ParentEntity(parent);
        const auto model = ecm.Component<cmp::Name>(parent);
        if (!model) return true;
        const auto &mn = model->Data();
        const auto &vn = name->Data();
        const bool buoy = IsBuoy(mn) && (vn == "body" || vn == "top_beacon");
        const bool dock = IsDock(mn) && vn.find("_beacon_") != std::string::npos;
        if (!buoy && !dock) return true;
        Lamp lamp;
        lamp.hz = Rate(mn);
        lamp.bulb = dock || vn == "top_beacon";
        lamp.material = material->Data();
        visuals.emplace(id, lamp);
        return true;
      });
    ecm.Each<cmp::Light, cmp::Name>(
      [&](const sim::Entity &id, const cmp::Light *light, const cmp::Name *name) {
        if (lights.count(id)) return true;
        auto base = name->Data();
        const std::string suffix = "_light";
        if (base.size() > suffix.size() &&
            base.compare(base.size()-suffix.size(), suffix.size(), suffix) == 0)
          base.resize(base.size()-suffix.size());
        bool dock = false;
        for (const auto &color : {"red", "green", "blue"}) {
          const std::string prefix = std::string("boat_dock_") + color + "_beacon_";
          dock = dock || base == prefix + "left" || base == prefix + "right";
        }
        if (!IsBuoy(base) && !dock) return true;
        Lamp lamp;
        lamp.hz = Rate(base);
        lamp.light = sim::convert<gz::msgs::Light>(light->Data());
        lamp.light.set_id(id);
        lights.emplace(id, lamp);
        return true;
      });
  }

 public:
  void Configure(const sim::Entity &, const std::shared_ptr<const sdf::Element> &sdf,
      sim::EntityComponentManager &, sim::EventManager &) override {
    entranceHz = sdf->Get<double>("entrance_hz", 0.5).first;
    defaultHz = sdf->Get<double>("default_hz", 1.0).first;
    exitHz = sdf->Get<double>("exit_hz", 0.0).first;
    if (!std::isfinite(entranceHz) || entranceHz <= 0 ||
        !std::isfinite(defaultHz) || defaultHz <= 0 ||
        !std::isfinite(exitHz) || exitHz < 0)
      throw std::runtime_error("Beacon rates must be finite and positive (exit_hz=0 means steady)");
  }
  void PreUpdate(const sim::UpdateInfo &info, sim::EntityComponentManager &ecm) override {
    if (info.paused) return;
    const double t = std::chrono::duration<double>(info.simTime).count();
    if (t < lastTime) {
      nextDiscovery = -1;
      for (auto &[id, lamp] : visuals) lamp.previous = -1;
      for (auto &[id, lamp] : lights) lamp.previous = -1;
    }
    lastTime = t;
    if (t >= nextDiscovery) {
      Discover(ecm);  // Also finds docks inserted after the world starts.
      nextDiscovery = t + 0.25;
    }
    for (auto it = visuals.begin(); it != visuals.end();) {
      const auto id = it->first;
      auto &lamp = it->second;
      if (!ecm.HasEntity(id)) { it = visuals.erase(it); continue; }
      const int on = On(t, lamp.hz);
      if (on != lamp.previous) {
        auto material = lamp.material;
        if (!on) {
          material.SetEmissive(gz::math::Color(0, 0, 0, 1));
          if (lamp.bulb) {
            auto dim = material.Diffuse();
            dim.R(dim.R() * 0.03f); dim.G(dim.G() * 0.03f); dim.B(dim.B() * 0.03f);
            material.SetDiffuse(dim);
            material.SetAmbient(dim);
            material.SetSpecular(gz::math::Color(0, 0, 0, 1));
          }
        }
        gz::msgs::Visual cmd;
        cmd.set_id(id);
        cmd.set_name(ecm.Component<cmp::Name>(id)->Data());
        *cmd.mutable_material() = sim::convert<gz::msgs::Material>(material);
        ecm.SetComponentData<cmp::VisualCmd>(id, cmd);
        lamp.previous = on;
      }
      ++it;
    }
    for (auto it = lights.begin(); it != lights.end();) {
      const auto id = it->first;
      auto &lamp = it->second;
      if (!ecm.HasEntity(id)) { it = lights.erase(it); continue; }
      const int on = On(t, lamp.hz);
      if (on != lamp.previous) {
        auto cmd = lamp.light;
        cmd.set_is_light_off(!on);
        cmd.set_intensity(on ? lamp.light.intensity() : 0.0);
        ecm.SetComponentData<cmp::LightCmd>(id, cmd);
        lamp.previous = on;
      }
      ++it;
    }
  }
};
}
GZ_ADD_PLUGIN(usv::NavigationBeacons, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(usv::NavigationBeacons, "usv::NavigationBeacons")
