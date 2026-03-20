#include "pinocchio_bridge.h"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio_bridge_utils.h"
#include <cassert>
#include <utility>

namespace pinocchio_bridge {
namespace pin = pinocchio;

class Model::ModelImpl {
public:
  pin::Model model;
  pin::Data data;
  ModelImpl(pin::Model model) : model(model), data(model) {}
};

// Forward impl

Model::Model(std::unique_ptr<ModelImpl> &&p_impl) : data(std::move(p_impl)) {}
Model::~Model() = default;

// Joints
//
size_t Model::joint_count() const {
  auto &d = *data;
  return d.model.joints.size();
}

size_t Model::get_joint_id(rust::Str name) const {
  auto &d = *data;
  return d.model.getJointId(std::string(name));
}

bool Model::get_joint(size_t id, size_t &q_idx, size_t &v_idx) const {
  auto &d = *data;
  if (id >= d.model.joints.size())
    return false;
  auto &j = d.model.joints[id];
  q_idx = static_cast<size_t>(j.idx_q());
  v_idx = static_cast<size_t>(j.idx_v());
  return true;
}

// State
int Model::nq() const {
  auto &d = *data;
  return d.model.nq;
}

int Model::nv() const {
  auto &d = *data;
  return d.model.nv;
}

bool Model::neutral(Slice out) const {
  auto &d = *data;
  MAP_MUT(out, nq()) << pin::neutral(d.model);
  return true;
}

bool Model::forward_kinematics(ConstSlice q) {
  auto &d = *data;
  pin::forwardKinematics(d.model, d.data, MAP(q, nq()));
  return true;
}

bool Model::difference(ConstSlice q0, ConstSlice q1, Slice v) {
  auto &d = *data;
  pin::difference(d.model, MAP(q0, nq()), MAP(q1, nq()), MAP_MUT(v, nv()));
  return true;
}

bool Model::rnea(ConstSlice q, ConstSlice v, ConstSlice a_cmd, Slice t) {
  auto &d = *data;
  MAP_MUT(t, nv()) =
      pin::rnea(d.model, d.data, MAP(q, nq()), MAP(v, nv()), MAP(a_cmd, nv()));
  return true;
}

bool Model::aba(ConstSlice q, ConstSlice v, ConstSlice t, Slice ddq) {
  auto &d = *data;
  pin::aba(d.model, d.data, MAP(q, nq()), MAP(v, nv()), MAP(t, nv()));
  MAP_MUT(ddq, nv()) = d.data.ddq;
  return true;
}

bool Model::integrate(ConstSlice q, ConstSlice vdt, Slice q_next) {
  auto &d = *data;
  MAP_MUT(q_next, nq()) = pin::integrate(d.model, MAP(q, nq()), MAP(vdt, nv()));
  return true;
}

std::unique_ptr<Model> Model::model_load(rust::Str path) {
  const auto fname = std::string(path);
  pin::Model model;
  pin::urdf::buildModel(fname, pin::JointModelFreeFlyer(), model);
  // Add damping & friction
  auto urdf_dom = urdf::parseURDFFile(fname);
  for (const auto &joint_pair : urdf_dom->joints_) {
    const auto &urdf_joint = joint_pair.second;
    if (urdf_joint->dynamics) {
      assert(model.existJointName(urdf_joint->name));
      pin::JointIndex id = model.getJointId(urdf_joint->name);
      model.damping[id] = urdf_joint->dynamics->damping;
      model.friction[id] = urdf_joint->dynamics->friction;
    }
  }
  auto p_impl = std::make_unique<ModelImpl>(model);
  return std::make_unique<Model>(std::move(p_impl));
}

std::unique_ptr<Model> model_load(rust::Str path) {
  return Model::model_load(path);
}
} // namespace pinocchio_bridge
