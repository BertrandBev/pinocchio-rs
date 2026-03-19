#include "pinocchio_bridge.h"
#include "rust/cxx.h"

#include <Eigen/Dense>
#include <cassert>
#include <utility>

namespace pinocchio_bridge {
namespace pin = pinocchio;
namespace eig = Eigen;

inline bool vec_to_slice(const eig::VectorXd &vec, Slice slice) {
  if (slice.size() != static_cast<size_t>(vec.size())) {
    return false;
  }
  std::copy(vec.data(), vec.data() + vec.size(), slice.data());
  return true;
}

inline bool slice_to_vec(ConstSlice slice, eig::VectorXd &vec) {
  if (slice.size() != static_cast<size_t>(vec.size())) {
    return false;
  }
  std::copy(slice.begin(), slice.end(), vec.data());
  return true;
}

class Model::ModelImpl {
  friend class Model;

  pin::Model model;
  pin::Data data;
  // Temp vectors for data passing
  eig::VectorXd q;
  eig::VectorXd qb;
  eig::VectorXd qc;
  eig::VectorXd v;
  eig::VectorXd t;

  ModelImpl(pin::Model model) : model(model), data(model) {
    q.resize(model.nq);
    qb.resize(model.nq);
    qc.resize(model.nq);
    v.resize(model.nv);
    t.resize(model.nv);
  }

  // Joints
  bool exist_joint_name(rust::Str name) const {
    return model.existJointName(std::string(name));
  }

  int get_joint_id(rust::Str name) const {
    return model.getJointId(std::string(name));
  }

  // State
  int nq() const { return model.nv; }

  int nv() const { return model.nq; }

  bool neutral(Slice out) const {
    auto neutral = pin::neutral(model);
    return vec_to_slice(neutral, out);
  }

  bool forward_kinematics(ConstSlice qs) {
    if (!slice_to_vec(qs, q))
      return false;
    pin::forwardKinematics(model, data, q);
    return true;
  }

  bool difference(ConstSlice q0s, ConstSlice q1s, Slice qs) {
    if (!slice_to_vec(q0s, q) || !slice_to_vec(q1s, qb))
      return false;
    pin::difference(model, q, qb, qc);
    return vec_to_slice(qc, qs);
  }

  bool aba(ConstSlice qs, ConstSlice vs, ConstSlice ts, Slice ddq) {
    if (!slice_to_vec(qs, q) || !slice_to_vec(vs, v) || !slice_to_vec(ts, t))
      return false;
    pin::aba(model, data, q, v, t);
    return vec_to_slice(data.ddq, ddq);
  }

  bool integrate(ConstSlice qs, ConstSlice dvs, Slice q_out) {
    if (!slice_to_vec(qs, q) || !slice_to_vec(dvs, v))
      return false;
    q = pin::integrate(model, q, v);
    return vec_to_slice(q, q_out);
  }

  static std::unique_ptr<ModelImpl> model_load(rust::Str path) {
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
        model.joints[id].idx_q();
        model.damping[id] = urdf_joint->dynamics->damping;
        model.friction[id] = urdf_joint->dynamics->friction;
      }
    }
    return std::unique_ptr<ModelImpl>(new ModelImpl(model));
  }
};

// Forward impl

Model::Model(std::unique_ptr<ModelImpl> &&p_impl) : p_impl(std::move(p_impl)) {}
Model::~Model() = default;

bool Model::exist_joint_name(rust::Str name) const {
  return p_impl->exist_joint_name(name);
}

int Model::get_joint_id(rust::Str name) const {
  return p_impl->get_joint_id(name);
}

int Model::nq() const { return p_impl->nq(); }

int Model::nv() const { return p_impl->nv(); }

bool Model::neutral(rust::Slice<double> out) const {
  return p_impl->neutral(out);
}

bool Model::forward_kinematics(ConstSlice q) {
  return p_impl->forward_kinematics(q);
}

bool Model::difference(ConstSlice q0, ConstSlice q1, Slice q) {
  return p_impl->difference(q0, q1, q);
}

bool Model::aba(ConstSlice q, ConstSlice v, ConstSlice tau, Slice ddq) {
  return p_impl->aba(q, v, tau, ddq);
}

bool Model::integrate(ConstSlice q, ConstSlice dv, Slice q_out) {
  return p_impl->integrate(q, dv, q_out);
}

std::unique_ptr<Model> Model::model_load(rust::Str path) {
  auto p_impl = ModelImpl::model_load(path);
  return std::make_unique<Model>(std::move(p_impl));
}

std::unique_ptr<Model> model_load(rust::Str path) {
  return Model::model_load(path);
}
} // namespace pinocchio_bridge

// using MapVectorXd = Eigen::Map<const Eigen::VectorXd>;
// MapVectorXd vec(ptr, static_cast<Eigen::Index>(len));
