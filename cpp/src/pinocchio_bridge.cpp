#include "pinocchio_bridge.h"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio_bridge_utils.h"
#include "rust/cxx.h"
#include <cassert>
#include <cstdint>
#include <ostream>
#include <utility>

#include <pinocchio/multibody/visitor.hpp>

struct AxisVisitor : public boost::static_visitor<Eigen::Vector3d> {
  template <typename JointModel>
  Eigen::Vector3d operator()(const JointModel &) const {
    return Eigen::Vector3d::Zero(); // non-revolute joints
  }
  Eigen::Vector3d
  operator()(const pinocchio::JointModelRevoluteUnalignedTpl<double> &j) const {
    return j.axis;
  }
  Eigen::Vector3d operator()(const pinocchio::JointModelRX &) const {
    return Eigen::Vector3d::UnitX();
  }
  Eigen::Vector3d operator()(const pinocchio::JointModelRY &) const {
    return Eigen::Vector3d::UnitY();
  }
  Eigen::Vector3d operator()(const pinocchio::JointModelRZ &) const {
    return Eigen::Vector3d::UnitZ();
  }
};

namespace pinocchio_bridge {
namespace pin = pinocchio;

class Model::ModelImpl {
public:
  pin::Model model;
  pin::Data data;
  ModelImpl(pin::Model model) : model(model), data(model) {}
};

// Forward impl

Model::Model(std::unique_ptr<ModelImpl> &&data) : data(std::move(data)) {}
Model::~Model() = default;
Model::Model(const Model &other)
    : data(std::make_unique<ModelImpl>(*other.data)) {}

// Joints
size_t Model::joint_count() const {
  auto &d = *data;
  return d.model.joints.size();
}

size_t Model::get_joint_id(rust::Str name) const {
  auto &d = *data;
  return d.model.getJointId(std::string(name));
}

bool Model::get_joint_name(size_t id, rust::String &name) const {
  auto &d = *data;
  INDEX_CHECK_LEN(id, joint_count());
  name = d.model.names[id];
  return true;
}

bool Model::get_joint_indices(size_t id, size_t &q_idx, size_t &v_idx) const {
  auto &d = *data;
  INDEX_CHECK_LEN(id, joint_count());
  auto &j = d.model.joints[id];
  q_idx = static_cast<size_t>(j.idx_q());
  v_idx = static_cast<size_t>(j.idx_v());
  return true;
}

bool Model::get_joint_axis(size_t id, Slice axis) const {
  auto &d = *data;
  INDEX_CHECK_LEN(id, joint_count());
  auto &j = d.model.joints[id];
  MAP_MUT(axis, 3) << boost::apply_visitor(AxisVisitor(),
                                           d.model.joints[id].toVariant());
  return true;
}

bool Model::get_joint_inertia(size_t id, double &mass, Slice lever,
                              Slice inertia) const {
  auto &d = *data;
  INDEX_CHECK_LEN(id, joint_count());
  const auto &i = d.model.inertias[id];
  mass = i.mass();
  MAP_MUT(lever, 3) << i.lever();
  MAP_MUT(inertia, 6) << i.inertia().data();
  return true;
}

bool Model::set_joint_inertia(size_t id, double mass, ConstSlice lever,
                              ConstSlice inertia) {
  auto &d = *data;
  INDEX_CHECK_LEN(id, joint_count());
  auto &i = d.model.inertias[id];
  i.mass() = mass;
  i.lever() << MAP(lever, 3);
  i.inertia().data() << MAP(inertia, 6);
  return true;
}

bool Model::get_joint_velocity(size_t id, Slice linear, Slice angular) const {
  auto &d = *data;
  INDEX_CHECK_LEN(id, joint_count());
  MAP_MUT(linear, 3) << d.data.v[id].linear();
  MAP_MUT(angular, 3) << d.data.v[id].angular();
  return true;
}

bool Model::omi(size_t id, Slice translation, Slice rotation) const {
  auto &d = *data;
  INDEX_CHECK_LEN(id, joint_count());
  auto &oMi = d.data.oMi[id];
  MAP_MUT(translation, 3) << oMi.translation();
  MAP_MUT(rotation, 9) << MAP_MAT_MUT(oMi.rotation(), 9);
  return true;
}

bool Model::limi(size_t id, Slice translation, Slice rotation) const {
  auto &d = *data;
  INDEX_CHECK_LEN(id, joint_count());
  auto &oMi = d.data.liMi[id];
  MAP_MUT(translation, 3) << oMi.translation();
  MAP_MUT(rotation, 9) << MAP_MAT_MUT(oMi.rotation(), 9);
  return true;
}

// Frames
size_t Model::frame_count() const {
  auto &d = *data;
  return d.model.frames.size();
}

size_t Model::get_frame_id(rust::Str name) const {
  auto &d = *data;
  return d.model.getFrameId(std::string(name));
}

bool Model::get_frame_name(size_t id, rust::String &name) const {
  auto &d = *data;
  INDEX_CHECK_LEN(id, frame_count());
  name = d.model.frames[id].name;
  return true;
}

bool Model::get_frame_pose(size_t id, Slice translation, Slice rotation) const {
  auto &d = *data;
  INDEX_CHECK_LEN(id, frame_count());
  auto &oMf = d.data.oMf[id];
  MAP_MUT(translation, 3) << oMf.translation();
  MAP_MUT(rotation, 9) << MAP_MAT_MUT(oMf.rotation(), 9);
  return true;
}

bool Model::get_frame_velocity(size_t id, Slice linear, Slice angular) const {
  auto &d = *data;
  INDEX_CHECK_LEN(id, frame_count());
  auto &v = d.data.v[d.model.frames[id].parentJoint];
  MAP_MUT(linear, 3) << v.linear();
  MAP_MUT(angular, 3) << v.angular();
  return true;
}

bool Model::get_frame_jacobian(size_t id, uint8_t reference_frame,
                               Slice jacobian) const {
  auto &d = *data;
  INDEX_CHECK_LEN(id, frame_count());
  INDEX_CHECK_LEN(reference_frame,
                  pin::ReferenceFrame::LOCAL_WORLD_ALIGNED + 1);
  const auto &frame = d.model.frames[id];
  auto mapped = Eigen::Map<Eigen::Matrix<double, 6, Eigen::Dynamic>>(
      jacobian.data(), 6, d.model.nv);
  pin::getFrameJacobian(d.model, d.data, frame.parentJoint, frame.placement,
                        static_cast<pin::ReferenceFrame>(reference_frame),
                        mapped);
  return true;
}

bool Model::omf(size_t id, Slice translation, Slice rotation) const {
  auto &d = *data;
  INDEX_CHECK_LEN(id, frame_count());
  auto &oMf = d.data.oMf[id];
  MAP_MUT(translation, 3) << oMf.translation();
  MAP_MUT(rotation, 9) << MAP_MAT_MUT(oMf.rotation(), 9);
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

bool Model::compute_joint_jacobians() {
  auto &d = *data;
  pin::computeJointJacobians(d.model, d.data);
  return true;
}

bool Model::update_frame_placements() {
  auto &d = *data;
  pin::updateFramePlacements(d.model, d.data);
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

bool Model::difference(ConstSlice q0, ConstSlice q1, Slice v) const {
  auto &d = *data;
  pin::difference(d.model, MAP(q0, nq()), MAP(q1, nq()), MAP_MUT(v, nv()));
  return true;
}

bool Model::integrate(ConstSlice q, ConstSlice vdt, Slice q_next) const {
  auto &d = *data;
  MAP_MUT(q_next, nq()) = pin::integrate(d.model, MAP(q, nq()), MAP(vdt, nv()));
  return true;
}

// Ctors
std::unique_ptr<Model> Model::clone() const {
  return std::make_unique<Model>(*this);
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
      model.lowerPositionLimit[id] = urdf_joint->limits->lower;
      model.upperPositionLimit[id] = urdf_joint->limits->upper;
    }
  }
  auto data = std::make_unique<ModelImpl>(model);
  return std::make_unique<Model>(std::move(data));
}

std::unique_ptr<Model> model_load(rust::Str path) {
  return Model::model_load(path);
}
} // namespace pinocchio_bridge
