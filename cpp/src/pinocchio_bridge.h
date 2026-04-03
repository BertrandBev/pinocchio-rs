#pragma once
#include <cstddef>
#include <memory>
#include <rust/cxx.h>

namespace pinocchio_bridge {
using ConstSlice = rust::Slice<const double>;
using Slice = rust::Slice<double>;

struct Joint {
  size_t q_idx;
  size_t v_idx;
};

class Model {
public:
  // Joints
  size_t joint_count() const;
  size_t get_joint_id(rust::Str name) const;
  bool get_joint_indices(size_t id, size_t &q_idx, size_t &v_idx) const;
  bool get_joint_axis(size_t id, Slice axis) const;
  bool get_joint_name(size_t id, rust::String &name) const;
  bool get_joint_inertia(size_t id, double &mass, Slice lever,
                         Slice inertia) const;
  bool set_joint_inertia(size_t id, double mass, ConstSlice lever,
                         ConstSlice inertia);
  bool get_joint_velocity(size_t id, Slice linear, Slice angular) const;
  bool omi(size_t id, Slice translation, Slice rotation) const;
  bool limi(size_t id, Slice translation, Slice rotation) const;

  // Frames
  size_t frame_count() const;
  size_t get_frame_id(rust::Str name) const;
  bool get_frame_name(size_t id, rust::String &name) const;
  bool get_frame_pose(size_t id, Slice translation, Slice rotation) const;
  bool get_frame_velocity(size_t id, Slice linear, Slice angular) const;
  bool get_frame_jacobian(size_t id, uint8_t reference_frame,
                          Slice jacobian) const;
  bool omf(size_t id, Slice translation, Slice rotation) const;

  // State
  int nq() const;
  int nv() const;
  bool neutral(rust::Slice<double> out) const;
  bool forward_kinematics(ConstSlice q);
  bool compute_joint_jacobians();
  bool update_frame_placements();
  bool rnea(ConstSlice q, ConstSlice v, ConstSlice a_cmd, Slice t);
  bool aba(ConstSlice q, ConstSlice v, ConstSlice t, Slice ddq);
  bool difference(ConstSlice q0, ConstSlice q1, Slice q) const;
  bool integrate(ConstSlice q, ConstSlice vdt, Slice q_next) const;

  // Ctors
  std::unique_ptr<Model> clone() const;
  static std::unique_ptr<Model> model_load(rust::Str path);

private:
  class ModelImpl;
  std::unique_ptr<ModelImpl> data;
  Model() = delete;

public:
  Model(std::unique_ptr<ModelImpl> &&p_impl);
  Model(const Model &other);
  ~Model();
};

// Namespace methods
std::unique_ptr<Model> model_load(rust::Str path);
} // namespace pinocchio_bridge
