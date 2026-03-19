#pragma once
#include <cstddef>
#include <memory>
#include <rust/cxx.h>

namespace pinocchio_bridge {
using ConstSlice = rust::Slice<const double>;
using Slice = rust::Slice<double>;

class Model {
public:
  // Joints
  bool exist_joint_name(rust::Str name) const;
  int get_joint_id(rust::Str name) const;

  // State
  int nq() const;
  int nv() const;
  bool neutral(rust::Slice<double> out) const;
  bool forward_kinematics(ConstSlice q);
  bool difference(ConstSlice q0, ConstSlice q1, Slice q);
  bool aba(ConstSlice q, ConstSlice v, ConstSlice tau, Slice ddq);
  bool integrate(ConstSlice q, ConstSlice dv, Slice q_out);

  static std::unique_ptr<Model> model_load(rust::Str path);

private:
  class ModelImpl;
  std::unique_ptr<ModelImpl> p_impl;
  Model() = delete;

public:
  Model(std::unique_ptr<ModelImpl> &&p_impl);
  ~Model();
};

// Namespace methods
std::unique_ptr<Model> model_load(rust::Str path);
} // namespace pinocchio_bridge
