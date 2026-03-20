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
  bool get_joint(size_t id, size_t &q_idx, size_t &v_idx) const;

  // State
  int nq() const;
  int nv() const;
  bool neutral(rust::Slice<double> out) const;
  bool forward_kinematics(ConstSlice q);
  bool difference(ConstSlice q0, ConstSlice q1, Slice q);
  bool rnea(ConstSlice q, ConstSlice v, ConstSlice a_cmd, Slice t);
  bool aba(ConstSlice q, ConstSlice v, ConstSlice t, Slice ddq);
  bool integrate(ConstSlice q, ConstSlice vdt, Slice q_next);

  static std::unique_ptr<Model> model_load(rust::Str path);

private:
  class ModelImpl;
  std::unique_ptr<ModelImpl> data;
  Model() = delete;

public:
  Model(std::unique_ptr<ModelImpl> &&p_impl);
  ~Model();
};

// Namespace methods
std::unique_ptr<Model> model_load(rust::Str path);
} // namespace pinocchio_bridge
