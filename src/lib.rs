use std::ops::Deref;

use cxx::UniquePtr;
use nalgebra::DVector;

#[cxx::bridge(namespace = "pinocchio_bridge")]
mod ffi {
    unsafe extern "C++" {
        include!("pinocchio_bridge/cpp/src/pinocchio_bridge.h");

        type Model;

        // Joints
        fn exist_joint_name(&self, name: &str) -> bool;
        fn get_joint_id(&self, name: &str) -> i32;

        // State
        fn nq(&self) -> i32;
        fn nv(&self) -> i32;
        fn neutral(&self, out: &mut [f64]) -> bool;
        fn forward_kinematics(self: Pin<&mut Self>, q: &[f64]) -> bool;
        fn difference(self: Pin<&mut Self>, q0: &[f64], q1: &[f64], q: &mut [f64]) -> bool;
        fn aba(self: Pin<&mut Self>, q: &[f64], v: &[f64], tau: &[f64], ddq: &mut [f64]) -> bool;
        fn integrate(self: Pin<&mut Self>, q: &[f64], dv: &[f64], q_out: &mut [f64]) -> bool;

        // Namespace methods
        fn model_load(path: &str) -> UniquePtr<Model>;
    }
}

mod model;
