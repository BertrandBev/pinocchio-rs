#[cxx::bridge(namespace = "pinocchio_bridge")]
mod ffi {
    unsafe extern "C++" {
        include!("pinocchio_bridge/cpp/src/pinocchio_bridge.h");

        type Model;

        // Joints
        fn joint_count(&self) -> usize;
        fn get_joint_id(&self, name: &str) -> usize;
        fn get_joint(&self, idx: usize, q_idx: &mut usize, v_idx: &mut usize) -> bool;

        // State
        fn nq(&self) -> i32;
        fn nv(&self) -> i32;
        fn neutral(&self, out: &mut [f64]) -> bool;
        fn forward_kinematics(self: Pin<&mut Self>, q: &[f64]) -> bool;
        fn difference(self: Pin<&mut Self>, q0: &[f64], q1: &[f64], v: &mut [f64]) -> bool;
        fn rnea(self: Pin<&mut Self>, q: &[f64], v: &[f64], a_cmd: &[f64], t: &mut [f64]) -> bool;
        fn aba(self: Pin<&mut Self>, q: &[f64], v: &[f64], t: &[f64], ddq: &mut [f64]) -> bool;
        fn integrate(self: Pin<&mut Self>, q: &[f64], vdt: &[f64], q_next: &mut [f64]) -> bool;

        // Namespace methods
        fn model_load(path: &str) -> Result<UniquePtr<Model>>;
    }
}

mod model;
pub use model::Model;
