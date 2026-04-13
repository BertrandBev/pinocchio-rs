#[cxx::bridge(namespace = "pinocchio_bridge")]
mod ffi {
    unsafe extern "C++" {
        include!("pinocchio_bridge.h");

        type Model;

        // Joints
        fn joint_count(&self) -> usize;
        fn get_joint_id(&self, name: &str) -> usize;
        fn get_joint_indices(&self, id: usize, q_idx: &mut usize, v_idx: &mut usize) -> bool;
        fn get_joint_axis(&self, id: usize, axis: &mut [f64]) -> bool;
        fn get_joint_name(&self, id: usize, name: &mut String) -> bool;
        fn get_joint_inertia(
            &self,
            id: usize,
            mass: &mut f64,
            lever: &mut [f64],
            inertia: &mut [f64],
        ) -> bool;
        fn set_joint_inertia(
            self: Pin<&mut Self>,
            id: usize,
            mass: f64,
            lever: &[f64],
            inertia: &[f64],
        ) -> bool;
        fn get_joint_velocity(&self, id: usize, linear: &mut [f64], angular: &mut [f64]) -> bool;
        fn omi(&self, id: usize, translation: &mut [f64], rotation: &mut [f64]) -> bool;
        fn limi(&self, id: usize, translation: &mut [f64], rotation: &mut [f64]) -> bool;

        // Frames
        fn frame_count(&self) -> usize;
        fn get_frame_id(&self, name: &str) -> usize;
        fn get_frame_name(&self, id: usize, name: &mut String) -> bool;
        fn get_frame_pose(&self, id: usize, translation: &mut [f64], rotation: &mut [f64]) -> bool;
        fn get_frame_velocity(&self, id: usize, linear: &mut [f64], angular: &mut [f64]) -> bool;
        fn get_frame_jacobian(&self, id: usize, reference_frame: u8, jacobian: &mut [f64]) -> bool;
        fn omf(&self, id: usize, translation: &mut [f64], rotation: &mut [f64]) -> bool;

        // State
        fn nq(&self) -> i32;
        fn nv(&self) -> i32;
        fn neutral(&self, out: &mut [f64]) -> bool;
        fn forward_kinematics(self: Pin<&mut Self>, q: &[f64]) -> bool;
        fn compute_joint_jacobians(self: Pin<&mut Self>) -> bool;
        fn update_frame_placements(self: Pin<&mut Self>) -> bool;
        fn rnea(self: Pin<&mut Self>, q: &[f64], v: &[f64], a_cmd: &[f64], t: &mut [f64]) -> bool;
        fn aba(self: Pin<&mut Self>, q: &[f64], v: &[f64], t: &[f64], ddq: &mut [f64]) -> bool;
        fn difference(&self, q0: &[f64], q1: &[f64], v: &mut [f64]) -> bool;
        fn integrate(&self, q: &[f64], vdt: &[f64], q_next: &mut [f64]) -> bool;

        // Namespace methods
        fn clone(&self) -> UniquePtr<Model>;
        fn model_load(path: &str, free_flyer: bool) -> Result<UniquePtr<Model>>;
    }
}

unsafe impl Send for ffi::Model {}
unsafe impl Sync for ffi::Model {}

mod model;
pub use model::*;
