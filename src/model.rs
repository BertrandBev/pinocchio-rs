use std::{borrow::Cow, ops::Deref, path::Path};

use crate::ffi;
use cxx::UniquePtr;
use nalgebra::{SMatrix, SVector};

pub struct Model<const NQ: usize, const NV: usize>(UniquePtr<ffi::Model>);

pub type SVec<const N: usize> = SVector<f64, N>;
pub type SMat<const R: usize, const C: usize> = SMatrix<f64, R, C>;

#[repr(u8)]
#[derive(Clone, Copy)]
pub enum ReferenceFrame {
    World = 0,
    Local,
    LocalWorldAligned,
}

impl<const NQ: usize, const NV: usize> Clone for Model<NQ, NV> {
    fn clone(&self) -> Self {
        Self(self.0.clone())
    }
}

impl<const NQ: usize, const NV: usize> Model<NQ, NV> {
    pub fn load(path: &str, free_flyer: bool) -> Result<Self> {
        if !Path::new(path).exists() {
            return Err(format!("file {} not found", path).into());
        }
        let loaded = ffi::model_load(path, free_flyer)?;
        if loaded.nq() != NQ as i32 {
            return Err(format!("loaded nq {} expected {}", loaded.nq(), NQ).into());
        }
        if loaded.nv() != NV as i32 {
            return Err(format!("loaded nv {} expected {}", loaded.nv(), NV).into());
        }
        Ok(Self(loaded))
    }

    // Joints
    pub fn get_joint_indices(&self, id: usize) -> JointIndices {
        let mut indices = JointIndices::default();
        self.0
            .get_joint_indices(id, &mut indices.q_idx, &mut indices.v_idx);
        indices
    }

    pub fn get_joint_name(&self, id: usize) -> String {
        let mut name = String::default();
        assert!(self.0.get_joint_name(id, &mut name));
        name
    }

    pub fn get_joint_axis(&self, id: usize) -> SVec<3> {
        let mut axis = SVec::<3>::default();
        assert!(self.0.get_joint_axis(id, axis.as_mut_slice()));
        axis
    }

    pub fn get_joint_inertia(&self, id: usize) -> JointInertia {
        let mut inertia = JointInertia::default();
        assert!(self.0.get_joint_inertia(
            id,
            &mut inertia.mass,
            inertia.lever.as_mut_slice(),
            inertia.inertia.as_mut_slice(),
        ));
        inertia
    }

    pub fn set_joint_inertia(&mut self, id: usize, inertia: &JointInertia) {
        assert!(self.0.pin_mut().set_joint_inertia(
            id,
            inertia.mass,
            inertia.lever.as_slice(),
            inertia.inertia.as_slice(),
        ));
    }

    pub fn get_joint_velocity(&self, id: usize) -> (SVec<3>, SVec<3>) {
        let mut linear = SVec::<3>::default();
        let mut angular = SVec::<3>::default();
        assert!(self
            .0
            .get_joint_velocity(id, linear.as_mut_slice(), angular.as_mut_slice()));
        return (linear, angular);
    }

    pub fn omi(&self, id: usize) -> (SVec<3>, SMat<3, 3>) {
        let mut translation = SVec::<3>::default();
        let mut rotation = SMat::<3, 3>::default();
        assert!(self
            .0
            .omi(id, translation.as_mut_slice(), rotation.as_mut_slice()));
        (translation, rotation)
    }

    pub fn limi(&self, id: usize) -> (SVec<3>, SMat<3, 3>) {
        let mut translation = SVec::<3>::default();
        let mut rotation = SMat::<3, 3>::default();
        assert!(self
            .0
            .limi(id, translation.as_mut_slice(), rotation.as_mut_slice()));
        (translation, rotation)
    }

    // Frames
    pub fn get_frame_name(&self, id: usize) -> String {
        let mut name = String::default();
        assert!(self.0.get_frame_name(id, &mut name));
        name
    }

    pub fn get_frame_pose(&self, id: usize) -> (SVec<3>, SMat<3, 3>) {
        let mut translation = SVec::<3>::default();
        let mut rotation = SMat::<3, 3>::default();
        assert!(self
            .0
            .get_frame_pose(id, translation.as_mut_slice(), rotation.as_mut_slice()));
        (translation, rotation)
    }

    pub fn get_frame_velocity(&self, id: usize) -> (SVec<3>, SVec<3>) {
        let mut linear = SVec::<3>::default();
        let mut angular = SVec::<3>::default();
        assert!(self
            .0
            .get_frame_velocity(id, linear.as_mut_slice(), angular.as_mut_slice()));
        (linear, angular)
    }

    pub fn get_frame_jacobian(
        &self,
        id: usize,
        reference_frame: ReferenceFrame,
        j: &mut SMat<6, NV>,
    ) {
        assert!(self
            .0
            .get_frame_jacobian(id, reference_frame as u8, j.as_mut_slice()));
    }

    pub fn omf(&self, id: usize) -> (SVec<3>, SMat<3, 3>) {
        let mut translation = SVec::<3>::default();
        let mut rotation = SMat::<3, 3>::default();
        assert!(self
            .0
            .omf(id, translation.as_mut_slice(), rotation.as_mut_slice()));
        (translation, rotation)
    }

    // State

    pub fn nq(&self) -> usize {
        NQ
    }

    pub fn nv(&self) -> usize {
        NV
    }

    pub fn neutral(&self, out: &mut SVec<NQ>) {
        assert!(self.0.neutral(out.as_mut_slice()));
    }

    pub fn forward_kinematics(&mut self, q: &SVec<NQ>) {
        assert!(self.0.pin_mut().forward_kinematics(q.as_slice()));
    }

    pub fn update_frame_placements(&mut self) {
        assert!(self.0.pin_mut().update_frame_placements());
    }

    pub fn compute_joint_jacobians(&mut self) {
        assert!(self.0.pin_mut().compute_joint_jacobians());
    }

    pub fn difference(&self, q0: &SVec<NQ>, q1: &SVec<NQ>, v: &mut SVec<NV>) {
        assert!(self
            .0
            .difference(q0.as_slice(), q1.as_slice(), v.as_mut_slice()));
    }

    pub fn aba(&mut self, q: &SVec<NQ>, v: &SVec<NV>, t: &SVec<NV>, ddq: &mut SVec<NV>) {
        assert!(self
            .0
            .pin_mut()
            .aba(q.as_slice(), v.as_slice(), t.as_slice(), ddq.as_mut_slice()));
    }

    pub fn rnea(&mut self, q: &SVec<NQ>, v: &SVec<NV>, a: &SVec<NV>, t: &mut SVec<NV>) {
        assert!(self
            .0
            .pin_mut()
            .rnea(q.as_slice(), v.as_slice(), a.as_slice(), t.as_mut_slice()));
    }

    pub fn integrate(&self, q: &SVec<NQ>, dv: &SVec<NV>, q_out: &mut SVec<NQ>) {
        assert!(self
            .0
            .integrate(q.as_slice(), dv.as_slice(), q_out.as_mut_slice()));
    }

    // Integration schemas
    pub fn semi_implicit_euler(
        &mut self,
        q: &mut SVec<NQ>,
        v: &mut SVec<NV>,
        t: &SVec<NV>,
        dt: f64,
    ) {
        let mut ddq = SVec::<NV>::zeros();
        self.aba(q, v, t, &mut ddq);
        v.axpy(dt, &ddq, 1.0); // v += ddq * dt
        let mut q_next = SVec::<NQ>::zeros();
        ddq.axpy(dt, &v, 0.0); // Re-use ddq = v * dt
        self.integrate(q, &ddq, &mut q_next);
        *q = q_next;
    }

    fn qv_diff(&mut self, qv: &(SVec<NQ>, SVec<NV>), t: &SVec<NV>) -> (SVec<NV>, SVec<NV>) {
        let mut ddq = SVec::<NV>::zeros();
        self.aba(&qv.0, &qv.1, t, &mut ddq);
        (qv.1.clone(), ddq)
    }

    fn qv_integrate(&mut self, qv: &mut (SVec<NQ>, SVec<NV>), dqv: &(SVec<NV>, SVec<NV>), dt: f64) {
        let mut q_next = SVec::<NQ>::zeros();
        let vdt = dqv.0.clone() * dt;
        self.integrate(&qv.0, &vdt, &mut q_next);
        qv.0 = q_next;
        qv.1 += dqv.1 * dt;
    }

    pub fn runge_kutta_4(&mut self, q: &mut SVec<NQ>, v: &mut SVec<NV>, t: &SVec<NV>, dt: f64) {
        let qv = (q.clone(), v.clone());

        // k1
        let k1 = self.qv_diff(&qv, t);

        // k2
        let mut qv_temp = qv.clone();
        self.qv_integrate(&mut qv_temp, &k1, dt / 2.0);
        let k2 = self.qv_diff(&qv_temp, t);

        // k3
        let mut qv_temp = qv.clone();
        self.qv_integrate(&mut qv_temp, &k2, dt / 2.0);
        let k3 = self.qv_diff(&qv_temp, t);

        // k4
        let mut qv_temp = qv.clone();
        self.qv_integrate(&mut qv_temp, &k3, dt);
        let k4 = self.qv_diff(&qv_temp, t);

        // Full step
        let mut qv_temp = qv.clone();
        self.qv_integrate(
            &mut qv_temp,
            &(
                k1.0 + k2.0 * 2.0 + k3.0 * 2.0 + k4.0,
                k1.1 + k2.1 * 2.0 + k3.1 * 2.0 + k4.1,
            ),
            dt / 6.0,
        );
        q.copy_from(&qv_temp.0);
        v.copy_from(&qv_temp.1);
    }

    // Additionnal debug functions
    pub fn dbg_joints(&self, f: &mut impl std::io::Write) {
        for k in 0..self.0.joint_count() {
            let (translation, rotation) = self.omi(k);
            let name = self.get_joint_name(k);
            writeln!(
                f,
                "joint {k}: {name}. translation: {translation}, rotation: {rotation}"
            )
            .unwrap();
        }
    }
}

impl<const NQ: usize, const NV: usize> Deref for Model<NQ, NV> {
    type Target = ffi::Model;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<const NQ: usize, const NV: usize> std::fmt::Debug for Model<NQ, NV> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "Model {{ {} joints; {} frames; nq: {}, nv: {} }}",
            self.joint_count(),
            self.frame_count(),
            self.nq(),
            self.nv()
        )?;
        for k in 0..self.joint_count() {
            writeln!(f, "- joint {k}: {}", self.get_joint_name(k))?;
        }
        for k in 0..self.frame_count() {
            writeln!(f, "- frame {k}: {}", self.get_frame_name(k))?;
        }
        Ok(())
    }
}

// Output types
#[derive(Debug, Clone, Default)]
pub struct JointIndices {
    pub q_idx: usize,
    pub v_idx: usize,
}

#[derive(Debug, Clone, Default)]
pub struct JointInertia {
    /// Joint mass
    pub mass: f64,
    /// Joint center of mass
    pub lever: SVec<3>,
    /// Column major symmetric inertia matrix
    pub inertia: SVec<6>,
}

// Custom result type
type Result<T> = std::result::Result<T, ModelError>;

#[derive(Debug, Clone)]
pub struct ModelError(Cow<'static, str>);

impl From<cxx::Exception> for ModelError {
    fn from(value: cxx::Exception) -> Self {
        Self(value.what().to_string().into())
    }
}

impl std::fmt::Display for ModelError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.0.fmt(f)
    }
}

impl From<String> for ModelError {
    fn from(value: String) -> Self {
        Self(value.into())
    }
}

impl std::error::Error for ModelError {}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{env, path::PathBuf};

    // #[test]
    // fn test_pinocchio_model() {
    //     let path = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap()).join("./src/model.urdf");
    //     let mut model = Model::<9, 8>::load(path.to_str().unwrap(), true).unwrap();
    //     assert_eq!(model.nq(), 9);
    //     assert_eq!(model.nv(), 8);
    //     let mut q: SVec<9> = Default::default();
    //     model.neutral(&mut q);
    //     model.forward_kinematics(&q);
    //     println!("{:?}", model);
    //     println!("q: {}", q);
    // }
}
