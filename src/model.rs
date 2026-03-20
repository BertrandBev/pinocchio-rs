use std::{borrow::Cow, ops::Deref};

use crate::ffi;
use cxx::UniquePtr;
use nalgebra::SVector;

pub struct Model<const NQ: usize, const NV: usize>(UniquePtr<ffi::Model>);

pub type SVec<const N: usize> = SVector<f64, N>;

impl<const NQ: usize, const NV: usize> Model<NQ, NV> {
    pub fn load(path: &str) -> Result<Self> {
        let loaded = ffi::model_load(path)?;
        if loaded.nq() != NQ as i32 {
            return Err(format!("loaded nq {} expected {}", loaded.nq(), NQ).into());
        }
        if loaded.nv() != NV as i32 {
            return Err(format!("loaded nv {} expected {}", loaded.nv(), NV).into());
        }
        Ok(Self(loaded))
    }

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

    pub fn difference(&mut self, q0: &SVec<NQ>, q1: &SVec<NQ>, v: &mut SVec<NV>) {
        assert!(self
            .0
            .pin_mut()
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
            .aba(q.as_slice(), v.as_slice(), a.as_slice(), t.as_mut_slice()));
    }

    pub fn integrate(&mut self, q: &SVec<NQ>, dv: &SVec<NV>, q_out: &mut SVec<NQ>) {
        assert!(self
            .0
            .pin_mut()
            .integrate(q.as_slice(), dv.as_slice(), q_out.as_mut_slice()));
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
        write!(f, "Model {{ nq: {}, nv: {} }}", self.nq(), self.nv())
    }
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

#[cfg(test)]
mod tests {
    use std::{env, path::PathBuf};

    use super::*;

    #[test]
    fn test_pinocchio_model() {
        let path = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap()).join("./src/model.urdf");
        let model = Model::<9, 8>::load(path.to_str().unwrap()).unwrap();
        assert_eq!(model.nq(), 9);
        assert_eq!(model.nv(), 8);
        let mut q: SVec<9> = Default::default();
        model.neutral(&mut q);
        println!("out: {}", q);
    }
}
