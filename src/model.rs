use std::ops::Deref;

use crate::ffi;
use cxx::UniquePtr;
use nalgebra::DVector;
type DVec = DVector<f64>;

// ffi model wrapper
pub struct Model(UniquePtr<ffi::Model>);
impl Model {
    pub fn load(path: &str) -> Self {
        Self(ffi::model_load(path))
    }

    fn neutral(&self, out: &mut DVec) {
        assert!(self.0.neutral(out.as_mut_slice()), "invalid size");
    }
}

impl Deref for Model {
    type Target = ffi::Model;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

#[cfg(test)]
mod tests {
    use std::{env, path::PathBuf};

    use super::*;
    use nalgebra::{dvector, DVector};

    #[test]
    fn test_pinocchio_model() {
        let path = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap()).join("./src/model.urdf");
        let mut model = Model::load(path.to_str().unwrap());
        dbg!(model.nq());
        dbg!(model.nv());
    }

    // #[test]
    // fn test_dynamic_system() {
    //     let mut a = dvector![1.0_f64, 2.0, 3.0];
    //     let mut b = DVector::zeros(3);
    //     let mut dy = Model::new_instance();
    //     let slice = a.as_mut_slice();
    //     dy.pin_mut().set_state(slice);
    //     dy.get_state(&mut b.as_mut_slice());
    //     assert_eq!(a, b);
    // }
}
