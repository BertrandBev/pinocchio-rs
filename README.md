## Pinocchio RS

[![Crates.io Version](https://img.shields.io/crates/v/pinocchio_rs?color=green)](https://crates.io/crates/pinocchio_rs)
[![Crates.io MSRV](https://img.shields.io/crates/msrv/pinocchio_rs)](https://crates.io/crates/pinocchio_rs)
[![Static Badge](https://img.shields.io/badge/handcoded%20-%20purple)](https://crates.io/crates/pinocchio_rs)

<p align="center">
  <img src="assets/anim.png" width="480">
</p>

Pinocchio-RS is a rust wrapper around [pinocchio](https://github.com/stack-of-tasks/pinocchio)

It can be used with pre-built binaries on linux or macos

```TOML
pinocchio_rs = { version = "latest"}
```

Or with a full source build (requires [https://xmake.io/](xmake) to be installed)

```TOML
pinocchio_rs = { version = "latest", features = ["source_build"]}
```

### Double pendulum

Here's how to setup a simple pendulum

```rust
// Load the model from a URDF descriptor
let mut model = Model::load(
            &format!("{}/model/double_pendulum.urdf", env!("CARGO_MANIFEST_DIR")),
            false,
        )
        .unwrap();

// Initialize the state vector
let mut q = SVec::default();
q[0] = PI / 2.0;
q[1] = PI / 4.0;
model.forward_kinematics(&q);

// Run sim loop
loop {
    // Zero torques on both joints
    let t = SVec::default();
    self.model.runge_kutta_4(&mut self.q, &mut self.v, &t, dt);
    // Or self.model.semi_implicit_euler(...) for faster iterations
    self.model.forward_kinematics(&self.q);
    // Run at 100Hz
    std::thread::sleep(std::time::Duration::from_millis(10));
}
```
