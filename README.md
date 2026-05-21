### Pinocchio RS

![Crates.io Version](https://img.shields.io/crates/v/pinocchio_rs?color=green)
![Crates.io MSRV](https://img.shields.io/crates/msrv/pinocchio_rs)
![Static Badge](https://img.shields.io/badge/handcoded%20-%20purple)

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
