# UR5 Teleop Sim

[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)
[![Linting](https://github.com/m-normand/ur5_teleop_sim/actions/workflows/ci-lint.yml/badge.svg)](https://github.com/m-normand/ur5_teleop_sim/actions/workflows/ci-lint.yml)

![sim_run](docs/sim_run.png)

A simple simulation that demonstrates teleop of a UR5 via keyboard

> [!WARNING]
> Work in progress... Mileage may vary...

- [Usage](#usage)
- [Installation](#installation)
- [Running](#running)
- [Subdirs](#subdirs)
  - [keyboard-teleop](#keyboard-teleop)
  - [ur5-sim](#ur5-sim)
- [To Do](#to-do)

## Usage

## Installation

The install script ensures the following are on your machine

- `docker`
- `make`
- `pre-commit`

To install

```bash
./install.sh
```

## Running

Use **Makefile** targets instead of invoking `docker` directly

```bash
make all   # Starts Sim
make stop  # Ends Sim
make logs  # Read docker logs
make build # To rebuild containers after each change
```

Generally it is sufficient to run

```bash
make all
```

## Subdirs

### keyboard-teleop

Package for converting key-strokes into `TwistStamped` for the robot to act on.

### ur5-sim

Large package that hosts `gazebo` + `rviz` + `ur5_moveit_config`.
The core simulation to be manipulated.

## To Do

- [ ] UR5 behaves strange from TwistStamped...
