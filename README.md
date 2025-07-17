# UR5 Teleop Sim

A simple simulation that demonstrates teleop of a UR5 with considerations
of Latency + Delays

## Installation

The install script ensures the following are on your machine

- `docker`
- `make`
- `pre-commit`

To install

```bash
./install.sh
```

## Usage

Use **Makefile** targets instead of invoking `docker` directly

```bash
make build # To rebuild containers after each change
make up    # Starts Sim
make down  # Ends Sim
make logs  # Read docker logs
```
