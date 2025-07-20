#!/usr/bin/env bash
# set_ur5_configuration.sh
# This script sets the UR5 robot's joints to a "HOME" position
# after the robot model has been spawned in Gazebo.
# Finally, it unpauses Gazebo.

SLEEP_RATE=1

loginfo() {
  echo "[INFO] [/set_ur5_configuration] $1"
}

wait_for_spawn_model_node_start() {
  local timeout=10
  local elapsed=0

  loginfo "Waiting for /spawn_gazebo_model node to start..."
  while ! rosnode list | grep -q "/spawn_gazebo_model"; do
    sleep $SLEEP_RATE
    elapsed=$((elapsed + SLEEP_RATE))

    if [[ $elapsed -ge $timeout ]]; then
      loginfo "Timeout waiting for /spawn_gazebo_model node to start."
      return 1
    fi
  done

  loginfo "/spawn_gazebo_model node is now running."
}

wait_for_spawn_model_node_end() {
  loginfo "Waiting for /spawn_gazebo_model node to finish..."
  while rosnode list | grep -q "/spawn_gazebo_model"; do
    sleep $SLEEP_RATE
  done
  loginfo "/spawn_gazebo_model node is now finished."
}

set_model_configuration() {
  loginfo "Setting joints to home configuration..."
  rosservice call /gazebo/set_model_configuration "model_name: 'robot'
urdf_param_name: ''
joint_names:
- 'shoulder_pan_joint'
- 'shoulder_lift_joint'
- 'elbow_joint'
- 'wrist_1_joint'
- 'wrist_2_joint'
- 'wrist_3_joint'
joint_positions:
- -1.1349
- -1.3759
- -1.497
- -1.8574
- 1.5622
- 3.5832" &>/dev/null
}

unpause_gazebo() {
  loginfo "Unpausing Gazebo physics..."
  rosservice call /gazebo/unpause_physics &>/dev/null
}

main() {
  wait_for_spawn_model_node_start
  wait_for_spawn_model_node_end
  set_model_configuration
  unpause_gazebo
}

main
