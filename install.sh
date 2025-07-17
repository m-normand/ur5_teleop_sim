#!/usr/bin/env bash
# Install script for the UR5 teleop simulation environment

_LOG_INFO() {
  printf "[ \033[1;32mOK\033[0m ] %b\n" "$1"
}

_LOG_WARN() {
  printf "[\033[1;33mWARN\033[0m] %b\n" "$1"
}

_LOG_ERROR() {
  printf "[ \033[1;31mERR\033[0m] %b\n" "$1"
}

install_docker() {
  command -v docker >/dev/null 2>&1 && {
    _LOG_INFO "Docker is already installed. Skipping installation."
    return
  }

  _LOG_INFO "Installing Docker..."

  # Add Docker's official GPG key
  sudo apt-get update
  sudo apt-get install ca-certificates curl gnupg
  sudo install -m 0755 -d /etc/apt/keyrings
  curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
  sudo chmod a+r /etc/apt/keyrings/docker.gpg

  # Add the repository to Apt sources
  # shellcheck source=/dev/null
  echo \
    deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
    "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable | \
    sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
  sudo apt-get update

  # Install Docker packages
  sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

  # Add current Linux user to "docker" Linux group (which was auto created when Docker was installed)
  sudo usermod -aG docker "$USER"
}

install_apt_reqs() {
  _LOG_INFO "Installing apt requirements"
  required_deps=("make" "pip")
  missing_deps=()

  for dep in "${required_deps[@]}"; do
    if ! command -v "$dep" &>/dev/null; then
      missing_deps+=("$dep")
    fi
  done

  if [[ -z "${missing_deps[*]}" ]]; then
    _LOG_INFO "Requirements '${required_deps[*]}' found"
    return 0
  else
    _LOG_WARN "Some dependencies not found"
    sudo apt update && sudo apt install "${missing_deps[*]}"
  fi

}

install_pip_reqs() {
  _LOG_INFO "Installing pip requirements"

  if command -v pre-commit &>/dev/null; then
    _LOG_INFO "Requirements 'pre-commit' found"
    return 0
  else
    _LOG_WARN "'pre-commit' not found"
    pip install pre-commit
  fi
}

install_precommit_hooks(){
  _LOG_INFO "Installing pre-commit hooks"
  pre-commit install --install-hooks 1>/dev/null
}

main() {
  install_docker
  install_apt_reqs
  install_pip_reqs
  install_precommit_hooks
}

main