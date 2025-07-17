_INFO  := "[ \33[1;32mOK\33[0m ] %b\n"
_WARN  := "[\33[1;33mWARN\33[0m] %b\n"
_ERROR := "[ \33[1;31mERR\33[0m] %b\n"

up: down
	@printf $(_INFO) "Spinning up UR5 teleop simulation"
	@xhost +local:root  # Allow Docker to access the X server
	@docker compose up -d --remove-orphans

logs:
	@docker compose logs --tail=100 -f

down:
	@printf $(_INFO) "Ensuring UR5 teleop simulation is down"
	@if docker ps | grep -q ur5; then \
		printf $(_WARN) "Shutting down UR5 teleop simulation"; \
		docker compose down; \
		xhost -local:ros; \
	fi

build:
	@printf $(_INFO) "Building UR5 teleop simulation image"
	@docker compose build
