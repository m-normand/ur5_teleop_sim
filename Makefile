_INFO  := "[ \33[1;32mOK\33[0m ] %b\n"
_WARN  := "[\33[1;33mWARN\33[0m] %b\n"
_ERROR := "[ \33[1;31mERR\33[0m] %b\n"


all: ur5 keyboard-container
	@$(MAKE) --no-print-directory keyboard-ui
	@$(MAKE) --no-print-directory stop

logs:
	@docker compose logs --tail=100 -f

stop: ur5-stop
	@docker compose down

ur5: ur5-stop
	@printf $(_INFO) "Giving docker access to X server"
	@xhost +local:rviz-user

	@printf $(_INFO) "Spinning up UR5 teleop simulation"
	@docker compose up -d --remove-orphans ur5-sim

ur5-stop:
	@printf $(_INFO) "Ensuring UR5 teleop simulation is down"
	@if docker ps | grep -q mnorma-ur5-sim; then \
		printf $(_WARN) "Shutting down UR5 teleop simulation"; \
		docker compose down ur5-sim; \
		printf $(_INFO) "Closing docker access to X server"; \
		xhost -local:rviz-user; \
	fi

keyboard-ui:
	@printf $(_INFO) "Attaching to keyboard-teleop"
	@docker exec -it mnorma-keyboard-teleop \
		bash -ic ". /ws/devel/setup.bash && rosrun keyboard_teleop keyboard_teleop_node"

keyboard-container:
	@printf $(_INFO) "Spinning up keyboard-teleop container"
	@docker compose up -d --remove-orphans keyboard-teleop; \

keyboard-stop:
	@printf $(_INFO) "Ensuring keyboard-teleop is down"
	@if docker ps | grep -q mnorma-keyboard-teleop; then \
		printf $(_WARN) "Shutting down keyboard-teleop"; \
		docker compose down keyboard-teleop; \
	fi

build:
	@printf $(_INFO) "Building UR5 teleop simulation images"
	@docker compose build
