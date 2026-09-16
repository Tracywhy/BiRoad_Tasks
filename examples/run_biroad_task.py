#!/usr/bin/env python3

import argparse

from rlbench.action_modes.action_mode import BimanualMoveArmThenGripper
from rlbench.action_modes.arm_action_modes import BimanualJointPosition
from rlbench.action_modes.gripper_action_modes import BimanualDiscrete
from rlbench.backend.utils import task_file_to_task_class
from rlbench.environment import Environment


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Launch and reset a BiRoAD bimanual RLBench task.")
    parser.add_argument(
        "--task",
        default="bimanual_put_item_in_drawer_random",
        help="Task module name from rlbench/bimanual_tasks without .py.")
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Launch CoppeliaSim without its graphical interface.")
    args = parser.parse_args()

    action_mode = BimanualMoveArmThenGripper(
        BimanualJointPosition(), BimanualDiscrete())
    environment = Environment(
        action_mode=action_mode,
        robot_setup="dual_panda",
        headless=args.headless)

    environment.launch()
    try:
        task_class = task_file_to_task_class(args.task, bimanual=True)
        task = environment.get_task(task_class)
        descriptions, _ = task.reset()
        print(f"Loaded task: {task.get_name()}")
        print("Descriptions:")
        for description in descriptions:
            print(f"- {description}")
    finally:
        environment.shutdown()


if __name__ == "__main__":
    main()
