# BiRoAD RLBench Task Suite

This repository provides the RLBench task definitions and CoppeliaSim model files used for the simulation evaluation in *BiRoAD: Learning Shared and Role-Adaptive Representations for Bimanual Manipulation*.

BiRoAD studies scene-conditioned role adaptation in bimanual manipulation. Different scene configurations can require the two arms to exchange their functional roles while preserving the same high-level task goal. The task suite is designed to evaluate whether a policy learns this role adaptation instead of associating a behavior with a fixed arm identity.

This repository contains task assets only. It does not include policy implementations, training code, demonstrations, or evaluation datasets.

## Task Suite

The core suite contains eight task families. Each family has a base-role configuration and a role-reversed configuration, giving 16 core task variants in total.

| Task family | Base-role configuration | Role-reversed configuration |
| --- | --- | --- |
| Handover Item | `bimanual_handover_item_easy_random` | `bimanual_handover_item_easy_symmetric_random` |
| Pick Laptop | `bimanual_pick_laptop_random` | `bimanual_pick_laptop_symmetric_random` |
| Pick Plate | `bimanual_pick_plate_random` | `bimanual_pick_plate_symmetric_random` |
| Put Bottle in Fridge | `bimanual_put_bottle_in_fridge` | `bimanual_put_bottle_in_fridge_symmetric` |
| Put Item in Drawer | `bimanual_put_item_in_drawer_random` | `bimanual_put_item_in_drawer_symmetric_random` |
| Sweep to Dustpan | `bimanual_sweep_to_dustpan_random` | `bimanual_sweep_to_dustpan_symmetric_random` |
| Take Tray out of Oven | `bimanual_take_tray_out_of_oven` | `bimanual_take_tray_out_of_oven_symmetric` |
| Take Shoes out of Box | `coordinated_take_shoes_out_of_box` | `coordinated_take_shoes_out_of_box_symmetric` |

The repository also includes two single-arm drawer variants:

- `bimanual_put_item_in_drawer_random_left`
- `bimanual_put_item_in_drawer_random_right`

These additional variants isolate left-arm-only and right-arm-only execution and are not part of the 16 core base-role/role-reversed task pairs.

## Task Design

The suite is built on RLBench and follows the bimanual setup of PerAct2 with two Franka Emika Panda arms. Across the paired configurations:

- Both configurations target the same high-level manipulation goal.
- Scene geometry and object placement determine the appropriate arm-role assignment.
- The role-reversed task exchanges the functional roles of the two arms.
- Randomized initializations broaden the object and interaction regions used for training and evaluation.

The BiRoAD experiments use these paired configurations to study both balanced and imbalanced role distributions. The paper evaluates balanced `50:50` and imbalanced `95:5` demonstration compositions. Demonstrations and split definitions are not included in this repository.

## Repository Structure

```text
BiRoad_Tasks/
|-- bimanual_tasks/   # Python task definitions
|-- task_ttms/        # CoppeliaSim task models
`-- README.md
```

Every Python task definition in `bimanual_tasks/` has a model with the same basename in `task_ttms/`.

## Installation

These tasks require a bimanual RLBench/PerAct2-compatible environment that provides `BimanualTask` and a two-arm Panda setup. They are not standalone tasks for an unmodified single-arm RLBench installation.

Clone this repository:

```bash
git clone https://github.com/Tracywhy/BiRoad_Tasks.git
cd BiRoad_Tasks
```

Copy the task definitions and models into the corresponding directories of your bimanual RLBench installation:

```bash
cp bimanual_tasks/*.py /path/to/RLBench/rlbench/bimanual_tasks/
cp task_ttms/*.ttm /path/to/RLBench/rlbench/task_ttms/
```

Keep each `.py` file paired with the `.ttm` file that has the same basename. The tasks can then be imported and launched through the same workflow used for other bimanual RLBench tasks in your environment.

## Citation

This task suite is built on [RLBench](https://github.com/stepjam/RLBench). Please cite RLBench when using these task assets:

```bibtex
@article{james2019rlbench,
  title={RLBench: The Robot Learning Benchmark \& Learning Environment},
  author={James, Stephen and Ma, Zicong and Rovick Arrojo, David and Davison, Andrew J.},
  journal={IEEE Robotics and Automation Letters},
  year={2020}
}
```
