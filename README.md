# BiRoAD RLBench Task Suite

This repository provides a self-contained bimanual RLBench environment and the task definitions and CoppeliaSim model files used for the simulation evaluation in *BiRoAD: Learning Shared and Role-Adaptive Representations for Bimanual Manipulation*.

BiRoAD studies scene-conditioned role adaptation in bimanual manipulation. Different scene configurations can require the two arms to exchange their functional roles while preserving the same high-level task goal. The task suite is designed to evaluate whether a policy learns this role adaptation instead of associating a behavior with a fixed arm identity.

The repository includes the PerAct2 bimanual RLBench runtime, its original single-arm and bimanual task assets, the matching bimanual PyRep fork, the dual-Panda environment, and the additional BiRoAD task suite. It does not include policy implementations, training code, demonstrations, or evaluation datasets.

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
|-- examples/             # RLBench examples and a BiRoAD task launcher
|-- rlbench/
|   |-- backend/          # RLBench task and scene backend
|   |-- tasks/            # Tasks inherited from the bimanual RLBench fork
|   |-- bimanual_tasks/   # Original bimanual tasks and BiRoAD additions
|   |-- robot_ttms/       # Robot models
|   `-- task_ttms/        # CoppeliaSim task models
|-- scripts/              # Repository-relative environment setup
|-- third_party/PyRep/    # Bimanual PyRep fork with dual-Panda support
|-- tools/                # Dataset generation and task utilities
|-- pyproject.toml
|-- requirements.txt
`-- README.md
```

Every Python task definition in `rlbench/bimanual_tasks/` has a model with the same basename in `rlbench/task_ttms/`.

## Installation

This repository must be used with its bundled bimanual RLBench and PyRep code rather than the standard single-arm packages from PyPI. The bundled PyRep fork provides `PandaLeft`, `PandaRight`, and the matching dual grippers; RLBench exposes them through `robot_setup="dual_panda"`. It requires CoppeliaSim 4.1 and currently targets Linux. Python 3.8 is recommended for compatibility with this stack.

Before installing, make sure that `python3`, `pip`, `curl`, a C/C++ compiler, and Python development headers are available. GPU-based headless rendering additionally requires a working NVIDIA/X or VirtualGL setup.

### Quick setup

Clone this repository:

```bash
git clone https://github.com/Tracywhy/BiRoad_Tasks.git
cd BiRoad_Tasks
```

Create and activate a Python 3.8 environment using either Conda or `venv`. Conda is recommended for this legacy CoppeliaSim/PyRep stack.

Using Conda:

```bash
conda create -n biroad-rlbench python=3.8 -y
conda activate biroad-rlbench
python -m pip install --upgrade pip setuptools wheel
```

Alternatively, using `venv`:

```bash
python3.8 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip setuptools wheel
```

Run the setup script with the Python interpreter from the activated environment:

```bash
PYTHON_BIN=python bash scripts/setup_environment.sh
source scripts/env.sh
```

By default, the script downloads CoppeliaSim 4.1 for Ubuntu 20.04 into `.deps/CoppeliaSim`, installs `third_party/PyRep` in editable mode, and then installs this bimanual RLBench package. The script finishes by verifying that both `dual_panda` and the dual-arm PyRep classes are available. No machine-specific path is stored in the repository.

If CoppeliaSim 4.1 is already installed next to this repository, point `COPPELIASIM_ROOT` to it using a path derived from the clone:

```bash
export COPPELIASIM_ROOT="$(pwd)/../CoppeliaSim"
PYTHON_BIN=python bash scripts/setup_environment.sh
source scripts/env.sh
```

`scripts/env.sh` derives the repository root from its own location. Source it in each new shell before launching RLBench.

## Dataset Generation

The included bimanual dataset generator uses the task definitions directly from `rlbench/bimanual_tasks/`:

```bash
python tools/dataset_generator_bimanual.py --help
```

Generated demonstrations are written to the output path selected at runtime; no local dataset path is hard-coded in this repository.

## Upstream Projects

This environment is based on the PerAct2 [bimanual RLBench fork](https://github.com/markusgrotz/RLBench) and bundles its matching [bimanual PyRep fork](https://github.com/markusgrotz/PyRep). Those projects extend the original [RLBench](https://github.com/stepjam/RLBench) and [PyRep](https://github.com/stepjam/PyRep) repositories. The original project licenses are retained in `LICENSE` and `third_party/PyRep/LICENSE`.

## Citation

Please cite the original RLBench project when using this environment or its task assets:

```bibtex
@article{james2019rlbench,
  title={RLBench: The Robot Learning Benchmark \& Learning Environment},
  author={James, Stephen and Ma, Zicong and Rovick Arrojo, David and Davison, Andrew J.},
  journal={IEEE Robotics and Automation Letters},
  year={2020}
}
```
