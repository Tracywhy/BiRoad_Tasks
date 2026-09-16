from typing import Union, Dict, Tuple

import gym
import numpy as np
from gym import spaces
from pyrep.const import RenderMode
from pyrep.objects.dummy import Dummy
from pyrep.objects.vision_sensor import VisionSensor

from rlbench.action_modes.action_mode import MoveArmThenGripper
from rlbench.action_modes.arm_action_modes import JointVelocity
from rlbench.action_modes.gripper_action_modes import Discrete
from rlbench.environment import Environment
from rlbench.observation_config import ObservationConfig


class RLBenchEnv(gym.Env):
    """An gym wrapper for RLBench."""

    metadata = {'render.modes': ['human', 'rgb_array']}

    def __init__(self, task_class, observation_mode='state',
                 render_mode: Union[None, str] = None):
        self._observation_mode = observation_mode
        self._render_mode = render_mode
        obs_config = ObservationConfig()
        if observation_mode == 'state':
            obs_config.set_all_high_dim(False)
            obs_config.set_all_low_dim(True)
        elif observation_mode == 'vision':
            obs_config.set_all(True)
        else:
            raise ValueError(
                'Unrecognised observation_mode: %s.' % observation_mode)

        action_mode = MoveArmThenGripper(JointVelocity(), Discrete())
        self.env = Environment(
            action_mode, obs_config=obs_config, headless=True)
        self.env.launch()
        self.task = self.env.get_task(task_class)

        _, obs = self.task.reset()

        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=self.env.action_shape)

        if observation_mode == 'state':
            self.observation_space = spaces.Box(
                low=-np.inf, high=np.inf, shape=obs.get_low_dim_data().shape)
        elif observation_mode == 'vision':
            observation_space = {
                "state": spaces.Box(
                    low=-np.inf, high=np.inf,
                    shape=obs.get_low_dim_data().shape),

                }
            for camera_name, config in obs_config.camera_configs.items():
                if config.rgb:
                    observation_space[f'{camera_name}_rgb'] = spaces.Box(
                        low=0, high=255,
                        shape=tuple(config.image_size) + (3,),
                        dtype=np.uint8)
                if config.depth:
                    observation_space[f'{camera_name}_depth'] = spaces.Box(
                        low=0, high=np.inf,
                        shape=tuple(config.image_size),
                        dtype=np.float32)
                if config.point_cloud:
                    observation_space[f'{camera_name}_point_cloud'] = spaces.Box(
                        low=-np.inf, high=np.inf,
                        shape=tuple(config.image_size) + (3,),
                        dtype=np.float32)
                if config.mask:
                    mask_shape = tuple(config.image_size)
                    if not config.masks_as_one_channel:
                        mask_shape += (3,)
                    observation_space[f'{camera_name}_mask'] = spaces.Box(
                        low=0, high=np.inf,
                        shape=mask_shape)

            self.observation_space = spaces.Dict(observation_space)

        if render_mode is not None:
            # Add the camera to the scene
            cam_placeholder = Dummy('cam_cinematic_placeholder')
            self._gym_cam = VisionSensor.create([640, 360])
            self._gym_cam.set_pose(cam_placeholder.get_pose())
            if render_mode == 'human':
                self._gym_cam.set_render_mode(RenderMode.OPENGL3_WINDOWED)
            else:
                self._gym_cam.set_render_mode(RenderMode.OPENGL3)

    def _extract_obs(self, obs) -> Dict[str, np.ndarray]:
        if self._observation_mode == 'state':
            return obs.get_low_dim_data()
        elif self._observation_mode == 'vision':
            extracted = {"state": obs.get_low_dim_data()}
            extracted.update({
                key: value for key, value in obs.perception_data.items()
                if value is not None
            })
            return extracted


    def render(self, mode='human') -> Union[None, np.ndarray]:
        if mode != self._render_mode:
            raise ValueError(
                'The render mode must match the render mode selected in the '
                'constructor. \nI.e. if you want "human" render mode, then '
                'create the env by calling: '
                'gym.make("reach_target-state-v0", render_mode="human").\n'
                'You passed in mode %s, but expected %s.' % (
                    mode, self._render_mode))
        if mode == 'rgb_array':
            frame = self._gym_cam.capture_rgb()
            frame = np.clip((frame * 255.).astype(np.uint8), 0, 255)
            return frame

    def reset(self) -> Dict[str, np.ndarray]:
        descriptions, obs = self.task.reset()
        del descriptions  # Not used.
        return self._extract_obs(obs)

    def step(self, action) -> Tuple[Dict[str, np.ndarray], float, bool, dict]:
        obs, reward, terminate = self.task.step(action)
        return self._extract_obs(obs), reward, terminate, {}

    def close(self) -> None:
        self.env.shutdown()
