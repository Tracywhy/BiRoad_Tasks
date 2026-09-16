from typing import List, Tuple
import numpy as np
from pyrep.objects.object import Object
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.conditions import DetectedCondition
from rlbench.backend.conditions import NothingGrasped
from rlbench.backend.task import BimanualTask
from collections import defaultdict
from rlbench.backend.spawn_boundary import SpawnBoundary

class BimanualPutBottleInFridgeSymmetric(BimanualTask):

    def init_task(self) -> None:
        bottle = Shape('bottle')
        self.register_graspable_objects([bottle])
        self.register_success_conditions(
            [DetectedCondition(bottle, ProximitySensor('success')),
             NothingGrasped(self.robot.right_gripper), NothingGrasped(self.robot.left_gripper)])

        self.waypoint_mapping = defaultdict(lambda: 'right')
        for i in range(4):
            self.waypoint_mapping[f'waypoint{i}'] = 'left'

        self.spawn_boundaries = [Shape('fridge_root')]

    def init_episode(self, index: int) -> List[str]:

        self._variation_index = index

        s = Shape('fridge_root')
        s.set_position([ 0.375, 0.2,  0.752])
        print(s.get_position())

        b = SpawnBoundary(self.spawn_boundaries)
        b.sample(Shape('bottle'), min_distance=0.1)


        return ['put bottle in fridge with swapped hands',
                'place the bottle inside the fridge using the other hand',
                'open the fridge and put the bottle in there with swapped hands',
                'open the fridge door, pick up the bottle, and leave it in the '
                'fridge using the other hand']

    def variation_count(self) -> int:
        return 1

    def boundary_root(self) -> Object:
        return Shape('fridge_root')

    def is_static_workspace(self):
        return True

    def base_rotation_bounds(self) -> Tuple[Tuple[float, float, float],
                                            Tuple[float, float, float]]:
        return (0.0, 0.0, -np.pi / 4), (0.0, 0.0, np.pi / 4)
