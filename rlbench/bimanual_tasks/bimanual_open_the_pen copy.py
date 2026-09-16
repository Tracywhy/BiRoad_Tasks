from collections import defaultdict
from typing import List, Tuple

import numpy as np
from pyrep.objects.dummy import Dummy
from pyrep.objects.joint import Joint
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.conditions import DetectedCondition, Condition
from rlbench.backend.conditions import JointCondition
from rlbench.backend.task import Task
from rlbench.backend.task import BimanualTask

class LiftedCondition(Condition):

    def __init__(self, item: Shape, min_height: float):
        self.item = item
        self.min_height = min_height

    def condition_met(self):
        pos = self.item.get_position()
        return pos[2] >= self.min_height, False


class BimanualOpenThePen(BimanualTask):

    def init_task(self) -> None:
        # Get pen objects
        self.pen_link0 = Shape('link_0_respondable')
        self.pen_link1 = Shape('link_1_respondable')
        
        # Register both link_0 and link_1 as graspable
        self.register_graspable_objects([self.pen_link0, self.pen_link1])

        # Waypoint mapping - adjust based on which hand picks up the pen
        self.waypoint_mapping = defaultdict(lambda: 'left')
        self.waypoint_mapping.update({'waypoint0': 'right', 'waypoint2': 'right'})

    def init_episode(self, index: int) -> List[str]:
        self._variation_index = index
        
        # Success when pen link1 is lifted 0.3m above its initial position
        initial_height = self.pen_link1.get_position()[2]
        target_height = initial_height + 0.3
        
        self.register_success_conditions([
            LiftedCondition(self.pen_link1, target_height)
        ])
        
        return ['pick up the pen',
                'grasp the pen',
                'lift the pen',
                'grab the pen and lift it up']

    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, -np.pi / 8], [0, 0, np.pi / 8]
