from collections import defaultdict
from typing import List, Tuple

import numpy as np
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.conditions import DetectedCondition
from rlbench.backend.conditions import NothingGrasped
from rlbench.backend.task import BimanualTask
from rlbench.backend.spawn_boundary import SpawnBoundary
from pyrep.objects.dummy import Dummy
from pyrep.objects.object import Object
from rlbench.backend.conditions import Condition


class LiftedCondition(Condition):

    def __init__(self, item: Shape, min_height: float):
        self.item = item
        self.min_height = min_height

    def condition_met(self):
        pos = self.item.get_position()
        return pos[2] >= self.min_height, False


class HandoverSuccessCondition(Condition):

    def __init__(self, item: Shape, left_sensor: ProximitySensor,
                 right_sensor: ProximitySensor, min_height: float = 0.8):
        self.item = item
        self.left_sensor = left_sensor
        self.right_sensor = right_sensor
        self.min_height = min_height
        self._left_ever_lifted = False
        self._right_ever_lifted = False

    def condition_met(self):
        left_has = self.left_sensor.is_detected(self.item)
        right_has = self.right_sensor.is_detected(self.item)
        lifted = self.item.get_position()[2] >= self.min_height

        if lifted:
            if left_has:
                self._left_ever_lifted = True
            if right_has:
                self._right_ever_lifted = True

        handover_done = (self._left_ever_lifted and self._right_ever_lifted
                         and lifted and (left_has or right_has))
        return handover_done, False

class BimanualHandoverItemEasySymmetricRandom(BimanualTask):

    def init_task(self) -> None:

        self.item = Shape('item')
        self.boundary = Shape('handover_item_boundary')

        self._boundary_init_pose = self.boundary.get_pose()
        self._item_init_orientation = self.item.get_orientation()

        self.register_graspable_objects([self.item])

        self.waypoint_mapping = defaultdict(lambda: 'right')
        self.waypoint_mapping.update({'waypoint0': 'left', 'waypoint5': 'left'})


    def init_episode(self, index: int) -> List[str]:

        self._variation_index = index

        right_success_sensor = ProximitySensor('Panda_rightArm_gripper_attachProxSensor')
        left_success_sensor = ProximitySensor('Panda_leftArm_gripper_attachProxSensor')

        self.boundary.set_pose(self._boundary_init_pose)

        b = SpawnBoundary([self.boundary])
        b.sample(self.item,
                min_distance=0.0,
                min_rotation=(0, 0, 0),
                max_rotation=(0, 0, 0))

        self.item.set_orientation(self._item_init_orientation)

        self.register_success_conditions(
            [HandoverSuccessCondition(self.item, left_success_sensor, right_success_sensor)])

        return [f'bring me the item with swapped hands',
                f'hand over the object using the other hand']

    def variation_count(self) -> int:
        return 1

    def boundary_root(self) -> Object:
        return Shape('handover_item_boundary')

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
