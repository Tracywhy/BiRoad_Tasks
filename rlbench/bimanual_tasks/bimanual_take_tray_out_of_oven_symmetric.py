from collections import defaultdict
from typing import List, Tuple
from pyrep.objects.shape import Shape
from pyrep.objects.object import Object
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.robot import BimanualRobot
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped
from rlbench.backend.task import BimanualTask

class BimanualTakeTrayOutOfOvenSymmetric(BimanualTask):

    def init_task(self) -> None:
        success_detector = ProximitySensor('success')
        tray = Shape('tray')
        assert(isinstance(self.robot, BimanualRobot))
        self.register_graspable_objects([tray])
        self.register_success_conditions(
            [DetectedCondition(tray, success_detector, negated=True),
             NothingGrasped(self.robot.right_gripper), NothingGrasped(self.robot.left_gripper)])

        self.waypoint_mapping = defaultdict(lambda: 'right')
        for i in range(4):
            self.waypoint_mapping[f'waypoint{i}'] = 'left'

    def init_episode(self, index: int) -> List[str]:
        return ['take tray out of oven with swapped hands',
                'open the oven and take the baking tray out using the other hand',
                'grasp the handle on the over door, open it, and remove the '
                'tray from the oven with swapped hands',
                'get the baking tray from the oven using the other hand',
                'get the tray with swapped hands',
                'take out the tray using the other hand',
                'retrieve the oven tray with swapped hands']

    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, -3.14 / 4.], [0, 0, 3.14 / 4.]

    def boundary_root(self) -> Object:
        return Shape('oven_boundary_root')
