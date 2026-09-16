from typing import List
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.conditions import Condition
from rlbench.backend.task import BimanualTask
from rlbench.backend.spawn_boundary import SpawnBoundary
from collections import defaultdict


class PickupSuccessCondition(Condition):
    def __init__(self, item: Shape, left_sensor: ProximitySensor,
                 right_sensor: ProximitySensor, lift_amount: float = 0.1):
        self.item = item
        self.left_sensor = left_sensor
        self.right_sensor = right_sensor
        self.lift_amount = lift_amount
        self._init_z = None

    def condition_met(self):
        if self._init_z is None:
            self._init_z = self.item.get_position()[2]

        left_has = self.left_sensor.is_detected(self.item)
        right_has = self.right_sensor.is_detected(self.item)
        lifted = self.item.get_position()[2] >= self._init_z + self.lift_amount

        return lifted and (left_has or right_has), False


class BimanualPickLaptopRandom(BimanualTask):

    def init_task(self) -> None:
        self.lid = Shape('lid')
        self.laptop_holder = Shape('laptop_holder')

        try:
            self.boundary = Shape('laptop_boundary')
            self._boundary_init_pose = self.boundary.get_pose()
            self._has_boundary = True
        except Exception:
            self._has_boundary = False

        self._laptop_holder_init_orientation = self.laptop_holder.get_orientation()

        self.waypoint_mapping = defaultdict(lambda: 'left')
        for i in range(1, 7, 2):
            self.waypoint_mapping.update({f'waypoint{i}': 'right'})

    def init_episode(self, index: int) -> List[str]:
        if self._has_boundary:
            self.boundary.set_pose(self._boundary_init_pose)
            b = SpawnBoundary([self.boundary])
            b.sample(self.laptop_holder,
                     min_distance=0.0,
                     min_rotation=(0, 0, 0),
                     max_rotation=(0, 0, 0))
            self.laptop_holder.set_orientation(self._laptop_holder_init_orientation)

        left_sensor = ProximitySensor('Panda_leftArm_gripper_attachProxSensor')
        right_sensor = ProximitySensor('Panda_rightArm_gripper_attachProxSensor')
        self.register_success_conditions(
            [PickupSuccessCondition(self.lid, left_sensor, right_sensor, lift_amount=0.1)])

        return ['pick up the laptop']

    def variation_count(self) -> int:
        return 1

    def is_static_workspace(self):
        return True

    def base_rotation_bounds(self):
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
