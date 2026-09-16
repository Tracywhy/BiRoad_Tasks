from typing import List, Tuple
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from rlbench.backend.task import BimanualTask
from rlbench.backend.spawn_boundary import SpawnBoundary
from collections import defaultdict
from rlbench.backend.conditions import Condition

class LiftedCondition(Condition):
    def __init__(self, item: Shape, min_height: float):
        self.item = item
        self.min_height = min_height

    def condition_met(self):
        pos = self.item.get_position()
        return pos[2] >= self.min_height, False

class BimanualPickPlateRandom(BimanualTask):

    def init_task(self) -> None:
        self.plate = Shape('plate')
        self.boundary = Shape('spawn_boundary')

        self._boundary_init_pose = self.boundary.get_pose()
        self._plate_init_orientation = self.plate.get_orientation()

        self.register_success_conditions([LiftedCondition(self.plate, 1.0)])
        self.register_graspable_objects([self.plate])

        self.waypoint_mapping = defaultdict(lambda: 'right')
        self.waypoint_mapping.update({
            'waypoint0': 'left',
            'waypoint2': 'left',
            'waypoint6': 'left'
        })

    def init_episode(self, index: int) -> List[str]:
        self.boundary.set_pose(self._boundary_init_pose)

        b = SpawnBoundary([self.boundary])
        b.sample(self.plate,
                min_distance=0.0,
                min_rotation=(0, 0, 0),
                max_rotation=(0, 0, 0))

        self.plate.set_orientation(self._plate_init_orientation)

        return ['pick up the plate']

    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
