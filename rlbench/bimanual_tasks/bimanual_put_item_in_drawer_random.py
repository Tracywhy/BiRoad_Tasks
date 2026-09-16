from collections import defaultdict
from typing import List, Tuple

import numpy as np
from pyrep.objects.dummy import Dummy
from pyrep.objects.joint import Joint
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.conditions import DetectedCondition
from rlbench.backend.task import Task
from rlbench.backend.task import BimanualTask
from rlbench.backend.spawn_boundary import SpawnBoundary

class BimanualPutItemInDrawerRandom(BimanualTask):

    def init_task(self) -> None:
        self._options = ['bottom', 'middle', 'top']
        self._anchors = [Dummy('waypoint_anchor_%s' % opt)
                         for opt in self._options]
        self._joints = [Joint('drawer_joint_%s' % opt)
                        for opt in self._options]
        self._waypoint1 = Dummy('waypoint1')
        self._item = Shape('item')

        try:
            self.boundary = Shape('item_boundary')
            self._boundary_init_pose = self.boundary.get_pose()
            self._has_boundary = True
        except:
            self._has_boundary = False

        self._item_init_orientation = self._item.get_orientation()

        self.register_graspable_objects([self._item])

        self.waypoint_mapping = defaultdict(lambda: 'right')
        self.waypoint_mapping.update({'waypoint0': 'left', 'waypoint1': 'left', 'waypoint2': 'left'})


    def init_episode(self, index) -> List[str]:
        self._variation_index = index
        option = self._options[index]
        anchor = self._anchors[index]
        self._waypoint1.set_position(anchor.get_position())

        drawer_frame = Shape('drawer_frame')
        current_orientation = drawer_frame.get_orientation()
        random_offset = np.random.uniform(-np.pi / 12, np.pi / 12)
        new_gamma = current_orientation[2] + random_offset
        drawer_frame.set_orientation([current_orientation[0], current_orientation[1], new_gamma])

        if self._has_boundary:
            self.boundary.set_pose(self._boundary_init_pose)

            b = SpawnBoundary([self.boundary])
            b.sample(self._item,
                    min_distance=0.0,
                    min_rotation=(0, 0, 0),
                    max_rotation=(0, 0, 0))

            self._item.set_orientation(self._item_init_orientation)

        success_sensor = ProximitySensor('success_' + option)
        self.register_success_conditions(
            [DetectedCondition(self._item, success_sensor)])
        return ['put the item in the %s drawer' % option,
                'put the block away in the %s drawer' % option,
                'open the %s drawer and place the block inside of it' % option,
                'leave the block in the %s drawer' % option]

    def variation_count(self) -> int:
        return 3

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
