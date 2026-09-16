from typing import List
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition
from rlbench.backend.task import BimanualTask
from collections import defaultdict
from rlbench.backend.spawn_boundary import SpawnBoundary


DIRT_NUM = 5


class BimanualSweepToDustpanSymmetricRandom(BimanualTask):

    def init_task(self) -> None:
        success_sensor = ProximitySensor('success')
        self.dirts = [Shape('dirt' + str(i)) for i in range(DIRT_NUM)]
        conditions = [DetectedCondition(dirt, success_sensor) for dirt in self.dirts]
        self.register_graspable_objects([Shape('broom'), Shape('Dustpan_4'),  Shape('Dustpan_5'), Shape('Dustpan_3')])
        self.register_success_conditions(conditions)
        self.waypoint_mapping = defaultdict(lambda: 'right')
        self.waypoint_mapping.update({'waypoint5': 'left', 'waypoint6': 'left', 'waypoint7': 'left'})

        self.broom_holder = Shape('broom_holder')
        self.dustpan_holder = Shape('Dustpan_4')

        try:
            self.broom_boundary = Shape('broom_boundary')
            self._broom_boundary_init_pose = self.broom_boundary.get_pose()
            self._has_broom_boundary = True
        except:
            self._has_broom_boundary = False

        try:
            self.dustpan_boundary = Shape('dustpan_boundary')
            self._dustpan_boundary_init_pose = self.dustpan_boundary.get_pose()
            self._has_dustpan_boundary = True
        except:
            self._has_dustpan_boundary = False

        self._broom_holder_init_orientation = self.broom_holder.get_orientation()
        self._dustpan_holder_init_orientation = self.dustpan_holder.get_orientation()

    def init_episode(self, index: int) -> List[str]:
        if self._has_broom_boundary:
            self.broom_boundary.set_pose(self._broom_boundary_init_pose)

            b = SpawnBoundary([self.broom_boundary])
            b.sample(self.broom_holder,
                    min_distance=0.0,
                    min_rotation=(0, 0, 0),
                    max_rotation=(0, 0, 0))

            self.broom_holder.set_orientation(self._broom_holder_init_orientation)

        # if self._has_dustpan_boundary:
        #     self.dustpan_boundary.set_pose(self._dustpan_boundary_init_pose)
        #     b = SpawnBoundary([self.dustpan_boundary])
        #     b.sample(self.dustpan_holder,
        #             min_distance=0.0,
        #             min_rotation=(0, 0, 0),
        #             max_rotation=(0, 0, 0))
        #     self.dustpan_holder.set_orientation(self._dustpan_holder_init_orientation)

        b = SpawnBoundary([Shape('dirt_boundary')])
        b.clear()
        for item in self.dirts:
            b.sample(item, min_distance=0.01, ignore_collisions=True)

        return ['sweep dirt to dustpan with swapped hands',
                'sweep the dirt up using the other hand',
                'use the broom to brush the dirt into the dustpan with swapped hands',
                'clean up the dirt using the other hand',
                'pick up the brush and clean up the table with swapped hands',
                'grasping the broom by its handle, clear way the dirt from the '
                'table using the other hand',
                'leave the table clean with swapped hands']

    def variation_count(self) -> int:
        return 1

    def is_static_workspace(self):
        return True

    def base_rotation_bounds(self):
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
