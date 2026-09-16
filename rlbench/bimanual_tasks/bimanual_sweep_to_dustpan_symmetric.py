from typing import List
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition
from rlbench.backend.task import BimanualTask
from collections import defaultdict
from rlbench.backend.spawn_boundary import SpawnBoundary


DIRT_NUM = 5


class BimanualSweepToDustpanSymmetric(BimanualTask):

    def init_task(self) -> None:
        success_sensor = ProximitySensor('success')
        self.dirts = [Shape('dirt' + str(i)) for i in range(DIRT_NUM)]
        conditions = [DetectedCondition(dirt, success_sensor) for dirt in self.dirts]
        self.register_graspable_objects([Shape('broom'), Shape('Dustpan_4'),  Shape('Dustpan_5'), Shape('Dustpan_3')])
        self.register_success_conditions(conditions)
        self.waypoint_mapping = defaultdict(lambda: 'right')
        self.waypoint_mapping.update({'waypoint5': 'left', 'waypoint6': 'left', 'waypoint7': 'left'})

    def init_episode(self, index: int) -> List[str]:

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
