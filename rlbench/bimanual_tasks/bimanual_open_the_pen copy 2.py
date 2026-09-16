from collections import defaultdict
from typing import List, Tuple
import logging

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
        
        # Track if we've detached link_0 from link_1
        self._link0_detached = False
        self._link0_original_parent = None

        # Waypoint mapping - adjust based on which hand picks up the pen
        self.waypoint_mapping = defaultdict(lambda: 'left')
        self.waypoint_mapping.update({'waypoint0': 'right', 'waypoint2': 'right'})
    
    def step(self) -> None:
        """Called every simulation step. 
        
        Check if link_1 is grasped by right gripper, and if so,
        detach link_0 to make it independently graspable.
        """
        # Check if right gripper has grasped link_1
        right_grasped = self.robot.right_gripper.get_grasped_objects()
        
        if self.pen_link1 in right_grasped and not self._link0_detached:
            # Right arm has grasped link_1, now detach link_0
            
            # Save original parent before detaching
            if self._link0_original_parent is None:
                self._link0_original_parent = self.pen_link0.get_parent()
            
            # Detach link_0 from hierarchy (set parent to None/world)
            self.pen_link0.set_parent(None, keep_in_place=True)
            
            # Ensure link_0 is dynamic and respondable
            self.pen_link0.set_dynamic(True)
            self.pen_link0.set_respondable(True)
            
            self._link0_detached = True
            logging.info("Link_0 detached from link_1 and set to dynamic/respondable")
    
    def cleanup(self) -> None:
        if self._link0_detached and self._link0_original_parent is not None:
            self.pen_link0.set_parent(self._link0_original_parent, keep_in_place=True)
            self.pen_link0.set_dynamic(False)
        self._link0_detached = False
        self._link0_original_parent = None

    def init_episode(self, index: int) -> List[str]:
        # 每次episode开始前，确保link_0恢复原状
        if self._link0_detached and self._link0_original_parent is not None:
            self.pen_link0.set_parent(self._link0_original_parent, keep_in_place=True)
            self.pen_link0.set_dynamic(False)
        self._link0_detached = False
        self._link0_original_parent = None
        
        self._variation_index = index
        # ... 后续不变
            
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
