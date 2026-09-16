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
        self.waypoint_mapping.update({'waypoint0': 'right', 'waypoint2': 'right', 'waypoint3': 'right'})
    
    def step(self) -> None:
        """Called every simulation step. 
        
        Check if link_1 is grasped by right gripper, and if so,
        detach link_0 to make it independently graspable.
        """
        # Check if right gripper has grasped link_1
        right_grasped = self.robot.right_gripper.get_grasped_objects()
        left_grasped = self.robot.left_gripper.get_grasped_objects()

        # Step 1: Detach link_0 when link_1 is grasped by right hand
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
        
        # Step 2: Ensure link_0 stays dynamic even after being released
        # This is critical because gripper.release() sets parent back to old_parent (None)
        # but doesn't maintain dynamic state
        if self._link0_detached:
            # Always ensure link_0 is dynamic when detached
            if not self.pen_link0.is_model_dynamic():
                self.pen_link0.set_dynamic(True)
                logging.debug("Re-enabled dynamic for link_0")
    
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
        
        # Success condition: pen cap (link_0) detected by success sensor
        success_sensor = ProximitySensor('success')
        self.register_success_conditions([
            DetectedCondition(self.pen_link0, success_sensor)
        ])
        
        return ['open the pen',
                'open the pen cap',
                'remove the cap from the pen',
                'take the cap off the pen',
                'uncover the pen by removing its cap']

    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, -np.pi / 8], [0, 0, np.pi / 8]
