from collections import defaultdict
from typing import List, Tuple
import logging

import numpy as np
from pyrep.objects.dummy import Dummy
from pyrep.objects.joint import Joint
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from pyrep.backend import sim
from rlbench.backend.conditions import DetectedCondition, Condition
from rlbench.backend.conditions import JointCondition
from rlbench.backend.task import Task
from rlbench.backend.task import BimanualTask


class BimanualOpenThePen(BimanualTask):

    def init_task(self) -> None:
        self.pen_link0 = Shape('link_0_respondable')
        self.pen_link1 = Shape('link_1_respondable')

        self.register_graspable_objects([self.pen_link0, self.pen_link1])

        self._link0_detached = False
        self._link0_original_parent = None
        self._link0_released = False

        self.waypoint_mapping = defaultdict(lambda: 'left')
        self.waypoint_mapping.update({
            'waypoint0': 'right',
            'waypoint2': 'right',
            'waypoint3': 'right',
            'waypoint4': 'right',
            'waypoint5': 'right',
        })

        # Detach link_0 after waypoint3 is reached
        self.register_waypoint_ability_end(5, self._detach_pen_cap)

    def _detach_pen_cap(self, waypoint) -> None:
        """Detach link_0 from link_1 after waypoint3 so left hand can grasp it."""
        right_grasped = self.robot.right_gripper.get_grasped_objects()

        if self.pen_link1 in right_grasped and not self._link0_detached:
            if self._link0_original_parent is None:
                self._link0_original_parent = self.pen_link0.get_parent()

            self.pen_link0.set_parent(None, keep_in_place=True)
            self.pen_link0.set_dynamic(True)
            self.pen_link0.set_respondable(True)

            # Reset velocity to prevent sudden movement
            try:
                sim.simResetDynamicObject(self.pen_link0.get_handle())
            except Exception:
                pass

            self._link0_detached = True
            logging.info("Link_0 detached from link_1 at waypoint3")

    def step(self) -> None:
        """Called every simulation step."""
        if not self._link0_detached:
            return

        # Ensure link_0 stays dynamic
        if not self.pen_link0.is_model_dynamic():
            self.pen_link0.set_dynamic(True)

        # Check if left gripper has opened but still holds link_0
        if not self._link0_released:
            left_grasped = self.robot.left_gripper.get_grasped_objects()
            if self.pen_link0 in left_grasped:
                open_amount = self.robot.left_gripper.get_open_amount()[0]
                if open_amount > 0.8:
                    # Force release
                    try:
                        self.robot.left_gripper.release()
                    except Exception:
                        self.pen_link0.set_parent(None, keep_in_place=True)

                    # Reset dynamics to prevent flying away
                    self.pen_link0.set_dynamic(False)
                    self.pen_link0.set_dynamic(True)
                    try:
                        sim.simResetDynamicObject(self.pen_link0.get_handle())
                    except Exception:
                        pass

                    self._link0_released = True
                    logging.info("Link_0 force released from left gripper")

    def cleanup(self) -> None:
        """Restore scene structure for reset."""
        if self._link0_detached and self._link0_original_parent is not None:
            try:
                self.pen_link0.set_parent(
                    self._link0_original_parent, keep_in_place=True)
                self.pen_link0.set_dynamic(False)
            except Exception:
                pass
        self._link0_detached = False
        self._link0_original_parent = None
        self._link0_released = False

    def init_episode(self, index: int) -> List[str]:
        # Restore link_0 before each episode
        if self._link0_detached and self._link0_original_parent is not None:
            try:
                self.pen_link0.set_parent(
                    self._link0_original_parent, keep_in_place=True)
                self.pen_link0.set_dynamic(False)
            except Exception:
                pass
        self._link0_detached = False
        self._link0_original_parent = None
        self._link0_released = False

        self._variation_index = index

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