"""
Method 1: Detach waypoint3/4 from link_1 after waypoint0.
Set parent to None so they become world-level objects.
"""
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
from rlbench.backend.task import BimanualTask


class BimanualOpenBottle(BimanualTask):

    def init_task(self) -> None:
        self.bottle_link0 = Shape('link_0_respondable')
        self.bottle_link1 = Shape('link_1_respondable')

        self.register_graspable_objects([self.bottle_link0, self.bottle_link1])

        self._link0_detached = False
        self._link0_original_parent = None
        self._link0_released = False

        self._waypoints_detached = False
        self._wp3_original_parent = None
        self._wp4_original_parent = None

        self.waypoint_mapping = defaultdict(lambda: 'left')
        self.waypoint_mapping.update({
            'waypoint0': 'right',
            'waypoint2': 'right',
            'waypoint3': 'right',
            'waypoint4': 'right',
        })

        # After waypoint0 executed: detach waypoint3/4 from link_1
        self.register_waypoint_ability_end(0, self._detach_waypoints)

        # After waypoint5 executed: detach link_0 (cap) from link_1 (body)
        self.register_waypoint_ability_end(5, self._detach_bottle_cap)

    # ---- Waypoint detach ----

    def _detach_waypoints(self, waypoint) -> None:
        """After right arm grabs bottle (waypoint0), detach waypoint3/4
        from link_1 so they stay at current world position."""
        if self._waypoints_detached:
            return
        try:
            wp3 = Dummy('waypoint3')
            wp4 = Dummy('waypoint4')

            self._wp3_original_parent = wp3.get_parent()
            self._wp4_original_parent = wp4.get_parent()

            wp3.set_parent(None, keep_in_place=True)
            wp4.set_parent(None, keep_in_place=True)

            self._waypoints_detached = True
            logging.info("Waypoint3/4 detached from link_1 to world")
        except Exception as e:
            logging.warning(f"Failed to detach waypoints: {e}")

    def _restore_waypoints(self):
        """Restore waypoint3/4 to original parents."""
        if not self._waypoints_detached:
            return
        try:
            if self._wp3_original_parent is not None:
                Dummy('waypoint3').set_parent(
                    self._wp3_original_parent, keep_in_place=True)
            if self._wp4_original_parent is not None:
                Dummy('waypoint4').set_parent(
                    self._wp4_original_parent, keep_in_place=True)
        except Exception:
            pass
        self._waypoints_detached = False
        self._wp3_original_parent = None
        self._wp4_original_parent = None

    # ---- Cap detach ----

    def _detach_bottle_cap(self, waypoint) -> None:
        """Detach link_0 from link_1 so left hand can grasp it."""
        right_grasped = self.robot.right_gripper.get_grasped_objects()

        if self.bottle_link1 in right_grasped and not self._link0_detached:
            if self._link0_original_parent is None:
                self._link0_original_parent = self.bottle_link0.get_parent()

            self.bottle_link0.set_parent(None, keep_in_place=True)
            self.bottle_link0.set_dynamic(True)
            self.bottle_link0.set_respondable(True)

            try:
                sim.simResetDynamicObject(self.bottle_link0.get_handle())
            except Exception:
                pass

            self._link0_detached = True
            logging.info("Link_0 detached from link_1")

    def _restore_link0(self):
        """Restore link_0 to original parent."""
        if self._link0_detached and self._link0_original_parent is not None:
            try:
                self.bottle_link0.set_parent(
                    self._link0_original_parent, keep_in_place=True)
                self.bottle_link0.set_dynamic(False)
            except Exception:
                pass
        self._link0_detached = False
        self._link0_original_parent = None
        self._link0_released = False

    # ---- Step ----

    def step(self) -> None:
        if not self._link0_detached:
            return

        if not self.bottle_link0.is_model_dynamic():
            self.bottle_link0.set_dynamic(True)

        if not self._link0_released:
            left_grasped = self.robot.left_gripper.get_grasped_objects()
            if self.bottle_link0 in left_grasped:
                open_amount = self.robot.left_gripper.get_open_amount()[0]
                if open_amount > 0.8:
                    try:
                        self.robot.left_gripper.release()
                    except Exception:
                        self.bottle_link0.set_parent(None, keep_in_place=True)

                    self.bottle_link0.set_dynamic(False)
                    self.bottle_link0.set_dynamic(True)
                    try:
                        sim.simResetDynamicObject(self.bottle_link0.get_handle())
                    except Exception:
                        pass

                    self._link0_released = True
                    logging.info("Link_0 force released from left gripper")

    # ---- Lifecycle ----

    def cleanup(self) -> None:
        self._restore_link0()
        self._restore_waypoints()

    def init_episode(self, index: int) -> List[str]:
        self._restore_link0()
        self._restore_waypoints()

        self._variation_index = index

        success_sensor = ProximitySensor('success')
        self.register_success_conditions([
            DetectedCondition(self.bottle_link0, success_sensor)
        ])

        return ['open the bottle',
                'open the bottle cap',
                'remove the cap from the bottle',
                'take the cap off the bottle',
                'uncover the bottle by removing its cap']

    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, -np.pi / 8], [0, 0, np.pi / 8]