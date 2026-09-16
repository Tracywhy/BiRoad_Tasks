from typing import List
from collections import defaultdict
import logging

from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.objects.joint import Joint
from rlbench.backend.task import BimanualTask
from rlbench.backend.conditions import Condition


class ButtonPressedCondition(Condition):
    """检测按钮是否被按下：joint 位移超过阈值"""

    def __init__(self, joint: Joint, threshold: float):
        self._joint = joint
        self._threshold = threshold

    def condition_met(self):
        pos = self._joint.get_joint_position()
        return abs(pos) >= self._threshold, False


class LiftedCondition(Condition):

    def __init__(self, item: Shape, min_height: float):
        self.item = item
        self.min_height = min_height

    def condition_met(self):
        pos = self.item.get_position()
        return pos[2] >= self.min_height, False


class BimanualPushPenbutton(BimanualTask):

    def init_task(self) -> None:
        self._link_1 = Shape('link_1_respondable')
        self._link_0 = Shape('link_0_respondable')
        self._joint_0 = Joint('joint_0')

        self.register_graspable_objects([self._link_1])

        # 用 joint 位移作为成功条件（物理按下时 joint 会移动）
        # 阈值 0.002 = 2mm，根据实际调整
        self.register_success_conditions([
            LiftedCondition(self._link_1, 0.1),
            ButtonPressedCondition(self._joint_0, 0.002),
        ])

        self.waypoint_mapping = defaultdict(lambda: 'left')
        self.waypoint_mapping.update({
            'waypoint0': 'right',
            'waypoint1': 'right',
        })

        # 保存 waypoint3/4 相对 link_0 的偏移（用于算相对 link_1 的位置）
        self._wp3 = Dummy('waypoint3')
        self._wp4 = Dummy('waypoint4')

        self._wp3_rel_pos = self._wp3.get_position(relative_to=self._link_1)
        self._wp3_rel_ori = self._wp3.get_orientation(relative_to=self._link_1)
        self._wp4_rel_pos = self._wp4.get_position(relative_to=self._link_1)
        self._wp4_rel_ori = self._wp4.get_orientation(relative_to=self._link_1)

        self._wp3_original_parent = self._wp3.get_parent()
        self._wp4_original_parent = self._wp4.get_parent()

        self._waypoints_moved = False

        # 右臂抓完笔（waypoint0 end）后，把 waypoint3/4 从 link_0 搬到 link_1
        self.register_waypoint_ability_end(0, self._move_waypoints_to_link1)

    def _move_waypoints_to_link1(self, waypoint) -> None:
        """把 waypoint3/4 从 link_0 搬到 link_1 下。
        这样即使 link_0 乱动，waypoint 位置仍然跟着 link_1（笔身）稳定。
        左臂会去正确的位置。"""
        if self._waypoints_moved:
            return
        try:
            # 先设到正确的世界位置（相对 link_1 的偏移）
            self._wp3.set_position(self._wp3_rel_pos, relative_to=self._link_1)
            self._wp3.set_orientation(self._wp3_rel_ori, relative_to=self._link_1)
            self._wp4.set_position(self._wp4_rel_pos, relative_to=self._link_1)
            self._wp4.set_orientation(self._wp4_rel_ori, relative_to=self._link_1)

            # 改挂到 link_1 下
            self._wp3.set_parent(self._link_1, keep_in_place=True)
            self._wp4.set_parent(self._link_1, keep_in_place=True)

            self._waypoints_moved = True
            logging.info("waypoint3/4 moved from link_0 to link_1")
        except Exception as e:
            logging.warning(f"Failed to move waypoints: {e}")

    def _restore_waypoints(self) -> None:
        if not self._waypoints_moved:
            return
        try:
            self._wp3.set_parent(self._wp3_original_parent, keep_in_place=True)
            self._wp4.set_parent(self._wp4_original_parent, keep_in_place=True)
        except Exception:
            pass
        self._waypoints_moved = False

    def init_episode(self, index: int) -> List[str]:
        self._restore_waypoints()
        return ['push the pen button']

    def cleanup(self) -> None:
        self._restore_waypoints()

    def variation_count(self) -> int:
        return 1