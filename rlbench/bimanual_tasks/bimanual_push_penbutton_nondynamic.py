from typing import List
from collections import defaultdict
import logging

from pyrep.objects.shape import Shape
from pyrep.objects.joint import Joint
from rlbench.backend.task import BimanualTask
from rlbench.backend.conditions import Condition


class ButtonPressedCondition(Condition):

    def __init__(self):
        self._pushed = False

    def set_pushed(self):
        self._pushed = True

    def condition_met(self):
        return self._pushed, False


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

        self._button_condition = ButtonPressedCondition()

        self.register_success_conditions([
            LiftedCondition(self._link_1, 0.1),
            self._button_condition,
        ])

        self.waypoint_mapping = defaultdict(lambda: 'left')
        self.waypoint_mapping.update({
            'waypoint0': 'right',
            'waypoint1': 'right',
        })

        # waypoint4 开始时按下按钮（左臂到达按钮位置）
        self.register_waypoint_ability_start(4, self._on_button_push)
        # 双保险
        self.register_waypoint_ability_end(4, self._on_button_push)

    def _on_button_push(self, waypoint) -> None:
        if self._button_condition._pushed:
            return
        try:
            # 直接设置 joint 位置，按下 5mm
            # joint 在父子树里，link_0 会沿 joint 轴移动
            # 正负方向根据你的 joint 设置调整
            self._joint_0.set_joint_position(0.005)
            self._button_condition.set_pushed()
            logging.info("Button pushed via set_joint_position!")
        except Exception as e:
            logging.warning(f"Failed to push button: {e}")

    def init_episode(self, index: int) -> List[str]:
        try:
            self._joint_0.set_joint_position(0)
        except Exception:
            pass
        self._button_condition._pushed = False
        return ['push the pen button']

    def cleanup(self) -> None:
        try:
            self._joint_0.set_joint_position(0)
        except Exception:
            pass

    def variation_count(self) -> int:
        return 1