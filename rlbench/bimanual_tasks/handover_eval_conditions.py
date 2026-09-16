from pyrep.objects.shape import Shape

from rlbench.backend.conditions import Condition


class LiftedCondition(Condition):

    def __init__(self, item: Shape, min_height: float):
        self.item = item
        self.min_height = min_height

    def condition_met(self):
        pos = self.item.get_position()
        return pos[2] >= self.min_height, False


class OrderInvariantHandoverCondition(Condition):
    """Success when either hand lifts the item first and the other receives it."""

    def __init__(self, task, item: Shape, min_height: float):
        self._task = task
        self._item = item
        self._min_height = min_height
        self.reset()

    def _is_lifted(self) -> bool:
        return self._item.get_position()[2] >= self._min_height

    def _is_grasped_by(self, gripper) -> bool:
        item_handle = self._item.get_handle()
        return any(
            obj.get_handle() == item_handle
            for obj in gripper.get_grasped_objects()
        )

    def _current_holder(self):
        right_holds = self._is_grasped_by(self._task.robot.right_gripper)
        left_holds = self._is_grasped_by(self._task.robot.left_gripper)
        if right_holds and not left_holds:
            return 'right'
        if left_holds and not right_holds:
            return 'left'
        if left_holds and right_holds:
            return 'both'
        return None

    def condition_met(self):
        holder = self._current_holder()
        lifted = self._is_lifted()

        self._task._handover_current_holder = holder
        self._task._handover_is_lifted = lifted

        if self._first_picker is None and holder in ('left', 'right') and lifted:
            self._first_picker = holder
            self._task._handover_first_picker = holder

        if self._first_picker is not None:
            receiver = 'left' if self._first_picker == 'right' else 'right'
            if holder == receiver and lifted:
                self._completed = True
                self._task._handover_final_holder = holder

        self._task._handover_transfer_complete = self._completed
        return self._completed, False

    def reset(self):
        self._first_picker = None
        self._completed = False
        self._task._handover_first_picker = None
        self._task._handover_final_holder = None
        self._task._handover_current_holder = None
        self._task._handover_is_lifted = False
        self._task._handover_transfer_complete = False
