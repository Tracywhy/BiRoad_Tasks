from rlbench.bimanual_tasks.bimanual_pick_plate import BimanualPickPlate
from collections import defaultdict

class BimanualPickPlateSymmetric(BimanualPickPlate):
    def __init__(self, pyrep, robot, name='bimanual_pick_plate_symmetric'):
        super(BimanualPickPlate, self).__init__(pyrep, robot, 'bimanual_pick_plate_symmetric')
    
    def init_task(self):
        super().init_task()
        self.waypoint_mapping = defaultdict(lambda: 'left')
        self.waypoint_mapping.update({'waypoint0': 'right', 'waypoint2': 'right', 'waypoint6': 'right'})
    
    def init_episode(self, index):
        return ['pick up the plate with swapped hands']
