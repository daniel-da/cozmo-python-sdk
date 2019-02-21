
import time

import cozmo
from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes
from cozmo.util import degrees, distance_mm, speed_mmps

def set_walls(robot):
    w_wall = robot.world.define_custom_wall(CustomObjectTypes.CustomType01, CustomObjectMarkers.Circles2, 710, 50, 50, 50, True)

    e_wall = robot.world.define_custom_wall(CustomObjectTypes.CustomType02, CustomObjectMarkers.Triangles2, 710, 50, 50, 50, True)

    n_wall = robot.world.define_custom_wall(CustomObjectTypes.CustomType03, CustomObjectMarkers.Diamonds2, 1330, 50, 50, 50, True)

    s_wall = robot.world.define_custom_wall(CustomObjectTypes.CustomType04, CustomObjectMarkers.Hexagons2, 1330, 50, 50, 50, True)

    return w_wall, e_wall, n_wall, s_wall


class Cockpit():
    def __init__(self, robot):
        self.robot  = robot


    def forward(self, mm, mmps):
        self.robot.drive_straight(distance_mm(mm), speed_mmps(mmps))

    def backards(self, mm, mmps):
        self.robot.drive_straight(distance_mm(-mm), speed_mmps(mmps))

    def turn_left(self, degs):
        self.robot.turn_in_place(degrees(degs))


    def turn_right(self, degs):
        self.robot.turn_in_place(degrees(-degs))

    def show_visible_objects(self):
        for o in self.robot.world.visible_objects:
            print(o)