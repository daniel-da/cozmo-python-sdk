# Copyright (c) 2019 Adolfo Duarte
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import asyncio
import math
import time

from cozmo import robot

from cozmo import behavior
from . import util
from .util import Pose, degrees, distance_mm, speed_mmps, Quaternion

from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes


class RobotUpgrades(object):

    walls = {}
    default_distance = 300
    default_speed = 100
    objects_found = []
    charging_pose = None
    charging_charger_pose = None
    docking_pose = None
    docking_charger_pose = None
    possible_docking_pose = None

    async def undock(self):
        await self.backup_onto_charger()
        if not self.is_charging:
            return
        self.record_charging_position()
        await self.drive_off_charger_contacts().wait_for_completed()
        await self.drive_forward(90)
        await self.turn_around()
        if self.world.charger.is_visible:
            await self.turn_around()
            self.record_docking_positions()

    def record_charging_position(self):
        self.charging_charger_pose = self.world.charger.pose
        self.charging_pose = self.pose

    def record_docking_positions(self):
        self.docking_pose = self.pose
        self.docking_charger_pose = self.world.charger.pose

    async def look_around_for_charger(self, search_angle=30):
        if search_angle is 0:
            return
        total_turn = 0
        found = False
        while total_turn < 360 + search_angle:
            if self.world.charger.is_visible:
                found = True
                break
            await self.turn_right(search_angle)
            total_turn += search_angle
        await self.turn_left(total_turn)
        return found

    def calculate_possible_docking_pose(self):
        charger_pose_delta = self.docking_charger_pose - self.world.charger.pose
        self.possible_docking_pose = self.docking_pose - charger_pose_delta

    def obtain_coordinates_from_walls(self):
        """Should reset the robot pose coordinates based on he location of known walls.
        returns new pose and certainity as a perentage"""

        # return util.Pose(x=0.0, y=0.0, z=0.0)
        pass

    def scout_world(self):
        """Robot searches world for  markers, like walls,
        charger, objects, etc... """
        pass

    def look(self, seconds):
        """Robot "looks" for seconds

        :param seconds: the number of seconds robot "looks" """

        # still not very well defined what "look" means
        # but it will probably have to do with some sort of recognigion
        # time.sleep(seconds)
        pass

    def look_around(self, total_angle, step_angle, look_time):
        """Robot looks total angle, stoping to look every step_angle

        for the look_time seconds

        :param total_angle: total angle to tun
        :param step_angle: degrees to take every step
        :param look_time: seconds to look at each step """

        look_angle = util.Angle(0)

        while look_angle < total_angle:
            self.turn_in_place(step_angle).wait_for_completed()
            self.look(look_time)
            look_angle += step_angle

    def current_position(self):
        _p = self.pose.position
        _q = self.pose.rotation
        return Pose(_p.x, _p.y, _p.z,
                    q0=_q.q1, q1=_q.q1, q2=_q.q2, q3=_q.q3,
                    origin_id=self.pose.origin_id
                    )

    async def drive_forward(self, distance=default_distance, speed=default_speed):
        await self.drive_straight(distance_mm(distance), speed_mmps(speed)).wait_for_completed()

    async def drive_backards(self, distance=default_distance, speed=default_speed):
        await self.drive_straight(distance_mm(distance * -1), speed_mmps(speed)).wait_for_completed()

    async def turn_left(self, d=90):
        await self.turn_in_place(degrees(d)).wait_for_completed()

    async def turn_right(self, d=90):
        await self.turn_in_place(degrees(-1 * d)).wait_for_completed()

    async def turn_around(self):
        await self.turn_in_place(degrees(180)).wait_for_completed()

    @property
    def knows_charger_location(self):
        return (True if
                self.world.charger
                and self.world.charger.pose.is_comparable(self.pose)
                else False)

    def distance_to_object(self, obj):
        ix = self.pose.position.x
        iy = self.pose.position.y
        iz = self.pose.position.z
        x = obj.pose.position.x
        y = obj.pose.position.y
        z = obj.pose.position.z
        return math.sqrt((x-ix)**2 + (y-iy)**2 + (z-iz)**2)

    def get_aligned_x_y(self, obj, hy):
        theta = obj.pose.rotation.angle_z.radians
        x_obj = obj.pose.position.x
        y_obj = obj.pose.position.y
        x_new = math.cos(theta + math.pi) * hy
        y_new = math.sin(theta + math.pi) * hy
        return x_new + x_obj, y_new + y_obj

    async def face_x_direction(self, anglepersecond=None):
        if anglepersecond is None:
            anglepersecond = degrees(45)
        print(anglepersecond)
        await self.turn_to_angle(degrees(00), anglepersecond=anglepersecond)

    async def face_y_direction(self, anglepersecond=None):
        if anglepersecond is None:
            anglepersecond = degrees(45)
        print(anglepersecond)
        await self.turn_to_angle(degrees(90), anglepersecond=anglepersecond)

    async def turn_to_angle(self, angle, anglepersecond=None):
        if anglepersecond is None:
            anglepersecond = degrees(45)
        print(anglepersecond)
        await self.turn_in_place(angle - self.pose.rotation.angle_z,
                                 speed=anglepersecond).wait_for_completed()

    async def move_to_new_x_y(self, x, y):
        x_c = self.pose.position.x
        dx = x - x_c
        await self.face_x_direction()
        await self.drive_forward(dx)
        y_c = self.pose.position.y
        dy = y - y_c
        await self.face_y_direction()
        await self.drive_forward(dy)

    async def align_with_object(self, obj, distance):
        _x, _y = self.get_aligned_x_y(obj, distance)
        await self.move_to_new_x_y(_x, _y)
        await self.turn_to_angle(obj.pose.rotation.angle_z)

    async def return_to_charger(self, final_distance=None, look_timeout=30):
        if not self.knows_charger_location:
            await self.look_around_for_charger(45)
            if not self.knows_charger_location:
                return
                #should probably say "cant find charger"
        await self.go_to_object(self.world.charger, distance_mm(100)).wait_for_completed()
        await self.look_around_for_charger(30)
        self.calculate_possible_docking_pose()
        self.go_to_pose(self.possible_docking_pose).wait_for_completed()
        await self.backup_onto_charger(10)


    def find_charger(self):
        """Find the charger by looking around. """
        pass


    @staticmethod
    def translate_pose_perpendicular(
            original_pose, d=distance_mm(100)):
        """returns a pose translated by the given

        distance perpendicular to the given pose """
        return original_pose - d

    def dock_with_charger(self):
        self.find_charger()
        if not self.knows_charger_location:
            '''Can't find charger '''
            return

        self.return_to_charger(distance_mm(100))

        docking_pose = self.translate_pose_perpendicular(
            self.world.charger.pose, distance_mm(100))
        self.go_to_pose(docking_pose)
        self.turn_in_place(degrees(180))
        self.backup_onto_charger()

    async def define_rectangle(self):

        self.walls['north'] = await self.world.define_custom_wall(
            CustomObjectTypes.CustomType02,
            CustomObjectMarkers.Triangles2,
            710, 50,
            25, 25,
            True)

        self.walls['west'] = await self.world.define_custom_wall(
            CustomObjectTypes.CustomType01,
            CustomObjectMarkers.Circles2,
            710, 50,
            25, 25,
            True)

        self.walls['north'] = await self.world.define_custom_wall(
            CustomObjectTypes.CustomType03,
            CustomObjectMarkers.Diamonds2,
            1330, 50,
            25, 25,
            True)

        self.walls['south'] = await self.world.define_custom_wall(
            CustomObjectTypes.CustomType04,
            CustomObjectMarkers.Hexagons2,
            1330, 50,
            25, 25,
            True)



    @staticmethod
    def stop(self):
        self.abort_all_actions()

    @staticmethod
    def record_objects(self):
        for o in self.world.visible_objects:
            self.objects_found[o.object_id] = o

    def show_visible_objects(self):
        for o in self.world.visible_objects:
            print(o)

    def enter_world(self, d, s):
        self.drive_off_charger_contacts().wait_for_completed()
        self.drive_straight(distance_mm(d), speed_mmps(s)).wait_for_completed()


