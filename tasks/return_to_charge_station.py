#!/usr/bin/env python3

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

'''Return to charging station.

Cozmo will find the charger. Return to to the charger. Dock with the charger.
'''

import sys
sys.path.insert(0,"/home/adolfo/dev/cozmo/cozmo-python-sdk")
#import tasks.return_to_charge_station as dut
#

import asyncio
import time

import cozmo
from cozmo.util import degrees, distance_mm, speed_mmps

VERSION = 0.04


def check_reload():
    print("You are in teh correct directory")
    print("Version: ", VERSION)


def charger_position_known(robot):
    if robot.world.charger and robot.world.charger.pose.is_comparable(robot.pose):
        return True
    return False

def look_for_charger(robot, turn_step):
    robot.abort_all_actions()
    robot.stop_all_motors()
    total_turn = 0
    robot.set_head_angle(degrees(0))
    speak(robot, "Looking for charger")
    while total_turn < 360:
        if robot.world.charger and robot.world.charger.pose.is_comparable(robot.pose):
            return
        total_turn += turn_step
        robot.turn_in_place(degrees(turn_step)).wait_for_completed()
        try:
            robot.world.wait_for_observed_charger(timeout=2)
            speak(robot, "Found charger")
            return

        except asyncio.TimeoutError:
            pass

    speak(robot, "Charger not found")


def get_to_charger(robot):
    for step in {180, 90 , 45, 30}:
        if charger_position_known(robot):
            break;
        look_for_charger(robot, step)

    robot.go_to_object(robot.world.charger, distance_mm(50))

def prep_test(robot):
    robot.wait_for_all_actions_completed()
    robot.drive_off_charger_contacts().wait_for_completed()
    robot.drive_straight(distance_mm(300), speed_mmps(60))

def speak(robot, text):
    action = robot.say_text(text, in_parallel=True)
    #print(type(action))
    robot.wait_for_all_actions_completed()
    #print(text)

def return_to_charger(robot):
    if robot.is_on_charger:
        robot.say_text("hmmmm charge. Feels so... electric")
        return

    speak(robot,"I need the charger")

    # Lower lift to clear camera view
    robot.move_lift(-3)
    robot.wait_for_all_actions_completed()
    print("lift movement completed")

    charger = robot.world.charger if robot.world.charger else None

    if charger:
        speak(robot,"I know of a charger")
    else:
        speak(robot,"I have no idea where the charger is")

    if charger:
        if charger.pose.is_comparable(robot.pose):
            speak(robot, "I think I know where it is")
        else:
            speak(robot, "I lost track of the charger")
            charger = None

    if not charger:
        speak(robot,"I used to know where the charger was, But I got lost")
        speak(robot,"Lets look for it")
        look_around = robot.start_behavior(
            cozmo.behavior.BehaviorTypes.LookAroundInPlace)

        try:
            charger = robot.world.wait_for_observed_charger(timeout=10)
            speak(robot, "There is the charger")

        except asyncio.TimeoutError:
            speak(robot, "I could not find the charger")
            charger = None

        finally:
            look_around.stop()

    if charger:
        speak(robot,"Driving back to charger")
        action  = robot.go_to_object(charger,  distance_mm(50.0))
        action.wait_for_completed()
        speak(robot,"Okey dokey, we are at the charger")
    else:
        speak(robot,"I can't get back to the charger")

# cozmo.robot.Robot.drive_off_charger_on_connect = False  # Cozmo can stay on charger for now
# cozmo.run_program(drive_to_charger, use_viewer=True, force_viewer_on_top=True)
