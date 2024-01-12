 #
# Licensed under 3-Clause BSD license in the License file. Copyright (c) 2021-2022 iRobot Corporation. All rights reserved.
#

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Create3
from irobot_edu_sdk.completer import Completer
from irobot_edu_sdk.packet import Packet
from irobot_edu_sdk.utils import bound
from irobot_edu_sdk.getter_types import Pose
from typing import Union
from struct import pack
import asyncio, math, time

class Create3(Create3): # Modified Create3 Package
    def __init__(self, backend):
        super().__init__(backend)

    async def navigate_to(self, x: Union[int, float], y: Union[int, float], heading: Union[int, float] = None, timeout: Union[int, float] = None):
        """ If heading is None, then it will be ignored, and the robot will arrive to its destination
        pointing towards the direction of the line between the destination and the origin points.
        Units:
            x, y: cm
            heading: deg
        """
        if self._disable_motors:
            return
        dev, cmd, inc = 1, 17, self.inc
        _heading = -1
        if heading is not None:
            _heading = int(heading * 10)
            _heading = bound(_heading, 0, 3599)
        if timeout is None:
            timeout = self.DEFAULT_TIMEOUT + int(math.sqrt(x * x + y * y) / 10) + 4
        payload = pack('>iih', int(x * 10), int(y * 10), _heading)
        completer = Completer()
        self._responses[(dev, cmd, inc)] = completer
        await self._backend.write_packet(Packet(dev, cmd, inc, payload))
        await completer.wait(timeout)

robot = Create3(Bluetooth())

# Settings to be configured
# Max Speed = 46 cm/s
speed = 20 # cm/s
speed_turn_dif = 0.15 # percentage difference
wall_detection_range = 150
wall_detection_error = 0.3 # percentage of error for +/-
fast_corners = True # Enable faster corner turning
nav_enable = True # Enable reverse navigation feature
nav_save_int = 1.0 # Navigation save interval in seconds

# Calculations
wall_detection_range_low = wall_detection_range * (1 - wall_detection_error)
wall_detection_range_high = wall_detection_range * (1 + wall_detection_error)
turn_speed = speed - (speed * speed_turn_dif)

# Variables
stop = False
sensors = []
sensor_max_index = 0
sensor_max_measure = 0
sensor_max_count = 0
position = Pose()
nav = [position]
state = None
wheel_left = 0
wheel_right = 0

state_dict = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall'
}

def inverse_angle(angle):
    if angle == 0:
        return 180 # or -180 as you wish
    else:
        if angle > 0:
            return angle - 180
        else:
            return angle + 180

def check_sign(data):
    if data > 0:
        return 1
    else:
        return -1

def is_between(old_pos, pos, diff):
    return (old_pos - check_sign(old_pos) * diff) < pos > (old_pos + check_sign(old_pos) * diff)

def change_state(data):
    global state, state_dict
    if data is not state:
        print('Wall follower - [%s] - %s' % (data, state_dict[data]))
        state = data

async def stop_movement():
    global stop
    stop = True
    await set_wheel_speeds_once(0, 0)
    
def start_movement():
    global stop
    stop = False

async def set_wheel_speeds_once(left, right):
    global wheel_left, wheel_right
    if left is not wheel_left or right is not wheel_right:
        await robot.set_wheel_speeds(left, right)
        wheel_left = left
        wheel_right = right
    else:
        await robot.wait(0)

async def find_wall():
    await set_wheel_speeds_once(speed, turn_speed)

async def turn_left():
    await set_wheel_speeds_once(-5, 5)

async def turn_right():
    if fast_corners:
        await set_wheel_speeds_once(speed, speed * 0.3)
    else:
        await set_wheel_speeds_once(5, -5)

async def reverse():
    await robot.move(-2)

async def follow_the_wall(): # Subroutine
    while not stop:
        # stop following the wall when other obsticles are detected
        if sensor_max_count > 1:
            break

        # if right sensor value is within distance accounted for error percent +/-
        # only send move commands once to free up communication bandwith
        if sensor_max_index == 6:
            # Adjust Left
            if sensor_max_measure > wall_detection_range_high:
                await set_wheel_speeds_once(turn_speed, speed)

            # Adjust Right
            elif sensor_max_measure < wall_detection_range_low:
                await set_wheel_speeds_once(speed, turn_speed)

            # Center
            elif wall_detection_range_low <= sensor_max_measure <= wall_detection_range_high:
                await set_wheel_speeds_once(speed, speed)

        # Right Corner / lost wall
        elif sensor_max_measure < wall_detection_range:
            await turn_right()
        else:
            break

async def get_position(): # Main Loop
    global position
    while True:
        position = (await robot.get_position())
        await robot.wait(0.5)
   
async def get_sensors(): # Main Loop
    global sensors, sensor_max_index, sensor_max_measure, sensor_max_count
    while True:
        sensors = (await robot.get_ir_proximity()).sensors
        sensor_max_index = sensors.index(max(sensors))
        sensor_max_measure = sensors[sensor_max_index]
        sensor_max_count = len([i for i in sensors if i > wall_detection_range])

        left = sensors[0] >= wall_detection_range
        m_left = sensors[1] >= wall_detection_range
        f_left = sensors[2] >= wall_detection_range
        front = sensors[3] >= wall_detection_range
        f_right = sensors[4] >= wall_detection_range
        m_right = sensors[5] >= wall_detection_range
        right = sensors[6] >= wall_detection_range
        
        if sensor_max_measure < wall_detection_range:
            change_state(0)
        elif not front and not f_left and not f_right and not m_right:
            if not left and not m_left and right:
                change_state(2)
            else:
                change_state(0)
        else:
            change_state(1)

async def save_nav(): # Main Loop
    last_save = time.time()

    if nav_enable:
        print('System - Reverse Navigation is enabled')

    while nav_enable:
        await robot.wait(0)
        current_time = time.time()

        if current_time - last_save > nav_save_int and not stop:
            nav.append(position)
            last_save = current_time

async def handle_state(): # Main Loop
    # only send move commands once to free up communication bandwith
    while True:
        if stop:
            await robot.wait(0)
        elif state == 0:
            await find_wall()
        elif state == 1:
            await turn_left()
        elif state == 2:
            await follow_the_wall()
        else:
            await robot.wait(0)

@event(robot.when_touched, [True, False])
async def navigate_home(robot):
    global nav

    if nav_enable:
        await stop_movement()
        reverse_nav = nav
        reverse_nav.reverse()
        error_dist = 40

        print('Navigation - Navigating back to start!')
        print('Navigation - Recorded points: %s' % (len(reverse_nav)))

        for i, pos in enumerate(reverse_nav):
            old_x = reverse_nav[i - 1].x
            old_y = reverse_nav[i - 1].y

            if i != 0 and is_between(old_x, pos.x, error_dist) and is_between(old_y, pos.y, error_dist):
                await robot.wait(0)
                continue
            elif i == 0 and is_between(position.x, pos.x, error_dist) and is_between(position.y, pos.y, error_dist):
                await robot.wait(0)
                continue
            else:
                await robot.navigate_to(pos.x, pos.y, timeout = nav_save_int)

            if pos.x == 0 and pos.y == 0:
                await set_wheel_speeds_once(0, 0)
                await robot.navigate_to(0, 0, 90)
                await robot.stop()
                print('Navigation - At start position.')
                nav = [Pose()] # cleans out navigation points.
                break

@event(robot.when_touched, [False, True])
async def button_continue(robot):
    if stop:
        print('System - Robot starts moving')
        start_movement()

@event(robot.when_bumped, [True, True])
async def bumped(robot):
    if not stop:
        print('Wall follower - Avoiding obstacles')
        await stop_movement()

        await reverse()
        await turn_left()
        
        heading = (await robot.get_position()).heading
        while True:
            if state == 2:
                break
            elif state == 0:
                pos = await robot.get_position()
                if abs(heading - pos.heading) >= 100:
                    await robot.turn_right(140)
                    break
            elif state == 1:
                heading = (await robot.get_position()).heading
                await robot.wait(0)

        start_movement()

@event(robot.when_play)
async def play(robot):
    # Better simultaneous processing
    await asyncio.gather(get_sensors(), handle_state(), get_position(), save_nav())

robot.play()
