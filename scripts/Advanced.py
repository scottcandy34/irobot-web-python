 #
# Licensed under 3-Clause BSD license in the License file. Copyright (c) 2021-2022 iRobot Corporation. All rights reserved.
#

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Color, Robot, Create3
from irobot_edu_sdk.music import Note
from irobot_edu_sdk.event import Event
from irobot_edu_sdk.getter_types import Pose
from typing import Callable, Awaitable, List
import asyncio, math, time

class Room:
    def __init__(self, x1 = 0, y1 = 0, x2 = 0, y2 = 0):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2

class CliffSensor:
    def __init__(self):
        self.left = False
        self.left_front = False
        self.right = False
        self.right_front = False

class Create3(Create3): # Modified Create3 Package
    def __init__(self, backend):
        super().__init__(backend)

        self.cliff_sensor = CliffSensor()

    def when_cliff_sensor(self, condition: list[bool, bool, bool, bool], callback: Callable[[bool], Awaitable[None]]):
        """Register when cliff callback of type: async def fn(over_cliff: bool)."""
        self._when_cliff_sensor.append(Event(condition, callback))

    async def _when_cliff_sensor_handler(self, packet):
        self.cliff_sensor.disable_motors = packet.payload[4] != 0
        for event in self._when_cliff_sensor:
            self.cliff_sensor.right = packet.payload[4] & 0x01 != 0
            self.cliff_sensor.right_front = packet.payload[4] & 0x02 != 0
            self.cliff_sensor.left_front = packet.payload[4] & 0x04 != 0
            self.cliff_sensor.left = packet.payload[4] & 0x08 != 0
            
            # An empty condition list means to trigger the event on every occurrence.
            if not event.condition and (self.cliff_sensor.left or self.cliff_sensor.left_front or self.cliff_sensor.right_front or self.cliff_sensor.right):  # Any.
                await event.run(self)
                continue
            if len(event.condition) > 1 and ((event.condition[0] == self.cliff_sensor.left) and (event.condition[1] == self.cliff_sensor.left_front) and (event.condition[2] == self.cliff_sensor.right_front) and (event.condition[3] == self.cliff_sensor.right)):
                await event.run(self)

robot = Create3(Bluetooth())
nav_robot = robot

# Settings to be configured
# Max Speed = 46 cm/s
speed = 20 # cm/s
speed_turn_dif = 0.15 # percentage difference
wall_detection_range = 150
wall_detection_error = 0.3 # percentage of error for +/-
fast_corners = True # Enable faster corner turning
object_detection_range = 150
robot_buffer = 300 # in mm
robot_radius = 180 # in mm
object_radius = 180 # in mm
room_enable = False # Enable room feature
rooms = [Room(-300, 230, -2700, 1600), Room(-2740, 1640, -230, 2700)] # room corners x1,y1,x2,y2
nav_enable = True # Enable reverse navigation feature
nav_save_int = 1.0 # Navigation save interval in seconds
nav_robot.DEFAULT_TIMEOUT = 1 # set default timeout for return navigation
lights_enable = False
auto_docking = False # Enable auto docking when battery low, Needs navigation save enabled
battery_low_warning = 20 # percentage

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
found_object = False
nav = [position]
room = None
state = None
wheel_left = 0
wheel_right = 0

state_dict = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall'
}

sensor_dict = {
    0: 'Left Sensor',
    1: 'Left Middle Sensor',
    2: 'Left Front Sensor',
    3: 'Front Sensor',
    4: 'Right Front Sensor',
    5: 'Right Middle Sensor',
    6: 'Right Sensor',
}

def get_object_postition():
    x_sign = 1
    y_sign = 1
    if 90 < position.heading < 270:
        x_sign = -1
    if 180 < position.heading < 360:
        y_sign = -1

    x = x_sign * math.cos(position.heading) * (robot_radius + object_radius) + position.x
    y = y_sign * math.sin(position.heading) * (robot_radius + object_radius) + position.y
    distance = math.ceil(math.sqrt((abs(x) ** 2) + (abs(y) ** 2)) / 10)
    return x, y, distance

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
    found_object = False

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

async def locate_object(): # Subroutine
    global found_object
    await robot.turn_left(90)
    while True:
        if sensor_max_index < 3:
            await set_wheel_speeds_once(-1, 1) # turn left
            await robot.play_note(2000, 0.05)
        elif sensor_max_index > 3:
            await set_wheel_speeds_once(1, -1) # turn right
            await robot.play_note(2000, 0.05)
        else:
            found_object = True
            await set_wheel_speeds_once(1, 1) # straight
            print('Room - Homed Object: Facing %s degrees' % (position.heading))

            while stop:
                await robot.play_note(3000, 0.5)
            found_object = False
            break

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

async def set_lights(): # Main Loop
    if lights_enable:
        print('System - Lights are enabled')
        print('Lights - Red and Blinking is stop is true')
        print('Lights - White and spining is when within room')
        print('Lights - Yellow is at state 0: %s' % (state_dict[0]))
        print('Lights - Blue is at state 1: %s' %  (state_dict[1]))
        print('Lights - Green is at state 2: %s' %  (state_dict[2]))

    while lights_enable:
        await robot.wait(0)
        if stop:
            await robot.set_lights(Robot.LIGHT_BLINK, Color(255, 0, 0)) # Red
            while stop:
                await robot.wait(0)
        elif room:
            await robot.set_lights(Robot.LIGHT_SPIN, Color(255, 255, 255)) # White
            while not stop and room:
                await robot.wait(0)
        elif state == 0:
            await robot.set_lights_rgb(255,255,0) # Yellow
            while not stop and not room and state != 0:
                await robot.wait(0)
        elif state == 1:
            await robot.set_lights_rgb(0, 0, 255) # Blue
            while not stop and not room and state != 1:
                await robot.wait(0)
        elif state == 2:
            await robot.set_lights_rgb(0, 255, 0) # Green
            while not stop and not room and state != 2:
                await robot.wait(0)

async def rooms(): # Main Loop
    global room
    detected_object = False

    if room_enable:
        print('System - Room detection is enabled')

    while room_enable:
        await robot.wait(0)
        room_detection = room
        for i in rooms:
            # checks for which number is bigger
            x1 = i.x1
            x2 = i.x2
            y1 = i.y1
            y2 = i.y2
            if i.x1 > i.x2:
                x1 = i.x2
                x2 = i.x1
            if i.y1 > i.y2:
                y1 = i.y2
                y2 = i.y1
            
            # checks if within room
            if x1 < position.x < x2 and y1 < position.y < y2:
                room = Room()
                room.x1 = x1
                room.x2 = x2
                room.y1 = y1
                room.y2 = y2
            else:
                room = None

        if (room is None and room_detection is not None):
            print('Room - Exiting: points (%s, %s) and (%s, %s)' % (room.x1, room.y1, room.x2, room.y2))
        elif (room is not None and room_detection is None):
            print('Room - Entering: points (%s, %s) and (%s, %s)' % (room.x1, room.y1, room.x2, room.y2))
        elif room is not None and room_detection is not None and room.x1 != room_detection.x1 and room.x2 != room_detection.x2:
            print('Room - Changing Room: points (%s, %s) and (%s, %s)' % (room.x1, room.y1, room.x2, room.y2))

        if room is not None and not detected_object:
            left = sensors[0] >= object_detection_range
            m_left = sensors[1] >= object_detection_range
            f_left = sensors[2] >= object_detection_range
            front = sensors[3] >= object_detection_range
            f_right = sensors[4] >= object_detection_range
            
            # if within x plane
            if room.x1 + robot_buffer < position.x < room.x2 - robot_buffer:
                if left or m_left or f_left or front or f_right:
                    print('Room - Detected Object: On X axis')
                    detected_object = True
                    await stop_movement()
            
            # if within y plane
            if room.y1 + robot_buffer < position.y < room.y2 - robot_buffer:
                if left or m_left or f_left or front or f_right:
                    print('Room - Detected Object: On Y axis')
                    detected_object = True
                    await stop_movement()

        elif room is None and detected_object:
            detected_object = False
            room_detection = None

        elif detected_object and not found_object:
            index = [left, m_left, f_left, front, f_right]
            sensor_triggered = index.index(max(index))
            print('Room - Homing in Object: [%s] - %s' % (sensor_triggered, sensor_dict[sensor_triggered]))
            await locate_object()

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

async def get_docking_sensors(): # Main Loop
    if auto_docking and nav_enable:
        print('System - Auto Docking is enabled')
    elif auto_docking and not nav_enable:
        print('System - Auto Docking needs Reverse Navigation enabled to function properly')
        
    while auto_docking:
        battery = await robot.get_battery_level()
        await robot.wait(5)
        if battery[1] < battery_low_warning:
            print('System - Battery lower than %s is %s Returning Home' % (battery_low_warning, battery[1]))
            await navigate_home(robot)
            await robot.turn_right(270)
            start_movement()

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
                await nav_robot.navigate_to(pos.x, pos.y)

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
    if found_object:
        await set_wheel_speeds_once(0, 0)
        object_x, object_y, distance = get_object_postition()
        print('Room - Object Found: Coordinates (%s, %s)' % (object_x, object_y))
        print('Room - Object Found: Distance from start: %s' % (distance))
    elif not stop:
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

# Act as a kidnaped sensor and stop robot processes
@event(robot.when_cliff_sensor, [True, True, True, True])
async def kidnaped(robot):
    print('System - Help Kidnaped!')
    change_state(0)
    await stop_movement()
    await robot.stop() # reset navigation

@event(robot.when_cliff_sensor, [False, False, False, False])
async def free(robot):
    global nav
    if stop:
        print('System - I\'m Free!')
        await robot.wait(1)
        nav = [Pose()] # cleans out navigation points.
        start_movement()

@event(robot.when_docking_sensor)
async def docking(robot):
    battery = await robot.get_battery_level()
    if auto_docking and not stop and battery[1] < battery_low_warning:
        print('System - Need more power: Docking')
        await stop_movement()
        await robot.stop()
        await robot.dock()

@event(robot.when_play)
async def play(robot):
    # auto undock from charger
    contact = (await robot.get_docking_values())['contacts']
    if (contact == 1):
        print('System - Undocking')
        await robot.undock()
        await robot.turn_right(90)
        await robot.stop() # reset navigation
    
    # Better simultaneous processing
    await asyncio.gather(get_sensors(), handle_state(), get_position(), save_nav(), rooms(), set_lights(), get_docking_sensors())

robot.play()
