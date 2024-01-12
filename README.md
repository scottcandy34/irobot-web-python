# iRobot Web Python Scripts

This is a repository of scripts that can control the iRobot Create3 Bot from the [Online Editor](https://python.irobot.com/). It uses the SDK found here [irobot-edu-python-sdk](https://github.com/iRobotEducation/irobot-edu-python-sdk), works with SDK v0.5.1.

## Scripts
### Advanced
- Wall Follower
- Reverse Navigation
- Room Detection
- Object Detection
- Low Battery Detection return to dock.
- handle kidnaped actions
- lights change for actions
#### Global
- set wheel speed
- set turn speed difference in percentage
- Runs everything simultaneously
- auto undocks from
#### Wall Follower
- stays within an error range to keep straight along a wall
- set wall detection range
- set wall detection range error percentage
- enable fast corners
- uses bumpers to identify if wall is dark object and navigates around
- get sensors data separate from telling the robot how to handle them.
#### Reverse Navigation
- enable or disable navigation
- set navigation save interval
- gets position actively
- save position to array every interval default to 1 sec
- press single dot button to trigger reverse navigation
- press double dot button to start the bot again
- ignores nav points if too close to bots current position within the radius of the bot
#### Room Detection
- enable or disable room detection
- set room coordinates in array
- will print out what room it is in
#### Object Detection
- set robot radius
- set object radius
- set robot buffer
- set object detection range
- detects object within a room
- once detected points robot towards object
- will print out the objects exact position calculated on where the robot is.
- will play noise if found object
#### Low Battery Detection
- enable auto docking when battery is low
- set battery low warning percentage
- will get battery level regularly
- if level is lower than warning percentage navigate home
- if near dock and has low battery it will auto dock
#### Handle Kidnaped Actions
- when robot is picked up it will reset state and stop all movement
- when it is freed it will hold for 1 sec then start all movement 
- it will clear navigation points because it has no idea where it is relative to be picked up and placed
#### Lights Change
- enable or disable lights
- lights will change dependent of what state its in
