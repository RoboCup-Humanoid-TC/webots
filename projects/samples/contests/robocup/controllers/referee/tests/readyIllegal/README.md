On this test scenario, we have a robot initially placed outside of the field and is going to be in an illegal popsition after finishing ready state:

The following should happen:

1. The robots are spawned.
2. During 2 minutes (real-time not simulated time), game state is `INITIAL` robots stay static.
3. AutoRef sends a message to the GameController, status is changed to `READY`.
4. Simulation is paused, the robot number 2 from the team that does not have the kick-off is manually moved to the center circle in it's own half, the
   robot number 2 from the team having the kick-off is manually moved to the center circle in it's own half, the robot number 1 of one of the teams is moved to opponent's field, The robot number 1 from the other team is moved to a valid position in its own field, simulation is resumed.
5. Time elapses and game state changes to `SET`.
6. The robot that is in the opponent's field is removed by the auto-referee.
7. The robot from the team that does not have the kivk-off and is in the center circle in it's own half is also removed.
8. The robot from the team that has the kick-off and is in the center circle in it's own half is not removed.
9. then the simulation is paused before `PLAY`. The robot number 1 from the other team is fallen into the opponent's
   half of the field, simulation is resumed.
10. Again before `PLAY` state, the robot touching opponent's field is removed the auto-referee.

The following information should be contained in logs (among others):

```
[0001.157|0000.000] Info: Spawned RED_PLAYER_1 RobocupRobot on port 10001 at halfTimeStartingPose: translation -3.5 -3.2 0.24, rotation 0 0 1 1.57
[0001.808|0000.000] Info: Spawned RED_PLAYER_2 RobocupRobot on port 10002 at halfTimeStartingPose: translation -3.5 3.2 0.24, rotation 0 0 1 -1.57
[0002.459|0000.000] Info: Spawned BLUE_PLAYER_1 RobocupRobot on port 10021 at halfTimeStartingPose: translation 3.5 -3.2 0.24, rotation 0 0 1 1.571592653589793
[0002.992|0000.000] Info: Spawned BLUE_PLAYER_2 RobocupRobot on port 10022 at halfTimeStartingPose: translation 3.5 3.2 0.24, rotation 0 0 1 4.711592653589793
[0002.996|0000.000] Info: Left side is red
[0002.996|0000.000] Info: Kickoff is red
[0002.996|0000.000] Info: Sending SIDE_LEFT:1 to GameController
[0002.996|0000.000] Info: Sending KICKOFF:1 to GameController
[0003.116|0000.048] Info: New state received from GameController: STATE_INITIAL
[0010.302|0012.040] Info: Sending STATE:READY to GameController
[0010.622|0014.600] Info: New state received from GameController: STATE_READY
[0231.955|0059.728] Info: New state received from GameController: STATE_SET
[SSSS.xxx|SSSS.xxx] INCAPABLE penalty for blue player 1: illegal set position in the opponent's field. sent to reentryStartingPose: translation x.xx x.xx x.xx orientation x.xx x.xx x.xx
[SSSS.xxx|SSSS.xxx] INCAPABLE penalty for blue player 2: illegal set position in the center circle while not having the kick-off. sent to reentryStartingPose: translation x.xx x.xx x.xx orientation x.xx x.xx x.xx
[SSSS.xxx|SSSS.xxx] INCAPABLE penalty for red player 1: illegal set position in the opponent's field. sent to reentryStartingPose: translation x.xx x.xx x.xx orientation x.xx x.xx x.xx
[0314.639|0064.720] Info: Sending STATE:PLAY to GameController
[0315.576|0065.608] Info: New state received from GameController: STATE_PLAYING
```
