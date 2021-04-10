On this test scenario, we have a robot initially placed outside of the field and is going to be in an illegal popsition after finishing ready state:

The following should happen:

1. The robot is spawned
2. During 2 minutes (real-time not simulated time), game state is `INITIAL` robot stays static.
3. AutoRef sends a message to the GameController, status is changed to `READY`
4. Simulation is paused, the robot is manually moved to an illegal position
   in the opponent's field, simulation is resumed.
5. Time elapses and game state changes to `SET`.
6. The robot is removed from the field by the auto-referee.

The following information should be contained in logs (among others):

```
[SSSS.xxx|0000.000] Info: Real time factor is set to 0.1.
[SSSS.xxx|0000.000] Info: Spawned RED_PLAYER_1 RobocupRobot on port 10001 at halfTimeStartingPose: translation -3.5 -3.2 0.459274, rotation 0 0.707107 0.707107 -3.14
[SSSS.xxx|0000.000] Info: Connected to GameControllerSimulator at localhost:8750
[SSSS.xxx|0000.000] Info: Red team is Funny Dingos
[SSSS.xxx|0000.000] Info: Blue team is Agile Lynxes
[SSSS.xxx|0000.000] Info: Left side is red
[SSSS.xxx|0000.000] Info: Kickoff is blue
[SSSS.xxx|0000.000] Info: Sending SIDE_LEFT:1 to GameController
[SSSS.xxx|0000.000] Info: Sending KICKOFF:2 to GameController
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_INITIAL
[SSSS.xxx|SSSS.xxx] Info: Sending STATE:READY to GameController
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_READY
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_SET
[SSSS.xxx|SSSS.xxx] Info: INCAPABLE penalty for red player 1: illegal set position. sent to reentryStartingPose: translation x.xx x.xx x.xx orientation x.xx x.xx x.xx
[SSSS.xxx|SSSS.xxx] Info: Sending STATE:PLAY to GameController
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_PLAYING
[SSSS.xxx|SSSS.xxx] Error: GameController sent "600 seconds remaining"!
```
