On this test scenario, there are two teams declared for a normal KidSize match, one team with 4 players, one with 3. Some valid and invalid positions are provided for the
robots of both teams to test the auto referee behavior:

- Team 1 (RED): 2 robots with invalid position
  - 1: partially on the border line (#3)
- Team 2 (BLUE): 2 robots have an invalid position
  - 1: outside of the field with on leg on the left half of the field and the other leg on the right half of the field (#4)
  - 2: on the wrong side

The following should happen:

1. All robots are spawned on their appropriate side, according to the team
   configuration files (just robot `Blue 2` is on the wrong side).
2. During 2 minutes (real-time not simulated time), game state is `INITIAL` all
   robots stay static. (#1)
3. The teams randomly receive a side they play on. (#5)
4. Kick-off is randomly set to a team. (#5)
3. AutoRef sends a message to the GameController, status is changed to `READY`. (#1)
4. Penalties are being called for illegaly positioned robots who are moved to
   appropriate locations.
5. Robots `Red 2`, `Red 3`, `Red 4`, and `Blue 3` are not penalized and can progress positioning themselves on the field. (#2)
6. The simulation is paused, All the robots are moved into their field. Here is some exact locations as a test (rotations are not altered):
   - `Red 1`:  `-3.5 -2 0.24`
   - `Red 2`:  `-3.5 2 0.24`
   - `Red 3`:  `-0.75 -2 0.24`
   - `Red 4`:  `-0.75 2 0.24`

   - `Blue 1`:  `-3.5 -2 0.24`
   - `Blue 2`:  `-3.5 2 0.24`
   - `Blue 3`:  `-0.75 -2 0.24`
   (Of course the positions for the team on the right side should be flipped)
   Then, the simulation is resumed.

7. Time elapses and game state changes to `SET`. (#5)
8. Time elapses and game state changes to `PLAY`. (#5)

The following information should be contained in logs (among others):

```
[SSSS.xxx|0000.000] Info: Spawned RED_PLAYER_1 RobocupRobot on port 10001 at halfTimeStartingPose: translation -3.5 -3.02 0.24, rotation 0 0 1 1.57
[SSSS.xxx|0000.000] Info: Spawned RED_PLAYER_2 RobocupRobot on port 10002 at halfTimeStartingPose: translation -3.5 3.06 0.24, rotation 0 0 1 -1.57
[SSSS.xxx|0000.000] Info: Spawned RED_PLAYER_3 RobocupRobot on port 10003 at halfTimeStartingPose: translation -0.75 -3.06 0.24, rotation 0 0 1 1.57
[SSSS.xxx|0000.000] Info: Spawned RED_PLAYER_4 RobocupRobot on port 10004 at halfTimeStartingPose: translation -0.75 3.06 0.24, rotation 0 0 1 -1.57
[SSSS.xxx|0000.000] Info: Spawned BLUE_PLAYER_1 RobocupRobot on port 10021 at halfTimeStartingPose: translation 0 -3.2 0.24, rotation 0 0 1 1.571592653589793
[SSSS.xxx|0000.000] Info: Spawned BLUE_PLAYER_2 RobocupRobot on port 10022 at halfTimeStartingPose: translation -2 3.06 0.24, rotation 0 0 1 4.711592653589793
[SSSS.xxx|0000.000] Info: Spawned BLUE_PLAYER_3 RobocupRobot on port 10023 at halfTimeStartingPose: translation 0.75 -3.06 0.24, rotation 0 0 1 1.571592653589793
[SSSS.xxx|0000.000] Info: Sending SIDE_LEFT:x to GameController
[SSSS.xxx|0000.000] Info: Sending KICKOFF:x to GameController
[SSSS.xxx|SSSS.xxx] Info: Sending STATE:READY to GameController
[SSSS.xxx|SSSS.xxx] Info: INCAPABLE penalty for red player 1: halfTimeStartingPose inside field. Sent to translation x.xx x.xx x.xx, rotation x.xx x.xx x.xx x.xx
[SSSS.xxx|SSSS.xxx] Info: INCAPABLE penalty for blue player 1: halfTimeStartingPose outside team side. Sent to translation x.xx x.xx x.xx, rotation x.xx x.xx x.xx x.xx
[SSSS.xxx|SSSS.xxx] Info: INCAPABLE penalty for blue player 2: halfTimeStartingPose outside team side. Sent to translation x.xx x.xx x.xx, rotation x.xx x.xx x.xx x.xx
[SSSS.xxx|SSSS.xxx] Setting state to SET
[SSSS.xxx|SSSS.xxx] Setting state to PLAY
```
