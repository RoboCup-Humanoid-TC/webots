On this test scenario, declaring the winner by the AutoRef is going to be tested:

The following should happen:

1. The robots are spawned and the game state is `INITIAL`.
2. Time elapses and game state changes to `READY`, then `SET`, and then `PLAY`.
3. One goal is scored for team red by using the UI of the GameController.
4. Game time passes untill the end.
5. AutoRef declares team red winner of the match after 10 minutes in the second half have expired. ([#12](https://github.com/RoboCup-Humanoid-TC/webots/issues/12))

The following information should be contained in logs (among others):

```
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_PLAYING.
[SSSS.xxx|SSSS.xxx] Info: End of first half.
[SSSS.xxx|SSSS.xxx] Info: Beginning of second half.
[SSSS.xxx|SSSS.xxx] Info: End of second half.
[SSSS.xxx|SSSS.xxx] Info: End of the game.
[SSSS.xxx|SSSS.xxx] Info: The score is 0-1.
[SSSS.xxx|SSSS.xxx] Info: The winner is the red team.
```
