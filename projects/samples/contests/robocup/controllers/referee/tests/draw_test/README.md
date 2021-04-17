On this test scenario, the procedure of determinig a game as a draw by the AutoRef is going to be tested:

The following should happen:

1. The robots are spawned and the game state is `INITIAL`.
2. Time elapses and game state changes to `READY`, then `SET`, and then `PLAY`.
3. One goal is scored for team red by using the UI of the GameController.
4. One goal is scored for team blue by using the UI of the GameController.
5. Game time passes untill the end.
6. AutoRef declares the game to be a draw. ([#13](https://github.com/RoboCup-Humanoid-TC/webots/issues/13))

The following information should be contained in logs (among others):

```
[SSSS.xxx|SSSS.xxx] Info: New state received from GameController: STATE_PLAYING.
[SSSS.xxx|SSSS.xxx] Info: End of first half.
[SSSS.xxx|SSSS.xxx] Info: Beginning of second half.
[SSSS.xxx|SSSS.xxx] Info: End of second half.
[SSSS.xxx|SSSS.xxx] Info: End of the game.
[SSSS.xxx|SSSS.xxx] Info: The score is 1-1.
[SSSS.xxx|SSSS.xxx] Info: This is a draw.
```
