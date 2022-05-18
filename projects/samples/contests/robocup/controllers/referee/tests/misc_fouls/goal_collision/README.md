# Goal collision

There are different parts in this scenario:

## What is tested

- A player that touches the goal continuously during 10 seconds, but manages
  to move away is not penalized
- A player that touches the goal for more than 15 seconds without interruption
  is penalized
- A player that touches the goal for 10 seconds, manages to get away for a few
  seconds and then touches the goal for 5 more seconds is penalized.

## Setup and description

This scenario involves three robots, with independent scenarios

- RED 1: goalkeeper
- RED 2: field player
- BLUE 1: goalkeeper


### A. RED 1

- Setup:
  - Once state reaches PLAYING, robot falls on the goal
  - After 10 seconds it gets away from the post
- Expectations:
  - RED 1 is never penalized

### B. RED 2

- Setup:
  - Once state reaches PLAYING, robot falls on the goal
  - It stays stuck on the goal for 15 seconds
- Expectations:
  - RED 2 is penalized approximately 15 seconds after colliding with the goal

### C. BLUE 1

- Setup:
  - Once state reaches PLAYING, robot falls on the goal
  - It stays stuck on the goal for 10 seconds and then moves away
  - 10 seconds later it falls on the goal again and does not move
- Expectations:
  - BLUE 1 stays unpenalized for approximately 25 seconds after first collision
  - Finally, BLUE 1 receives a penalty

## Implementation note

Since robots get 'inactive' if they are not moving, it is required to reset the
position of the robots multiple times to ensure the robot does not become
inactive.
In a 'real-world' scenario, this should not be a problem since robots will use
their motors to move and therefore not be considered as inactive.
