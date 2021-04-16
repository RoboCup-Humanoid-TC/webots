from controller import Supervisor
import json
from types import SimpleNamespace

class Action():
    def __init__(self, time, action_object, action_type, values):
        self.time = time
        self.object = action_object
        self.type = action_type
        self.values = values
        self.done = False


with open('actions.json') as json_file:
    actions = json.loads(json_file.read(), object_hook=lambda d: SimpleNamespace(**d))

Actions = []
for item in actions:
    action = Action(item[0], item[1], item[2], item[3])
    Actions.append(action)

robot = Supervisor()

timestep = int(robot.getBasicTimeStep())
simulated_time = 0

while robot.step(timestep) != -1:
    simulated_time += timestep
    for action in Actions:
        if (simulated_time >= action.time) and (not action.done):
            if (action.type == "POSITION"):
                robot.getFromDef(action.object).getField('translation').setSFVec3f(action.values)
            elif(action.type == "FORCE"):
                robot.getFromDef(action.object).addForce(action.values, False)
            action.done = True

    pass
