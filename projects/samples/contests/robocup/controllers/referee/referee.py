# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import copy
import json
import math
import numpy as np
import os
import random
import socket
import subprocess
import sys
import time
import traceback
import yaml

from scipy.spatial import ConvexHull

from types import SimpleNamespace

from controller import Supervisor, Node

from gamestate import GameState
from field import Field
from forceful_contact_matrix import ForcefulContactMatrix
from logger import logger
from geometry import distance2, rotate_along_z, aabb_circle_collision, polygon_circle_collision, update_aabb
from display import Display
from game import Game
from team import Team
from sim_time import SimTime
from blackboard import blackboard


# game interruptions requiring a free kick procedure
GAME_INTERRUPTIONS = {
    'DIRECT_FREEKICK': 'direct free kick',
    'INDIRECT_FREEKICK': 'indirect free kick',
    'PENALTYKICK': 'penalty kick',
    'CORNERKICK': 'corner kick',
    'GOALKICK': 'goal kick',
    'THROWIN': 'throw in'
}


class Referee:
    def __init__(self):
        # start the webots supervisor
        self.supervisor = Supervisor()
        self.time_step = int(self.supervisor.getBasicTimeStep())
        self.sim_time = SimTime()
        with open('referee_config.yaml') as f:
            config = yaml.safe_load(f)
        self.config = SimpleNamespace(**config)
        self.config.GOAL_HALF_WIDTH = self.config.GOAL_WIDTH / 2
        self.config.SIMULATED_TIME_INTERRUPTION_PHASE_0 = \
            int(self.config.SIMULATED_TIME_INTERRUPTION_PHASE_0 * 1000 / self.time_step)
        self.config.SIMULATED_TIME_BEFORE_PLAY_STATE = \
            int(self.config.SIMULATED_TIME_BEFORE_PLAY_STATE * 1000 / self.time_step)
        self.config.SIMULATED_TIME_SET_PENALTY_SHOOTOUT = \
            int(self.config.SIMULATED_TIME_SET_PENALTY_SHOOTOUT * 1000 / self.time_step)

        self.game_controller_socket = None

        self.blackboard = blackboard
        self.blackboard.supervisor = self.supervisor
        self.blackboard.sim_time = self.sim_time
        self.blackboard.config = self.config
        self.blackboard.start_real_time = time.time()
        self.logger = logger

        # determine configuration file name
        self.game_config_file = os.environ['WEBOTS_ROBOCUP_GAME'] if 'WEBOTS_ROBOCUP_GAME' in os.environ \
            else os.path.join(os.getcwd(), 'game.json')
        if not os.path.isfile(self.game_config_file):
            self.logger.error(f'Cannot read {self.game_config_file} game config file.')
            self.clean_exit()

        # read game configuration
        with open(self.game_config_file) as json_file:
            game_data = json.loads(json_file.read())
        game_data['blackboard'] = self.blackboard
        field_size = game_data['class'].lower()
        game_data['field_size'] = field_size
        self.field = Field(field_size)
        self.blackboard.field = self.field

        self.game = Game(**game_data)
        self.blackboard.game = self.game

        try:
            self.red_team = Team.from_json(self.game.red.config)
            self.blue_team = Team.from_json(self.game.blue.config)
        except Exception:
            self.clean_exit()
        self.red_team.color = 'red'
        self.blue_team.color = 'blue'
        self.red_team.setup()
        self.blue_team.setup()

        self.blackboard.red_team = self.red_team
        self.blackboard.blue_team = self.blue_team

        self.forceful_contact_matrix = ForcefulContactMatrix(len(self.red_team.players),
                                                             len(self.blue_team.players),
                                                             self.config.FOUL_PUSHING_PERIOD,
                                                             self.config.FOUL_PUSHING_TIME, self.time_step)
        self.display = Display()
        self.others = []
        self.game_controller_send_id = 0
        self.game_controller_send_unanswered = {}
        self.game_controller_last_sent_message = None
        self.game_controller_udp_filter = os.environ['GAME_CONTROLLER_UDP_FILTER'] \
            if 'GAME_CONTROLLER_UDP_FILTER' in os.environ else None

        self.setup()
        self.display.update()

        self.status_update_last_real_time = None
        self.status_update_last_sim_time = None

        try:
            self.main_loop()
        except Exception:
            self.logger.error(f"Unexpected exception in main referee loop: {traceback.format_exc()}")

        self.clean_exit()

    def announce_final_score(self):
        """ Prints score of the match to the console and saves it to the log. """
        if not hasattr(self.game, "state"):
            return
        red_team_idx = self.team_index('red')
        blue_team_idx = self.team_index('blue')
        red_score = self.game.state.teams[red_team_idx].score
        blue_score = self.game.state.teams[blue_team_idx].score
        # TODO: store and print score before penalty shootouts
        self.logger.info(f"FINAL SCORE: {red_score}-{blue_score}")

    def clean_exit(self):
        """Save logs and clean all subprocesses"""
        self.announce_final_score()
        if hasattr(self.game, "controller") and self.game_controller_socket:
            self.logger.info("Closing 'controller' socket")
            self.game_controller_socket.close()
        if hasattr(self.game, "controller_process") and self.game.controller_process:
            self.logger.info("Terminating 'game_controller' process")
            self.game.controller_process.terminate()
        if hasattr(self.game, "udp_bouncer_process") and self.udp_bouncer_process:
            self.logger.info("Terminating 'udp_bouncer' process")
            self.udp_bouncer_process.terminate()
        if hasattr(self.game, 'over') and self.game.over:
            self.logger.info("Game is over")
            if hasattr(self.game, 'press_a_key_to_terminate') and self.game.press_a_key_to_terminate:
                print('Press a key to terminate')
                keyboard = self.supervisor.getKeyboard()
                keyboard.enable(self.time_step)
                while self.supervisor.step(self.time_step) != -1:
                    if keyboard.getKey() != -1:
                        break
            else:
                waiting_steps = self.config.END_OF_GAME_TIMEOUT * 1000 / self.time_step
                self.logger.info(f"Waiting {waiting_steps} simulation steps before exiting")
                while waiting_steps > 0:
                    self.supervisor.step(self.time_step)
                    waiting_steps -= 1
                self.logger.info("Finished waiting")
        if hasattr(self.game, 'record_simulation'):
            if self.game.record_simulation.endswith(".html"):
                self.logger.info("Stopping animation recording")
                self.supervisor.animationStopRecording()
            elif self.game.record_simulation.endswith(".mp4"):
                self.logger.info("Starting encoding")
                self.supervisor.movieStopRecording()
                while not self.supervisor.movieIsReady():
                    self.supervisor.step(self.time_step)
                self.logger.info("Encoding finished")
        self.logger.info("Exiting webots properly")

        # Note: If self.supervisor.step is not called before the 'simulationQuit', information is not shown
        self.supervisor.step(self.time_step)
        self.supervisor.simulationQuit(0)
        sys.exit()

    def print_status(self):
        """
        Print/Log status about current real time factor and state the game is in.
        Period of printing is limited by the STATUS_PRINT_PERIOD in the config
        """
        now = time.time()
        if self.status_update_last_real_time is None or self.status_update_last_sim_time is None:
            self.status_update_last_real_time = now
            self.status_update_last_sim_time = self.sim_time.get_sec()
        elif now - self.status_update_last_real_time > self.config.STATUS_PRINT_PERIOD:
            elapsed_real = now - self.status_update_last_real_time
            elapsed_simulation = self.sim_time.get_sec() - self.status_update_last_sim_time
            speed_factor = elapsed_simulation / elapsed_real
            messages = [f"Avg speed factor: {speed_factor:.3f} (over last {elapsed_real:.2f} seconds)"]
            if self.game.state is None:
                messages.append("No messages received from GameController yet")
            else:
                messages.append(f"state: {self.game.state.game_state}, remaining time: {self.game.state.seconds_remaining}")
                if self.game.state.secondary_state in GAME_INTERRUPTIONS:
                    messages.append(f"  sec_state: {self.game.state.secondary_state} phase: "
                                    f"{self.game.state.secondary_state_info[1]}")
            if self.game.penalty_shootout:
                messages.append(f"{self.get_penalty_shootout_msg()}")
            messages = [f"STATUS: {m}" for m in messages]
            self.logger.info(messages)
            self.status_update_last_real_time = now
            self.status_update_last_sim_time = self.sim_time.get_sec()

    def toss_a_coin_if_needed(self, attribute):  # attribute should be either "side_left" or "kickoff"
        # If game.json contains such an attribute, use it to determine field side and kick-off
        # Supported values are "red", "blue" and "random". Default value is "random".
        if hasattr(self.game, attribute):
            if getattr(self.game, attribute) == 'red':
                setattr(self.game, attribute, self.game.red.id)
            elif getattr(self.game, attribute) == 'blue':
                setattr(self.game, attribute, self.game.blue.id)
            elif getattr(self.game, attribute) != 'random':
                self.logger.error(f'Unsupported value for "{attribute}" in game.json file: '
                                  f'{getattr(self.game, attribute)}, using "random".')
                setattr(self.game, attribute, 'random')
        else:
            setattr(self.game, attribute, 'random')
        if getattr(self.game, attribute) == 'random':  # toss a coin to determine a random team
            setattr(self.game, attribute, self.game.red.id if bool(random.getrandbits(1)) else self.game.blue.id)

    def spawn_team(self, team, red_on_right):
        """
        Spawn robots defined in the team.json file at their defined starting poses
        :param team: Team object containing the team configuration
        :param red_on_right: whether or not right is on the right, used for flipping the spawn poses
        """
        children = self.supervisor.getRoot().getField('children')
        color = team.color
        nb_players = len(team.players)
        for number, player in team.players.items():
            model = player['proto']
            n = int(number) - 1
            port = self.game.red.ports[n] if color == 'red' else self.game.blue.ports[n]
            if red_on_right:  # symmetry with respect to the central line of the field
                self.flip_poses(player)
            defname = color.upper() + '_PLAYER_' + number
            halfTimeStartingTranslation = player['halfTimeStartingPose']['translation']
            halfTimeStartingRotation = player['halfTimeStartingPose']['rotation']
            string = f'DEF {defname} {model}{{name "{color} player {number}" ' + \
                     f'translation {halfTimeStartingTranslation[0]} ' + \
                     f'{halfTimeStartingTranslation[1]} {halfTimeStartingTranslation[2]} ' + \
                     f'rotation {halfTimeStartingRotation[0]} ' + \
                     f'{halfTimeStartingRotation[1]} {halfTimeStartingRotation[2]} ' + \
                     f'{halfTimeStartingRotation[3]} controllerArgs ["{port}" "{nb_players}"'
            hosts = self.game.red.hosts if color == 'red' else self.game.blue.hosts
            for h in hosts:
                string += f', "{h}"'
            string += '] }}'
            children.importMFNodeFromString(-1, string)
            player['robot'] = self.supervisor.getFromDef(defname)
            player['position'] = player['robot'].getCenterOfMass()
            self.logger.info(f'Spawned {defname} {model} on port {port} at halfTimeStartingPose: translation (' +
                             f'{halfTimeStartingTranslation[0]} {halfTimeStartingTranslation[1]} ' +
                             f'{halfTimeStartingTranslation[2]}), rotation ({halfTimeStartingRotation[0]} ' +
                             f'{halfTimeStartingRotation[1]} {halfTimeStartingRotation[2]} {halfTimeStartingRotation[3]}).')

    def team_index(self, color):
        """
        Returns index of the team in self.game.state.teams of the specified team.
        :param color: team color 'red' or 'blue'
        :return: index of the team in self.game.state.teams
        """
        if color not in ['red', 'blue']:
            raise RuntimeError(f'Wrong color passed to team_index(): \'{color}\'.')
        id = self.game.red.id if color == 'red' else self.game.blue.id
        index = 0 if self.game.state.teams[0].team_number == id else 1
        if self.game.state.teams[index].team_number != id:
            raise RuntimeError(f'Wrong team number set in team_index(): {id} != {self.game.state.teams[index].team_number}')
        return index

    def game_controller_receive(self):
        """
        Receive new message from gamecontroller and update the game object (including game.state) accordingly
        Also update display.
        """
        data = None
        ip = None
        while True:
            if self.game_controller_udp_filter and ip and ip not in self.others:
                self.others.append(ip)
                self.logger.warning(f'Ignoring UDP packets from {ip} not matching '
                                    f'GAME_CONTROLLER_UDP_FILTER={self.game_controller_udp_filter}.')
            try:
                data, peer = self.game.udp.recvfrom(GameState.sizeof())  # TODO move udp from game.py to referee.py
                ip, port = peer
                if self.game_controller_udp_filter is None or self.game_controller_udp_filter == ip:
                    break
                else:
                    continue
            except BlockingIOError:
                return
            except Exception as e:
                self.logger.error(f'UDP input failure: {e}')
                return
        self.previous_seconds_remaining = self.game.state.seconds_remaining if self.game.state else 0
        previous_secondary_seconds_remaining = self.game.state.secondary_seconds_remaining if self.game.state else 0
        previous_state = self.game.state.game_state if self.game.state else None
        previous_sec_state = self.game.state.secondary_state if self.game.state else None
        previous_sec_phase = self.game.state.secondary_state_info[1] if self.game.state else None
        if self.game.state:
            red = 0 if self.game.state.teams[0].team_color == 'RED' else 1
            blue = 1 if red == 0 else 0
            previous_red_score = self.game.state.teams[red].score
            previous_blue_score = self.game.state.teams[blue].score
        else:
            previous_red_score = 0
            previous_blue_score = 0

        self.game.state = GameState.parse(data)

        if previous_state != self.game.state.game_state:
            self.logger.info(f'New state received from GameController: {self.game.state.game_state}.')
            if self.game.wait_for_state is not None:
                if 'STATE_' + self.game.wait_for_state != self.game.state.game_state:
                    self.logger.warning(f'Received unexpected state from GameController: {self.game.state.game_state} ' +
                                        f'while expecting {self.game.wait_for_state}')
                else:
                    self.logger.info(f"State has successfully changed to {self.game.wait_for_state}")
                    self.game.wait_for_state = None
        new_sec_state = self.game.state.secondary_state
        new_sec_phase = self.game.state.secondary_state_info[1]
        if previous_sec_state != new_sec_state or previous_sec_phase != new_sec_phase:
            self.logger.info(f'New secondary state received from GameController: {new_sec_state}, phase {new_sec_phase}.')
            if self.game.wait_for_sec_state is not None or self.game.wait_for_sec_phase is not None:
                if 'STATE_' + self.game.wait_for_sec_state != new_sec_state or new_sec_phase != self.game.wait_for_sec_phase:
                    self.logger.warning(f'Received unexpected secondary state from GameController: '
                                        f'{new_sec_state}:{new_sec_phase} '
                                        f'while expecting {self.game.wait_for_sec_state}:{self.game.wait_for_sec_phase}')
                else:
                    self.logger.info(f"State has successfully changed to {new_sec_state}:{new_sec_phase}")
                    self.game.wait_for_sec_state = None
                    self.game.wait_for_sec_phase = None
        if self.game.state.game_state == 'STATE_PLAYING' and \
                self.game.state.secondary_seconds_remaining == 0 and previous_secondary_seconds_remaining > 0:
            if self.game.in_play is None and self.game.phase == 'KICKOFF':
                self.logger.info('Ball in play, can be touched by any player (10 seconds elapsed after kickoff).')
                self.game.in_play = self.sim_time.get_ms()
                self.game.ball_last_move = self.sim_time.get_ms()
        if self.previous_seconds_remaining != self.game.state.seconds_remaining:
            allow_in_play = self.game.wait_for_sec_state is None and self.game.wait_for_sec_phase is None
            if allow_in_play and self.game.state.secondary_state == "STATE_NORMAL" \
                    and self.game.interruption_seconds is not None:
                if self.game.interruption_seconds - self.game.state.seconds_remaining > self.config.IN_PLAY_TIMEOUT:
                    if self.game.in_play is None:
                        self.logger.info('Ball in play, can be touched by any player (10 seconds elapsed).')
                        self.game.ball_in_play(self.sim_time.get_ms())
            self.display.update()
        red = 0 if self.game.state.teams[0].team_color == 'RED' else 1
        blue = 1 if red == 0 else 0
        if previous_red_score != self.game.state.teams[red].score or \
                previous_blue_score != self.game.state.teams[blue].score:
            self.display.update()
        # print(self.game.state.game_state)
        secondary_state = self.game.state.secondary_state
        secondary_state_info = self.game.state.secondary_state_info
        if secondary_state[0:6] == 'STATE_' and secondary_state[6:] in GAME_INTERRUPTIONS:
            kick = secondary_state[6:]
            step = secondary_state_info[1]
            delay = (self.sim_time.get_ms() - self.game.interruption_step_time) / 1000
            if step == 0 and self.game.interruption_step != step:
                self.game.interruption_step = step
                self.game.interruption_step_time = self.sim_time.get_ms()
                self.logger.info(f'Awarding a {GAME_INTERRUPTIONS[kick]}.')
            elif (step == 1 and self.game.interruption_step != step and self.game.state.secondary_seconds_remaining <= 0 and
                  delay >= self.config.SIMULATED_TIME_INTERRUPTION_PHASE_1):
                self.game.interruption_step = step
                self.game_controller_send(f'{kick}:{secondary_state_info[0]}:PREPARE')
                self.game.interruption_step_time = self.sim_time.get_ms()
                self.logger.info(f'Prepare for {GAME_INTERRUPTIONS[kick]}.')
            elif step == 2 and self.game.interruption_step != step and self.game.state.secondary_seconds_remaining <= 0:
                self.game.interruption_step = step
                opponent_team = self.blue_team if secondary_state_info[0] == self.game.red.id else self.red_team
                self.check_team_away_from_ball(opponent_team, self.field.opponent_distance_to_ball)
                if kick == "PENALTYKICK":
                    self.check_penalty_kick_positions()
                self.game_controller_send(f'{kick}:{secondary_state_info[0]}:EXECUTE')
                self.logger.info(f'Execute {GAME_INTERRUPTIONS[kick]}.')
                self.game.interruption_seconds = self.game.state.seconds_remaining
                if self.game.interruption_seconds == 0:
                    self.game.interruption_seconds = None
        elif secondary_state not in ['STATE_NORMAL', 'STATE_OVERTIME', 'STATE_PENALTYSHOOT']:
            print(f'GameController {self.game.state.game_state}:{secondary_state}: {secondary_state_info}')
        self.update_penalized()
        if previous_state != self.game.state.game_state or \
                previous_sec_state != new_sec_state or previous_sec_phase != new_sec_phase or \
                previous_secondary_seconds_remaining != self.game.state.secondary_seconds_remaining or \
                self.game.state.seconds_remaining <= 0:
            self.display.update()

    def game_controller_send(self, message):
        if message[:6] == 'STATE:' or message[:6] == 'SCORE:' or message == 'DROPPEDBALL':
            # we don't want to send twice the same STATE or SCORE message
            if self.game_controller_last_sent_message == message:
                return False
            self.game_controller_last_sent_message = message
            if message[6:] in ['READY', 'SET']:
                self.game.wait_for_state = message[6:]
            elif message[6:] == 'PLAY':
                self.game.wait_for_state = 'PLAYING'
            elif (message[:6] == 'SCORE:' or
                  message == 'DROPPEDBALL'):
                self.game.wait_for_state = 'FINISHED' if self.game.penalty_shootout else 'READY'
            elif message[6:] == "PENALTY-SHOOTOUT":
                self.game.wait_for_state = 'INITIAL'
        if ':' in message:
            msg_start = message.split(':', 1)[0]
            if msg_start in GAME_INTERRUPTIONS:
                if 'ABORT' in message or 'EXECUTE' in message:
                    self.game.wait_for_sec_state = 'NORMAL'
                    self.game.wait_for_sec_phase = 0
                else:
                    self.game.wait_for_sec_state = msg_start
                    if 'READY' in message:
                        self.game.wait_for_sec_phase = 1
                    elif 'PREPARE' in message:
                        self.game.wait_for_sec_phase = 2
                    else:
                        self.game.wait_for_sec_phase = 0
                self.logger.info(f"Waiting for secondary state: {self.game.wait_for_sec_state}:{self.game.wait_for_sec_phase}")
        self.game_controller_send_id += 1
        if message[:6] != 'CLOCK:':
            self.logger.info(f'Sending {self.game_controller_send_id}:{message} to GameController.')
        message = f'{self.game_controller_send_id}:{message}\n'
        self.game_controller_socket.sendall(message.encode('ascii'))
        # self.logger.info(f'sending {message.strip()} to GameController')
        self.game_controller_send_unanswered[self.game_controller_send_id] = message.strip()
        answered = False
        sent_id = self.game_controller_send_id
        while True:
            try:
                answers = self.game_controller_socket.recv(1024).decode('ascii').split('\n')
                for answer in answers:
                    if answer == '':
                        continue
                    try:
                        id, result = answer.split(':')
                        if int(id) == sent_id:
                            answered = True
                    except ValueError:
                        self.logger.error(f'Cannot split {answer}')
                        self.clean_exit()
                    try:
                        answered_message = self.game_controller_send_unanswered[int(id)]
                        del self.game_controller_send_unanswered[int(id)]
                    except KeyError:
                        self.logger.error(f'Received acknowledgment message for unknown message: {id}')
                        self.clean_exit()
                    if result == 'OK':
                        continue
                    if result == 'INVALID':
                        self.logger.error(f'Received invalid answer from GameController for message {answered_message}.')
                        self.clean_exit()
                    elif result == 'ILLEGAL':
                        info_msg = f"Received illegal answer from GameController for message {answered_message}."
                        if "YELLOW" in message:
                            self.logger.warning(info_msg)
                        else:
                            self.logger.error(info_msg)
                            self.clean_exit()
                    else:
                        self.logger.error(f'Received unknown answer from GameController: {answer}.')
                        self.clean_exit()
            except BlockingIOError:
                if answered or ':CLOCK:' in message:
                    break
                else:  # keep sending CLOCK messages to keep the GameController happy
                    self.logger.info(f'Waiting for GameController to answer to {message.strip()}.')
                    time.sleep(0.2)
                    self.game_controller_send_id += 1
                    clock_message = f'{self.game_controller_send_id}:CLOCK:{self.sim_time.get_ms()}\n'
                    self.game_controller_socket.sendall(clock_message.encode('ascii'))
                    self.game_controller_send_unanswered[self.game_controller_send_id] = clock_message.strip()
        # We are waiting for a specific update from the GC before testing anything else
        while self.game.wait_for_state is not None or self.game.wait_for_sec_state is not None \
                or self.game.wait_for_sec_phase is not None:
            self.game_controller_receive()

        return True

    def append_solid(self, solid, active_tag=None):  # we list only the hands and feet
        solids = []
        tagged_solids = dict()
        name_field = solid.getField('name')
        if name_field:
            name = name_field.getSFString()
            tag_start = name.rfind('[')
            tag_end = name.rfind(']')
            if tag_start != -1 and tag_end != -1:
                active_tag = name[tag_start+1:tag_end]
            if name.endswith("[hand]") or name.endswith("[foot]"):
                solids.append(solid)
            if active_tag is not None:
                tagged_solids[name] = active_tag
        children = solid.getProtoField('children') if solid.isProto() else solid.getField('children')
        for i in range(children.getCount()):
            child = children.getMFNode(i)
            if child.getType() in [Node.ROBOT, Node.SOLID, Node.GROUP, Node.TRANSFORM,
                                   Node.ACCELEROMETER, Node.CAMERA, Node.GYRO, Node.TOUCH_SENSOR]:
                s, ts = self.append_solid(child, active_tag)
                solids.extend(s)
                tagged_solids.update(ts)
                continue
            if child.getType() in [Node.HINGE_JOINT, Node.HINGE_2_JOINT, Node.SLIDER_JOINT, Node.BALL_JOINT]:
                endPoint = child.getProtoField('endPoint') if child.isProto() else child.getField('endPoint')
                solid = endPoint.getSFNode()
                if solid.getType() == Node.NO_NODE or solid.getType() == Node.SOLID_REFERENCE:
                    continue
                s, ts = self.append_solid(solid, None)  # active tag is reset after a joint
                solids.extend(s)
                tagged_solids.update(ts)
        return solids, tagged_solids

    def list_player_solids(self, player, color, number):
        robot = player['robot']
        # Tagged solids: Keys: name of solid, Values: name of tag
        player['solids'], player['tagged_solids'] = self.append_solid(robot)
        if len(player['solids']) != 4:
            self.logger.info(f"Tagged solids: {player['tagged_solids']}")
            self.logger.error(f'{color} player {number}: invalid number of [hand]+[foot], '
                              f'received {len(player["solids"])}, expected 4.')
            self.clean_exit()

    def list_team_solids(self, team):
        for number in team.players:
            self.list_player_solids(team.players[number], team.color, number)

    def list_solids(self):
        self.list_team_solids(self.red_team)
        self.list_team_solids(self.blue_team)

    def show_polygon(self, vertices):
        polygon = self.supervisor.getFromDef('POLYGON')
        if polygon:
            polygon.remove()
        material = 'Material { diffuseColor 1 1 0 }'
        appearance = f'Appearance {{ material {material} }}'
        point = 'point ['
        for vertex in vertices:
            point += ' ' + str(vertex[0]) + ' ' + str(vertex[1]) + f' {self.field.turf_depth + 0.001},'  # 1 mm above turf
        point = point[:-1]
        point += ' ]'
        coord = f'Coordinate {{ {point} }}'
        coordIndex = '['
        for i in range(len(vertices)):
            coordIndex += ' ' + str(i)
        coordIndex += ' -1 ]'
        geometry = f'IndexedFaceSet {{ coord {coord} coordIndex {coordIndex} }}'
        shape = f'DEF POLYGON Shape {{ appearance {appearance} geometry {geometry} castShadows FALSE isPickable FALSE }}'
        children = self.supervisor.getRoot().getField('children')
        children.importMFNodeFromString(-1, shape)

    def update_team_ball_holding(self, team):
        players_close_to_the_ball = []
        numbers = []
        goalkeeper_number = None
        for number, player in team.players.items():
            d = distance2(player['position'], self.game.ball_position)
            if d <= self.field.ball_vincity:
                if self.is_goalkeeper(team, number):
                    goalkeeper_number = number
                players_close_to_the_ball.append(player)
                numbers.append(number)

        goalkeeper_hold_ball = False
        if goalkeeper_number is not None:  # goalkeeper is in vincity of ball
            goalkeeper = team.players[goalkeeper_number]
            points = np.empty([4, 2])
            aabb = None
            i = 0
            for solid in goalkeeper['solids']:
                position = solid.getPosition()
                aabb = update_aabb(aabb, position)
                points[i] = [position[0], position[1]]
                i += 1
            # check for collision between AABB of goalkeeper and ball
            if aabb_circle_collision(aabb, self.game.ball_position[0], self.game.ball_position[1],
                                     self.game.ball_radius * self.config.BALL_HOLDING_RATIO):
                hull = ConvexHull(points)
                hull_vertices = np.take(points, hull.vertices, 0)
                goalkeeper_hold_ball = polygon_circle_collision(hull_vertices, self.game.ball_position,
                                                                self.game.ball_radius * self.config.BALL_HOLDING_RATIO)

        n = len(players_close_to_the_ball)
        hold_ball = False
        if n > 0:
            aabb = None
            points = np.empty([4 * n, 2])
            i = 0
            for player in players_close_to_the_ball:
                for solid in player['solids']:
                    position = solid.getPosition()
                    aabb = update_aabb(aabb, position)
                    points[i] = [position[0], position[1]]
                    i += 1
            # check for collision between AABB of players and ball
            if aabb_circle_collision(aabb, self.game.ball_position[0], self.game.ball_position[1],
                                     self.game.ball_radius * self.config.BALL_HOLDING_RATIO):
                hull = ConvexHull(points)
                hull_vertices = np.take(points, hull.vertices, 0)
                hold_ball = polygon_circle_collision(hull_vertices, self.game.ball_position,
                                                     self.game.ball_radius * self.config.BALL_HOLDING_RATIO)
        players_holding_time_window = team.players_holding_time_window
        index = int(self.sim_time.get_ms() / self.time_step) % len(players_holding_time_window)
        players_holding_time_window[index] = hold_ball

        goalkeeper_holding_time_window = team.goalkeeper_holding_time_window
        index = int(self.sim_time.get_ms() / self.time_step) % len(goalkeeper_holding_time_window)
        goalkeeper_holding_time_window[index] = goalkeeper_hold_ball

        color = team.color
        if hasattr(team, 'hold_ball'):
            if not hold_ball:
                delay = int((self.sim_time.get_ms() - team.hold_ball) / 100) / 10
                self.logger.info(f'{color.capitalize()} team released the ball after {delay} seconds.')
                del team.hold_ball
        elif hold_ball:
            team.hold_ball = self.sim_time.get_ms()
            self.logger.info(f'{color.capitalize()} team ({numbers}) is holding the ball.')

        if hasattr(team, 'goalkeeper_hold_ball'):
            if not goalkeeper_hold_ball:
                delay = int((self.sim_time.get_ms() - team.goalkeeper_hold_ball) / 100) / 10
                self.logger.info(f'{color.capitalize()} goalkeeper released the ball after {delay} seconds.')
                del team.goalkeeper_hold_ball
        elif goalkeeper_hold_ball:
            team.goalkeeper_hold_ball = self.sim_time.get_ms()
            self.logger.info(f'{color.capitalize()} goalkeeper is holding the ball.')

    def update_ball_holding(self):
        self.update_team_ball_holding(self.red_team)
        self.update_team_ball_holding(self.blue_team)

    def update_team_contacts(self, team):
        early_game_interruption = self.is_early_game_interruption()
        color = team.color
        for number, player in team.players.items():
            robot = player['robot']
            if robot is None:
                continue
            l1 = len(player['velocity_buffer'])     # number of iterations
            l2 = len(player['velocity_buffer'][0])  # should be 6 (velocity vector size)
            player['velocity_buffer'][int(self.sim_time.get_ms() / self.time_step) % l1] = robot.getVelocity()
            sum = [0] * l2
            for v in player['velocity_buffer']:
                for i in range(l2):
                    sum[i] += v[i]
            player['velocity'] = [s / l1 for s in sum]
            contact_points = robot.getContactPoints(True)
            n = len(contact_points)
            player['contact_points'] = []
            if n == 0:  # robot is asleep
                player['asleep'] = True
                continue
            player['asleep'] = False
            player['position'] = robot.getCenterOfMass()
            # if less then 3 contact points, the contacts do not include contacts with the ground,
            # so don't update the following value based on ground collisions
            if n >= 3:
                player['outside_circle'] = True        # true if fully outside the center cicle
                player['outside_field'] = True         # true if fully outside the field
                player['inside_field'] = True          # true if fully inside the field
                player['on_outer_line'] = False        # true if robot is partially on the line surrounding the field
                player['inside_own_side'] = True       # true if fully inside its own side (half field side)
                player['outside_goal_area'] = True     # true if fully outside of any goal area
                player['outside_penalty_area'] = True  # true if fully outside of any penalty area
                outside_turf = True                    # true if fully outside turf
                fallen = False
            else:
                outside_turf = False
                fallen = True
            for i in range(n):
                point = contact_points[i].point
                node = self.supervisor.getFromId(contact_points[i].node_id)
                if not node:
                    continue
                name_field = node.getField('name')
                member = 'unknown body part'
                if name_field:
                    name = name_field.getSFString()
                    if name in player['tagged_solids']:
                        member = player['tagged_solids'][name]
                if point[2] > self.field.turf_depth:  # not a contact with the ground
                    if not early_game_interruption and point in self.ball.contact_points:  # ball contact
                        if member in ['arm', 'hand']:
                            player['ball_handling_last'] = self.sim_time.get_ms()
                            if player['ball_handling_start'] is None:
                                player['ball_handling_start'] = self.sim_time.get_ms()
                                self.logger.info(f'Ball touched the {member} of {color} player {number}.')
                            if (self.game.throw_in and
                                    self.game.ball_position[2] >
                                    self.field.turf_depth + self.game.ball_radius + self.config.BALL_LIFT_THRESHOLD):
                                self.game.throw_in_ball_was_lifted = True
                        else:  # the ball was touched by another part of the robot
                            # if the ball was hit by any player, we consider the throw-in (if any) complete
                            self.game.throw_in = False
                        if self.game.ball_first_touch_time == 0:
                            self.game.ball_first_touch_time = self.sim_time.get_ms()
                        self.game.ball_last_touch_time = self.sim_time.get_ms()
                        if self.game.penalty_shootout_count >= 10:  # extended penalty shootout
                            self.game.penalty_shootout_time_to_touch_ball[self.game.penalty_shootout_count - 10] = \
                                60 - self.game.state.seconds_remaining
                        if self.game.ball_last_touch_team != color or self.game.ball_last_touch_player_number != int(number):
                            self.game.set_ball_touched(color, int(number))
                            self.game.ball_last_touch_time_for_display = self.sim_time.get_ms()
                            action = 'kicked' if self.game.kicking_player_number is None else 'touched'
                            self.logger.info(f'Ball {action} by {color} player {number}.')
                            if self.game.kicking_player_number is None:
                                self.game.kicking_player_number = int(number)
                        elif self.sim_time.get_ms() - self.game.ball_last_touch_time_for_display >= 1000:
                            # dont produce too many touched messages
                            self.game.ball_last_touch_time_for_display = self.sim_time.get_ms()
                            self.logger.info(f'Ball touched again by {color} player {number}.')
                        step = self.game.state.secondary_state_info[1]
                        if step != 0 and self.game.state.secondary_state[6:] in GAME_INTERRUPTIONS:
                            self.game_interruption_touched(team, number)
                        continue
                    # the robot touched something else than the ball or the ground
                    player['contact_points'].append(point)  # this list will be checked later for robot-robot collisions
                    continue
                if distance2(point, [0, 0]) < self.field.circle_radius:
                    player['outside_circle'] = False
                if self.field.point_inside(point, include_turf=True):
                    outside_turf = False
                if self.field.point_inside(point):
                    player['outside_field'] = False
                    if abs(point[0]) > self.field.size_x - self.field.penalty_area_length and \
                            abs(point[1]) < self.field.penalty_area_width / 2:
                        player['outside_penalty_area'] = False
                        if abs(point[0]) > self.field.size_x - self.field.goal_area_length and \
                                abs(point[1]) < self.field.goal_area_width / 2:
                            player['outside_goal_area'] = False
                    if not self.field.point_inside(point, include_turf=False, include_border_line=False):
                        player['on_outer_line'] = True
                else:
                    player['inside_field'] = False
                if self.game.side_left == (self.game.red.id if color == 'red' else self.game.blue.id):
                    if point[0] > -self.field.line_half_width:
                        player['inside_own_side'] = False
                else:
                    if point[0] < self.field.line_half_width:
                        player['inside_own_side'] = False
                # check if the robot has fallen
                if member == 'foot':
                    continue
                fallen = True
                if 'fallen' in player:  # was already down
                    continue
                self.logger.info(f'{color.capitalize()} player {number} has fallen down.')
                player['fallen'] = self.sim_time.get_ms()
            if not player['on_outer_line']:
                player['on_outer_line'] = not (player['inside_field'] or player['outside_field'])
            if not fallen and 'fallen' in player:  # the robot has recovered
                delay = (int((self.sim_time.get_ms() - player['fallen']) / 100)) / 10
                self.logger.info(f'{color.capitalize()} player {number} just recovered after {delay} seconds.')
                del player['fallen']
            if outside_turf:
                if player['left_turf_time'] is None:
                    player['left_turf_time'] = self.sim_time.get_ms()
            else:
                player['left_turf_time'] = None

    def update_ball_contacts(self):
        self.ball.contact_points = []
        new_contact_points = self.ball.getContactPoints()
        for contact in new_contact_points:
            point = contact.point
            if point[2] <= self.field.turf_depth:  # contact with the ground
                continue
            self.ball.contact_points.append(point)
            break

    def update_contacts(self):
        """Only updates the contact of objects which are not asleep"""
        self.update_ball_contacts()
        self.update_team_contacts(self.red_team)
        self.update_team_contacts(self.blue_team)

    def find_robot_contact(self, team, point):
        for number, player in team.players.items():
            if point in player['contact_points']:
                return number
        return None

    def update_team_robot_contacts(self, team):
        for number, player in team.players.items():
            contact_points = player['contact_points']
            if len(contact_points) == 0:
                continue
            opponent_team = self.red_team if team == self.blue_team else self.blue_team
            for point in contact_points:
                opponent_number = self.find_robot_contact(opponent_team, point)
                if opponent_number is not None:
                    if team == self.red_team:
                        red_number = number
                        blue_number = opponent_number
                    else:
                        red_number = opponent_number
                        blue_number = number
                    fcm = self.forceful_contact_matrix
                    if (not fcm.contact(red_number, blue_number, self.sim_time.get_ms() - self.time_step) and
                            not fcm.contact(red_number, blue_number, self.sim_time.get_ms())):
                        self.logger.info(f'{self.sim_time.get_ms()}: contact between {team.color} player {number} and '
                                         f'{opponent_team.color} player {opponent_number}.')
                    fcm.set_contact(red_number, blue_number, self.sim_time.get_ms())

    def update_robot_contacts(self):
        self.forceful_contact_matrix.clear(self.sim_time.get_ms())
        self.update_team_robot_contacts(self.red_team)
        self.update_team_robot_contacts(self.blue_team)

    def update_histories(self):
        for team in [self.red_team, self.blue_team]:
            for number, player in team.players.items():
                # Remove old ball_distances
                if len(player['history']) > 0 \
                        and (self.sim_time.get_ms() - player['history'][0][0] >
                             self.config.INACTIVE_GOALKEEPER_TIMEOUT * 1000):
                    player['history'].pop(0)
                # If enough time has elapsed, add an entry
                if len(player['history']) == 0 or \
                        (self.sim_time.get_ms() - player['history'][-1][0]) > self.config.BALL_DIST_PERIOD * 1000:
                    ball_dist = distance2(player['position'], self.game.ball_position)
                    own_goal_area = player['inside_own_side'] and not player['outside_goal_area']
                    entry = (self.sim_time.get_ms(), {"ball_distance": ball_dist, "own_goal_area": own_goal_area})
                    player['history'].append(entry)

    def update_team_penalized(self, team):
        color = team.color
        index = self.team_index(color)
        for number, player in team.players.items():
            if player['robot'] is None:
                continue
            p = self.game.state.teams[index].players[int(number) - 1]
            if p.number_of_red_cards > 0:
                # sending red card robot far away from the field
                t = copy.deepcopy(player['reentryStartingPose']['translation'])
                t[0] = 50
                t[1] = (10 + int(number)) * (1 if color == 'red' else -1)
                self.reset_player(color, number, 'reentryStartingPose', t)
                customData = player['robot'].getField('customData')
                customData.setSFString('red_card')  # disable all devices of the robot
                player['penalized'] = 'red_card'
                # FIXME: unfortunately, player['robot'].remove() crashes webots
                # Once this is fixed, we should remove the robot, which seems to be a better solution
                # than moving it away from the field
                player['robot'] = None
                self.logger.info(f'Sending {color} player {number} to {t}. (team_index: {index})')
                if 'stabilize' in player:
                    del player['stabilize']
                player['outside_field'] = True
            elif 'enable_actuators_at' in player:
                timing_ok = self.sim_time.get_ms() >= player['enable_actuators_at']
                penalty_ok = 'penalized' not in player or p.penalty == 0
                if timing_ok and penalty_ok:
                    customData = player['robot'].getField('customData')
                    self.logger.info(f'Enabling actuators of {color} player {number}.')
                    customData.setSFString('')
                    del player['enable_actuators_at']
                    if 'penalized' in player:
                        del player['penalized']

    def update_penalized(self):
        self.update_team_penalized(self.red_team)
        self.update_team_penalized(self.blue_team)

    def already_penalized(self, player):
        return 'penalized' in player

    def send_penalty(self, player, penalty, reason, log=None):
        if 'yellow_card' not in player and self.already_penalized(player):
            return
        player['penalty'] = penalty
        player['penalty_reason'] = reason
        if log is not None:
            self.logger.info(log)

    def forceful_contact_foul(self, team, number, opponent_team, opponent_number, distance_to_ball, message):
        player = team.players[number]
        if player['outside_penalty_area']:
            area = 'outside penalty area'
        else:
            area = 'inside penalty area'
        self.logger.info(f'{team.color.capitalize()} player {number} committed a forceful contact foul on '
                         f'{opponent_team.color} player {opponent_number} ({message}) {area}.')
        self.forceful_contact_matrix.clear_all()
        opponent = opponent_team.players[opponent_number]
        immunity_timeout = self.sim_time.get_ms() + self.config.FOUL_PENALTY_IMMUNITY * 1000
        opponent['penalty_immunity'] = immunity_timeout
        player['penalty_immunity'] = immunity_timeout
        freekick_team_id = self.game.blue.id if team.color == "red" else self.game.red.id
        foul_far_from_ball = distance_to_ball > self.config.FOUL_BALL_DISTANCE
        if self.game.penalty_shootout and self.is_penalty_kicker(team, number):
            self.logger.info(f'Kicker {team.color} {number} performed forceful contact during penaltykick -> end of trial')
            self.next_penalty_shootout()
        self.logger.info(f"Ball in play: {self.game.in_play}, foul far from ball: {foul_far_from_ball}")
        if foul_far_from_ball or not self.game.in_play or self.game.penalty_shootout:
            self.send_penalty(player, 'PHYSICAL_CONTACT', 'forceful contact foul')
        else:
            offence_location = team.players[number]['position']
            if self.is_game_interruption():
                self.game_interruption_forceful_contact(team, player)
            else:
                self.interruption('FREEKICK', freekick_team_id, offence_location)

    def goalkeeper_inside_own_goal_area(self, team, number):
        if self.is_goalkeeper(team, number):
            goalkeeper = team.players[number]
            if not goalkeeper['outside_goal_area'] and goalkeeper['inside_own_side']:
                return True
        return False

    def moves_to_ball(self, player, velocity, velocity_squared):
        if velocity_squared < self.config.FOUL_SPEED_THRESHOLD * self.config.FOUL_SPEED_THRESHOLD:
            return True
        rx = self.game.ball_position[0] - player['position'][0]
        ry = self.game.ball_position[1] - player['position'][1]
        vx = velocity[0]
        vy = velocity[1]
        angle = math.acos((rx * vx + ry * vy) / (math.sqrt(rx * rx + ry * ry) * math.sqrt(vx * vx + vy * vy)))
        return angle < self.config.FOUL_DIRECTION_THRESHOLD

    def readable_number_list(self, number_list, width=5, nb_digits=2):
        fmt = f"%{width}.{nb_digits}f"
        return f"[{' '.join([fmt % elem for elem in number_list])}]"

    def check_team_forceful_contacts(self, team, number, opponent_team, opponent_number):
        p1 = team.players[number]
        if 'penalty_immunity' in p1:
            if p1['penalty_immunity'] < self.sim_time.get_ms():
                del p1['penalty_immunity']
            else:
                return
        p2 = opponent_team.players[opponent_number]
        d1 = distance2(p1['position'], self.game.ball_position)
        d2 = distance2(p2['position'], self.game.ball_position)
        p1_str = f"{team.color} {number}"
        p2_str = f"{opponent_team.color} {opponent_number}"
        debug_messages = [f"Check if {p1_str} is performing a foul on {p2_str}",
                          f"{p1_str:6s}: at {self.readable_number_list(p1['position'])}, dist to ball: {d1:.2f}",
                          f"{p2_str:6s}: at {self.readable_number_list(p2['position'])}, dist to ball: {d2:.2f}"]
        if self.goalkeeper_inside_own_goal_area(opponent_team, opponent_number):
            self.logger.info(debug_messages)
            self.forceful_contact_foul(team, number, opponent_team, opponent_number, d1, 'goalkeeper')
            return True
        if team == self.red_team:
            red_number = number
            blue_number = opponent_number
        else:
            red_number = opponent_number
            blue_number = number
        if self.forceful_contact_matrix.long_collision(red_number, blue_number):
            if d1 < self.config.FOUL_VINCITY_DISTANCE and d1 - d2 > self.config.FOUL_DISTANCE_THRESHOLD:
                collision_time = self.forceful_contact_matrix.get_collision_time(red_number, blue_number)
                debug_messages.append(f"Pushing time: {collision_time} > {self.config.FOUL_PUSHING_TIME} "
                                      f"over the last {self.config.FOUL_PUSHING_PERIOD}")
                debug_messages.append(f"Difference of distance: {d1-d2} > {self.config.FOUL_DISTANCE_THRESHOLD}")
                self.logger.info(debug_messages)
                self.forceful_contact_foul(team, number, opponent_team, opponent_number, d1, 'long_collision')
                return True
        v1 = p1['velocity']
        v2 = p2['velocity']
        v1_squared = v1[0] * v1[0] + v1[1] * v1[1]
        v2_squared = v2[0] * v2[0] + v2[1] * v2[1]
        if not v1_squared > self.config.FOUL_SPEED_THRESHOLD * self.config.FOUL_SPEED_THRESHOLD:
            return False
        debug_messages.append(f"{p1_str:6s}: velocity: {self.readable_number_list(v1[:3])}, "
                              f"speed: {math.sqrt(v1_squared):.2f}")
        debug_messages.append(f"{p2_str:6s}: velocity: {self.readable_number_list(v2[:3])}, "
                              f"speed: {math.sqrt(v2_squared):.2f}")
        if d1 < self.config.FOUL_VINCITY_DISTANCE:
            debug_messages.append(f"{p1_str} is close to the ball ({d1:.2f} < {self.config.FOUL_VINCITY_DISTANCE})")
            if self.moves_to_ball(p2, v2, v2_squared):
                if not self.moves_to_ball(p1, v1, v1_squared):
                    self.logger.info(debug_messages)
                    self.forceful_contact_foul(team, number, opponent_team, opponent_number, d1,
                                               'opponent moving towards the ball, charge')
                    return True
                if d1 - d2 > self.config.FOUL_DISTANCE_THRESHOLD:
                    debug_messages.append(f"{p2_str} is significantly closer to the ball than {p1_str}: "
                                          f"({d1-d2:.2f} < {self.config.FOUL_DISTANCE_THRESHOLD})")
                    self.logger.info(debug_messages)
                    self.forceful_contact_foul(team, number, opponent_team, opponent_number, d1,
                                               'opponent moving towards the ball, charge from behind')
                    return True
        elif math.sqrt(v1_squared) - math.sqrt(v2_squared) > self.config.FOUL_SPEED_THRESHOLD:
            self.logger.info(debug_messages)
            self.forceful_contact_foul(team, number, opponent_team, opponent_number, d1,
                                       f'violent collision: {math.sqrt(v1_squared)} - {math.sqrt(v2_squared)} '
                                       f'> {self.config.FOUL_SPEED_THRESHOLD}')
            return True
        return False

    def check_forceful_contacts(self):
        self.update_robot_contacts()
        fcm = self.forceful_contact_matrix
        for red_number in self.red_team.players:
            for blue_number in self.blue_team.players:
                if not fcm.contact(red_number, blue_number, self.sim_time.get_ms()):
                    continue  # no contact
                if self.check_team_forceful_contacts(self.red_team, red_number, self.blue_team, blue_number):
                    continue
                if self.check_team_forceful_contacts(self.blue_team, blue_number, self.red_team, red_number):
                    continue

    def check_team_ball_holding(self, team):
        color = team.color
        players_holding_time_window = team.players_holding_time_window
        size = len(players_holding_time_window)
        sum = 0
        for i in range(size):
            if players_holding_time_window[i]:
                sum += 1
        if sum > size / 2:
            self.logger.info(f'{color.capitalize()} team has held the ball for too long.')
            return True
        return False

    def check_ball_holding(self):  # return the team id which gets a free kick in case of ball holding from the other team
        if self.check_team_ball_holding(self.red_team):
            return self.game.blue.id
        if self.check_team_ball_holding(self.blue_team):
            return self.game.red.id
        return None

    def reset_ball_handling(self, player):
        player['ball_handling_start'] = None
        player['ball_handling_last'] = None

    def check_team_ball_handling(self, team):
        for number, player in team.players.items():
            if player['ball_handling_start'] is None:  # ball is not handled
                continue
            duration = (player['ball_handling_last'] - player['ball_handling_start']) / 1000
            color = team.color
            if self.sim_time.get_ms() - player['ball_handling_last'] >= 1000:  # ball was released one second ago or more
                self.reset_ball_handling(player)
                if self.game.throw_in:
                    was_throw_in = self.game.throw_in
                    self.game.throw_in = False
                    if was_throw_in and not self.game.throw_in_ball_was_lifted:
                        sentence = 'throw-in with the hand or arm while the ball was not lifted by at least ' + \
                                   f'{self.config.BALL_LIFT_THRESHOLD * 100} cm'
                        self.send_penalty(player, 'BALL_MANIPULATION', sentence,
                                          f'{color.capitalize()} player {number} {sentence}.')
                        continue
            goalkeeper = self.goalkeeper_inside_own_goal_area(team, number)
            if not goalkeeper and not self.game.throw_in:
                self.reset_ball_handling(player)
                sentence = 'touched the ball with its hand or arm'
                if self.game.penalty_shootout and self.is_penalty_kicker(team, number):
                    self.logger.info("Kicker {color.capitalize()} {number} has fallen down and not recovered -> end of trial")
                else:
                    self.send_penalty(player, 'BALL_MANIPULATION', sentence,
                                      f'{color.capitalize()} player {number} {sentence}.')
                continue
            ball_on_the_ground = self.game.ball_position[2] <= self.field.turf_depth + self.game.ball_radius
            if self.game.throw_in:
                # a player can handle the ball for 10 seconds for throw-in, no more
                if duration >= self.config.BALL_HANDLING_TIMEOUT:
                    self.reset_ball_handling(player)
                    sentence = f'touched the ball with its hand or arm for more than {self.config.BALL_HANDLING_TIMEOUT} ' + \
                        'seconds during throw-in'
                    self.send_penalty(player, 'BALL_MANIPULATION', sentence,
                                      f'{color.capitalize()} player {number} {sentence}.')
                    continue
            # goalkeeper case
            if duration >= self.config.BALL_HANDLING_TIMEOUT:
                # ball handled by goalkeeper for 1 second or more
                if (self.game.ball_previous_touch_team == self.game.ball_last_touch_team and
                        self.sim_time.get_ms() - player['ball_handling_start'] >= 1000):
                    self.reset_ball_handling(player)
                    sentence = 'handled the ball after it received it from teammate' \
                        if self.game.ball_last_touch_player_number == int(number) else 'released the ball and retook it'
                    self.logger.info(f'{color.capitalize()} goalkeeper {number} {sentence}.')
                    return True  # a freekick will be awarded
                self.reset_ball_handling(player)
                self.logger.info(f'{color.capitalize()} goalkeeper {number} handled the ball up for more than '
                                 f'{self.config.BALL_HANDLING_TIMEOUT} seconds.')
                return True  # a freekick will be awarded
            if ball_on_the_ground and duration >= self.config.GOALKEEPER_GROUND_BALL_HANDLING:
                self.reset_ball_handling(player)
                self.logger.info(f'{color.capitalize()} goalkeeper {number} handled the ball on the ground for more than '
                                 f'{self.config.GOALKEEPER_GROUND_BALL_HANDLING} seconds.')
                return True  # a freekick will be awarded
        return False  # not free kick awarded

    def check_ball_handling(self):  # return the team id which gets a free kick in case of wrong ball handling by a goalkeeper
        if self.check_team_ball_handling(self.red_team):
            return self.game.blue.id
        if self.check_team_ball_handling(self.blue_team):
            return self.game.red.id
        return None

    def check_team_fallen(self, team):
        color = team.color
        penalty = False
        for number, player in team.players.items():
            if self.already_penalized(player):
                continue
            if 'fallen' in player and self.sim_time.get_ms() - player['fallen'] > 1000 * self.config.FALLEN_TIMEOUT:
                del player['fallen']
                if self.game.penalty_shootout and self.is_penalty_kicker(team, number):
                    self.logger.info("Kicker {color.capitalize()} {number} has fallen down and not recovered -> end of trial")
                    self.next_penalty_shootout()
                else:
                    self.send_penalty(player, 'INCAPABLE', 'fallen down',
                                      f'{color.capitalize()} player {number} has fallen down and not recovered in 20 seconds.')
                penalty = True
        return penalty

    def check_fallen(self):
        red = self.check_team_fallen(self.red_team)
        blue = self.check_team_fallen(self.blue_team)
        return red or blue

    def check_team_inactive_goalkeeper(self, team):
        # Since there is only one goalkeeper, we can safely return once we have a 'proof' that the goalkeeper is not inactive
        for number, player in team.players.items():
            if not self.is_goalkeeper(team, number):
                continue
            if self.already_penalized(player) or len(player['history']) == 0:
                return
            # If player was out of his own goal area recently, it can't be considered as inactive
            if not all([e[1]["own_goal_area"] for e in player['history']]):
                return
            ball_distances = [e[1]["ball_distance"] for e in player['history']]
            if max(ball_distances) > self.config.INACTIVE_GOALKEEPER_DIST:
                return
            # In order to measure progress toward the ball, we just look if one of distance measured is significantly lower
            # than what happened previously. active_dist keeps track of the threshold below which we consider the player as
            # moving toward the ball.
            active_dist = 0
            for d in ball_distances:
                if d < active_dist:
                    return
                new_active_dist = d - self.config.INACTIVE_GOALKEEPER_PROGRESS
                if new_active_dist > active_dist:
                    active_dist = new_active_dist
            self.logger.info(f'Goalkeeper did not move toward the ball over the '
                             f'last {self.config.INACTIVE_GOALKEEPER_TIMEOUT} seconds')
            self.send_penalty(player, 'INCAPABLE', 'Inactive goalkeeper')

    def check_inactive_goalkeepers(self):
        self.check_team_inactive_goalkeeper(self.red_team)
        self.check_team_inactive_goalkeeper(self.blue_team)

    def check_team_away_from_ball(self, team, distance):
        for number, player in team.players.items():
            if self.already_penalized(player):
                continue
            d = distance2(player['position'], self.game.ball_position)
            if d < distance:
                color = team.color
                self.send_penalty(player, 'INCAPABLE', f'too close to ball: {d:.2f} m., should be at least {distance:.2f} m.' +
                                  f'{color.capitalize()} player {number} is too close to the ball: ' +
                                  f'({d:.2f} m., should be at least {distance:.2f} m.)')

    def check_penalty_kick_positions(self):
        for team in [self.red_team, self.blue_team]:
            defending_team = (team.color == 'red') ^ (self.game.state.secondary_state_info[0] == self.game.red.id)
            has_striker = False
            for number, player in team.players.items():
                if self.already_penalized(player):
                    continue
                player_x = player['position'][0]
                ahead_of_ball = self.game.ball_position[0] * player_x > 0 and abs(player_x) > self.field.penalty_mark_x
                near_ball = distance2(self.game.ball_position, player['position']) < self.field.opponent_distance_to_ball
                if ahead_of_ball:
                    if not defending_team:
                        # In rules version from June 21st 2021, there are no constraint on the position of the striker.
                        # Here, we assume that the striker is not allowed to be in front of the ball
                        self.send_penalty(player, 'INCAPABLE', "Field player of the attacking team in front of penalty mark")
                    elif self.is_goalkeeper(team, number):
                        on_goal_line_or_behind = (player['on_outer_line'] or player['outside_field']) and \
                                                 abs(player['position'][1]) <= self.config.GOAL_WIDTH
                        if not on_goal_line_or_behind:
                            self.send_penalty(player, 'INCAPABLE', "Defending goalkeeper between goal line and penalty mark")
                    else:
                        self.send_penalty(player, 'INCAPABLE',
                                          "Field player of the defending team in front of the penalty mark")
                elif near_ball:
                    if defending_team:
                        self.send_penalty(player, 'INCAPABLE', "Player of the defending team near ball (penaltykick)")
                    elif has_striker:
                        self.send_penalty(player, 'INCAPABLE', "Player of the attacking team has already a striker")
                    else:
                        has_striker = True
                        self.logger.info(f"Player {team.color} {number} is considered as striker")

    def check_team_start_position(self, team):
        penalty = False
        for number, player in team.players.items():
            if self.already_penalized(player):
                continue
            if not player['outside_field']:
                self.send_penalty(player, 'INCAPABLE', 'halfTimeStartingPose inside field')
                penalty = True
            elif not player['inside_own_side']:
                self.send_penalty(player, 'INCAPABLE', 'halfTimeStartingPose outside team side')
                penalty = True
        return penalty

    def check_start_position(self):
        red = self.check_team_start_position(self.red_team)
        blue = self.check_team_start_position(self.blue_team)
        return red or blue

    def check_team_kickoff_position(self, team):
        color = team.color
        team_id = self.game.red.id if color == 'red' else self.game.blue.id
        penalty = False
        for number, player in team.players.items():
            if self.already_penalized(player):
                continue
            if not player['inside_field']:
                self.send_penalty(player, 'INCAPABLE', 'outside of field at kick-off')
                penalty = True
            elif not player['inside_own_side']:
                self.send_penalty(player, 'INCAPABLE', 'outside team side at kick-off')
                penalty = True
            elif self.game.kickoff != team_id and not player['outside_circle']:
                self.send_penalty(player, 'INCAPABLE', 'inside center circle during opponent\'s kick-off')
                penalty = True
        return penalty

    def check_kickoff_position(self):
        red = self.check_team_kickoff_position(self.red_team)
        blue = self.check_team_kickoff_position(self.blue_team)
        return red or blue

    def check_team_dropped_ball_position(self, team):
        for number, player in team.players.items():
            if self.already_penalized(player):
                continue
            if not player['outside_circle']:
                self.send_penalty(player, 'INCAPABLE', 'inside center circle during dropped ball')
            elif not player['inside_own_side']:
                self.send_penalty(player, 'INCAPABLE', 'outside team side during dropped ball')

    def check_dropped_ball_position(self):
        self.check_team_dropped_ball_position(self.red_team)
        self.check_team_dropped_ball_position(self.blue_team)

    def check_team_outside_turf(self, team):
        color = team.color
        for number, player in team.players.items():
            if self.already_penalized(player):
                continue
            if player['left_turf_time'] is None:
                continue
            if self.sim_time.get_ms() - player['left_turf_time'] < self.config.OUTSIDE_TURF_TIMEOUT * 1000:
                continue
            if self.game.penalty_shootout and self.is_penalty_kicker(team, number):
                self.logger.info(f'Kicker {color.capitalize()} {number} left the field -> end of trial')
                self.next_penalty_shootout()
            else:
                self.send_penalty(player, 'INCAPABLE', f'left the field for more than '
                                  f'{self.config.OUTSIDE_TURF_TIMEOUT} seconds {color.capitalize()} '
                                  f'player {number} left the field for more than {self.config.OUTSIDE_TURF_TIMEOUT} seconds.')

    def check_outside_turf(self):
        self.check_team_outside_turf(self.red_team)
        self.check_team_outside_turf(self.blue_team)

    def check_team_penalized_in_field(self, team):
        color = team.color
        penalty = False
        for number, player in team.players.items():
            if 'penalized' not in player:
                continue  # skip non penalized players
            if player['penalized'] == 'red_card':
                continue  # skip red card players
            if player['outside_field']:
                continue
            player['yellow_card'] = True
            self.logger.info(f'Penalized {color} player {number} re-entered the field...')
            self.send_penalty(player, 'INCAPABLE', 'penalized player re-entered field',
                              f'Penalized {color} player {number} re-entered the field: shown yellow card.')
            penalty = True
        return penalty

    def check_penalized_in_field(self):
        red = self.check_team_penalized_in_field(self.red_team)
        blue = self.check_team_penalized_in_field(self.blue_team)
        return red or blue

    def check_circle_entrance(self, team):
        """
        Checks if a player from the opposing team has entered the center circle.
        If so, the player is sent to penalty.
        :param team: opposing team
        :return:
        """
        penalty = False
        for number, player in team.players.items():
            if self.already_penalized(player):
                continue
            if not player['outside_circle']:
                color = team.color
                self.send_penalty(player, 'INCAPABLE', 'entered circle during oppenent\'s kick-off',
                                  f'{color.capitalize()} player {number} entering circle during opponent\'s kick-off.')
                penalty = True
        return penalty

    def check_ball_must_kick(self, team):
        if self.game.ball_last_touch_team is None:
            return False  # nobody touched the ball
        if self.game.dropped_ball or self.game.ball_last_touch_team == self.game.ball_must_kick_team:
            return False  # no foul
        for number, player in team.players.items():
            if not self.game.ball_last_touch_player_number == int(number):
                continue
            if self.already_penalized(player):
                continue
            color = team.color
            self.send_penalty(player, 'INCAPABLE', 'non-kicking player touched ball not in play',
                              f'Non-kicking {color} player {number} touched ball not in play. Ball was touched by wrong team.')
            break
        return True

    def is_game_interruption(self):
        if not hasattr(self.game, "state"):
            return False
        return self.game.state.secondary_state[6:] in GAME_INTERRUPTIONS

    def is_early_game_interruption(self):
        """
        Return true if the active state is a game interruption and phase is 0.

        Note: During this step, robots are allowed to commit some infringements such as touching a ball that is not in play.
        """
        return self.is_game_interruption() and self.game.state.secondary_state_info[1] == 0

    def game_interruption_touched(self, team, number):
        """
        Applies the associated actions for when a robot touches the ball during step 1 and 2 of game interruptions

        1. If opponent touches the ball, robot receives a WARN and RETAKE is sent
        2. If team with game_interruption touched the ball, player receives a WARN and ABORT is sent
        """
        # Warnings only applies in step 1 and 2 of game interruptions
        team_id = self.game.red.id if team.color == 'red' else self.game.blue.id
        opponent = team_id != self.game.interruption_team
        if opponent:
            self.game.in_play = None
            self.game.ball_set_kick = True
            self.game.interruption_countdown = self.config.SIMULATED_TIME_INTERRUPTION_PHASE_0
            self.logger.info(f"Ball touched by opponent, retaking {GAME_INTERRUPTIONS[self.game.interruption]}")
            self.logger.info(f"Reset interruption_countdown to {self.game.interruption_countdown}")
            self.game_controller_send(f'{self.game.interruption}:{self.game.interruption_team}:RETAKE')
        else:
            self.game.in_play = self.sim_time.get_ms()
            self.logger.info(f"Ball touched before execute, aborting {GAME_INTERRUPTIONS[self.game.interruption]}")
            self.game_controller_send(f'{self.game.interruption}:{self.game.interruption_team}:ABORT')
        self.game_controller_send(f'CARD:{team_id}:{number}:WARN')

    def game_interruption_ball_holding(self, team):
        """
        Applies the associated actions for when a robot does ball holding duing an interruption

        1. If opponent commits ball holding, RETAKE is sent
        2. If team with game_interruption does ball holding game interruption continues
        """
        # Warnings only applies in step 1 and 2 of game interruptions
        opponent = team != self.game.interruption_team
        if opponent:
            self.game.in_play = None
            self.game.ball_set_kick = True
            self.game.interruption_countdown = self.config.SIMULATED_TIME_INTERRUPTION_PHASE_0
            self.logger.info(f"Ball holding by opponent team, retaking {GAME_INTERRUPTIONS[self.game.interruption]}")
            self.logger.info(f"Reset interruption_countdown to {self.game.interruption_countdown}")
            self.game_controller_send(f'{self.game.interruption}:{self.game.interruption_team}:RETAKE')

    def game_interruption_forceful_contact(self, team, player):
        """
        Applies the associated actions for when a robot does a forceful contact duing an interruption

        1. If opponent commits commits, RETAKE is sent, 30 second removal penalty for opponent
        2. If team with game_interruption does forceful game interruption continues
        """
        opponent = team != self.game.interruption_team
        if opponent:
            self.send_penalty(player, 'PHYSICAL_CONTACT', 'forceful contact foul')
            self.game.in_play = None
            self.game.ball_set_kick = True
            self.game.interruption_countdown = self.config.SIMULATED_TIME_INTERRUPTION_PHASE_0
            self.logger.info(f"Forceful contact by opponent team during interruption, retaking {GAME_INTERRUPTIONS[self.game.interruption]}")
            self.logger.info(f"Reset interruption_countdown to {self.game.interruption_countdown}")
            self.game_controller_send(f'{self.game.interruption}:{self.game.interruption_team}:RETAKE')

    def game_interruption_ball_handling(self, team):
        """
        Applies the associated actions for when a robot does ball handling duing an interruption

        1. Ball Handling is performed by any team during interruption -> retake
        """
        # TODO may result in endless loop if team always tries to pick up the ball
        self.game.in_play = None
        self.game.ball_set_kick = True
        self.game.interruption_countdown = self.config.SIMULATED_TIME_INTERRUPTION_PHASE_0
        self.logger.info(f"Ball handling during interruption by team {team}: {GAME_INTERRUPTIONS[self.game.interruption]}")
        self.logger.info(f"Reset interruption_countdown to {self.game.interruption_countdown}")
        self.game_controller_send(f'{self.game.interruption}:{self.game.interruption_team}:RETAKE')

    def get_first_available_spot(self, team_color, number, reentry_pos):
        """Return the first available spot to enter on one side of the field given the reentry_pos"""
        if not self.is_other_robot_near(team_color, number, reentry_pos, self.field.robot_radius):
            return reentry_pos
        preferred_dir = 1 if reentry_pos[1] > self.game.ball_position[1] else -1
        max_iterations = math.ceil(reentry_pos[1] / self.field.penalty_offset)
        basic_offset = np.array([self.field.penalty_offset, 0, 0])
        initial_pos = np.array(reentry_pos)
        for i in range(1, max_iterations):
            for direction in [preferred_dir, -preferred_dir]:
                current_pos = initial_pos + direction * i * basic_offset
                opposite_sides = current_pos[0] * initial_pos[0] < 0  # current_pos should be on the other side
                out_of_field = abs(current_pos[0]) > self.field.size_x
                if opposite_sides or out_of_field:
                    continue
                if not self.is_other_robot_near(team_color, number, current_pos, self.field.robot_radius):
                    return current_pos.tolist()
        return None

    def place_player_at_penalty(self, player, team, number):
        color = team.color
        t = copy.deepcopy(player['reentryStartingPose']['translation'])
        r = copy.deepcopy(player['reentryStartingPose']['rotation'])
        t[0] = self.field.penalty_mark_x if t[0] > 0 else -self.field.penalty_mark_x
        if (self.game.ball_position[1] > 0 and t[1] > 0) or (self.game.ball_position[1] < 0 and t[1] < 0):
            t[1] = -t[1]
            r = rotate_along_z(r)
        # check if position is already occupied by a penalized robot
        self.logger.info(f"placing player {color} {number} at {t}")
        pos = self.get_first_available_spot(color, number, t)
        self.logger.info(f"-> pos: {pos}")
        if pos is None:
            t[1] = -t[1]
            r = rotate_along_z(r)
            pos = self.get_first_available_spot(color, number, t)
            if pos is None:
                raise RuntimeError("No spot available, this should not be possible")
        self.reset_player(color, number, None, pos, r)

    def send_team_penalties(self, team):
        color = team.color
        for number, player in team.players.items():
            if 'yellow_card' in player:
                self.game_controller_send(f'CARD:{self.game.red.id if color == "red" else self.game.blue.id}:{number}:YELLOW')
                del player['yellow_card']
            if 'penalty' in player:
                penalty = player['penalty']
                reason = player['penalty_reason']
                del player['penalty']
                del player['penalty_reason']
                player['penalty_immunity'] = self.sim_time.get_ms() + self.config.FOUL_PENALTY_IMMUNITY * 1000
                team_id = self.game.red.id if color == 'red' else self.game.blue.id
                self.game_controller_send(f'PENALTY:{team_id}:{number}:{penalty}')
                self.place_player_at_penalty(player, team, number)
                player['penalized'] = self.config.REMOVAL_PENALTY_TIMEOUT
                # Once removed from the field, the robot will be in the air, therefore its status will not be updated.
                # Thus, we need to make sure it will not be considered in the air while falling
                player['outside_field'] = True
                self.logger.info(f'{penalty} penalty for {color} player {number}: {reason}.')

    def send_penalties(self):
        self.send_team_penalties(self.red_team)
        self.send_team_penalties(self.blue_team)

    def stabilize_team_robots(self, team):
        color = team.color
        for number, player in team.players.items():
            robot = player['robot']
            if robot is None:
                continue
            if 'stabilize' in player:
                if player['stabilize'] == 0:
                    self.logger.info(f'Stabilizing {color} player {number}')
                    robot.resetPhysics()
                    robot.getField('translation').setSFVec3f(player['stabilize_translation'])
                    robot.getField('rotation').setSFRotation(player['stabilize_rotation'])
                    del player['stabilize']
                else:
                    player['stabilize'] -= 1

    def stabilize_robots(self):
        self.stabilize_team_robots(self.red_team)
        self.stabilize_team_robots(self.blue_team)

    def flip_pose(self, pose):
        pose['translation'][0] = -pose['translation'][0]
        pose['rotation'][3] = math.pi - pose['rotation'][3]

    def flip_poses(self, player):
        self.flip_pose(player['halfTimeStartingPose'])
        self.flip_pose(player['reentryStartingPose'])
        self.flip_pose(player['shootoutStartingPose'])
        self.flip_pose(player['goalKeeperStartingPose'])

    def flip_sides(self):  # flip sides (no need to notify GameController, it does it automatically)
        self.game.side_left = self.game.red.id if self.game.side_left == self.game.blue.id else self.game.blue.id
        for team in [self.red_team, self.blue_team]:
            for number, player in team.players.items():
                self.flip_poses(player)
        self.display.update()

    def reset_player(self, color, number, pose, custom_t=None, custom_r=None):
        team = self.red_team if color == 'red' else self.blue_team
        player = team.players[number]
        robot = player['robot']
        if robot is None:
            return
        robot.loadState('__init__')
        self.list_player_solids(player, color, number)
        translation = robot.getField('translation')
        rotation = robot.getField('rotation')
        t = custom_t if custom_t else player[pose]['translation']
        r = custom_r if custom_r else player[pose]['rotation']
        translation.setSFVec3f(t)
        rotation.setSFRotation(r)
        robot.resetPhysics()
        player['stabilize'] = 5  # stabilize after 5 simulation steps
        player['stabilize_translation'] = t
        player['stabilize_rotation'] = r
        player['position'] = t
        self.logger.info(f'{color.capitalize()} player {number} reset to {pose}: ' +
                         f'translation ({t[0]} {t[1]} {t[2]}), rotation ({r[0]} {r[1]} {r[2]} {r[3]}).')
        self.logger.info(f'Disabling actuators of {color} player {number}.')
        robot.getField('customData').setSFString('penalized')
        player['enable_actuators_at'] = self.sim_time.get_ms() + int(self.config.DISABLE_ACTUATORS_MIN_DURATION * 1000)

    def reset_teams(self, pose):
        for number in self.red_team.players:
            self.reset_player('red', str(number), pose)
        for number in self.blue_team.players:
            self.reset_player('blue', str(number), pose)

    def is_goalkeeper(self, team, id):
        n = self.game.state.teams[0].team_number
        if (n == self.game.red.id and team == self.red_team) or (n == self.game.blue.id and team == self.blue_team):
            index = 0
        else:
            index = 1
        return self.game.state.teams[index].players[int(id) - 1].goalkeeper

    def get_penalty_attacking_team(self):
        first_team = self.game.penalty_shootout_count % 2 == 0
        if first_team == (self.game.kickoff == self.game.red.id):
            return self.red_team
        return self.blue_team

    def get_penalty_defending_team(self):
        first_team = self.game.penalty_shootout_count % 2 == 0
        if first_team == (self.game.kickoff != self.game.red.id):
            return self.red_team
        return self.blue_team

    def player_has_red_card(self, player):
        return 'penalized' in player and player['penalized'] == 'red_card'

    def is_penalty_kicker(self, team, id):
        for number, player in team.players.items():
            if self.player_has_red_card(player):
                continue
            return id == number

    def penalty_kicker_player(self):
        default = self.game.penalty_shootout_count % 2 == 0
        attacking_team = self.red_team if (self.game.kickoff == self.game.blue.id) ^ default else self.blue_team
        for number, player in attacking_team.players.items():
            if self.player_has_red_card(player):
                continue
            return player
        return None

    def get_penalty_shootout_msg(self):
        trial = self.game.penalty_shootout_count + 1
        name = "penalty shoot-out"
        if self.game.penalty_shootout_count >= 10:
            name = f"extended {name}"
            trial -= 10
        return f"{name} {trial}/10"

    def set_penalty_positions(self):
        self.logger.info(f"Setting positions for {self.get_penalty_shootout_msg()}")
        default = self.game.penalty_shootout_count % 2 == 0
        attacking_color = 'red' if (self.game.kickoff == self.game.blue.id) ^ default else 'blue'
        if attacking_color == 'red':
            defending_color = 'blue'
            attacking_team = self.red_team
            defending_team = self.blue_team
        else:
            defending_color = 'red'
            attacking_team = self.blue_team
            defending_team = self.red_team
        for number, player in attacking_team.players.items():
            if self.player_has_red_card(player):
                continue
            if self.is_penalty_kicker(attacking_team, number):
                self.reset_player(attacking_color, number, 'shootoutStartingPose')
            else:
                self.reset_player(attacking_color, number, 'halfTimeStartingPose')
        for number, player in defending_team.players.items():
            if self.player_has_red_card(player):
                continue
            if self.is_goalkeeper(defending_team, number) and self.game.penalty_shootout_count < 10:
                self.reset_player(defending_color, number, 'goalKeeperStartingPose')
                player['invalidGoalkeeperStart'] = None
            else:
                self.reset_player(defending_color, number, 'halfTimeStartingPose')
        x = -self.field.penalty_mark_x if (self.game.side_left == self.game.kickoff) ^ default else self.field.penalty_mark_x
        self.ball.resetPhysics()
        self.game.reset_ball_touched()
        self.game.set_penalty(attacking_team.color, x)

    def stop_penalty_shootout(self):
        self.logger.info(f"End of {self.get_penalty_shootout_msg()}")
        if self.game.penalty_shootout_count == 20:  # end of extended penalty shootout
            return True
        diff = abs(self.game.state.teams[0].score - self.game.state.teams[1].score)
        if self.game.penalty_shootout_count == 10 and diff > 0:
            return True
        if self.game.kickoff == self.game.state.teams[0].team_number:
            kickoff_team = self.game.state.teams[0]
        else:
            kickoff_team = self.game.state.teams[1]
        kickoff_team_leads = (kickoff_team.score >= self.game.state.teams[0].score and
                              kickoff_team.score >= self.game.state.teams[1].score)
        penalty_shootout_count = self.game.penalty_shootout_count % 10  # supports both regular and extended shootout kicks
        if (penalty_shootout_count == 6 and diff == 3) or (penalty_shootout_count == 8 and diff == 2):
            return True  # no need to go further, score is like 3-0 after 6 shootouts or 4-2 after 8 shootouts
        if penalty_shootout_count == 7:
            if diff == 3:  # score is like 4-1
                return True
            if diff == 2 and not kickoff_team_leads:  # score is like 1-3
                return True
        elif penalty_shootout_count == 9:
            if diff == 2:  # score is like 5-3
                return True
            if diff == 1 and not kickoff_team_leads:  # score is like 3-4
                return True
        return False

    def next_penalty_shootout(self):
        self.game.penalty_shootout_count += 1
        if not self.game.penalty_shootout_goal and self.game.state.game_state[:8] != "FINISHED":
            self.logger.info("Sending state finish to end current_penalty_shootout")
            self.game_controller_send('STATE:FINISH')
        self.game.penalty_shootout_goal = False
        if self.stop_penalty_shootout():
            self.game.over = True
            return
        if self.game.penalty_shootout_count == 10:
            self.logger.info('Starting extended penalty shootout without a goalkeeper and goal area entrance allowed.')
        # Only prepare next penalty if team has a kicker available
        self.flip_sides()
        self.logger.info(f'fliped sides: game.side_left = {self.game.side_left}')
        if self.penalty_kicker_player():
            self.game_controller_send('STATE:SET')
            self.set_penalty_positions()
        else:
            self.logger.info("Skipping penalty trial because team has no kicker available")
            self.game_controller_send('STATE:SET')
            self.next_penalty_shootout()
        return

    def check_penalty_goal_line(self):
        """
        Checks that the goalkeepers of both teams respect the goal line rule and apply penalties if required
        """
        defending_team = self.get_penalty_defending_team()
        for number, player in defending_team.players.items():
            ignore_player = self.already_penalized(player) or not self.is_goalkeeper(defending_team, number)
            if self.game.in_play is not None or ignore_player:
                player['invalidGoalkeeperStart'] = None
                continue
            on_goal_line_or_behind = (player['on_outer_line'] or player['outside_field']) and \
                abs(player['position'][1]) <= self.config.GOAL_WIDTH
            if on_goal_line_or_behind and player['inside_own_side']:
                player['invalidGoalkeeperStart'] = None
            else:
                if player['invalidGoalkeeperStart'] is None:
                    player['invalidGoalkeeperStart'] = self.sim_time.get_ms()
                elif self.sim_time.get_ms() - player['invalidGoalkeeperStart'] > self.config.INVALID_GOALKEEPER_TIMEOUT * 1000:
                    self.logger.info(f'Goalkeeper of team {defending_team.color} is not on goal line '
                                     f'since {self.config.INVALID_GOALKEEPER_TIMEOUT} sec')
                    self.send_penalty(player, 'INCAPABLE', "Not on goal line during penalty")

    def interruption(self, interruption_type, team=None, location=None, is_goalkeeper_ball_manipulation=False):
        if location is not None:
            self.game.ball_kick_translation[:2] = location[:2]
        if interruption_type == 'FREEKICK':
            own_side = (self.game.side_left == team) ^ (self.game.ball_position[0] < 0)
            inside_penalty_area = self.field.circle_fully_inside_penalty_area(self.game.ball_position, self.game.ball_radius)
            if inside_penalty_area and own_side:
                if is_goalkeeper_ball_manipulation:
                    # move the ball on the penalty line parallel to the goal line
                    dx = self.field.size_x - self.field.penalty_area_length
                    dy = self.field.penalty_area_width / 2
                    moved = False
                    if abs(location[0]) > dx:
                        self.game.ball_kick_translation[0] = dx * (-1 if location[0] < 0 else 1)
                        moved = True
                    if abs(location[1]) > dy:
                        self.game.ball_kick_translation[1] = dy * (-1 if location[1] < 0 else 1)
                        moved = True
                    if moved:
                        self.logger.info(f'Moved the ball on the penalty line at {self.game.ball_kick_translation}')
                    interruption_type = 'INDIRECT_FREEKICK'
                else:
                    interruption_type = 'PENALTYKICK'
                    self.game.ball_kick_translation = [self.field.penalty_mark_x, 0, 0.08]
                    if location[0] < 0:
                        self.game.ball_kick_translation[0] *= -1
            else:
                interruption_type = 'DIRECT_FREEKICK'
            self.game.can_score = interruption_type != 'INDIRECT_FREEKICK'
        self.game.in_play = None
        self.game.can_score_own = False
        self.game.ball_set_kick = True
        self.game.interruption = interruption_type
        self.game.phase = interruption_type
        self.game.ball_first_touch_time = 0
        self.game.interruption_countdown = self.config.SIMULATED_TIME_INTERRUPTION_PHASE_0
        self.logger.info(f'Interruption countdown set to {self.game.interruption_countdown}')
        if not team:
            self.game.interruption_team = self.game.red.id if self.game.ball_last_touch_team == 'blue' else self.game.blue.id
        else:
            self.game.interruption_team = team
        self.game.ball_must_kick_team = 'red' if self.game.interruption_team == self.game.red.id else 'blue'
        self.game.reset_ball_touched()
        self.logger.info(f'Ball not in play, will be kicked by a player from the {self.game.ball_must_kick_team} team.')
        color = 'red' if self.game.interruption_team == self.game.red.id else 'blue'
        self.logger.info(f'{GAME_INTERRUPTIONS[interruption_type].capitalize()} awarded to {color} team.')
        self.game_controller_send(f'{self.game.interruption}:{self.game.interruption_team}')

    def throw_in(self, left_side):
        # set the ball on the touch line for throw in
        sign = -1 if left_side else 1
        self.game.ball_kick_translation[0] = self.game.ball_exit_translation[0]
        self.game.ball_kick_translation[1] = sign * (self.field.size_y - self.field.line_half_width)
        self.game.can_score = False  # disallow direct goal
        self.game.throw_in = True
        self.game.throw_in_ball_was_lifted = False
        self.interruption('THROWIN')

    def corner_kick(self, left_side):
        # set the ball in the right corner for corner kick
        sign = -1 if left_side else 1
        self.game.ball_kick_translation[0] = sign * (self.field.size_x - self.field.line_half_width)
        self.game.ball_kick_translation[1] = self.field.size_y - self.field.line_half_width \
            if self.game.ball_exit_translation[1] > 0 \
            else -self.field.size_y + self.field.line_half_width
        self.game.can_score = True
        self.interruption('CORNERKICK')

    def goal_kick(self):
        # set the ball at intersection between the centerline and touchline
        self.game.ball_kick_translation[0] = 0
        self.game.ball_kick_translation[1] = self.field.size_y - self.field.line_half_width \
            if self.game.ball_exit_translation[1] > 0 \
            else -self.field.size_y + self.field.line_half_width
        self.game.can_score = True
        self.interruption('GOALKICK')

    def move_ball_away(self):
        """Places ball far away from field for phases where the referee is supposed to hold it in it's hand"""
        target_location = [100, 100, self.game.ball_radius + 0.05]
        self.ball.resetPhysics()
        self.game.ball_translation.setSFVec3f(target_location)
        self.logger.info("Moved ball out of the field temporarily")

    def kickoff(self):
        color = 'red' if self.game.kickoff == self.game.red.id else 'blue'
        self.logger.info(f'Kick-off is {color}.')
        self.game.set_kickoff(color)
        self.game.reset_ball_touched()
        self.move_ball_away()
        self.logger.info(f'Ball not in play, will be kicked by a player from the {self.game.ball_must_kick_team} team.')

    def dropped_ball(self):
        self.logger.info(f'The ball didn\'t move for the past {self.config.DROPPED_BALL_TIMEOUT} seconds.')
        self.game.ball_last_move = self.sim_time.get_ms()
        self.game_controller_send('DROPPEDBALL')
        self.game.set_dropped_ball()

    def is_robot_near(self, position, min_dist):
        for team in [self.red_team, self.blue_team]:
            for number, player in team.players.items():
                if distance2(position, player['position']) < min_dist:
                    return True
        return False

    def is_other_robot_near(self, robot_color, robot_number, position, min_dist):
        """Test if another robot than the robot defined by team_color and number is closer than min_dist from position"""
        for team in [self.red_team, self.blue_team]:
            for number, player in team.players.items():
                if team.color == robot_color and number == robot_number:
                    continue
                if distance2(position, player['position']) < min_dist:
                    return True
        return False

    def reset_pos_penalized_robots_near(self, position, min_dist):
        for team in [self.red_team, self.blue_team]:
            for number, player in team.players.items():
                if 'penalized' in player and distance2(position, player['position']) < min_dist:
                    self.place_player_at_penalty(player, team, number)

    def penalize_fallen_robots_near(self, position, min_dist):
        for team in [self.red_team, self.blue_team]:
            for number, player in team.players.items():
                if 'fallen' in player:
                    # If we wait for the end of the tick, the rest of the game interruption procedure will ignore the penalty
                    self.place_player_at_penalty(player, team, number)
                    self.send_penalty(player, "INCAPABLE", "Fallen close to ball during GameInterruption")

    def get_alternative_ball_locations(self, original_pos):
        if (self.game.interruption_team == self.game.red.id) ^ (self.game.side_left == self.game.blue.id):
            prefered_x_dir = 1
        else:
            prefered_x_dir = -1
        prefered_y_dir = -1 if original_pos[1] > 0 else 1
        offset_x = prefered_x_dir * self.field.place_ball_safety_dist * np.array([1, 0, 0])
        offset_y = prefered_y_dir * self.field.place_ball_safety_dist * np.array([0, 1, 0])
        locations = []
        if self.game.interruption == "DIRECT_FREEKICK" or self.game.interruption == "INDIRECT_FREEKICK":
            # TODO If indirect free kick in opponent penalty area on line parallel to goal line, move it along this line
            for dist_mult in range(1, self.config.GAME_INTERRUPTION_PLACEMENT_NB_STEPS+1):
                locations.append(original_pos + offset_x * dist_mult)
                locations.append(original_pos + offset_y * dist_mult)
                locations.append(original_pos - offset_y * dist_mult)
                locations.append(original_pos - offset_x * dist_mult)
        elif self.game.interruption == "THROWIN":
            for dist_mult in range(1, self.config.GAME_INTERRUPTION_PLACEMENT_NB_STEPS+1):
                locations.append(original_pos + offset_x * dist_mult)
            for dist_mult in range(1, self.config.GAME_INTERRUPTION_PLACEMENT_NB_STEPS+1):
                locations.append(original_pos - offset_x * dist_mult)
        return locations

    def get_obstacles_positions(self, team, number):
        """Returns the list of potential obstacles in case the indicated robot is moved"""
        # TODO: add goal posts for safety
        obstacles = []
        for o_team in [self.blue_team, self.red_team]:
            for o_number, o_player in o_team.players.items():
                if team.color == o_team.color and number == o_number:
                    continue
                obstacles.append(o_player['position'])
        return obstacles

    def move_robots_away(self, target_location):
        for team in [self.blue_team, self.red_team]:
            for number, player in team.players.items():
                if player['robot'] is None:
                    continue
                initial_pos = np.array(player['position'])
                if distance2(initial_pos, target_location) < self.field.place_ball_safety_dist:
                    obstacles = self.get_obstacles_positions(team, number)
                    player_2_ball = initial_pos - np.array(target_location)
                    dist = np.linalg.norm(player_2_ball[:2])
                    if dist < 0.001:
                        player_2_ball = [1, 0, 0]
                        dist = 1
                    offset = player_2_ball / dist * self.field.place_ball_safety_dist
                    for dist_mult in range(1, self.config.GAME_INTERRUPTION_PLACEMENT_NB_STEPS+1):
                        allowed = True
                        pos = target_location + offset * dist_mult
                        for o in obstacles:
                            if distance2(o, pos) < self.field.place_ball_safety_dist:
                                allowed = False
                                break
                        if allowed:
                            pos[2] = initial_pos[2]
                            diff = pos - initial_pos
                            initial_t = np.array(player['robot'].getField('translation').getSFVec3f())
                            dst_t = initial_t + diff
                            self.logger.info(f"Moving {team.color} player {number} from {initial_pos} to {pos}")
                            # Pose of the robot is not changed
                            player['robot'].getField('translation').setSFVec3f(dst_t.tolist())
                            player['position'] = dst_t.tolist()
                            break

    def game_interruption_place_ball(self, target_location, enforce_distance=True):
        if enforce_distance:
            target_location[2] = 0  # Set position along z-axis to 0 for all 'self.field.point_inside' checks
            step = 1
            self.logger.info(f"GI placing ball to {target_location}")
            while step <= 4 and self.is_robot_near(target_location, self.field.place_ball_safety_dist):
                if step == 1:
                    self.logger.info('Reset of penalized robots')
                    self.reset_pos_penalized_robots_near(target_location, self.field.place_ball_safety_dist)
                elif step == 2:
                    self.logger.info('Penalizing fallen robots')
                    self.penalize_fallen_robots_near(target_location, self.field.place_ball_safety_dist)
                elif step == 3:
                    self.logger.info('Finding alternative locations')
                    for loc in self.get_alternative_ball_locations(target_location):
                        self.logger.info(f"Testing alternative location: {loc}")
                        # TODO: ?should it only allow point outside penalty area?
                        if self.field.point_inside(loc) and not self.is_robot_near(loc, self.field.place_ball_safety_dist):
                            self.logger.info(f"Set alternative location to: {loc}")
                            target_location = loc.tolist()
                            break
                elif step == 4:
                    self.logger.info(f"Pushing robots away from {target_location}")
                    self.move_robots_away(target_location)
                step += 1
        target_location[2] = self.game.ball_radius
        self.ball.resetPhysics()
        self.game.ball_translation.setSFVec3f(target_location)
        self.game.ball_set_kick = False
        self.game.reset_ball_touched()
        self.logger.info(f'Ball respawned at {target_location[0]} {target_location[1]} {target_location[2]}.')

    def setup(self):
        # speed up non-real time tests
        if self.game.minimum_real_time_factor == 0:
            self.config.REAL_TIME_BEFORE_FIRST_READY_STATE = 5
            self.config.HALF_TIME_BREAK_REAL_TIME_DURATION = 2

        # check game type
        if self.game.type not in ['NORMAL', 'KNOCKOUT', 'PENALTY']:
            self.logger.error(f'Unsupported game type: {self.game.type}.')
            self.clean_exit()

        if self.game.minimum_real_time_factor == 0:
            self.logger.info('Simulation will run as fast as possible, real time waiting times will be minimal.')
        else:
            self.logger.info(
                f'Simulation will guarantee a maximum {1 / self.game.minimum_real_time_factor:.2f}x speed for each time step.')

        # check team name length (should be at most 12 characters long, trim them if too long)
        if len(self.red_team.name) > 12:
            self.red_team.name = self.red_team.name[:12]
        if len(self.blue_team.name) > 12:
            self.blue_team.name = self.blue_team.name[:12]

        # check if the host parameter of the game.json file correspond to the actual host
        host = socket.gethostbyname(socket.gethostname())
        if host != '127.0.0.1' and host != self.game.host:
            self.logger.warning(f'Host is not correctly defined in game.json file, '
                                f'it should be {host} instead of {self.game.host}.')

        try:
            JAVA_HOME = os.environ['JAVA_HOME']
            try:
                GAME_CONTROLLER_HOME = os.environ['GAME_CONTROLLER_HOME']
                if not os.path.exists(GAME_CONTROLLER_HOME):
                    self.logger.error(f'{GAME_CONTROLLER_HOME} (GAME_CONTROLLER_HOME) folder not found.')
                    self.game.controller_process = None
                    self.clean_exit()
                else:
                    path = os.path.join(GAME_CONTROLLER_HOME, 'build', 'jar', 'config',
                                        f'hl_sim_{self.game.field_size}', 'teams.cfg')
                    red_line = f'{self.game.red.id}={self.red_team.name}\n'
                    blue_line = f'{self.game.blue.id}={self.blue_team.name}\n'
                    with open(path, 'w') as file:
                        file.write((red_line + blue_line) if self.game.red.id < self.game.blue.id else (blue_line + red_line))
                    command_line = [os.path.join(JAVA_HOME, 'bin', 'java'), '-jar', 'GameControllerSimulator.jar']
                    if self.game.minimum_real_time_factor < 1:
                        command_line.append('--fast')
                    command_line.append('--minimized')
                    command_line.append('--config')
                    command_line.append(self.game_config_file)
                    if hasattr(self.game, 'game_controller_extra_args'):
                        for arg in self.game.game_controller_extra_args:
                            command_line.append(arg)
                    if hasattr(self.game, 'use_bouncing_server') and self.game.use_bouncing_server:
                        command_line.append('-b')
                        command_line.append(self.game.host)
                        self.udp_bouncer_process = subprocess.Popen(["python3", "udp_bouncer.py", self.game_config_file])
                    else:
                        self.udp_bouncer_process = None
                    self.game.controller_process = subprocess.Popen(command_line,
                                                                    cwd=os.path.join(GAME_CONTROLLER_HOME, 'build', 'jar'))
            except KeyError:
                self.game.controller_process = None
                self.logger.error('GAME_CONTROLLER_HOME environment variable not set, unable to launch GameController.')
                self.clean_exit()
        except KeyError:
            self.game.controller_process = None
            self.logger.error('JAVA_HOME environment variable not set, unable to launch GameController.')
            self.clean_exit()

        self.toss_a_coin_if_needed('side_left')
        self.toss_a_coin_if_needed('kickoff')

        children = self.supervisor.getRoot().getField('children')
        if (hasattr(self.game, "texture_seed")):
            random.seed(self.game.texture_seed)
        bg = random.choice(['stadium_dry', 'shanghai_riverside', 'ulmer_muenster', 'sunset_jhbcentral',
                            'sepulchral_chapel_rotunda', 'paul_lobe_haus', 'kiara_1_dawn'])
        luminosity = random.random() * 0.5 + 0.75  # random value between 0.75 and 1.25
        ball_texture = random.choice(['telstar', 'teamgeist', 'europass', 'jabulani', 'tango'])
        random.seed()
        children.importMFNodeFromString(-1, f'RoboCupBackground {{ texture "{bg}" luminosity {luminosity}}}')
        children.importMFNodeFromString(-1, f'RoboCupMainLight {{ texture "{bg}" luminosity {luminosity}}}')
        children.importMFNodeFromString(-1, f'RoboCupOffLight {{ texture "{bg}" luminosity {luminosity}}}')
        children.importMFNodeFromString(-1, f'RoboCupTopLight {{ texture "{bg}" luminosity {luminosity}}}')
        children.importMFNodeFromString(-1, f'RobocupSoccerField {{ size "{self.game.field_size}" }}')
        ball_size = 1 if self.game.field_size == 'kid' else 5

        # the ball is initially very far away from the field
        children.importMFNodeFromString(-1, f'DEF BALL RobocupTexturedSoccerBall'
                                            f'{{ translation 100 100 0.5 size {ball_size} texture "{ball_texture}" }}')
        self.ball = self.blackboard.supervisor.getFromDef('BALL')
        self.game.ball_translation = self.blackboard.supervisor.getFromDef('BALL').getField('translation')

        self.game.state = None
        self.spawn_team(self.red_team, self.game.side_left == self.game.blue.id)
        self.spawn_team(self.blue_team, self.game.side_left == self.game.red.id)

        players_ball_holding_time_window_size = int(1000 * self.config.PLAYERS_BALL_HOLDING_TIMEOUT / self.time_step)
        goalkeeper_ball_holding_time_window_size = int(1000 * self.config.GOALKEEPER_BALL_HOLDING_TIMEOUT / self.time_step)
        self.red_team.players_holding_time_window = np.zeros(players_ball_holding_time_window_size, dtype=bool)
        self.red_team.goalkeeper_holding_time_window = np.zeros(goalkeeper_ball_holding_time_window_size, dtype=bool)
        self.blue_team.players_holding_time_window = np.zeros(players_ball_holding_time_window_size, dtype=bool)
        self.blue_team.goalkeeper_holding_time_window = np.zeros(goalkeeper_ball_holding_time_window_size, dtype=bool)

        self.list_solids()  # prepare lists of solids to monitor in each robot to compute the convex hulls

        self.game.reset_ball_touched()

        self.previous_seconds_remaining = 0

        try:
            if self.game.controller_process:
                self.game_controller_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                retry = 0
                while True:
                    try:
                        self.game_controller_socket.connect(('localhost', 8750))
                        self.game_controller_socket.setblocking(False)
                        break
                    except socket.error as msg:
                        retry += 1
                        if retry <= 10:
                            self.logger.warning(f'Could not connect to GameController at localhost:8750: {msg}. '
                                                f'Retrying ({retry}/10)...')
                            time.sleep(retry)  # give some time to allow the GameControllerSimulator to start-up
                            self.supervisor.step(0)
                        else:
                            self.logger.error('Could not connect to GameController at localhost:8750.')
                            self.game_controller_socket = None
                            self.clean_exit()
                            break
                self.logger.info('Connected to GameControllerSimulator at localhost:8750.')
                try:
                    self.game.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
                    self.game.udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    if hasattr(self.game, 'use_bouncing_server') and self.game.use_bouncing_server:
                        # In case we are using the bouncing server we have to select which interface
                        # is used because messages are not broadcast
                        self.game.udp.bind((self.game.host, 3838))
                    else:
                        self.game.udp.bind(('0.0.0.0', 3838))
                    self.game.udp.setblocking(False)
                except Exception:
                    self.logger.error("Failed to set up UDP socket to listen to GC messages")
            else:
                self.game_controller_socket = None
        except Exception:
            self.logger.error(f"Failed connecting to GameController with the following exception {traceback.format_exc()}")
            self.clean_exit()

        try:
            self.display.update()
            self.logger.info(f'Game type is {self.game.type}.')
            self.logger.info(f'Red team is "{self.red_team.name}", '
                             f'playing on {"left" if self.game.side_left == self.game.red.id else "right"} side.')
            self.logger.info(f'Blue team is "{self.blue_team.name}", '
                             f'playing on {"left" if self.game.side_left == self.game.blue.id else "right"} side.')
            self.game_controller_send(f'SIDE_LEFT:{self.game.side_left}')

            if hasattr(self.game, 'supervisor'):  # optional supervisor used for CI tests
                children.importMFNodeFromString(-1,
                                                f'DEF TEST_SUPERVISOR Robot '
                                                f'{{ supervisor TRUE controller "{self.game.supervisor}" }}')

            if self.game.penalty_shootout:
                self.logger.info(f'{"Red" if self.game.kickoff == self.game.red.id else "Blue"} '
                                 f'team will start the penalty shoot-out.')
                self.game.phase = 'PENALTY-SHOOTOUT'
                self.game.ready_real_time = None
                self.logger.info(f'Penalty start: Waiting {self.config.REAL_TIME_BEFORE_FIRST_READY_STATE} '
                                 f'seconds (real-time) before going to SET')
                # real time for set state (penalty-shootout)
                self.game.set_real_time = time.time() + self.config.REAL_TIME_BEFORE_FIRST_READY_STATE
                self.game_controller_send(f'KICKOFF:{self.game.kickoff}')
            else:
                self.logger.info(f'Regular start: Waiting {self.config.REAL_TIME_BEFORE_FIRST_READY_STATE} '
                                 f'seconds (real-time) before going to READY')
                # real time for ready state (initial kick-off)
                self.game.ready_real_time = time.time() + self.config.REAL_TIME_BEFORE_FIRST_READY_STATE
                self.kickoff()
                self.game_controller_send(f'KICKOFF:{self.game.kickoff}')
        except Exception:
            self.logger.error(f"Failed setting initial state: {traceback.format_exc()}")
            self.clean_exit()

        if hasattr(self.game, 'record_simulation'):
            try:
                if self.game.record_simulation.endswith(".html"):
                    self.supervisor.animationStartRecording(self.game.record_simulation)
                elif self.game.record_simulation.endswith(".mp4"):
                    self.supervisor.movieStartRecording(self.game.record_simulation, width=1280, height=720, codec=0,
                                                        quality=100, acceleration=1, caption=False)
                    if self.supervisor.movieFailed():
                        raise RuntimeError("Failed to Open Movie")
                else:
                    raise RuntimeError(f"Unknown extension for record_simulation: {self.game.record_simulation}")
            except Exception:
                self.logger.error(f"Failed to start recording with exception: {traceback.format_exc()}")
                self.clean_exit()

    def main_loop(self):
        previous_real_time = time.time()
        while self.supervisor.step(self.time_step) != -1 and not self.game.over:
            if hasattr(self.game, 'max_duration') and (time.time() - self.blackboard.start_real_time) > self.game.max_duration:
                self.logger.info(f'Interrupting game automatically after {self.game.max_duration} seconds')
                break
            self.print_status()
            self.game_controller_send(f'CLOCK:{self.sim_time.get_ms()}')
            self.game_controller_receive()
            if self.game.state is None:
                self.sim_time.progress_ms(self.time_step)
                continue
            self.stabilize_robots()
            send_play_state_after_penalties = False
            previous_position = copy.deepcopy(self.game.ball_position)
            self.game.ball_position = self.game.ball_translation.getSFVec3f()
            if self.game.ball_position != previous_position:
                self.game.ball_last_move = self.sim_time.get_ms()
            self.update_contacts()  # check for collisions with the ground and ball
            if not self.game.penalty_shootout:
                self.update_ball_holding()  # check for ball holding for field players and goalkeeper
            self.update_histories()
            if self.game.state.game_state == 'STATE_PLAYING' and not self.is_early_game_interruption():
                self.check_outside_turf()
                self.check_forceful_contacts()
                self.check_inactive_goalkeepers()
                if self.game.in_play is None:
                    # During period after the end of a game interruption, check distance of opponents
                    if self.game.phase in GAME_INTERRUPTIONS and self.game.state.secondary_state[6:] == "NORMAL":
                        opponent_team = self.red_team if self.game.ball_must_kick_team == 'blue' else self.blue_team
                        self.check_team_away_from_ball(opponent_team, self.field.opponent_distance_to_ball)
                    # If ball is not in play after a kick_off, check for circle entrance for the defending team
                    if self.game.phase == 'KICKOFF' and self.game.kickoff != self.config.DROPPED_BALL_TEAM_ID:
                        defending_team = self.red_team if self.game.kickoff == self.game.blue.id else self.blue_team
                        self.check_circle_entrance(defending_team)
                    if self.game.ball_first_touch_time != 0:
                        d = distance2(self.game.ball_kick_translation, self.game.ball_position)
                        if d > self.config.BALL_IN_PLAY_MOVE:
                            self.logger.info(f'{self.game.ball_kick_translation} {self.game.ball_position}')
                            self.logger.info(f'Ball in play, can be touched by any player (moved by {d * 100:.2f} cm).')
                            self.game.in_play = self.sim_time.get_ms()
                        elif not self.is_game_interruption():  # The game interruption case is handled in update_team_contacts
                            team = self.red_team if self.game.ball_must_kick_team == 'blue' else self.blue_team
                            self.check_ball_must_kick(team)
                else:
                    if self.sim_time.get_ms() - self.game.ball_last_move > self.config.DROPPED_BALL_TIMEOUT * 1000:
                        self.dropped_ball()
                    if self.game.ball_left_circle is None and self.game.phase == 'KICKOFF':
                        if distance2(self.game.ball_kick_translation, self.game.ball_position) > \
                                self.field.circle_radius + self.game.ball_radius:
                            self.game.ball_left_circle = self.sim_time.get_ms()
                            self.logger.info('The ball has left the center circle after kick-off.')

                    ball_touched_by_opponent = (self.game.ball_last_touch_team and
                                                self.game.ball_last_touch_team != self.game.ball_must_kick_team)
                    ball_touched_by_teammate = (self.game.kicking_player_number is not None and
                                                self.game.ball_last_touch_player_number != self.game.kicking_player_number)
                    ball_touched_in_play = self.game.in_play is not None and self.game.in_play < self.game.ball_last_touch_time
                    if not self.game.can_score:
                        if self.game.phase == 'KICKOFF':
                            ball_touched_after_leaving_the_circle = self.game.ball_left_circle is not None \
                                    and self.game.ball_left_circle < self.game.ball_last_touch_time
                            if ball_touched_by_opponent or ball_touched_by_teammate or ball_touched_after_leaving_the_circle:
                                self.game.can_score = True
                        elif self.game.phase == 'THROWIN':
                            if ball_touched_by_teammate or ball_touched_by_opponent or ball_touched_in_play:
                                self.game.can_score = True
                    if not self.game.can_score_own:
                        if ball_touched_by_opponent or ball_touched_by_teammate or ball_touched_in_play:
                            self.game.can_score_own = True

                if self.game.penalty_shootout:
                    self.check_penalty_goal_line()
                    ball_in_goal_area = self.field.circle_fully_inside_goal_area(self.game.ball_position,
                                                                                 self.game.ball_radius)
                    # It is unclear that using getVelocity is the good approach, because even when the ball
                    # is clearly not moving anymore, it still provides values above 1e-3.
                    ball_vel = self.ball.getVelocity()[:3]
                    if ball_in_goal_area and np.linalg.norm(ball_vel) < self.config.STATIC_SPEED_EPS:
                        self.logger.info(f"Ball stopped in goal area at {self.game.ball_position}")
                        self.next_penalty_shootout()
                    if self.game.penalty_shootout_count < 10:  # detect entrance of kicker in the goal area
                        kicker = self.penalty_kicker_player()
                        if kicker is None or (not kicker['outside_goal_area'] and not kicker['inside_own_side']):
                            # if no kicker is available or if the kicker is not fully outside the opponent goal area,
                            # we stop the kick and continue
                            self.next_penalty_shootout()
                            if self.game.over:
                                break
                    else:  # extended penalty shootouts
                        if ball_in_goal_area:
                            c = self.game.penalty_shootout_count - 10
                            if self.game.penalty_shootout_time_to_reach_goal_area[c] is None:
                                self.game.penalty_shootout_time_to_reach_goal_area[c] = 60 - self.game.state.seconds_remaining
                if self.previous_seconds_remaining != self.game.state.seconds_remaining:
                    self.display.update()
                    self.previous_seconds_remaining = self.game.state.seconds_remaining
                    # TODO find out why GC can send negative 'seconds_remaining' when secondary state is penaltykick
                    if self.game.state.game_state != "STATE_FINISHED" and self.game.state.seconds_remaining <= 0 and \
                            not self.game.state.secondary_state == "PENALTYKICK":
                        self.logger.info(f"Sending FINISH because seconds remaining = {self.game.state.seconds_remaining}")
                        self.game_controller_send('STATE:FINISH')
                        if self.game.penalty_shootout:  # penalty timeout was reached
                            self.next_penalty_shootout()
                            if self.game.over:
                                break
                        elif self.game.state.first_half:
                            game_type = 'knockout ' if self.game.type == 'KNOCKOUT' and self.game.overtime else ''
                            self.logger.info(f'End of {game_type}first half.')
                            self.flip_sides()
                            self.reset_teams('halfTimeStartingPose')
                            self.game.kickoff = self.game.blue.id if self.game.kickoff == self.game.red.id \
                                else self.game.red.id
                        elif self.game.type == 'NORMAL':
                            self.logger.info('End of second half.')
                        elif self.game.type == 'KNOCKOUT':
                            if not self.game.overtime:
                                self.logger.info('End of second half.')
                                self.flip_sides()
                                self.reset_teams('halfTimeStartingPose')
                                self.game.kickoff = self.game.blue.id if self.game.kickoff == self.game.red.id \
                                    else self.game.red.id
                                self.game.overtime = True
                            else:
                                self.logger.info('End of knockout second half.')
                                self.game.finished_overtime = True
                        else:
                            self.logger.error(f'Unsupported game type: {self.game.type}.')
                            self.clean_exit()
                if (self.game.interruption_countdown == 0 and self.game.ready_countdown == 0 and
                        self.game.ready_real_time is None and not self.game.throw_in and
                        (self.game.ball_position[1] - self.game.ball_radius >= self.field.size_y or
                         self.game.ball_position[1] + self.game.ball_radius <= -self.field.size_y or
                         self.game.ball_position[0] - self.game.ball_radius >= self.field.size_x or
                         self.game.ball_position[0] + self.game.ball_radius <= -self.field.size_x)):
                    self.logger.info(f'Ball left the field at ({self.game.ball_position[0]} {self.game.ball_position[1]} '
                                     f'{self.game.ball_position[2]}) after being touched by {self.game.ball_last_touch_team} '
                                     f'player {self.game.ball_last_touch_player_number}.')
                    self.game.ball_exit_translation = self.game.ball_position
                    scoring_team = None
                    right_way = None
                    if self.game.ball_exit_translation[1] - self.game.ball_radius > self.field.size_y:
                        if self.game.penalty_shootout:
                            self.next_penalty_shootout()
                        else:
                            self.game.ball_exit_translation[1] = self.field.size_y - self.field.line_half_width
                            self.throw_in(left_side=False)
                    elif self.game.ball_exit_translation[1] + self.game.ball_radius < -self.field.size_y:
                        if self.game.penalty_shootout:
                            self.next_penalty_shootout()
                        else:
                            self.throw_in(left_side=True)
                    if self.game.ball_exit_translation[0] - self.game.ball_radius > self.field.size_x:
                        right_way = self.game.ball_last_touch_team == 'red' and self.game.side_left == self.game.red.id or \
                                    self.game.ball_last_touch_team == 'blue' and self.game.side_left == self.game.blue.id
                        if self.config.GOAL_HALF_WIDTH > self.game.ball_exit_translation[1] > -self.config.GOAL_HALF_WIDTH and \
                                self.game.ball_exit_translation[2] < self.field.goal_height:
                            scoring_team = self.game.side_left  # goal
                        elif self.game.penalty_shootout:
                            self.next_penalty_shootout()
                        else:
                            if right_way:
                                self.goal_kick()
                            else:
                                self.corner_kick(left_side=False)
                    elif self.game.ball_exit_translation[0] + self.game.ball_radius < -self.field.size_x:
                        right_way = self.game.ball_last_touch_team == 'red' and self.game.side_left == self.game.blue.id or \
                                    self.game.ball_last_touch_team == 'blue' and self.game.side_left == self.game.red.id
                        if self.config.GOAL_HALF_WIDTH > self.game.ball_exit_translation[1] > -self.config.GOAL_HALF_WIDTH and \
                                self.game.ball_exit_translation[2] < self.field.goal_height:
                            # goal
                            scoring_team = self.game.red.id if self.game.blue.id == self.game.side_left else self.game.blue.id
                        elif self.game.penalty_shootout:
                            self.next_penalty_shootout()
                        else:
                            if right_way:
                                self.goal_kick()
                            else:
                                self.corner_kick(left_side=True)
                    if scoring_team:
                        goal = 'red' if scoring_team == self.game.blue.id else 'blue'
                        if self.game.penalty_shootout_count >= 10:  # extended penalty shootouts
                            extended_penalty_idx = self.game.penalty_shootout_count - 10
                            self.game.penalty_shootout_time_to_score[extended_penalty_idx] = \
                                60 - self.game.state.seconds_remaining
                        if not self.game.penalty_shootout:
                            self.game.kickoff = self.game.blue.id if scoring_team == self.game.red.id else self.game.red.id
                        i = self.team_index(self.game.ball_last_touch_team)
                        if not self.game.can_score:
                            if self.game.phase == 'KICKOFF':
                                self.logger.info(f'Invalidated direct score in {goal} goal from kick-off position.')
                                self.goal_kick()
                            elif self.game.phase in GAME_INTERRUPTIONS:
                                self.logger.info(f'Invalidated direct score in {goal} goal from '
                                                 f'{GAME_INTERRUPTIONS[self.game.phase]}.')
                                if not right_way:  # own_goal
                                    self.corner_kick(left_side=scoring_team != self.game.side_left)
                                else:
                                    self.goal_kick()

                        elif not right_way and not self.game.can_score_own:
                            if self.game.phase == 'KICKOFF':
                                self.logger.info(f'Invalidated direct score in {goal} goal from kick-off position.')
                            elif self.game.phase in GAME_INTERRUPTIONS:
                                self.logger.info(f'Invalidated direct score in {goal} goal from '
                                                 f'{GAME_INTERRUPTIONS[self.game.phase]}.')
                            if self.game.penalty_shootout:
                                self.next_penalty_shootout()
                            else:
                                self.corner_kick(left_side=scoring_team != self.game.side_left)

                        elif (self.game.ball_last_touch_player_number is not None and
                              self.game.state.teams[i].players[self.game.ball_last_touch_player_number - 1]
                                  .secs_till_unpenalized == 0):
                            self.game_controller_send(f'SCORE:{scoring_team}')
                            self.logger.info(f'Score in {goal} goal by {self.game.ball_last_touch_team} '
                                             f'player {self.game.ball_last_touch_player_number}')
                            if self.game.penalty_shootout:
                                self.game.penalty_shootout_goal = True
                                self.next_penalty_shootout()
                            else:
                                self.game.ready_countdown = self.config.SIMULATED_TIME_INTERRUPTION_PHASE_0
                                self.logger.info(f"Ready countdown was set to {self.game.ready_countdown}")
                                self.kickoff()
                        elif not right_way:  # own goal
                            self.game_controller_send(f'SCORE:{scoring_team}')
                            self.logger.info(f'Score in {goal} goal by {self.game.ball_last_touch_team} player ' +
                                             f'{self.game.ball_last_touch_player_number} (own goal)')
                            self.game.ready_countdown = self.config.SIMULATED_TIME_INTERRUPTION_PHASE_0
                            self.logger.info(f"Ready countdown was set to {self.game.ready_countdown}")
                            self.kickoff()
                        else:
                            self.logger.info(f'Invalidated score in {goal} goal by penalized {self.game.ball_last_touch_team} '
                                             f'player {self.game.ball_last_touch_player_number}')
                            if self.game.penalty_shootout:
                                self.next_penalty_shootout()
                            else:
                                self.goal_kick()
            elif self.game.state.game_state == 'STATE_READY':
                self.game.play_countdown = 0
                # the GameController will automatically change to the SET state once the state READY is over
                # the referee should wait a little time since the state SET started before sending the PLAY state
            elif self.game.state.game_state == 'STATE_SET':
                if self.game.play_countdown == 0:
                    if self.game.penalty_shootout:
                        self.logger.info("Waiting for penalty shootout")
                        self.game.play_countdown = self.config.SIMULATED_TIME_SET_PENALTY_SHOOTOUT
                    else:
                        self.logger.info("Waiting for classic play")
                        self.game.play_countdown = self.config.SIMULATED_TIME_BEFORE_PLAY_STATE
                    if self.game.ball_set_kick:
                        self.game_interruption_place_ball(self.game.ball_kick_translation, enforce_distance=False)
                else:
                    if self.game.penalty_shootout:
                        self.check_penalty_goal_line()
                    else:
                        if self.game.dropped_ball:
                            self.check_dropped_ball_position()
                        else:
                            self.check_kickoff_position()
                    self.game.play_countdown -= 1
                    if self.game.play_countdown == 0:
                        self.game.ready_countdown = 0
                        send_play_state_after_penalties = True
            elif self.game.state.game_state == 'STATE_FINISHED':
                if self.game.penalty_shootout:
                    if self.game.state.seconds_remaining <= 0:
                        self.next_penalty_shootout()
                elif self.game.state.first_half:
                    self.logger.info("Received state FINISHED: end of first half")
                    self.game.ready_real_time = None
                elif self.game.type == 'KNOCKOUT':
                    if self.game.ready_real_time is None:
                        if self.game.state.teams[0].score != self.game.state.teams[1].score:
                            self.game.over = True
                            break
                        elif self.game.finished_overtime:
                            self.logger.info('Beginning of penalty shout-out.')
                            self.game_controller_send('STATE:PENALTY-SHOOTOUT')
                            self.game.penalty_shootout = True
                            self.logger.info(f'Going to SET in {self.config.HALF_TIME_BREAK_REAL_TIME_DURATION} '
                                             f'seconds (real-time)')
                            self.game.set_real_time = time.time() + self.config.HALF_TIME_BREAK_REAL_TIME_DURATION
                        elif self.game.overtime:
                            self.logger.info('Beginning of the knockout first half.')
                            self.game_controller_send('STATE:OVERTIME-FIRST-HALF')
                            self.logger.info(f'Going to READY in {self.config.HALF_TIME_BREAK_REAL_TIME_DURATION} '
                                             f'seconds (real-time)')
                            self.game.ready_real_time = time.time() + self.config.HALF_TIME_BREAK_REAL_TIME_DURATION
                else:
                    self.game.over = True
                    break

            elif self.game.state.game_state == 'STATE_INITIAL':
                if self.game.penalty_shootout:
                    if self.game.set_real_time <= time.time():
                        self.logger.info("Starting first penalty")
                        self.set_penalty_positions()
                        self.game_controller_send('STATE:SET')
                elif self.game.ready_real_time is not None:
                    # initial kick-off (1st, 2nd half, extended periods, penalty shootouts)
                    if self.game.ready_real_time <= time.time():
                        self.logger.info('Real-time to wait elapsed, moving to READY')
                        self.game.ready_real_time = None
                        self.check_start_position()
                        self.game_controller_send('STATE:READY')
                elif self.game.ready_countdown > 0:
                    self.game.ready_countdown -= 1
                    if self.game.ready_countdown == 0:  # kick-off after goal or dropped ball
                        self.check_start_position()
                        self.game_controller_send('STATE:READY')
                elif not self.game.state.first_half:
                    game_type = ''
                    if self.game.overtime:
                        game_type = 'overtime '
                    self.logger.info(f'Beginning of {game_type}second half.')
                    self.kickoff()
                    self.logger.info(f'Going to READY in {self.config.HALF_TIME_BREAK_REAL_TIME_DURATION} seconds (real-time)')
                    self.game.ready_real_time = time.time() + self.config.HALF_TIME_BREAK_REAL_TIME_DURATION

            if self.game.interruption_countdown > 0:
                self.game.interruption_countdown -= 1
                if self.game.interruption_countdown == 0:
                    if self.game.ball_set_kick:
                        self.game_interruption_place_ball(self.game.ball_kick_translation, enforce_distance=True)
                    if self.game.interruption:
                        self.game_controller_send(f'{self.game.interruption}:{self.game.interruption_team}:READY')

            if self.game.state.game_state != 'STATE_INITIAL':
                self.check_fallen()                                # detect fallen robots

            if self.game.state.game_state == 'STATE_PLAYING' and self.game.in_play:
                if not self.game.penalty_shootout:
                    ball_holding = self.check_ball_holding()       # check for ball holding fouls
                    if ball_holding:
                        if self.is_game_interruption():
                            self.game_interruption_ball_holding(ball_holding)
                        else:
                            self.interruption('FREEKICK', ball_holding, self.game.ball_position)
                ball_handling = self.check_ball_handling()  # return team id if ball handling is performed by goalkeeper
                if ball_handling and not self.game.penalty_shootout:
                    if self.is_game_interruption():
                        self.game_interruption_ball_handling(ball_handling)
                    else:
                        self.interruption('FREEKICK', ball_handling, self.game.ball_position, is_goalkeeper_ball_manipulation=True) # TODO check logic here 
            self.check_penalized_in_field()                    # check for penalized robots inside the field
            if self.game.state.game_state != 'STATE_INITIAL':  # send penalties if needed
                self.send_penalties()
                if send_play_state_after_penalties:
                    self.game_controller_send('STATE:PLAY')

            self.sim_time.progress_ms(self.time_step)

            if self.game.minimum_real_time_factor != 0:
                # slow down the simulation to guarantee a miminum amount of real time between each step
                t = time.time()
                delta_time = previous_real_time - t + self.game.minimum_real_time_factor * self.time_step / 1000
                if delta_time > 0:
                    time.sleep(delta_time)
                previous_real_time = time.time()

        # for some reason, the simulation was terminated before the end of the match (may happen during tests)
        if not self.game.over:
            self.logger.info('Game interrupted before the end.')
        else:
            self.logger.info('End of the game.')
            if self.game.state.teams[0].score > self.game.state.teams[1].score:
                winner = 0
                loser = 1
            else:
                winner = 1
                loser = 0
            self.logger.info(f'The score is {self.game.state.teams[winner].score}-{self.game.state.teams[loser].score}.')
            if self.game.state.teams[0].score != self.game.state.teams[1].score:
                self.logger.info(f'The winner is the {self.game.state.teams[winner].team_color.lower()} team.')
            elif self.game.penalty_shootout_count < 20:
                self.logger.info('This is a draw.')
            else:  # extended penatly shoutout rules to determine the winner
                count = [0, 0]
                for i in range(5):
                    if self.game.penalty_shootout_time_to_reach_goal_area[2 * i] is not None:
                        count[0] += 1
                    if self.game.penalty_shootout_time_to_reach_goal_area[2 * i + 1] is not None:
                        count[1] += 1
                if self.game.kickoff == self.game.red.id:
                    count_red = count[0]
                    count_blue = count[1]
                else:
                    count_red = count[1]
                    count_blue = count[0]
                self.logger.info('The during the extended penalty shootout, the ball reached '
                                 f'the red goal area {count_blue} times and the blue goal area {count_red} times.')
                if count_red > count_blue:
                    self.logger.info('The winner is the red team.')
                elif count_blue > count_red:
                    self.logger.info('The winner is the blue team.')
                else:
                    count = [0, 0]
                    for i in range(5):
                        if self.game.penalty_shootout_time_to_touch_ball[2 * i] is not None:
                            count[0] += 1
                        if self.game.penalty_shootout_time_to_touch_ball[2 * i + 1] is not None:
                            count[1] += 1
                    if self.game.kickoff == self.game.red.id:
                        count_red = count[0]
                        count_blue = count[1]
                    else:
                        count_red = count[1]
                        count_blue = count[0]
                    self.logger.info(f'The ball was touched {count_red} times by the red player '
                                     f'and {count_blue} times by the blue player.')
                    if count_red > count_blue:
                        self.logger.info('The winner is the red team.')
                    elif count_blue > count_red:
                        self.logger.info('The winner is the blue team.')
                    else:
                        sum = [0, 0]
                        for i in range(5):
                            t = self.game.penalty_shootout_time_to_score[2 * i]
                            sum[0] += 60 if t is None else t
                            t = self.game.penalty_shootout_time_to_score[2 * i + 1]
                            sum[1] += 60 if t is None else t
                        if self.game.kickoff == self.game.red.id:
                            sum_red = sum[0]
                            sum_blue = sum[1]
                        else:
                            sum_red = sum[1]
                            sum_blue = sum[0]
                        self.logger.info(f'The red team took {sum_red} seconds to score '
                                         f'while blue team took {sum_blue} seconds.')
                        if sum_blue < sum_red:
                            self.logger.info('The winner is the blue team.')
                        elif sum_red < sum_blue:
                            self.logger.info('The winner is the red team.')
                        else:
                            sum = [0, 0]
                            for i in range(5):
                                t = self.game.penalty_shootout_time_to_reach_goal_area[2 * i]
                                sum[0] += 60 if t is None else t
                                t = self.game.penalty_shootout_time_to_reach_goal_area[2 * i + 1]
                                sum[1] += 60 if t is None else t
                            if self.game.kickoff == self.game.red.id:
                                sum_red = sum[0]
                                sum_blue = sum[1]
                            else:
                                sum_red = sum[1]
                                sum_blue = sum[0]
                            self.logger.info(f'The red team took {sum_red} seconds to send the ball to the goal area ' +
                                             f'while blue team took {sum_blue} seconds.')
                            if sum_blue < sum_red:
                                self.logger.info('The winner is the blue team.')
                            elif sum_red < sum_blue:
                                self.logger.info('The winner is the red team.')
                            else:
                                sum = [0, 0]
                                for i in range(5):
                                    t = self.game.penalty_shootout_time_to_touch_ball[2 * i]
                                    sum[0] += 60 if t is None else t
                                    t = self.game.penalty_shootout_time_to_touch_ball[2 * i + 1]
                                    sum[1] += 60 if t is None else t
                                if self.game.kickoff == self.game.red.id:
                                    sum_red = sum[0]
                                    sum_blue = sum[1]
                                else:
                                    sum_red = sum[1]
                                    sum_blue = sum[0]
                                self.logger.info(f'The red team took {sum_red} seconds to touch the ball ' +
                                                 f'while blue team took {sum_blue} seconds.')
                                if sum_blue < sum_red:
                                    self.logger.info('The winner is the blue team.')
                                elif sum_red < sum_blue:
                                    self.logger.info('The winner is the red team.')
                                else:
                                    self.logger.info('Tossing a coin to determine the winner.')
                                    if bool(random.getrandbits(1)):
                                        self.logger.info('The winer is the red team.')
                                    else:
                                        self.logger.info('The winer is the blue team.')


if __name__ == '__main__':
    Referee()
