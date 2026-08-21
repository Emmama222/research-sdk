from research-sdk.SSL.game_controller.Message import RefereeMessage,GameEvent
from research-sdk.SSL.game_controller.common import Command,Stage,GameEventType,Team,PacketType, GameState
from research-sdk.network.ssl_sockets import GameControl

from research-sdk.process_workers.worker import BaseWorker
from multiprocessing import Queue
from enum import Enum,auto
import time
import time



class GCfsm (BaseWorker):
    def __init__(self,is_running,logger):
        super().__init__(is_running,logger)


        self.last_ref_msg = None
        # state, command, event, stage
        self.current_command = None
        self.current_event = None
        self.current_stage = None
        self.current_state = None
        # cards
        self.fouls = 0
        self.yellow_cards = 0
        self.yellow_card_active:int = 0
        self.red_cards = 0
        self.robots_active = 0
        self.max_robots = 6 #small size league team member

        # last known ball_left_field_location
        self.last_blf_location = None
        self.recv = GameControl(is_running=is_running)


    def setup(self,*args):
        output_q, us_yellow, us_positive, team_name = args

        self.output_q = output_q
        self.us_yellow = us_yellow
        self.us_positive = us_positive
        self.team_name = team_name
        self.logger.info (f"[GCP] : Setup Complete {self.output_q=}, {us_yellow=}, {us_positive=}, {team_name=}")

        self.us_positive = us_positive
        self.team_name = team_name
        self.logger.info (f"[GCP] : Setup Complete {self.output_q=}, {us_yellow=}, {us_positive=}, {team_name=}")

    def step(self):
        # listen from GameControl socket
        new_data = self.recv.listen()
        # if the socket says None
        if new_data is None:
            self.logger.error("[GCP] received None from Socket")
            # time.sleep(1) # wait one sec
            raise AttributeError("received None from Socket") # if this is none, continue

        new_ref_msg:RefereeMessage = RefereeMessage.from_proto(new_data)
        # no previous packets
        if self.last_ref_msg is not None:
            # check if the timestamp is before
            if new_ref_msg.packet_timestamp < self.last_ref_msg.packet_timestamp:
                return

        # otherwise :
        self.last_ref_msg = new_ref_msg
        # check team color if this changes, basically resets everything
        self.check_color_side(new_ref_msg)
        # check for card and foul changes, add / remove robot from field
        self.check_cards(new_ref_msg)
        # check for state changes, forward new decided state (see GameState Enum)
        self.check_state(new_ref_msg)
        self.forward_gc_status(new_ref_msg)
        # check for game event : ball placement location (for now)
        self.check_game_events(new_ref_msg)

    def check_cards(self,new_ref_msg:RefereeMessage):
        update_numbers = False
        if self.us_yellow is None or new_ref_msg.yellow is None or new_ref_msg.blue is None:
            return # no color identified = > do nothing

        # check yellow cards in our team
        yellow_cards = new_ref_msg.yellow.yellow_cards if self.us_yellow == True else new_ref_msg.blue.yellow_cards

        if self.yellow_cards != yellow_cards :  # number not equal
            print(f"yellow card number changed : {yellow_cards}")
            self.yellow_cards = yellow_cards

        # check how many are still active
        yellow_card_active:int = len(new_ref_msg.yellow.yellow_card_times) if self.us_yellow==True else len(new_ref_msg.blue.yellow_card_times)

        if yellow_card_active != self.yellow_card_active: # if this has changes (more / less)
            print(f"yellow card times changed : {yellow_card_active}")
            self.yellow_card_active = yellow_card_active
            # we need to update our active robot numbers
            update_numbers = True


        # check red cards in our Team
        red_cards = new_ref_msg.yellow.red_cards if self.us_yellow == True else new_ref_msg.blue.red_cards
        # check if there's number changes from the record
        if self.red_cards != red_cards :
            print(f"red card number changed : {red_cards}")
            self.red_cards = red_cards
            # update active robot *red card = permanently remove
            update_numbers = True


        # checking fouls in our team
        fouls = new_ref_msg.yellow.foul_counter if self.us_yellow == True else new_ref_msg.blue.foul_counter
        if self.fouls != fouls:
            print(f"Foul Counter has changed : {fouls}")
            self.fouls = fouls # 3 fouls = 1 yellow card

        if update_numbers is True :
            self.fouls = fouls # 3 fouls = 1 yellow card

        if update_numbers is True :
            self.update_robot_numbers()


    def update_robot_numbers(self):
        # robots away = how many we need to take out
        # robots away = how many we need to take out
        robots_away = self.red_cards + self.yellow_card_active
        # check how many robots should be active now
        robots_active =  self.max_robots - robots_away

        robots_active =  self.max_robots - robots_away

        if robots_active <= 0:
            robots_active = 0
        # if this is different from our record
        if robots_active == self.robots_active:
            return
        else:
            packet = (PacketType.ROBOTS_ACTIVE,robots_active)
            self.output_q.put_nowait(packet)
            self.robots_active = robots_active


    def check_color_side(self,new_ref_msg:RefereeMessage):
        our_team_name :str = self.team_name
        us_positive:bool = None
        us_yellow:bool = None

        if new_ref_msg.yellow.name == our_team_name:
            us_yellow = True
        elif new_ref_msg.blue.name == our_team_name:
            us_yellow = False

        # us_positive = True means WE ARE on the +x half (our goal at +x, attack toward -x).
        # blue on +x → yellow on -x → us_positive=False for yellow.
        # blue on -x → yellow on +x → us_positive=True for yellow.
        if new_ref_msg.blue_team_on_positive_half is None:
            pass
        elif new_ref_msg.blue_team_on_positive_half is True:
            us_positive = False if us_yellow == True else True
        elif new_ref_msg.blue_team_on_positive_half is False:
            us_positive = True if  us_yellow == True else False

        if self.us_yellow != us_yellow or self.us_positive != us_positive:
            self.us_yellow = us_yellow
            self.us_positive = us_positive
            print(f"we are now yellow : {us_yellow} , positive: {us_positive}")
            packet = (PacketType.SWITCH_TEAM, {"YELLOW" : self.us_yellow,"POSITIVE": self.us_positive})
            self.output_q.put_nowait(packet)


        elif self.us_yellow is None:
            return




    def check_state(self,new_ref_msg:RefereeMessage):
        self.update_state(new_ref_msg.command, new_ref_msg.stage)
        self.current_stage = new_ref_msg.stage
        self.current_command = new_ref_msg.command
        # Forward ball placement target whenever it is present
        if new_ref_msg.designated_position is not None:
            pos = (new_ref_msg.designated_position.x, new_ref_msg.designated_position.y)
            self.output_q.put_nowait((PacketType.BALL_PLACEMENT_POS, pos))

    def update_state(self, command, stage):
        if not isinstance(command, Command) or not isinstance(stage, Stage):
            return

        if command == Command.HALT:
            state = GameState.HALTED
        elif command == Command.STOP:
            state = GameState.STOPPED
        elif command == Command.PREPARE_KICKOFF_YELLOW:
            state = GameState.OUR_PREPARE_KICKOFF if self.us_yellow is True else GameState.ENEMY_PREPARE_KICKOFF
        elif command == Command.PREPARE_KICKOFF_BLUE:
            state = GameState.OUR_PREPARE_KICKOFF if self.us_yellow is False else GameState.ENEMY_PREPARE_KICKOFF
        elif command == Command.PREPARE_PENALTY_YELLOW:
            state = GameState.OUR_PREPARE_PENALTY if self.us_yellow is True else GameState.ENEMY_PREPARE_PENALTY
        elif command == Command.PREPARE_PENALTY_BLUE:
            state = GameState.OUR_PREPARE_PENALTY if self.us_yellow is False else GameState.ENEMY_PREPARE_PENALTY
        elif command in {Command.DIRECT_FREE_YELLOW, Command.INDIRECT_FREE_YELLOW}:
            state = GameState.OUR_FREE_KICK if self.us_yellow is True else GameState.ENEMY_FREE_KICK
        elif command in {Command.DIRECT_FREE_BLUE, Command.INDIRECT_FREE_BLUE}:
            state = GameState.OUR_FREE_KICK if self.us_yellow is False else GameState.ENEMY_FREE_KICK
        elif command == Command.BALL_PLACEMENT_YELLOW:
            state = GameState.OUR_BALL_PLACEMENT if self.us_yellow is True else GameState.ENEMY_BALL_PLACEMENT
        elif command == Command.BALL_PLACEMENT_BLUE:
            state = GameState.OUR_BALL_PLACEMENT if self.us_yellow is False else GameState.ENEMY_BALL_PLACEMENT

        elif command == Command.FORCE_START:
            state = GameState.RUNNING
        elif command in {Command.DIRECT_FREE_YELLOW, Command.INDIRECT_FREE_YELLOW}:
            state = GameState.OUR_FREE_KICK if self.us_yellow is True else GameState.ENEMY_FREE_KICK
        elif command in {Command.DIRECT_FREE_BLUE, Command.INDIRECT_FREE_BLUE}:
            state = GameState.OUR_FREE_KICK if self.us_yellow is False else GameState.ENEMY_FREE_KICK
        elif command == Command.NORMAL_START:
            if self.current_command == Command.PREPARE_KICKOFF_YELLOW:
                state = GameState.OUR_KICKOFF if self.us_yellow is True else GameState.ENEMY_KICKOFF
            elif self.current_command == Command.PREPARE_KICKOFF_BLUE:
                state = GameState.OUR_KICKOFF if self.us_yellow is False else GameState.ENEMY_KICKOFF
            elif self.current_command == Command.PREPARE_PENALTY_YELLOW:
                state = GameState.OUR_PENALTY_SHOOTOUT if self.us_yellow is True else GameState.ENEMY_PENALTY_SHOOTOUT
            elif self.current_command == Command.PREPARE_PENALTY_BLUE:
                state = GameState.OUR_PENALTY_SHOOTOUT if self.us_yellow is False else GameState.ENEMY_PENALTY_SHOOTOUT
            else:
                state = GameState.RUNNING
        elif command in {Command.TIMEOUT_YELLOW, Command.TIMEOUT_BLUE}:
            state = GameState.HALTED
        else:
            if stage in (Stage.NORMAL_HALF_TIME, Stage.EXTRA_HALF_TIME):
                state = GameState.HALF_TIME
            else:
                state = GameState.HALTED

        if state != self.current_state:
            packet = (PacketType.NEW_STATE, state)
            print(f"new state: {state}")
            self.output_q.put(packet)
            self.current_state = state

    def forward_gc_status(self, new_ref_msg: RefereeMessage):
        our_info = None
        if self.us_yellow is True:
            our_info = new_ref_msg.yellow
        elif self.us_yellow is False:
            our_info = new_ref_msg.blue

        packet = (
            PacketType.GC_STATUS,
            {
                "stage": new_ref_msg.stage,
                "command": new_ref_msg.command,
                "state": self.current_state,
                "us_yellow": self.us_yellow,
                "us_positive": self.us_positive,
                "yellow_cards": None if our_info is None else our_info.yellow_cards,
                "red_cards": None if our_info is None else our_info.red_cards,
                "fouls": None if our_info is None else our_info.foul_counter,
                "yellow_card_times": [] if our_info is None else list(our_info.yellow_card_times),
                "packet_timestamp": new_ref_msg.packet_timestamp,
                "received_at": time.time(),
            },
        )
        self.output_q.put_nowait(packet)



    def check_game_events(self,new_ref_msg:RefereeMessage):
        game_events = new_ref_msg.game_events
        if len(game_events) == 0:
            return

        for e in game_events:
            if e.type == GameEventType.BALL_LEFT_FIELD_TOUCH_LINE or e.type == GameEventType.BALL_LEFT_FIELD_GOAL_LINE:
                # self.forward_ball_location(e.event_data)
                print("ball_left_field")
            if e.type == GameEventType.BOT_SUBSTITUTION :
                if e.by_team == Team.YELLOW if self.us_yellow == True else Team.BLUE:
                    print("we sub robot")

    def forward_ball_location(self,event_data):
        location = event_data.location.vector
        if location is not None and self.last_blf_location != location:
                packet = (PacketType.BLF_LOCATION, location)
                self.output_q.put_nowait(packet)
