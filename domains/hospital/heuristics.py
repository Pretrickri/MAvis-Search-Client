# coding: utf-8
#
# Copyright 2021 The Technical University of Denmark
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#    http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from __future__ import annotations
import sys
import itertools
import numpy as np
from utils import pos_add, pos_sub, APPROX_INFINITY
from collections import deque, defaultdict

import domains.hospital.state as h_state
import domains.hospital.goal_description as h_goal_description
import domains.hospital.level as h_level

class HospitalGoalCountHeuristics:

    # Write your own implementation of the goal count heuristics here. Remember that the goal count heuristics is simply the number of goals that are not satisfied in the current state. 


    def __init__(self):
        pass

    def preprocess(self, level: h_level.HospitalLevel):
        # This function will be called a single time prior to the search allowing us to preprocess the level such as
        # pre-computing lookup tables or other acceleration structures
        pass

    def h(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription) -> int:

        numGoals = len(goal_description.goals)
        print(f'goal_description.goals: {numGoals}', file = sys.stderr)
        for index in range(goal_description.num_sub_goals()):
            agentsubGoal = goal_description.get_sub_goal(index)
            if agentsubGoal.is_goal(state):
                numGoals -= 1

        print(f'Heuristic: {numGoals}', file = sys.stderr)
        return numGoals


class HospitalAdvancedHeuristics:

    # Write your own implementation of the advanced heuristics here.

    def __init__(self):
        self.distances = None
        self.goal_chars = None
        self.agent_chars = None
        self.goals = None

    def preprocess(self, level: h_level.HospitalLevel):

        pass

    def h(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription) -> int:

        # MAVIS 2 HEURISTICS
        # -------------------

        # totalHeuristics = 0

        # dict = {'0' : (0, 0)}
        
        # for agent in state.agent_positions:
        #     agent_position = agent[0]
        #     agent_index = agent[1]
        #     dict[agent_index] = agent_position

        # for (goal_position, goal_char, is_positive_literal) in goal_description.goals:
        #     totalHeuristics += self.vectorSimpleSum(goal_position, dict[goal_char])
        
        # return totalHeuristics # heuristic = 0



        # MAVIS 3 HEURISTICS
        # ------------------
        # THIS ONE DOESN'T SUPPORT MULTIPLE BOXES OF DIFFERENT TYPES.

        # totalHeuristics = 0
        # # print(goal_description.agent_goals, file=sys.stderr)
        
        # # WHERE WE DON'T HAVE A BOX IN THE GOAL
        # if state.box_at(goal_description.box_goals[0][0]) != -1:

        #     print("BOX NOT ON GOAL", file=sys.stderr)
        #     boxGoalPos = goal_description.box_goals[0][0]

        #     dBoxBoxgoal = APPROX_INFINITY
        #     chosenBox = tuple() # (tuple[0,0], '')

        #     # Find the box that is closest to the box goal
        #     # Find distance between box and box goal
        #     for box in state.box_positions:
        #         distance = self.ManhattanSum(box[0], boxGoalPos)

        #         if distance == 1 :
        #             chosenBox = box
        #             break
        #         if distance < dBoxBoxgoal:
        #             dBoxBoxgoal = distance
        #             chosenBox = box
            
        #     # Find distance between agent and box
        #     dAgentBox = self.ManhattanSum(state.agent_positions[0][0], chosenBox[0])
        #     # print("dstAgentAndBox = " + str(dAgentBox))

        #     # Find distance between box goal and agent goal
        #     dBoxgoalAgentgoal = 0
        #     if len(goal_description.agent_goals) != 0:
        #         dBoxgoalAgentgoal = self.ManhattanSum(goal_description.agent_goals[0][0], goal_description.box_goals[0][0])
        #     # print("dstBoxGoalAgentGoal = " + str(dBoxgoalAgentgoal))

        #     # Calculates total heuristics
        #     totalHeuristics = dBoxBoxgoal + dAgentBox + dBoxgoalAgentgoal

        # # WHERE WE DO HAVE A BOX IN THE BOX GOAL
        # else:
        #     if len(goal_description.agent_goals) != 0:
        #         dAgentAgentgoal = self.ManhattanSum(state.agent_positions[0][0], goal_description.agent_goals[0][0])
        #     totalHeuristics = dAgentAgentgoal



        # MAVIS 3 - ATTEMPT 2
        # -------------------

        if self.is_box_goal(state, goal_description):
            print("STATE TWO", file=sys.stderr)
            totalHeuristics = self.heuristics_state_two(state, goal_description)
        else:
            print("STATE ONE", file=sys.stderr)
            totalHeuristics = self.heuristics_state_one(state, goal_description)

        return totalHeuristics


    def ManhattanSum(self, point1: tuple, point2: tuple):
        """Returns the Manhattan Sum between two coordinates"""
        row1, col1 = point1
        row2, col2 = point2
        #print(f"point1row1 = {row1}, point1col1 = {col1}", file = sys.stderr)

        simpleSum = abs((row2 - row1) + (col2 - col1))
        return simpleSum

    def is_box_goal(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription):
        """Returns whether the given state satisfies all box goals in the goal description"""
        for (goal_position, goal_char, is_positive_literal) in goal_description.box_goals:
            char = state.box_at(goal_position)
            if is_positive_literal and goal_char != char[1]:
                return False
            elif not is_positive_literal and goal_char == char[1]:
                return False

        return True
    
    def heuristics_state_one(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription) -> int:
        """This heuristics is called when we are in the first state of the level. That is, when all the boxes are NOT on their
        respective goals"""

        # print("INSIDE STATE ONE", file=sys.stderr)

        boxDict = dict() # NOT SURE IF THIS WORKS
        box_goalDict = dict()
        totalHeuristics = 0

        # ADD ALL GOALS TO A DICTIONARY TO MAKE LOOK UP EASIER
        for (goal_pos, goal_char, ipL) in goal_description.box_goals:
            box_goalDict[goal_char] = goal_pos

        # ITERATE THROUGH ALL BOXES, FINDING THE DISTANCE BETWEEN THEM AND THEIR RESPECTIVE GOALS
        # ADD THE DISTANCES TO THE HEURISTIC
        for (pos, char) in state.box_positions:
            # IN CASE WE HAVE MORE THAN ONE BOX OF THE SAME TYPE, WE CHECK WHICH IS CLOSER TO ITS GOAL
            if char in boxDict:
                currentDist = self.ManhattanSum(box_goalDict[char], boxDict[char]) # The current "smallest distance"
                newDist = self.ManhattanSum(box_goalDict[char], pos) # The new "smallest distance"
                if newDist < currentDist:
                    # replace the currentDist by the newDist in the totalHeuristics
                    totalHeuristics -= currentDist
                    totalHeuristics += newDist
                    boxDict[char] = pos
                    #------------------------------#
            else:
                boxDict[char] = pos
                totalHeuristics += self.ManhattanSum(box_goalDict[char], pos)
        
        # FINDING THE DISTANCE FROM THE FARTHEST BOX_GOAL TO THE AGENT_GOAL
        # ADDING THE DISTANCE TO THE HEURISTIC
        if len(goal_description.agent_goals) != 0: # i.e. We have an agent goal
            biggest = 0
            for (goal_pos, goal_char, ipL) in goal_description.box_goals:
                dist = self.ManhattanSum(goal_pos, goal_description.agent_goals[0][0])
                if biggest < dist:
                    biggest = dist

            totalHeuristics += biggest
        
        print("TOTAL HEURISTIC1 = " + str(totalHeuristics), file=sys.stderr)
        return totalHeuristics
    
    def heuristics_state_two(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription) -> int:
        """This heuristics is called whenever the first heuristic state is satisfied. That is, when all the boxes are on their respective
        goals. Now the agent just have to go back to its goal."""
        if len(goal_description.agent_goals) != 0:
            totalHeuristics = self.ManhattanSum(state.agent_positions[0][0], goal_description.agent_goals[0][0])
            print("TOTAL HEURISTIC2 = " + str(totalHeuristics), file=sys.stderr)
            return totalHeuristics
        return 0