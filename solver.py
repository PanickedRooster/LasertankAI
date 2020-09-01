#!/usr/bin/python
import sys
# from laser_tank import LaserTankMap
from queue import Queue
from queue import PriorityQueue
import numpy as np
import hashlib
from laser_tank import LaserTankMap
import time
import copy


"""
Template file for you to implement your solution to Assignment 1.

COMP3702 2020 Assignment 1 Support Code
"""


#
#
# Code for any classes or functions you need can go here.
#
#

class LaserTankState:
    def __init__(self, lasertank, cost, flag_pos):
        self.lasertank = lasertank
        self.flag_pos = flag_pos
        self.moves = lasertank.MOVES
        self.value_for_priority = 0
        self.cost = cost
        self.id = self.__hash__()

    def __eq__(self, other):
        return self.id == other.id

    def __hash__(self):
        stuff = [item for sublist in self.lasertank.grid_data for item in sublist]
        return hash((self.lasertank.player_x, self.lasertank.player_y, self.lasertank.player_heading) + tuple(stuff))

    def __lt__(self, other):
        return self.value_for_priority < other.value_for_priority

    def get_neighborlist(self):
        neighborlist = []
        for action in self.moves:
            data = [row[:] for row in self.lasertank.grid_data]
            temp = LaserTankMap(self.lasertank.x_size, self.lasertank.y_size, data, self.lasertank.player_x,
                                self.lasertank.player_y, self.lasertank.player_heading)
            temp.apply_move(action)
            neighbor = LaserTankState(temp, 1, self.flag_pos)
            neighborlist.append((neighbor, action))
        return neighborlist

    def estimate_cost_to_go(self, heuristic_mode):
        if heuristic_mode == 'zeroed':
            cost_to_go_estimate = 0
        elif heuristic_mode == 'manhattan':
            cost_to_go_estimate = abs(self.flag_pos[0] - self.lasertank.player_x)
            cost_to_go_estimate += abs(self.flag_pos[1] - self.lasertank.player_y)
        else:
            raise NotImplementedError(heuristic_mode)
        return cost_to_go_estimate

def write_output_file(filename, actions):
    """
    Write a list of actions to an output file. You should use this method to write your output file.
    :param filename: name of output file
    :param actions: list of actions where is action is in LaserTankMap.MOVES
    """
    f = open(filename, 'w')
    for i in range(len(actions)):
        f.write(str(actions[i]))
        if i < len(actions) - 1:
            f.write(',')
    f.write('\n')
    f.close()

def search_bfs(start):
    begin_clock = time.time()
    log = dict()
    log['nvextex_explored_(with_duplicates)'] = 0

    explored = set()
    fringe = Queue()
    fringe.put((start, []))

    while fringe.qsize() > 0:
        current, path = fringe.get()
        explored.add(current.id)

        log['nvextex_explored_(with_duplicates)'] += 1
        for neighbor, action in current.get_neighborlist():
            if neighbor.lasertank.is_finished():
                log['nvertex_in_fringe_at_termination'] = fringe.qsize()
                log['nvextex_explored'] = len(explored)
                log['action_path'] = path + [action]
                log['elapsed_time_in_minutes'] = (time.time() - begin_clock)
                print(log['elapsed_time_in_minutes'])
                print(log['nvextex_explored'])
                return log['action_path']
            if neighbor.id in explored:
                # print("skipped")
                continue

            fringe.put((neighbor, path + [action]))

    # raise RuntimeError('No Solution!')

def search_astar(start, heuristic_mode='manhattan'):
    begin_clock = time.time()
    log = dict()
    log['nvextex_explored_(with_duplicates)'] = 0

    fringe = PriorityQueue()
    fringe.put(start)
    explored = {start.id: start.cost} # a dict of `vertex: cost_so_far`
    path = {start.id: []} # a dict of `vertex: actions`
    log['nvextex_explored_(with_duplicates)'] += 1

    while fringe.qsize() > 0:
        current = fringe.get()
        if current.lasertank.is_finished():
            log['nvertex_in_fringe_at_termination'] = fringe.qsize()
            log['nvextex_explored'] = len(explored)
            log['action_path'] = path[current.id]
            log['solution_path_cost'] = explored[current.id]
            log['elapsed_time_in_minutes'] = (time.time() - begin_clock)
            print(log['action_path'], log['elapsed_time_in_minutes'])
            return log['action_path']

        for neighbor, action in current.get_neighborlist():
            cost_so_far = explored[current.id] + current.cost
            if (neighbor.id not in explored) or (cost_so_far < explored[neighbor.id]):
                explored[neighbor.id] = cost_so_far
                path[neighbor.id] = path[current.id] + [action]
                log['nvextex_explored_(with_duplicates)'] += 1

                vfp = cost_so_far + current.estimate_cost_to_go(heuristic_mode)
                neighbor.value_for_priority = vfp
                fringe.put(neighbor)

def main(arglist):
    input_file = arglist[0]
    output_file = arglist[1]

    # Read the input testcase file
    game_map = LaserTankMap.process_input_file(input_file)
    flag_pos = (0,0)
    for i in range(game_map.x_size):
        for j in range(game_map.y_size):
            if game_map.grid_data[j][i] == game_map.FLAG_SYMBOL:
                flag_pos = (i, j)
    enter = LaserTankState(game_map, 0, flag_pos)

    actions = []

    # print(game_map)

    # actions.append(search_bfs(enter))
    actions = search_astar(enter)
    # actions.append(search_astar(enter))

    #
    #
    # Code for your main method can go here.
    #
    # Your code should find a sequence of actions for the agent to follow to reach the goal, and store this sequence
    # in 'actions'.
    #
    #

    # Write the solution to the output file
    write_output_file(output_file, actions)


if __name__ == '__main__':
    main(sys.argv[1:])

