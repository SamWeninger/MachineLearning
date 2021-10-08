#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os #for time functions
from search import * #for search engines
from sokoban import SokobanState, Direction, PROBLEMS #for Sokoban specific classes and problems
import math

def sokoban_goal_state(state):
  '''
  @return: Whether all boxes are stored.
  '''
  for box in state.boxes:
    if box not in state.storage:
      return False
  return True

def heur_manhattan_distance(state):
#IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #We want an admissible heuristic, which is an optimistic heuristic.
    #It must never overestimate the cost to get from the current state to the goal.
    #The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    #When calculating distances, assume there are no obstacles on the grid.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.
    
    # state -> SokobanState
    manhattan_distance = 0

    for box in state.boxes:
      manhattan_distance += manhattan_box_distance(box, state.storage)[0]
    return manhattan_distance

# helper for calculating manhattan distance
def manhattan_box_distance(box, goal_states): 
  hval = math.inf
  closest_state = None
  for goal_state in goal_states:
    dist = abs(box[0] - goal_state[0]) + abs(box[1] - goal_state[1])
    if dist < hval:
      hval = dist
      closest_state = goal_state
  return hval, closest_state


# Check for unsolveable cases, cases where boxes are stuck
def box_stuck(box, state, storage_spots):
  x, y = state.width - 1, state.height - 1 # sokobon board endpoints

  if box[0] == 0 or box[0] == x: # L/R edge
    if box[1] == 0 or box[1] == y: # corner
      return True
    
    # y-locations of goal states on same x-edge as box
    goal_x = {goal_state[1] for goal_state in storage_spots if goal_state[0] == box[0]}
    if not goal_x: # box stuck on edge
      return True
    else:
      blocking_boxes = []
      updated_obstacles = {obs[1] for obs in state.obstacles if obs[0] == box[0]}

      # check for adjacent boxes on edges
      for other_box in set(state.boxes) - set(box):
        if box[0] == other_box[0]:
          blocking_boxes.append(other_box[1])
          # obstacle blocking box on edge
          if abs(box[1] - other_box[1]) == 1: 
            return True

      # check for obstacles blocking boxes from goal states
      blocking_boxes.sort()
      blocking_len, i = len(blocking_boxes), 0

      while blocking_len > 1 and i < blocking_len - 1:
        if abs(blocking_boxes[i] - blocking_boxes[i+1]) == 1:
          updated_obstacles.add(blocking_boxes[i])
        i+=1
      for obstacle in updated_obstacles:
        # obstacle blocking box on edge
        if (box[1] < obstacle and obstacle < min(goal_x)) or (box[1] > obstacle and obstacle > max(goal_x)): 
          return True


  elif box[1] == 0 or box[1] == y: # Up/Down edge
   
    # x-locations of goal states on same y-edge as box
    goal_y = {goal_state[0] for goal_state in storage_spots if goal_state[1] == box[1]}
    
    if not goal_y: # box stuck on edge
      return True
    else:
      blocking_boxes = []
      updated_obstacles = {obs[0] for obs in state.obstacles if obs[1] == box[1]}

      # check for adjacent boxes on edges
      for other_box in set(state.boxes) - set(box):
        if box[1] == other_box[1]:
          blocking_boxes.append(other_box[0])
          # obstacle blocking box on edge
          if abs(box[0] - other_box[0]) == 1: 
            return True
      
      # check for obstacles blocking boxes from goal states
      blocking_boxes.sort()
      blocking_len, i = len(blocking_boxes), 0
      while blocking_len > 1 and i < blocking_len - 1:
        if abs(blocking_boxes[i] - blocking_boxes[i+1]) == 1:
          updated_obstacles.add(blocking_boxes[i])
        i+=1
      for obstacle in updated_obstacles:
        # obstacle blocking box on edge
        if (box[0] < obstacle and obstacle < min(goal_y)) or (box[0] > obstacle and obstacle > max(goal_y)): 
          return True

  return False # state is solveable


# check the number of obstacles in the way of a box and a goal state
def num_obstacles(box, closest_state, obstacles):
  oval = 0 # return

  # get the area endpoints that contain the box and goal state
  # this area or sub-section of the Sokobon board is checked for obstacles
  x_range = sorted([box[0], closest_state[0]])
  y_range = sorted([box[1], closest_state[1]])
  x_min, x_max = x_range[0], x_range[1]
  y_min, y_max = y_range[0], y_range[1]

  # count obstacles in sub-section of Sokobon board
  for obstacle in obstacles:
    if x_min <= obstacle[0] <= x_max and y_min <= obstacle[1] <= y_max:
      oval += 1

  return oval

# distance from robots to boxes (reward for robot moving closer to a box)
def heur_robot_distance(box, robots): 
  rval = math.inf
  
  for robot in robots:
    rval = min(rval, abs(box[0] - robot[0]) + abs(box[1] - robot[1]))
  return rval

# Check if a box is cornered (stuck) between obstacles
def obstacle_trap(box, state):
  up, down, left, right = (box[0], box[1] - 1), (box[0], box[1] + 1), (box[0] - 1, box[1]), (box[0] + 1, box[1])

  up_obstacle, down_obstacle, left_obstacle, right_obstacle = False, False, False, False

  # Check for obstacles adjacent to box
  if up in state.obstacles:
    up_obstacle = True
  if down in state.obstacles:
    down_obstacle = True
  if left in state.obstacles:
    left_obstacle = True
  if right in state.obstacles:
    right_obstacle = True

  if (up_obstacle and left_obstacle) or (left_obstacle and down_obstacle) or (down_obstacle and right_obstacle) or (right_obstacle and up_obstacle):
    return math.inf
  return 0

#SOKOBAN HEURISTICS
def trivial_heuristic(state):
  '''trivial admissible sokoban heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''
  count = 0
  for box in state.boxes:
    if box not in state.storage:
        count += 1
  return count

def heur_alternate(state):
#IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #heur_manhattan_distance has flaws.
    #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    #Your function should return a numeric value for the estimate of the distance to the goal.
    manhattan_distance = 0

    # only apply heuristic to boxes NOT in goal states
    updated_boxes = set(state.boxes) - set(state.storage)
    updated_storage = set(state.storage) - set(state.boxes)

    # evaluate each box in given state
    for box in updated_boxes:
      if box_stuck(box, state, updated_storage):
        return math.inf # stuck/unsolveable (infinite) state, don't evaluate

      # box-specific heuristic variables
      hval, rval, oval = 0, 0, 0

      # box-manhattan distance calculation (greedy)
      hval, goal_state = manhattan_box_distance(box, updated_storage)
      updated_storage.discard(goal_state) # one goal state per box

      # box-robot calculation (greedy)
      rval = heur_robot_distance(box, state.robots)

      # nieve obstacle check
      oval = num_obstacles(box, goal_state, state.obstacles)

      # obstacle cornering trap
      tval = obstacle_trap(box, state)

      manhattan_distance += hval + rval + oval + tval
      
    return manhattan_distance

def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.
    Use this function stub to encode the standard form of weighted A* (i.e. g + w*h)

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
  
    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    return sN.gval + weight*sN.hval

def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
#IMPLEMENT
  '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of anytime weighted astar algorithm'''

  # initialize search engine
  se = SearchEngine('custom', 'full')
  wrapped_fval_function = (lambda sN: fval_function(sN, weight))
  se.init_search(initial_state, sokoban_goal_state, heur_fn=heur_fn, fval_function=wrapped_fval_function)

  # return, cost parameters
  final = False
  costbound = math.inf

  # start timer
  search_start_time = os.times()[0]
  while os.times()[0] - search_start_time < timebound: # iteratively search within time bounds
    # Search for sokoban goal state
    search_result = se.search(timebound=(timebound - (os.times()[0] - search_start_time)), costbound=(math.inf, math.inf, costbound))[0] 
    
    weight = weight*.8 # decrement weight
    
    if search_result: # solution found, redefine the result and new cost bound
      if not final or search_result.gval + heur_fn(search_result) < final.gval + heur_fn(final):
        final = search_result

      costbound = min(costbound, final.gval + heur_fn(final))
    else: # no solution
      break

  return final

def anytime_gbfs(initial_state, heur_fn, timebound = 10):
#IMPLEMENT
  '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of anytime greedy best-first search'''
  
  # initialize search engine
  se = SearchEngine('best_first', 'full')
  se.init_search(initial_state, sokoban_goal_state, heur_fn=heur_fn)
  
  # return, cost parameters
  final = False
  costbound = math.inf

  # start timer
  search_start_time = os.times()[0] 
  while os.times()[0] - search_start_time < timebound: # iteratively search within time bounds
    # Search for sokoban goal state
    search_result = se.search(timebound=(timebound - (os.times()[0] - search_start_time)), costbound=(costbound, math.inf, math.inf))[0]
    
    if search_result: # solution found, redefine the result and new cost bound
      if not final or search_result.gval < final.gval:
        final = search_result
      costbound = min(costbound, final.gval)
    else: # no solution
      break

  return final