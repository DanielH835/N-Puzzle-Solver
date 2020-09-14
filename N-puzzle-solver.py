from __future__ import division
from __future__ import print_function

import sys
import math
import time
import queue as Q
import heapq 
import resource

class PuzzleState(object):
    """
        The PuzzleState stores a board configuration and implements
        movement instructions to generate valid children.
    """
    
    def __init__(self, config, n, parent=None, action="Initial", cost=0):
        """
        :param config->List : Represents the n*n board, for e.g. [0,1,2,3,4,5,6,7,8] represents the goal state.
        :param n->int : Size of the board
        :param parent->PuzzleState
        :param action->string
        :param cost->int
        """
        if n*n != len(config) or n < 2:
            raise Exception("The length of config is not correct!")
        if set(config) != set(range(n*n)):
            raise Exception("Config contains invalid/duplicate entries : ", config)

        self.n        = n
        self.cost     = cost
        self.parent   = parent
        self.action   = action
        self.config   = config
        self.children = []

        # Get the index and (row, col) of empty block
        self.blank_index = self.config.index(0)

    def display(self):
        """ Display this Puzzle state as a n*n board """
        for i in range(self.n):
            print(self.config[3*i : 3*(i+1)])

    def move_up(self):
        """ 
        Moves the blank tile one row up.
        :return a PuzzleState with the new configuration
        """
        newConfig = list(self.config)
        
        if self.blank_index != 0 and self.blank_index != 1 and self.blank_index != 2:
            temp = newConfig[self.blank_index]
            newConfig[self.blank_index] = newConfig[self.blank_index-3]
            newConfig[self.blank_index-3] = temp
            
            newState = PuzzleState(newConfig, self.n, self, "Up", self.cost+1)
            
            return newState
      
    def move_down(self):
        """
        Moves the blank tile one row down.
        :return a PuzzleState with the new configuration
        """
        newConfig = list(self.config)
        
        if self.blank_index != 6 and self.blank_index != 7 and self.blank_index != 8:
            temp = newConfig[self.blank_index]
            newConfig[self.blank_index] = newConfig[self.blank_index+3]
            newConfig[self.blank_index+3] = temp
            
            newState = PuzzleState(newConfig, self.n, self, "Down", self.cost+1)
            
            return newState
      
    def move_left(self):
        """
        Moves the blank tile one column to the left.
        :return a PuzzleState with the new configuration
        """
        newConfig = list(self.config)
        
        if self.blank_index != 0 and self.blank_index != 3 and self.blank_index != 6:
            temp = newConfig[self.blank_index]
            newConfig[self.blank_index] = newConfig[self.blank_index-1]
            newConfig[self.blank_index-1] = temp
            
            newState = PuzzleState(newConfig, self.n, self, "Left", self.cost+1)
            
            return newState

    def move_right(self):
        """
        Moves the blank tile one column to the right.
        :return a PuzzleState with the new configuration
        """
        newConfig = list(self.config)
        
        if self.blank_index != 2 and self.blank_index != 5 and self.blank_index != 8:
            temp = newConfig[self.blank_index]
            newConfig[self.blank_index] = newConfig[self.blank_index+1]
            newConfig[self.blank_index+1] = temp
            
            newState = PuzzleState(newConfig, self.n, self, "Right", self.cost+1)
            
            return newState
      
    def expand(self):
        """ Generate the child nodes of this node """
        
        # Node has already been expanded
        if len(self.children) != 0:
            return self.children
        
        # Add child nodes in order of UDLR
        children = [
            self.move_up(),
            self.move_down(),
            self.move_left(),
            self.move_right()]

        # Compose self.children of all non-None children states
        self.children = [state for state in children if state is not None]
        return self.children

def writeOutput(path, cost, expanded, maxDepth, time):
    
    ram = (resource.getrusage(resource.RUSAGE_SELF))

    with open('output.txt', 'w') as f:
        f.write("path_to_goal: " + str(path) + "\n")
        f.write("cost_of_path: " + str(cost) + "\n")
        f.write("nodes_expanded: " + str(expanded) + "\n")
        f.write("search_depth: " + str(cost) + "\n")
        f.write("max_search_depth: " + str(maxDepth) + "\n")
        f.write("running_time: " + str(time) + "\n")
        f.write("max_ram_usage: " + str(ram[2] / 1000000) + "\n")
    f.closed
    
def findPath(finalState):
    
    path = list()

    while finalState.action != "Initial":
        path.append(finalState.action)
        finalState = finalState.parent
   
    path.reverse()    
    return path

def bfs_search(initial_state):
    """BFS search"""
    start_time  = time.time()
    
    frontier = Q.Queue()
    frontier.put(initial_state)
    frontierSet = set()
    frontierSet.add(tuple(initial_state.config))
    explored = set()
    countChild = 0
    maxDepth = 0
    
    while not frontier.empty():
        state = frontier.get()
        explored.add(tuple(state.config))
        frontierSet.remove(tuple(state.config))
        
        if test_goal(state) == True:
            path = findPath(state)
            cost = len(path)
            writeOutput(path, cost, countChild, maxDepth,(time.time() - start_time))
            return True
        
        countChild += 1
        
        children = state.expand()
        for child in children:
            if tuple(child.config) not in frontierSet and tuple(child.config) not in explored:
                frontier.put(child)  
                frontierSet.add(tuple(child.config))
                
                if child.cost > maxDepth:
                    maxDepth = child.cost
        
    return False
     
def dfs_search(initial_state):
    """DFS search"""
    start_time  = time.time()
    
    frontier = list()
    frontier.append(initial_state)
    frontierSet = set()
    frontierSet.add(tuple(initial_state.config))
    explored = set()
    countChild = 0
    maxDepth = 0
    
    while len(frontier) != 0:
        state = frontier.pop()
        explored.add(tuple(state.config))
        frontierSet.remove(tuple(state.config))
        
        if test_goal(state) == True:
            path = findPath(state)
            cost = len(path)
            writeOutput(path, cost, countChild, maxDepth,(time.time() - start_time))
            return True
        
        countChild += 1
        
        children = state.expand()
        children.reverse()
        for child in children:
            if tuple(child.config) not in frontierSet and tuple(child.config) not in explored:
                frontier.append(child)
                frontierSet.add(tuple(child.config))
                
                if child.cost > maxDepth:
                    maxDepth = child.cost
    
    return False

def A_star_search(initial_state):
    """A * search"""
    start_time = time.time()
    
    totalCost = calculate_total_cost(initial_state) + initial_state.cost
    count = 0
    frontier = []
    heapq.heapify(frontier)
    heapq.heappush(frontier, (totalCost, count, initial_state))
    count += 1
    frontierSet = set()
    frontierSet.add(tuple(initial_state.config))    
    explored = set()
    countChild = 0
    maxDepth = 0
    check = False
    
    while len(frontier) != 0:
        state = heapq.heappop(frontier)
        explored.add(tuple(state[2].config))
        frontierSet.remove(tuple(state[2].config))
        
        if test_goal(state[2]) == True:
            path = findPath(state[2])
            cost = len(path)
            writeOutput(path, cost, countChild, maxDepth,(time.time() - start_time))
            return True 
        
        if check == True:
            countChild += 1
            check = False
        children = state[2].expand()
        for child in children:
            if tuple(child.config) not in frontierSet and tuple(child.config) not in explored:
                check = True
                totalCost = calculate_total_cost(child) + child.cost
                heapq.heappush(frontier, (totalCost, count, child))
                count += 1
                frontierSet.add(tuple(child.config))
                
                if child.cost > maxDepth:
                    maxDepth = child.cost

    return False
    
def calculate_total_cost(state):
    """calculate the total estimated cost of a state"""

    manDist = 0
    
    for i in range(0, 9):
        manDist += calculate_manhattan_dist(i, state.config[i], 3)
     
    return manDist
    

def calculate_manhattan_dist(idx, value, n):
    """calculate the manhattan distance of a tile"""
    
    distance = abs((value // n) - (idx // n)) + abs((value % n) - (idx % n))
    
    return distance
    

def test_goal(puzzle_state):
    """test the state is the goal state or not"""
    
    my_list = list(puzzle_state.config)
    
    for i in range(0, len(my_list)):
        if my_list[i] != i:
            return False
    
    return True
        

# Main Function that reads in Input and Runs corresponding Algorithm
def main():
    search_mode = sys.argv[1].lower()
    begin_state = sys.argv[2].split(",")
    begin_state = list(map(int, begin_state))
    board_size  = int(math.sqrt(len(begin_state)))
    hard_state  = PuzzleState(begin_state, board_size)
    start_time  = time.time()
    
    if   search_mode == "bfs": bfs_search(hard_state)
    elif search_mode == "dfs": dfs_search(hard_state)
    elif search_mode == "ast": A_star_search(hard_state)
    else: 
        print("Enter valid command arguments !")
        
    end_time = time.time()
    print("Program completed in %.3f second(s)"%(end_time-start_time))

if __name__ == '__main__':
    main()
