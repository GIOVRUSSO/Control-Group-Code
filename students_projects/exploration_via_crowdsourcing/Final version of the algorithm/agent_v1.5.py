import math
import numpy as np
import copy
import gui as gui
import random as r
from timeit import default_timer as timer


# This is the calculatione of the Kullback–Leibler divergence
def kl_div(p, q):
    sum = 0
    i = 0
    for x in p:
        sum = sum + p[i]*math.log2(p[i]/q[i])
        i = i + 1
    return sum

# This function calculate the expectation
def expectation(board, x, source):
    sum = 0
    i = 0
    # Iteration for every element of the source
    for p in source:
        pos = board._to_coordinates(x)
        # Determining what is the next position
        if(i == 1):
            pos[0] = pos[0] - 1 # left
        elif(i == 2):
            pos[1] = pos[1] + 1 # up
        elif(i == 3):
            pos[0] = pos[0] + 1 #rigth
        elif(i == 4):
            pos[1] = pos[1] - 1 #down
        i = i + 1
        # I verify if the next position is in the grid
        if pos[0] < board.dimension and pos[1] < board.dimension and pos[0] >= 0 and pos[1] >= 0:
            # If it is I multiply the source element with the grid element and I increment the sum
            sum = sum + p * board.grid[pos[0],pos[1]]
            #print("Cell value: ", self.grid[pos[0],pos[1]])
        else:
            sum = sum + p * -100
            #print("Outside")
    return sum


class Agent:

    def __init__ (self, name, position, board):
        self.name = name
        self.position = position
        self.board = board
        self.board.set_cell_reward(self.position, -10)

    # ---------- SOURCES ------------- #

    stay = [0.95, 0.01, 0.01, 0.01, 0.01]
    
    left = [0.01, 0.95, 0.01, 0.01, 0.01]
    
    up = [0.01, 0.01, 0.95, 0.01, 0.01]
    
    rigth = [0.01, 0.01, 0.01, 0.95, 0.01]
    
    down = [0.01, 0.01, 0.01, 0.01, 0.95]

    sources = [stay, left, up, rigth, down]

    # ---------------------------------- #

    # Convert the position from the input state to the coordinate state
    def _to_coordinates(self, x):
        return [x % self.board.dimension, x // self.board.dimension]

    # Convert the position from the coordinate state to the input state
    def _convert(self, pos):
        return pos[1]*self.board.dimension + pos[0] 

    # Move the agent to the cell given in input if the cell is near
    def advance(self, cell):
        
        print("Advance to the next cell...")
        pos = self._to_coordinates(self.position)
        cell = self._to_coordinates(cell)
        
        # If the cell isn't near a wrong call to teh method is made so I decide to stay
        if cell[0] != pos[0] and cell[1] != pos[1]:
            raise RuntimeError("<< Error in calling advance: the cell isn't near! >>")
            return self.position

        if(cell[0] > pos[0]): # go right
            self.board.set_cell_reward(self.position, +1)
            pos[0] = pos[0] + 1
            self.position = self._convert(pos)
            print("Movement to RIGHT ->", self.position)
            self.board.set_cell_reward(self.position, -10)
        elif(cell[0] < pos[0]): # go left
            # I mark the precedent cell with a little positive value to not stuck and prefer to go back
            self.board.set_cell_reward(self.position, +1)
            # I simulate the movement in the coordinate space
            pos[0] = pos[0] - 1
            # I convert the movement in the input space and I update the state of the agent
            self.position = self._convert(pos)
            print("Movement to LEFT ->", self.position)
            # I mark the new cell corresponding to the new state with a negative value because
            # if it is not the goal I don't want to stay here
            self.board.set_cell_reward(self.position, -10)
            # I return the index of the next cell
            return self.position
        elif(cell[1] > pos[1]): # go up
            self.board.set_cell_reward(self.position, +1)
            pos[1] = pos[1] + 1
            self.position = self._convert(pos)
            print("Movement to UP ->", self.position)
            self.board.set_cell_reward(self.position, -10)
            return self.position
        elif(cell[1] < pos[1]): # go down
            self.board.set_cell_reward(self.position, +1)
            pos[1] = pos[1] - 1
            self.position = self._convert(pos)
            print("Movement to DOWN ->", self.position)
            self.board.set_cell_reward(self.position, -10)
            return self.position
        else:
            print("STAY in position ", self.position)
            self.board.set_cell_reward(self.position, -10)
            return self.position


    # It work on a copy of the current board in phase of simulation
    def discover(self, goal, board):

        # This function in the case of multiple min element stores indexes of this
        # and return randomly one of this indexes
        def find_min_index(path):
            x = path[0]
            index = [0]
            for i in range(1,len(path)):
                if path[i] < x :
                    x = path[i]
                    index = [i]
                elif path[i] == x :
                    index.append(i)
            return index[r.randint(0,len(index)-1)]

        # Basic condition that if it is verified it does not execute the rest of the algorithm
        if self.position == goal:
            print("Discovery reached goal: ", self.position)
            return goal

        # Conversion of the position in coordinates to work with two index
        pos = self._to_coordinates(self.position)
        goal = self._to_coordinates(goal)

        # Saturation check
        if board.grid[pos[0],pos[1]] < -29:
            # print("UNREACHABLE GOAL")
            return -1 

        # Loading the pmf
        pmf = self.next_cell(pos,goal)

        # In this step I calculate the Kullback–Leibler divergence for every source.
        # For each iteration I have a source and I compute the KL of this source and the pmf and
        # at the result of the operation is subtracted the expectation of this source respect to the
        # current position.
        # The final value is stored in a list, so at the end I have a number of value in the list
        # that is equal the number of sources
        v = []
        for s in self.sources:
            v.append(kl_div(s, pmf) - expectation(board, self.position, s))
            # Decomment this line for DEBUG
            #print("KL:", kl_div(pmf, s) ,"Expectation:", board.expectation(self.position, s), "Res:", kl_div(pmf, s) - board.expectation(self.position, s))

        # I take the index of a minimum value of the list
        i = find_min_index(v)
        # print("vector:",v)
        # At this index corresponds a movement
        if(i == 1): # LEFT
            # I mark the precedent cell with a little positive value to not stuck and prefer to go back
            board.set_cell_reward(self.position, +1)
            # I simulate the movement in the coordinate space
            pos[0] = pos[0] - 1
            # I convert the movement in the input space and I update the state of the agent
            self.position = self._convert(pos)
        elif(i == 2):   # UP
            board.set_cell_reward(self.position, +1)
            pos[1] = pos[1] + 1
            self.position = self._convert(pos)
        elif(i == 3):   # RIGHT
            board.set_cell_reward(self.position, +1)
            pos[0] = pos[0] + 1
            self.position = self._convert(pos)
        elif(i == 4):   # DOWN
            self.board.set_cell_reward(self.position, +1)
            pos[1] = pos[1] - 1
            self.position = self._convert(pos)
        # STAY
        print("Discovery state coordinates: ", self.position, "(", pos, "->", goal, ")")
        # I mark the new cell corresponding to the new state with a negative value because
        # if it is not the goal I don't want to stay here
        board.set_cell_reward(self.position, -10)
        # I return the index of the next cell
        return self.position


    # Look forward for n_steps
    def move(self, goal, n_steps, n_advance=100):

        # Final check
        if(self.position == goal):
            # print("### GOAL ACHIEVED ###")
            return [goal]

        # Saving the current position of the agent
        x = self.position
        # Path to be generated by the discovery
        forward_path = [x]
        # This is a fase of simulation so the discover works on the copy of the board
        board = copy.deepcopy(self.board)
        print("Start the discovery...")
        # I call the discover n_steps time
        while(n_steps > 0):
            pos = self.discover(goal, board)
            if(pos == -1):
                return -1
            forward_path.append(pos)
            n_steps = n_steps - 1
        print("Output of the discovery: ",forward_path)
        # I save the next position in the case of a total loop
        next_cell = forward_path[1]
        forward_path = self.optimal_path(forward_path)
        print("Optimized output of the discovery: ", forward_path)
        # After the simulation is necessary to move really the agent and store the result
        # I restore the position
        self.position = x
        # I take the next position of the path: remember that forward_path[0] is the current position
        if(len(forward_path) < 2):
            print("Max self loop detected")
            self.board.set_cell_reward(next_cell, -10)
            self.advance(self.position)
            return forward_path
        # I remove the current position from the path
        forward_path = forward_path[1:]
        path_lenght = len(forward_path)
        if n_advance > path_lenght:
            n_advance = path_lenght
        for i in range(0, n_advance):
            self.advance(forward_path[i])
        return forward_path[:n_advance]


    # This in the new pmf
    def next_cell(self, pos, goal):
        delta_x = goal[0] - pos[0]
        delta_y = goal[1] - pos[1]
        # Goal achieved
        if(delta_x == 0 and delta_y == 0):
            return self.stay()
        # Ladder style
        if(delta_x > 0):
            if(delta_x >= abs(delta_y)):
                return self.rigth
        if(delta_x < 0):
            if(abs(delta_x) >= abs(delta_y)):
                return self.left
        if(delta_y > 0):
            if(abs(delta_x) < delta_y):
                return self.up
        if(delta_y < 0):
            if(abs(delta_x) < abs(delta_y)):
                return self.down
        raise RuntimeError('<< Position NOT valid! >>')


    # This method optimize the path removing self-loop
    def optimal_path(self, path):
        if(len(path) == 0): 
            return 'Path not generated'
        final_path = []
        while (len(path) > 0):
            cell = path[0]
            i = len(path) - 1
            # I search in the remaining path from rigth to left
            while(i > 0):
                # print("path di i:", path[i:])
                # print("path: ", path)
                if (cell == path[i]):
                    # I remove the element before the index
                    del path[:i]
                    i = 0
                # Two adiacent cells have other cells in between
                elif (self._is_near(cell, path[i])):
                    # I remove the element before the index of the current
                    # print("Elementi del ciclo esterno da rimuovere: ",path[1:i])
                    del path[1:i]
                    i = 0
                else:
                    i = i - 1
            # Adding the new cell
            final_path.append(cell)
            # Removing the cell from the remaining path
            path = path[1:]
        return final_path

    def _is_near(self, cell1, cell2):
        pos1 = self._to_coordinates(cell1)
        pos2 = self._to_coordinates(cell2)
        if(pos1[0] == pos2[0]):
            if(abs(pos1[1]-pos2[1]) == 1):
                return True
        elif(pos1[1] == pos2[1]): 
            if(abs(pos1[0]-pos2[0]) == 1):
                return True
        return False


# This class represent the space in which the agent can discover
class Board:

    # The board is implemented as a square grid in which the dimension represent the
    # number of cells on a side
    # Initially the value of the cell is zero for all 
    def __init__ (self, dimension):
        self.dimension = dimension
        self.grid = np.zeros((dimension,dimension))

    # Same as previous
    def _to_coordinates(self, x):
        return [x % self.dimension, x // self.dimension]

    # This function add a negative value to the grid in the position indicated by the list
    # that corresponds to an obstacle that does not let the agent through
    def set_obstacle(self, obs_list):
        for o in obs_list:
            pos = self._to_coordinates(o)
            self.grid[pos[0], pos[1]] = -100

    # This add a custom negative value to the cell
    def set_cell_reward(self, x, value):
        pos = self._to_coordinates(x)
        self.grid[pos[0], pos[1]] = self.grid[pos[0], pos[1]] + value

    def update(self, ffactor = 1):
        if(ffactor < 1):
            return -1
        i = 0
        for i in range(self.dimension):
            j = 0
            for j in range(self.dimension):
                if(self.grid[i][j] < 0 and self.grid[i][j] > -100):
                    if(self.grid[i][j] < -ffactor):
                        self.grid[i][j] = self.grid[i][j] + ffactor
                    else:
                        self.grid[i][j] = 0



def main():

    dim = int(input("Insert grid dimension (e.g. '3' for a grid 3x3): "))
    user_list = []
    n = int(input("Enter number of obstacles : "))
    for i in range(0, n):
        element = int(input())
        user_list.append(element)
    start = int(input("Insert the initial position of the agent: "))
    end = int(input("Insert the desired final position of the agent: "))
    n_step = int(input("Insert the resolution of the discovery: "))
    n_advance = int(input("Insert how many cells advance max in each step: "))
    ffactor = int(input("Insert the value of the forgetting factor (0 to disable): "))

    start_t = timer()

    path = []
    b = Board(dim)
    b.set_obstacle(user_list)
    a = Agent("car", start, b)
    i = 0
    x = [-1]
    # While the last element of the returned list is different from the goal
    while(x[-1] != end):
        i = i+1
        print("\nSTEP", i)
        x = a.move(end, n_step, n_advance)
        if(x == -1):
            print("\n### UNREACHABLE GOAL ###")
            return
        path.extend(x)
        b.update(ffactor)
        # print("Path of while: ", path)
    print("\n### GOAL ACHIEVED ###")
    end_t = timer()-start_t
    print("Path: ", path)
    print("Time taken:", end_t)

    input("\nPress any button to start simulation...\n")
    gui.start_simulation(dim, user_list, path, start)


if __name__ == '__main__':
    main()