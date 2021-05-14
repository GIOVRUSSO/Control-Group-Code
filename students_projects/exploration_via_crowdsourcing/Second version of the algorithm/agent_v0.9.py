import math
import numpy as np
import copy

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

    def _convert(self, pos):
        return pos[1]*self.board.dimension + pos[0] 

    # Move the agent to the cell given in input if the cell is near
    def advance(self, cell):
        ### NO CHECK CONDITION OF NEAR CELL
        print("<<<Calling the advance>>>")
        print("Advance to", cell)
        pos = self._to_coordinates(self.position)
        cell = self._to_coordinates(cell)
        # print("Current pos", pos,"Advance to", cell)
        if(cell[0] > pos[0]): # go right
            self.board.set_cell_reward(self.position, +1)
            print("# GO RIGHT")
            pos[0] = pos[0] + 1
            self.position = self._convert(pos)
        elif(cell[0] < pos[0]): # go left
            # I mark the precedent cell with a little positive value to not stuck and prefer to go back
            self.board.set_cell_reward(self.position, +1)
            print("# GO LEFT")
            # I simulate the movement in the coordinate space
            pos[0] = pos[0] - 1
            # I convert the movement in the input space and I update the state of the agent
            self.position = self._convert(pos)
        elif(cell[1] > pos[1]): # go up
            self.board.set_cell_reward(self.position, +1)
            print("# GO UP")
            pos[1] = pos[1] + 1
            self.position = self._convert(pos)
        elif(cell[1] < pos[1]): # go down
            self.board.set_cell_reward(self.position, +1)
            print("# GO DOWN")
            pos[1] = pos[1] - 1
            self.position = self._convert(pos)
        else:
            print("# STAY")
        # I mark the new cell corresponding to the new state with a negative value because
        # if it is not the goal I don't want to stay here
        self.board.set_cell_reward(self.position, -10)
        # I return the index of the next cell
        return self.position


    # It work on a copy of the current board in phase of simulation
    def discover(self, goal, board):

        # Basic condition that if it is verified it does not execute the rest of the algorithm
        if self.position == goal:
            print("Discovery reached goal: ", self.position)
            return goal

        # Conversion of the position in coordinates to work with two index
        pos = self._to_coordinates(self.position)
        goal = self._to_coordinates(goal)
        print("Discovery state coordinates: ", self.position, "(", pos, "->", goal, ")")

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
            v.append(self.kl_div(s, pmf) - board.expectation(self.position, s))
            # Decomment this line for DEBUG
            #print("KL:", self.kl_div(pmf, s) ,"Expectation:", board.expectation(x, s), "Res:", self.kl_div(pmf, s) - board.expectation(x, s))

        # I take the index of the minimum value of the list
        i = v.index(min(v))
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
            forward_path.append(next_cell)
            self.advance(forward_path[1])
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

    # This is the calculatione of the Kullback–Leibler divergence
    def kl_div(self, p, q):
        sum = 0
        i = 0
        for x in p:
            sum = sum + p[i]*math.log2(p[i]/q[i])
            i = i + 1
        return sum

    # This method optimize the path removing self-loop
    def optimal_path(self, path):
        length = len(path)
        if(length == 0): 
            return 'Path not generated'
        final_path = []
        k = 0
        while (k < length):
            cell = path[k]
            # If the element is present in temporary list I nedd to remove some cells
            if (cell in final_path):
                i = final_path.index(cell)
                # I remove the element after the index
                del final_path[i+1:]
            # This is a new cell, never traversed
            else:
                final_path.append(cell)
            k = k + 1
        return final_path
    

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

    # This function calculate the expectation
    def expectation(self, x, source):
        sum = 0
        i = 0
        # Iteration for every element of the source
        for p in source:
            pos = self._to_coordinates(x)
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
            if pos[0] < self.dimension and pos[1] < self.dimension and pos[0] >= 0 and pos[1] >= 0:
                # If it is I multiply the source element with the grid element and I increment the sum
                sum = sum + p * self.grid[pos[0],pos[1]]
                #print("Cell value: ", self.grid[pos[0],pos[1]])
            else:
                sum = sum + p * -100
                #print("Outside")
        return sum



def main():
    path = []
    b = Board(3)
    b.set_obstacle([3, 4, 5])
    a = Agent("car", 0, b)
    i = 0
    x = [-1]
    # While the last element of the returned list is different from the goal
    while(x[-1] != 8):
        i = i+1
        print("\nSTEP >>", i)
        x = a.move(8, 5)
        if(x == -1):
            print("\n### UNREACHABLE GOAL ###")
            return
        path.extend(x)
        # print("Path of while: ", path)
    print("\n### GOAL ACHIEVED ###")
    print("Path: ", path)


if __name__ == '__main__':
    main()