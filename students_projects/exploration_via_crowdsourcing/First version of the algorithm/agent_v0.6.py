import math
import numpy as np

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

    # Important method: it makes the movement of the agent
    def move(self, x, goal):

        # Basic condition that if it is verified it does not execute the rest of the algorithm
        if x == goal:
            print("GOAL ACHIEVED: ", x)
            return 0

        # Conversion of the position in coordinates to work with two index
        pos = self._to_coordinates(x)
        goal = self._to_coordinates(goal)
        print("State coordinates: ", x, "(", pos, "->", goal, ")")

        # Saturation check
        if self.board.grid[pos[0],pos[1]] < -99:
            print("UNREACHABLE GOAL")
            return 0

        def _convert(pos):
            return pos[1]*self.board.dimension + pos[0]   

        # Loading the pmf
        pmf = self.return_target(x)
        #pmf = self.next_cell(pos,goal)

        # In this step I calculate the Kullback–Leibler divergence for every source.
        # For each iteration I have a source and I compute the KL of this source and the pmf and
        # at the result of the operation is subtracted the expectation of this source respect to the
        # current position.
        # The final value is stored in a list, so at the end I have a number of value in the list
        # that is equal the number of sources
        v = []
        for s in self.sources:
            v.append(self.kl_div(s, pmf) - self.board.expectation(x, s))
            # Decomment this code for DEBUG
            #print("KL:", self.kl_div(pmf, s) ,"Expectation:", self.board.expectation(x, s), "Res:", self.kl_div(pmf, s) - self.board.expectation(x, s))

        # I take the index of the minimum value of the list
        i = v.index(min(v))
        # At this index corresponds a movement
        if(i == 1):
            # I mark the precedent cell with a little positive value to not stuck and prefer to go back
            self.board.set_cell_reward(self.position, +1)
            print(i, " -> LEFT")
            # I simulate the movement in the coordinate space
            pos[0] = pos[0] - 1
            # I convert the movement in the input space and I update the state of the agent
            self.position = _convert(pos)
            # I mark the new cell corresponding to the new state with a negative value because
            # if it is not the goal I don't want to stay here
            self.board.set_cell_reward(self.position, -10)
            # I return the pmf corresponding to the movement
            return self.left
        if(i == 2):
            self.board.set_cell_reward(self.position, +1)
            print(i, " -> UP")
            pos[1] = pos[1] + 1
            self.position = _convert(pos)
            self.board.set_cell_reward(self.position, -10)
            return self.up
        if(i == 3):
            self.board.set_cell_reward(self.position, +1)
            print(i, " -> RIGHT")
            pos[0] = pos[0] + 1
            self.position = _convert(pos)
            self.board.set_cell_reward(self.position, -10)
            return self.rigth
        if(i == 4):
            self.board.set_cell_reward(self.position, +1)
            print(i, " -> DOWN")
            pos[1] = pos[1] - 1
            self.position = _convert(pos)
            self.board.set_cell_reward(self.position, -10)
            return self.down
        print(i, " -> STAY")
        self.board.set_cell_reward(self.position, -10)
        return self.stay

    # This is the pmf given to me
    def return_target(self, x): 
        if x==0 or x==3 or x==4 or x==6 or x==7:
            return([0.01, 0.01, 0.01, 0.95, 0.01])
        if x==1 or x==2 or x==5:
            return([0.01, 0.01, 0.95, 0.01, 0.01])
        return([0.95, 0.01, 0.01, 0.01, 0.01])

    # This in the new pmf, not in the presentation for the next step
    def next_cell(self, pos, goal):
        delta_x = goal[0] - pos[0]
        delta_y = goal[1] - pos[1]
        # Goal achieved
        if(delta_x == 0 and delta_y == 0):
            return self.stay()
        # MAke decision= algorithm 1
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
    

# This class represent the space in which the agent can move
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
    b = Board(3)
    b.set_obstacle([3, 4, 5])
    a = Agent("car", 0, b)
    while(a.move(a.position, 8) != 0):
        pass


if __name__ == '__main__':
    main()