import sys
import pygame
from pygame.locals import KEYDOWN, K_q
import os
import time

# CONSTANTS:
SCREENSIZE = WIDTH, HEIGHT = 600, 600
CONTAINER_WIDTH_HEIGHT = 600
BLACK = (0, 0, 0)
GREY = (160, 160, 160)
# VARS:
_VARS = {'surf': False}

# Test method to simulate the graphical interface without 
# starting the crowdsourcing algorithm
def test():
    pygame.init()
    _VARS['surf'] = pygame.display.set_mode(SCREENSIZE)
    b = BoardGUI(5, [6], 24)
    m = Movement(b, 0)
    cells_list = [0, 1, 2, 3, 4, 9, 14, 19, 24]
    for c in cells_list:
      checkEvents()
      m.move(c)
      pygame.display.update()
    final()

# Method invoked by the crowdsourcing algorithm to which the calculated 
# path passes and all the info on the grid necessary for the graphic 
# representation
def start_simulation(dim, obs_list, path, start_position):
  pygame.init()
  _VARS['surf'] = pygame.display.set_mode(SCREENSIZE)
  b = BoardGUI(dim, obs_list, path[-1])
  m = Movement(b, start_position)
  for c in path:
    checkEvents()
    m.move(c)
    pygame.display.update()
  final()

# Method that detects pressing the "q" key to end the simulation
def checkEvents():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()
        elif event.type == KEYDOWN and event.key == K_q:
            pygame.quit()
            sys.exit()

# Method that puts the simulation on hold without engaging the cpu 
# too much while waiting for the termination signal
def final():
  while True:
    pygame.time.delay(50)
    checkEvents()


# This class represents the game grid and needs to know how many rows or 
# columns it is composed of to be represented but also the indices of 
# the cells containing obstacles
class BoardGUI:

  def __init__ (self, divisions, obs_list, goal):
    self.divisions = divisions  # number of row or columns
    self.obs_list = obs_list  # index of obstacles
    self.goal = goal # index of the goal cell
    self.cellsize = self.drawGrid(divisions) # dimension in pixel of the single cell
    self.obstacle = self.loadImage('obstacle.png')  # loading the representation of the obstacle
    self.goal_img = self.loadImage('goal.png')  # loading the representation of the goal cell
    self.background = pygame.transform.scale(pygame.image.load(os.path.join('sand.jpg')), (WIDTH, HEIGHT)) # loading the background
    self.drawImage(obs_list) # drawing the obstacles
    
  # This is an useful method that takes in input a cell index and return
  # the coordinate in pixel of the cell that corresponds to the point to
  # start drawing  
  def _to_screen_coordinates(self, x):
    pos = [x % self.divisions, x // self.divisions]
    pos[1] = self.divisions - pos[1] - 1
    return [pos[0]*self.cellsize, pos[1]*self.cellsize]

  # Load the image data into a variable
  def loadImage(self, name):
    myimage = pygame.image.load(os.path.join(name))
    myimage.convert_alpha()
    myimage = pygame.transform.scale(myimage, (int(self.cellsize/2),int(self.cellsize/2)))
    return myimage

  # Draw the image loaded in self.obstacle in all the cells present in the list 
  # passed in input
  def drawImage(self, obs_list):
    for o in obs_list:
      pos = self._to_screen_coordinates(o)
      pos[0] = pos[0] + self.cellsize/4
      pos[1] = pos[1] + self.cellsize/4
      _VARS['surf'].blit(self.obstacle, pos)

  def drawObject(self, index, img_data):
    pos = self._to_screen_coordinates(index)
    pos[0] = pos[0] + self.cellsize/4
    pos[1] = pos[1] + self.cellsize/4
    _VARS['surf'].blit(img_data, pos)

  # Main method that deals with drawing the rows and columns of the grid:
  # it draws the lines on the background set previously
  def drawGrid(self, divisions):

      cont_x, cont_y = 0, 0  # TOP LEFT OF CONTAINER

      # DRAW Grid Border:
      # TOP lEFT TO RIGHT
      pygame.draw.line(
        _VARS['surf'], BLACK,
        (cont_x, cont_y),
        (CONTAINER_WIDTH_HEIGHT + cont_x, cont_y), 2)
      # # BOTTOM lEFT TO RIGHT
      pygame.draw.line(
        _VARS['surf'], BLACK,
        (cont_x, CONTAINER_WIDTH_HEIGHT + cont_y),
        (CONTAINER_WIDTH_HEIGHT + cont_x, CONTAINER_WIDTH_HEIGHT + cont_y), 2)
      # # LEFT TOP TO BOTTOM
      pygame.draw.line(
        _VARS['surf'], BLACK,
        (cont_x, cont_y),
        (cont_x, cont_y + CONTAINER_WIDTH_HEIGHT), 2)
      # # RIGHT TOP TO BOTTOM
      pygame.draw.line(
        _VARS['surf'], BLACK,
        (CONTAINER_WIDTH_HEIGHT + cont_x, cont_y),
        (CONTAINER_WIDTH_HEIGHT + cont_x, CONTAINER_WIDTH_HEIGHT + cont_y), 2)

      # Get cell size, just one since its a square grid
      cellSize = CONTAINER_WIDTH_HEIGHT/divisions

      # VERTICAL DIVISIONS: (0,1,2) for grid(3) for example
      for x in range(divisions):
          pygame.draw.line(
            _VARS['surf'], BLACK,
            (cont_x + (cellSize * x), cont_y),
            (cont_x + (cellSize * x), CONTAINER_WIDTH_HEIGHT + cont_y), 2)
      # # HORIZONTAl DIVISIONS
          pygame.draw.line(
            _VARS['surf'], BLACK,
            (cont_x, cont_y + (cellSize*x)),
            (cont_x + CONTAINER_WIDTH_HEIGHT, cont_y + (cellSize*x)), 2)

      return cellSize


class Movement:

  def __init__(self, board, start):
    self.board = board
    self.cellsize = board.cellsize
    self.pos = board._to_screen_coordinates(start) # object current co-ordinates 
    self.drone = self.loadImage('drone.png')
    self.first_time = True

  # Load the image into a variable
  def loadImage(self, name):
    myimage = pygame.image.load(os.path.join(name))
    myimage.convert_alpha()
    myimage = pygame.transform.scale(myimage, (int(self.cellsize*2/3), int(self.cellsize*2/3)))
    return myimage

  
  def move(self, next_cell):

    # Calculation of the position in pixels of the next cell
    next_pos = self.board._to_screen_coordinates(next_cell)
    # The margin represents the distance from the edge of the cell 
    # from which to start drawing the object
    margin = int(self.cellsize/6)

    # Calculation of the distance in pixels along the axes
    delta_x = next_pos[0] - self.pos[0]
    delta_y = next_pos[1] - self.pos[1]

    step = 8
    last_step = int(self.cellsize % step)

    n_step = int(self.cellsize/step) + 1

    # Initial delay
    if(self.first_time):
        self.first_time = False
        _VARS['surf'].blit(self.board.background, (0, 0))
        self.board.drawGrid(self.board.divisions)
        self.board.drawImage(self.board.obs_list)
        self.board.drawObject(self.board.goal, self.board.goal_img)
        _VARS['surf'].blit(self.drone, (self.pos[0] + margin, self.pos[1] + margin))
        pygame.display.update()
        pygame.time.delay(1000)

    for i in range(n_step):
    
      # This is a movement delay: determines how often we move 
      # the agent a certain step by updating the grid
      pygame.time.delay(50)

      # Checking if the movement is directed to the right
      if(delta_x > 0):
        if(delta_x >= abs(delta_y)):
          # Checking if it is the last step
          if i == n_step-1:
            # The last step is shorter than the others
            self.pos[0] = self.pos[0] + last_step
          else:
            # Moving the agent of a step along x axes to the right
            self.pos[0] = self.pos[0] + step
      if(delta_x < 0):
        if(abs(delta_x) >= abs(delta_y)):
          if i == n_step-1:
            self.pos[0] = self.pos[0] - last_step
          else:
            self.pos[0] = self.pos[0] - step
      if(delta_y > 0):
        if(abs(delta_x) < delta_y):
          if i == n_step-1:
            self.pos[1] = self.pos[1] + last_step
          else:
            self.pos[1] = self.pos[1] + step
      if(delta_y < 0):
        if(abs(delta_x) < abs(delta_y)):
          if i == n_step-1:
            self.pos[1] = self.pos[1] - last_step
          else:
            self.pos[1] = self.pos[1] - step


      # Adding the margin to the cell coordinates
      x = self.pos[0] + margin
      y = self.pos[1] + margin

      # Drawing the grid with the agent
      _VARS['surf'].blit(self.board.background, (0, 0))
      self.board.drawGrid(self.board.divisions)
      self.board.drawImage(self.board.obs_list)
      self.board.drawObject(self.board.goal, self.board.goal_img)
      _VARS['surf'].blit(self.drone, (x, y))

      pygame.display.update()



if __name__ == '__main__':
    test()
