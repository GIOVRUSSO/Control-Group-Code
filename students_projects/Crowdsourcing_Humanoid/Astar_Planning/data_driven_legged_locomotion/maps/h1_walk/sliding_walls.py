from .map import *

sliding_walls_obstacles = [
    SlidingWall(start_pos=np.array([3.33, 0.0]), end_pos=np.array([3.33, 3.33]), shift_vect=np.array([0.0, 3.33])),
    SlidingWall(start_pos=np.array([6.66, 0.0]), end_pos=np.array([6.66, 3.33]), shift_vect=np.array([0.0, 3.33])),
    SlidingWall(start_pos=np.array([0.0, 6.66]), end_pos=np.array([3.33, 6.66]), shift_vect=np.array([3.33, 0.0])),
]