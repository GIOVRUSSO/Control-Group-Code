from .map import *

from data_driven_legged_locomotion.utils.quaternions import yaw_to_quat

door_length = 1.0
top_left = np.array([0.0, 2.0 + door_length*2 + 3.0 + 0.2])
top_right = np.array([5.0, 2.0 + door_length*2 + 3.0 + 0.2])
top_mean = (top_left + top_right) / 2

living_room_obstacles = [
    Wall(start_pos=np.array([0.0, 0.0]), end_pos=top_left),
    Wall(start_pos=np.array([0.0, 0.0]), end_pos=np.array([5.0, 0.0])),
    Wall(start_pos=np.array([5.0, 0.0]), end_pos=np.array([5.0, 2.0])),
    Wall(start_pos=np.array([5.0, 2.0 + door_length]), end_pos=np.array([5.0, 2.0 + door_length + 3.0])),
    Wall(start_pos=np.array([5.0, 2.0 + door_length*2 + 3.0]), end_pos=top_right),
    Wall(start_pos=top_left, end_pos=top_right),
    
    # Lower room
    Door(pos=np.array([5.0, 2.0]), quat=yaw_to_quat(np.pi/6), shift_yaw=-np.pi*2/3),
    Couch(np.array([2.5, 1.9])),
    TV(np.array([2.5, 0.035])),
    Pouf(np.array([4.0, 1.0])),
    #Lamp(np.array([4.0, 1.0])),
    
    # Middle room
    Table(np.array([2.5, 4.5])),
    Bookshelf(np.array([4.93, 4.5]), quat=yaw_to_quat(np.pi/2)),
    
    # Upper room
    Shelf(pos = top_mean - np.array([0.0, 0.07])),
    Lamp(top_left + np.array([0.5, -0.5])),
    Door(pos=np.array([5.0, 2.0 + door_length*2 + 3.0]), quat=yaw_to_quat(np.pi/2), shift_yaw=np.pi*2/3),
]