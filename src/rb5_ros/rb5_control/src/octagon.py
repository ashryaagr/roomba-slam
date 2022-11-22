import numpy as np
theta = np.pi/4
angle = 0
L = [[0.0, 0.0, 0.0]]
curr_state = L[0]
side = 0.5

print(curr_state, ",")
for i in range(8):
    angle += theta
    if angle>np.pi:
        angle -= 2*np.pi
    elif angle < -np.pi:
        angle += 2*np.pi
    curr_state = [curr_state[0]+side*np.sin(angle), curr_state[1]+side*np.cos(angle), angle]
    curr_state = [round(x, 3) for x in curr_state]
    #L.append()
    print([curr_state[0], -curr_state[1], curr_state[2]], ",")

waypoint_octagon = np.array([
    [0.0, 0.0, 0.0] ,
    [0.707, -0.707, 0.785] ,
    [1.707, -0.707, 1.571] ,
    [2.414, 0.0, 2.356] ,
    [2.414, 1.0, 3.142] ,
    [1.707, 1.707, -2.356] ,
    [0.707, 1.707, -1.571] ,
    [-0.0, 1.0, -0.785] ,
    [0.0, -0.0, 0.0] ,
])
# waypoint_octagon[:, 1] = -waypoint_octagon[:, 1]
# print(waypoint_octagon)