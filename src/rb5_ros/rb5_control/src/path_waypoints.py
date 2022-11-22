from VisibilityRoadMap.visibility_road_map import VisibilityRoadMap, ObstaclePolygon
from VoronoiRoadMap.voronoi_road_map import VoronoiRoadMapPlanner
import numpy as np
# obstacles = [ObstaclePolygon([1.021, 1.021, 1.25, 1.25], [1.021, 1.479, 1.479, 1.021])]

# minTime = VisibilityRoadMap(0.2)
maxSafety = VoronoiRoadMapPlanner()

def giveWaypoints(plan_output):
    xs,ys = plan_output
    waypoint = []
    for (x, y) in zip(xs, ys):
        waypoint.append([x, y, 0])
    return waypoint

# waypoint_minTime = giveWaypoints(minTime.planning(0, 0, 2.25, 2.25, obstacles))
# print(waypoint_minTime)
# [[0, 0, 0], [1.3914213562373094, 0.8795786437626905, 0], [2.25, 2.25, 0]]


os = np.array([[0, 0], [0, 1], [0, 2], [1, 0], [1, 1], [1, 2],
                   [2, 0], [2, 1], [2, 2]]).T

# points = np.array([
#     [-0.15, 0], [-0.15, 0.5], [-0.15, 1], [-0.15, 1.5], [-0.15, 2], [-0.15, 2.65],
#     [0, 2.65], [0.5, 2.65], [1, 2.65], [1.5, 2.65], [2, 2.65], [2.5, 2.65],
#     [2.65, 2.5], [2.65, 2], [2.65, 1.5], [2.65, 1], [2.65, 0.5], [2.65, -0.15],
#     [2.5, -0.15], [2, -0.15], [1.5, -0.15], [1, -0.15], [0.5, -0.15], [-0.15, -0.15],
#     [1.021, 1.021],
#     [1.021, 1.479],
#     [1.25 , 1.479],
#     [1.25 , 1.021]
#     ]
    
#     )

points = np.array([
    [-0.15, 0], [-0.15, 0.5], [-0.15, 1], [-0.15, 1.5], [-0.15, 2], [-0.15, 2.65],
    [0, 2.65], [0.5, 2.65], [1, 2.65], [1.5, 2.65], [2, 2.65], [2.5, 2.65],
    [2.65, 2.5], [2.65, 2], [2.65, 1.5], [2.65, 1], [2.65, 0.5], [2.65, -0.15],
    [2.5, -0.15], [2, -0.15], [1.5, -0.15], [1, -0.15], [0.5, -0.15], [-0.15, -0.15],
    [1.021, 1.021],
    [1.021, 1.479],
    [1.25 , 1.479],
    [1.25 , 1.021]
    ]
)

# obst_pts1 = [[1.021,x] for x in np.arange(1.021,1.479,0.02)]
# points = np.vstack([points,obst_pts1])

from scipy.spatial import Voronoi, voronoi_plot_2d
vor = Voronoi(points)
fig = voronoi_plot_2d(vor)
plt.show()
##############
waypoint_maxSafety = giveWaypoints(maxSafety.planning(0.1, 0.1, 1.5, 1.75, os[0], os[1], 0.1))
print(waypoint_maxSafety)

from scipy.spatial import Voronoi, voronoi_plot_2d
vor = Voronoi(points)
fig = voronoi_plot_2d(vor)
plt.show()

waypoint_maxSafety = np.array([[0, 0],
[ 0.25      ,  0.25      ],
[ 0.75      ,  0.44017165],
[ 0.53557388,  0.53557388],
[ 0.75      ,  0.44017165],
[ 1.1355    ,  0.43325833],
[ 1.25      ,  0.40881341],
[ 1.75      ,  0.51555978],
[ 1.89290968,  0.60709032],
[ 1.9460925 ,  0.75      ],
[ 1.9535925 ,  1.25      ],
[ 1.9460925 ,  1.75      ],
[ 1.89290968,  1.89290968],
[ 2.25      ,  2.25      ]])

waypoint_minTime = [[0, 0, 0], [1.3914213562373094, 0.8795786437626905, 0], [2.25, 2.25, 0]]