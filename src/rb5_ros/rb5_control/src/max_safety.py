from scipy.spatial import Voronoi, voronoi_plot_2d
from scipy.spatial.distance import euclidean
from scipy.sparse.csgraph import dijkstra
from scipy.sparse import csr_matrix
import numpy as np
import matplotlib.pyplot as plt
import copy

def closest_vertex_to_start_and_goal(start,goal,vor):
    vor_ridge_vertices = np.unique(vor.ridge_vertices)

    start_dists = [euclidean(start,vor.vertices[i]) for i in vor_ridge_vertices]
    closest_to_start = vor_ridge_vertices[np.argmin(start_dists)]

    goal_dists = [euclidean(goal,vor.vertices[i]) for i in vor_ridge_vertices]
    closest_to_goal = vor_ridge_vertices[np.argmin(goal_dists)]

    return closest_to_start,closest_to_goal

def is_intersecting(A,B,C,D):
    def ccw(A,B,C):
        return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

    # Return true if line segments AB and CD intersect
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

def prune_voronoi(vor, obstacle1, obstacle_2):
    pruned_vor = copy.deepcopy(vor)

    obstacle_2 = [
        [1.021, 1.021],
        [1.021, 1.479],
        [1.25 , 1.479],
        [1.25 , 1.021]    
    ]

    obstacle_2_line_segments = [
        [obstacle_2[0],obstacle_2[1]],
        [obstacle_2[1],obstacle_2[2]],
        [obstacle_2[2],obstacle_2[3]],
        [obstacle_2[3],obstacle_2[0]],
    ]

    for i in range(len(pruned_vor.ridge_vertices)-1,-1,-1):
        rv = pruned_vor.ridge_vertices[i]
        if rv[0]<0 or rv[1]<0:
            del pruned_vor.ridge_vertices[i]
        else:
            for j in range(len(obstacle_2_line_segments)):
                if is_intersecting(pruned_vor.vertices[rv[0]],pruned_vor.vertices[rv[1]],*obstacle_2_line_segments[j]):
                    del pruned_vor.ridge_vertices[i]
                    break

    return pruned_vor


def vor_dijkstra(start_point,goal_point,vor):
    closest_to_start,closest_to_goal = closest_vertex_to_start_and_goal(start_point,goal_point,vor)

    vor_ridge_vertices = np.unique(vor.ridge_vertices)
    v_to_i = {}
    i_to_v = {}
    for v in vor_ridge_vertices:
        if v not in v_to_i:
            v_to_i[v] = len(v_to_i)
            i_to_v[v_to_i[v]] = v

    graph = np.zeros((len(vor.ridge_vertices)+2,len(vor.ridge_vertices)+2))
    for v1,v2 in vor.ridge_vertices:
        ed = euclidean(vor.vertices[v1],vor.vertices[v2])
        graph[v_to_i[v1],v_to_i[v2]] = ed
        graph[v_to_i[v2],v_to_i[v1]] = ed

    graph[-2,v_to_i[closest_to_start]] = euclidean(start_point, vor.vertices[closest_to_start])
    graph[v_to_i[closest_to_start],-2] = euclidean(start_point, vor.vertices[closest_to_start])

    graph[-1,v_to_i[closest_to_goal]] = euclidean(goal_point, vor.vertices[closest_to_goal])
    graph[v_to_i[closest_to_goal],-1] = euclidean(goal_point, vor.vertices[closest_to_goal])
    
    graph = csr_matrix(graph)

    # print(graph)

    _, predecessors = dijkstra(csgraph=graph, directed=False, indices=-1, return_predecessors=True)

    reqd_path = []
    n=graph.shape[0]-2
    while n!=-9999:
        reqd_path.append(n)
        n = predecessors[n]

    reqd_path[0] = start_point
    reqd_path[-1]=goal_point
    reqd_path[1:-1] = [list(vor.vertices[i_to_v[x]]) for x in reqd_path[1:-1]]

    return reqd_path


def sample_path(path):
    reqd_path = [path[0]]
    cur_pos = path[0]
    for i in range(1,len(path)-1):
        if euclidean(cur_pos,path[i]) < 0.20:
            continue
        reqd_path.append(path[i])
        cur_pos = path[i]
    reqd_path.append(path[-1])

    return reqd_path


if __name__ == "__main__":

    # wall
    obstacle_1 = [
        [[-0.15,x] for x in np.arange(-0.15,2.65,0.15)],
        [[x,2.65] for x in np.arange(-0.15,2.65,0.15)],
        [[2.65,x] for x in np.arange(2.65,-0.15,-0.15)],
        [[x,-0.15] for x in np.arange(2.65,-0.15,-0.15)]
    ]
    obstacle_1 = np.reshape(obstacle_1,(-1,2))

    obstacle_2 = [
        [[1.021,x] for x in np.arange(1.021,1.479,0.1)],
        [[x,1.479] for x in np.arange(1.021,1.25,0.05)],
        [[1.25,x] for x in np.arange(1.479,1.021,-0.1)],
        [[x,1.021] for x in np.arange(1.25,1.021,-0.05)]        
    ]

    obstacle_2 = np.reshape(obstacle_2,(-1,2))

    # center obstacle
    # obstacle_2 = [
    #     [1.021, 1.021],
    #     [1.021, 1.479],
    #     [1.25 , 1.479],
    #     [1.25 , 1.021]    
    # ]

    all_obstacles = np.vstack([obstacle_1,obstacle_2])

    vor = Voronoi(all_obstacles)

    pruned_vor = prune_voronoi(vor,obstacle_1,obstacle_2)

    shortest_path = vor_dijkstra([0,0],[2,2],pruned_vor)

    reqd_path = sample_path(shortest_path)

    fig,axes = plt.subplots(2,2)
    axes = axes.flatten()

    voronoi_plot_2d(vor,axes[0],show_vertices=False,line_colors="orange")
    axes[0].title.set_text('Voronoi Diagram')    

    voronoi_plot_2d(pruned_vor,axes[1],show_vertices=False,line_colors="orange")
    axes[1].title.set_text('Pruned Voronoi Diagram')

    X = [x[0] for x in obstacle_1]
    Y = [y[1] for y in obstacle_1]
    axes[2].plot(X,Y,linestyle='-', marker='o', color='blue',markersize=4)
    obst2 = obstacle_2 
    X = [x[0] for x in obst2]
    Y = [y[1] for y in obst2]
    axes[2].plot(X,Y,linestyle='-', marker='o', color='blue', label='obstacle',markersize=4)   
    X = [x[0] for x in shortest_path]
    Y = [y[1] for y in shortest_path]
    axes[2].plot(X,Y,linestyle='--', marker='o', color='green', label='max safety path',markersize=4)
    axes[2].title.set_text('Dijkstra Shortest Path')

    X = [x[0] for x in obstacle_1]
    Y = [y[1] for y in obstacle_1]
    axes[3].plot(X,Y,linestyle='-', marker='o', color='blue',markersize=4)
    obst2 = obstacle_2
    X = [x[0] for x in obst2]
    Y = [y[1] for y in obst2]
    axes[3].plot(X,Y,linestyle='-', marker='o', color='blue', label='obstacle',markersize=4)   
    X = [x[0] for x in reqd_path]
    Y = [y[1] for y in reqd_path]
    axes[3].plot(X,Y,linestyle='--', marker='o', color='green', label='max safety path',markersize=4)
    axes[3].title.set_text('Sampled Dijkstra Shortest path')

    print(reqd_path)

    plt.tight_layout()
    plt.savefig("voronoi.png", bbox_inches="tight")
    plt.show()

