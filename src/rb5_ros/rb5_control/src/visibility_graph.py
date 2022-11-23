from dijkstar import Graph, find_path
import math
import numpy as np
import matplotlib.pyplot as plt

class VisibilityAlgorithm:

    def __init__(self, margin):
        self.margin = margin
        self.expand_distance = margin
        self.nodes = []
        self.graph = Graph()
    
    def plan(self, start, goal, obstacles):

        self.get_visibility_graph(start, goal, obstacles)

        self.edge_selection(obstacles)

        # Run dijkstra algorithm on the tree
        order_nodes = find_path(self.graph, 0, 1)
        return [self.nodes[i] for i in order_nodes.nodes]

    def get_visibility_graph(self, start, goal, obstacles):
        # add start and goal as nodes
        self.nodes = [
                 (start[0], start[1]),
                 (goal[0], goal[1])
            ]
        self.vertices_in_config_space(obstacle)

    def vertices_in_config_space(self, obstacle):
        xs = obstacle.T[0]
        ys = obstacle.T[1]
        

        n = len(obstacle)

        for i in range(len(obstacle)):
            off_x, off_y = self.calc_offsets(
                xs[(i - 1) % n], ys[(i - 1) % n],
                xs[i], ys[i],
                xs[(i + 1) % n], ys[(i + 1) % n],
            )
            self.nodes.append([xs[i]+off_x, ys[i]+off_y])

    
    def calc_offsets(self, pre_x, pre_y, x, y, next_x, next_y):
        pre_theta = math.atan2(y - pre_y, x - pre_x)
        next_theta = math.atan2(next_y - y, next_x - x)
        theta = math.atan2(
            np.sin(pre_theta) + np.sin(next_theta), 
            np.cos(pre_theta) + np.cos(next_theta)) + np.pi/2

        return [self.expand_distance*np.cos(theta), self.expand_distance*np.sin(theta)]

    def edge_selection(self, obstacles):

        for (i, target_node) in enumerate(self.nodes):
            for (node_id, node) in enumerate(self.nodes):

                # We don't consider very short edges
                if np.hypot(target_node[0] - node[0],
                            target_node[1] - node[1]) <= 0.1:
                    continue
                
                for obstacle in obstacles:
                    if not self.is_edge_non_intersecting(target_node, node, obstacle):
                        break
                else:
                    self.graph.add_edge(i, node_id, self.cost(i, node_id))

    def cost(self, i, j):
        p1 = self.nodes[i]
        p2 = self.nodes[j]
        return math.sqrt((p2[0]-p1[0])**2+(p2[1]-p2[0])**2)

    def is_edge_non_intersecting(self, target_node, node, obstacle):

        for i in range(len(obstacle) - 1):

            if VisibilityAlgorithm.is_seg_intersect(
                (target_node[0], target_node[1]), 
                (node[0], node[1]), 
                (obstacle[i][0], obstacle[i][1]), 
                (obstacle[i + 1][0], obstacle[i + 1][1])):
                return False

        return True

    def is_seg_intersect(p1, q1, p2, q2):
        
        def on_segment(p, q, r):
            if ((q[0] <= max(p[0], r[0])) and (q[0] >= min(p[0], r[0])) and
                    (q[1] <= max(p[1], r[1])) and (q[1] >= min(p[1], r[1]))):
                return True
            return False

        def orientation(p, q, r):
            val = (float(q[1] - p[1]) * (r[0] - q[0])) - (
                    float(q[0] - p[0]) * (r[1] - q[1]))
            if val > 0:
                return 1
            if val < 0:
                return 2
            return 0

        # Find the 4 orientations required for
        # the general and special cases
        o1 = orientation(p1, q1, p2)
        o2 = orientation(p1, q1, q2)
        o3 = orientation(p2, q2, p1)
        o4 = orientation(p2, q2, q1)

        if (o1 != o2) and (o3 != o4):
            return True
        if (o1 == 0) and on_segment(p1, p2, q1):
            return True
        if (o2 == 0) and on_segment(p1, q2, q1):
            return True
        if (o3 == 0) and on_segment(p2, p1, q2):
            return True
        if (o4 == 0) and on_segment(p2, q1, q2):
            return True

        return False

if __name__ == '__main__':

    va = VisibilityAlgorithm(0.2)
    
    obstacle = np.array([[1.021, 1.021, 1.25, 1.25, 1.021], [1.021, 1.479, 1.479, 1.021, 1.021]]).T
    obstacles = [obstacle[[0, 1, 2, 3]]]
    a = va.plan([0, 0], [2.25, 2.25], obstacles)
    print(a)
    # b = np.array(a).T

    # plt.plot(obstacle.T[0], obstacle.T[1], 'bo-')
    # # plt.plot(b[0], b[1], 'go-')
    # plt.text(0, 0.1, s="start")
    # plt.text(2.25, 2.3, s="goal")
    # print(va.nodes)
    # nodes = [(0, 0), (2.25, 2.25), [0.8795786437626905, 0.8795786437626905], [0.8795786437626905, 1.6204213562373095], [1.3914213562373094, 1.6204213562373095], [1.3914213562373094, 0.8795786437626905]]
    # for node in nodes:
    #     for target_node in nodes:
    #         if node==target_node: continue
    #         x = [node[0], target_node[0]]
    #         y = [node[1], target_node[1]]
    #         if va.is_edge_non_intersecting(node, target_node, obstacle):
    #             plt.plot(x, y, 'ro-')
    #         else:
    #             plt.plot(x, y, 'ro:')

    # plt.show()