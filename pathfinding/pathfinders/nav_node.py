import pygame


class NavNode:
    def __init__(self, position):
        self.position = position
        self.neighbours = []

    def add_neighbour(self, neighbour):

        self.neighbours.append(neighbour)

    def __str__(self):
        return "[" + str(self.position[0]) + ", " + str(self.position[1]) + "]"

    __repr__ = __str__


class NavigationNode:
    """
    Better Navigation node than the one above.
    """
    def __init__(self, position: pygame.math.Vector2):
        self.position = position
        self.neighbour_connections = []

    def add_connection(self, node):
        distance = self.position.distance_to(node.position)
        connection_edge = {'node': node, 'distance': distance}
        self.neighbour_connections.append(connection_edge)


class PathFinderNode:
    def __init__(self, nav_node, parent_path_node, depth,
                 fixed_path_cost=0.0, straight_line_distance_to_end=0.0,
                 total_path_cost_estimate=0.0):
        self.nav_node = nav_node
        self.parent_path_node = parent_path_node
        self.depth = depth

        self.fixed_path_cost = fixed_path_cost
        self.distance_to_end = straight_line_distance_to_end
        self.total_path_cost_estimate = total_path_cost_estimate

    def __lt__(self, other):
        return self

    def __str__(self):
        return str(self.nav_node)

    __repr__ = __str__
