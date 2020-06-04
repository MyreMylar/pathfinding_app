import random

import pygame

from ..pathfinders.nav_node import NavNode


class JunctionPoint:
    def __init__(self, top_left, square_size, x_pos, y_pos):
        self.grid_x_pos = x_pos
        self.grid_y_pos = y_pos

        screen_x_pos = top_left[0] + (self.grid_x_pos * square_size)
        screen_y_pos = top_left[1] + (self.grid_y_pos * square_size)

        self.nav_node = NavNode(pygame.math.Vector2(screen_x_pos, screen_y_pos))


class MazeWall:
    def __init__(self, top_left, square_size, start_index, end_index):
        self.is_vert_wall = False
        self.is_horiz_wall = False

        if start_index[0] < end_index[0]:
            first_id = start_index[0]
            second_id = end_index[0]
            self.is_vert_wall = True
        else:
            first_id = end_index[0]
            second_id = start_index[0]

        if start_index[1] < end_index[1]:
            third_id = start_index[1]
            fourth_id = end_index[1]
            self.is_horiz_wall = True
        else:
            third_id = end_index[1]
            fourth_id = start_index[1]

        self.id = str(first_id) + str(second_id) + str(third_id) + str(fourth_id)
        self.start_index = start_index
        self.end_index = end_index

        self.start_pos = (top_left[0] + (start_index[1] * square_size),
                          top_left[1] + (start_index[0] * square_size))
        self.end_pos = (top_left[0] + (end_index[1] * square_size),
                        top_left[1] + (end_index[0] * square_size))
        self.collided = False

        if self.start_pos[0] < self.end_pos[0]:
            left = self.start_pos[0]
            width = self.end_pos[0] - self.start_pos[0]
        else:
            left = self.end_pos[0]
            width = self.start_pos[0] - self.end_pos[0]
            if width == 0:
                width = 4

        if self.start_pos[1] < self.end_pos[1]:
            top = self.start_pos[1]
            height = self.end_pos[1] - self.start_pos[1]
        else:
            top = self.start_pos[1]
            height = self.start_pos[1] - self.end_pos[1]
            if height == 0:
                height = 4
        self.rect = pygame.Rect((left, top), (width, height))


def add_new_wall_if_unique(possible_new_wall, maze_walls):
    wall_already_exists = False
    for wall in maze_walls:
        if wall.id == possible_new_wall.id:
            wall_already_exists = True
    if not wall_already_exists:
        maze_walls.append(possible_new_wall)

    return maze_walls


def create_maze(top_left, square_size, width=11, height=16, complexity=.75, density=.75):
    # Only odd shapes
    shape = ((width // 2) * 2 + 1, (height // 2) * 2 + 1)
    # Adjust complexity and density relative to maze size
    complexity = int(complexity * (5 * (shape[0] + shape[1])))
    density = int(density * ((shape[0] // 2) * (shape[1] // 2)))
    # Build actual maze
    maze_shape = []
    for x in range(0, shape[0]):
        column = []
        for y in range(0, shape[1]):
            column.append(0)
        maze_shape.append(column)
    # Fill borders
    for x in range(0, shape[0]):
        maze_shape[x][0] = 1
        maze_shape[x][-1] = 1
    for y in range(0, shape[1]):
        maze_shape[0][y] = 1
        maze_shape[-1][y] = 1

    # Make aisles
    for i in range(density):
        x, y = int(random.randint(0, shape[0] // 2) * 2), int(random.randint(0, shape[1] // 2) * 2)
        maze_shape[x][y] = 1
        for j in range(complexity):
            neighbours = []
            if x > 1:
                neighbours.append((x - 2, y))
            if x < shape[0] - 2:
                neighbours.append((x + 2, y))
            if y > 1:
                neighbours.append((x, y - 2))
            if y < shape[1] - 2:
                neighbours.append((x, y + 2))
            if len(neighbours):
                x_, y_ = neighbours[int(random.randint(0, len(neighbours) - 1))]
                if maze_shape[x_][y_] == 0:
                    maze_shape[x_][y_] = 1
                    maze_shape[x_ + (x - x_) // 2][y_ + (y - y_) // 2] = 1
                    x, y = x_, y_

    # clears dead ends
    for x in range(1, shape[0] - 1):
        for y in range(1, shape[1] - 1):
            if maze_shape[x][y] == 0:
                wall_neighbours = []
                exits = 0
                if maze_shape[x][y + 1] == 0:
                    exits += 1
                elif (y + 1) != shape[0] - 1:
                    wall_neighbours.append((y + 1, x))
                if maze_shape[x + 1][y] == 0:
                    exits += 1
                elif (x + 1) != shape[1] - 1:
                    wall_neighbours.append((y, x + 1))
                if maze_shape[x - 1][y] == 0:
                    exits += 1
                elif (x - 1) != 0:
                    wall_neighbours.append((y, x - 1))
                if maze_shape[x][y - 1] == 0:
                    exits += 1
                elif (y - 1) != 0:
                    wall_neighbours.append((y - 1, x))

                if exits <= 1:
                    y_, x_ = wall_neighbours[int(random.randint(0, len(wall_neighbours) - 1))]
                    maze_shape[x_][y_] = 0

    entry_x = 1
    entry_y = shape[1]-1
    exit_x = shape[0] - 2
    exit_y = 0
    maze_shape[1][shape[1]-1] = 0  # entry doorway
    maze_shape[shape[0] - 2][0] = 0  # exit doorway

    maze_walls = []
    junction_points = []
    maze_entrance = None
    maze_exit = None

    maze_exit_junction = None

    for x in range(0, shape[0]):
        for y in range(0, shape[1]):
            if maze_shape[x][y] == 1:
                # check for neighbours with a wall, if a wall is on it's own we just won't draw it
                if y + 1 < shape[0]:
                    if maze_shape[x][y + 1] == 1:
                        maze_walls = add_new_wall_if_unique(MazeWall(top_left, square_size,
                                                                     (y, x), (y + 1, x)), maze_walls)
                if y - 1 > 0:
                    if maze_shape[x][y - 1] == 1:
                        maze_walls = add_new_wall_if_unique(MazeWall(top_left, square_size,
                                                                     (y, x), (y - 1, x)), maze_walls)
                if x + 1 < shape[1]:
                    if maze_shape[x + 1][y] == 1:
                        maze_walls = add_new_wall_if_unique(MazeWall(top_left, square_size,
                                                                     (y, x), (y, x + 1)), maze_walls)
                if x - 1 > 0:
                    if maze_shape[x - 1][y] == 1:
                        maze_walls = add_new_wall_if_unique(MazeWall(top_left, square_size,
                                                                     (y, x), (y, x - 1)), maze_walls)
            elif maze_shape[x][y] == 0:
                if y < shape[1]-1:
                    below = maze_shape[x][y + 1]
                else:
                    below = None
                if y > 0:
                    above = maze_shape[x][y - 1]
                else:
                    above = None
                if x > 0:
                    left = maze_shape[x - 1][y]
                else:
                    left = None
                if x < shape[0]-1:
                    right = maze_shape[x + 1][y]
                else:
                    right = None
                if above is not None and below is not None and left is not None and right is not None:
                    if (below == 1) and (above == 1) and (left == 0) and (right == 0):
                        pass  # in horizontal corridor
                    elif (below == 0) and (above == 0) and (left == 1) and (right == 1):
                        pass  # vertical corridor
                    else:  # must be a point at which we can or need to change direction
                        junction_points.append(JunctionPoint(top_left, square_size, x, y))
                else:
                    if x == exit_x and y == exit_y:
                        maze_exit = JunctionPoint(top_left, square_size, x, y)
                        junction_points.append(maze_exit)
                        maze_exit_junction = JunctionPoint(top_left, square_size, x, y + 1)
                        junction_points.append(maze_exit_junction)
                    if x == entry_x and y == entry_y:
                        maze_entrance = JunctionPoint(top_left, square_size, x, y)
                        junction_points.append(maze_entrance)
                        maze_entrance_junction = JunctionPoint(top_left, square_size, x, y - 1)
                        junction_points.append(maze_entrance_junction)

    for point in junction_points:
        # locate neighbours in the four possible directions if they exist
        x_explore = point.grid_x_pos + 1
        found_wall_or_neighbour = False
        while x_explore < shape[1] and not found_wall_or_neighbour:  # direction 1
            if maze_shape[x_explore][point.grid_y_pos] == 0:
                for neighbour_point in junction_points:
                    if neighbour_point.grid_y_pos == point.grid_y_pos and neighbour_point.grid_x_pos == x_explore:
                        point.nav_node.neighbours.append(neighbour_point.nav_node)
                        found_wall_or_neighbour = True
                        break
                x_explore += 1
            elif maze_shape[x_explore][point.grid_y_pos] == 1:
                found_wall_or_neighbour = True
                if x_explore - 1 != point.grid_x_pos:
                    for neighbour_point in junction_points:
                        if neighbour_point.grid_x_pos == x_explore - 1 and\
                                neighbour_point.grid_y_pos == point.grid_y_pos:
                            point.nav_node.neighbours.append(neighbour_point.nav_node)
                            break

        x_explore = point.grid_x_pos - 1
        found_wall_or_neighbour = False
        while x_explore >= 0 and not found_wall_or_neighbour:  # direction 2
            if maze_shape[x_explore][point.grid_y_pos] == 0:
                for neighbour_point in junction_points:
                    if neighbour_point.grid_y_pos == point.grid_y_pos and neighbour_point.grid_x_pos == x_explore:
                        point.nav_node.neighbours.append(neighbour_point.nav_node)
                        found_wall_or_neighbour = True
                        break
                x_explore -= 1
            elif maze_shape[x_explore][point.grid_y_pos] == 1:
                found_wall_or_neighbour = True
                if x_explore + 1 != point.grid_x_pos:
                    for neighbour_point in junction_points:
                        if neighbour_point.grid_x_pos == x_explore + 1 and\
                                neighbour_point.grid_y_pos == point.grid_y_pos:
                            point.nav_node.neighbours.append(neighbour_point.nav_node)
                            break

        y_explore = point.grid_y_pos + 1
        found_wall_or_neighbour = False
        while y_explore < shape[0] and not found_wall_or_neighbour:  # direction 3
            if maze_shape[point.grid_x_pos][y_explore] == 0:
                for neighbour_point in junction_points:
                    if neighbour_point.grid_y_pos == y_explore and neighbour_point.grid_x_pos == point.grid_x_pos:
                        point.nav_node.neighbours.append(neighbour_point.nav_node)
                        found_wall_or_neighbour = True
                        break
                y_explore += 1
            elif maze_shape[point.grid_x_pos][y_explore] == 1:
                found_wall_or_neighbour = True
                if y_explore - 1 != point.grid_y_pos:
                    for neighbour_point in junction_points:
                        if neighbour_point.grid_y_pos == y_explore - 1 and\
                                neighbour_point.grid_x_pos == point.grid_x_pos:
                            point.nav_node.neighbours.append(neighbour_point.nav_node)
                            break

        y_explore = point.grid_y_pos - 1
        found_wall_or_neighbour = False
        while y_explore >= 0 and not found_wall_or_neighbour:  # direction 4
            if maze_shape[point.grid_x_pos][y_explore] == 0:
                for neighbour_point in junction_points:
                    if neighbour_point.grid_y_pos == y_explore and\
                            neighbour_point.grid_x_pos == point.grid_x_pos:
                        point.nav_node.neighbours.append(neighbour_point.nav_node)
                        found_wall_or_neighbour = True
                        break

                y_explore -= 1
            elif maze_shape[point.grid_x_pos][y_explore] == 1:
                found_wall_or_neighbour = True
                if y_explore + 1 != point.grid_y_pos:
                    for neighbour_point in junction_points:
                        if neighbour_point.grid_y_pos == y_explore + 1 and\
                                neighbour_point.grid_x_pos == point.grid_x_pos:
                            point.nav_node.neighbours.append(neighbour_point.nav_node)
                            break

    return maze_walls, junction_points, maze_entrance, maze_exit
