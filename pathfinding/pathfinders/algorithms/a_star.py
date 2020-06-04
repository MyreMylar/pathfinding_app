import math

import pygame
import pygame_gui

from ..nav_node import PathFinderNode


class AStarFinder:
    def __init__(self, start_nav_node, end_nav_node, incremental=False, max_path_search_size=2000):
        self.name = "A*"
        self.end_nav_node = end_nav_node

        x_diff = start_nav_node.position[0] - self.end_nav_node.position[0]
        y_diff = start_nav_node.position[1] - self.end_nav_node.position[1]
        straight_line_distance_to_end_node = math.sqrt(x_diff ** 2 + y_diff ** 2)

        self.start_path_node = PathFinderNode(start_nav_node, None, 0, 0, straight_line_distance_to_end_node,
                                              straight_line_distance_to_end_node)
        self.max_search_size = max_path_search_size
        self.incremental = incremental
        self.final_path = []

        self.open_node_list = []
        self.closed_node_list = []  # nodes we have already evaluated by adding their neighbours to the open list
        self.current_path_node = self.start_path_node

        self.node_progress = []

        self.add_current_path_node_neighbours_to_open_list()

        self.search_size = 0
        self.current_fixed_path_cost = 0.0
        self.time_to_increment = False
        self.finished = False

        # drawing info
        self.tool_tip = None
        self.progress_label = None
        self.finished_path_info_label = None
        self.path_colour = pygame.Color("#FFAA00")
        self.path_colour_2 = pygame.Color("#882222AA")
        self.path_colour_3 = pygame.Color("#22AA22AA")
        self.path_colour_4 = pygame.Color("#444499AA")

    def get_name(self):
        return self.name

    def shutdown(self):
        if self.tool_tip is not None:
            self.tool_tip.kill()
        if self.progress_label is not None:
            self.progress_label.kill()
        if self.finished_path_info_label is not None:
            self.finished_path_info_label.kill()

    def increment_algorithm(self):
        self.time_to_increment = True

    def update(self):
        if self.current_path_node is not None:
            valid_search_length = self.search_size < self.max_search_size
            valid_current_node = self.current_path_node is not None
            reached_end_of_path = self.current_path_node.nav_node == self.end_nav_node or self.finished
            need_to_wait_for_increment = self.incremental and not self.time_to_increment
            if valid_current_node and valid_search_length and not reached_end_of_path and not need_to_wait_for_increment:
                lowest_path_cost = self.current_fixed_path_cost + 99999999.0
                lowest_path_cost_node = None
                for path_node in self.open_node_list:
                    if path_node.total_path_cost_estimate < lowest_path_cost:
                        lowest_path_cost = path_node.total_path_cost_estimate
                        lowest_path_cost_node = path_node

                self.current_path_node = lowest_path_cost_node
                if self.current_path_node is not None:
                    self.node_progress.append([lowest_path_cost_node, lowest_path_cost])
                    self.add_current_path_node_neighbours_to_open_list()

                self.search_size += 1
                if self.incremental:
                    self.time_to_increment = False

            else:
                # unwind our successful path
                if valid_search_length and reached_end_of_path and not self.finished:
                    self.finished = True
                    while self.current_path_node is not None and self.current_path_node.parent_path_node is not None:
                        self.final_path.append(self.current_path_node)
                        self.current_path_node = self.current_path_node.parent_path_node
                    self.final_path.reverse()
                    self.current_path_node = None

    def add_current_path_node_neighbours_to_open_list(self):
        # add current Node neighbours to open list (if not in closed list?)
        for neighbour in self.current_path_node.nav_node.neighbours:
            if not self.is_nav_node_in_closed_list(neighbour) and not self.is_nav_node_in_open_list(neighbour):
                x_diff = self.current_path_node.nav_node.position[0] - neighbour.position[0]
                y_diff = self.current_path_node.nav_node.position[1] - neighbour.position[1]
                distance_to_neighbour = math.sqrt(x_diff ** 2 + y_diff ** 2)

                x_diff = neighbour.position[0] - self.end_nav_node.position[0]
                y_diff = neighbour.position[1] - self.end_nav_node.position[1]
                distance_to_end_node = math.sqrt(x_diff ** 2 + y_diff ** 2)

                fixed_path_cost = self.current_path_node.fixed_path_cost + distance_to_neighbour
                total_path_cost_estimate = fixed_path_cost + distance_to_end_node
                self.open_node_list.append(PathFinderNode(neighbour, self.current_path_node,
                                                          self.current_path_node.depth + 1,
                                                          fixed_path_cost, distance_to_end_node,
                                                          total_path_cost_estimate))

        self.closed_node_list.append(self.current_path_node)
        if self.current_path_node in self.open_node_list:
            self.open_node_list.remove(self.current_path_node)

    def is_nav_node_in_closed_list(self, nav_node):
        is_in_closed_list = False
        for path_node in self.closed_node_list:
            x_match = path_node.nav_node.position[0] == nav_node.position[0]
            y_match = path_node.nav_node.position[1] == nav_node.position[1]
            if x_match and y_match:
                is_in_closed_list = True
        return is_in_closed_list

    def is_nav_node_in_open_list(self, nav_node):
        is_in_open_list = False
        for path_node in self.open_node_list:
            x_match = path_node.nav_node.position[0] == nav_node.position[0]
            y_match = path_node.nav_node.position[1] == nav_node.position[1]
            if x_match and y_match:
                is_in_open_list = True
        return is_in_open_list

    def draw_information(self, window_surface, ui_manager, maze_square_size):
        mouse_position = pygame.mouse.get_pos()
        hovering_anything = False
        for path_node in self.closed_node_list:
            position = path_node.nav_node.position
            closed_node_rect = pygame.Rect(0, 0, maze_square_size, maze_square_size)
            closed_node_rect.center = position
            pygame.draw.rect(window_surface, self.path_colour_4, closed_node_rect)

        for path_node in self.open_node_list:
            position = path_node.nav_node.position
            open_node_rect = pygame.Rect(0, 0, maze_square_size, maze_square_size)
            open_node_rect.center = position
            if open_node_rect.collidepoint(mouse_position[0], mouse_position[1]):
                hovering_anything = True
                if self.tool_tip is None:
                    tool_tip_str = ("<b>Total Path Cost Estimate: </b>" + str(path_node.total_path_cost_estimate) +
                                    "<br><b>Straight Line to End Estimate: </b>" + str(path_node.distance_to_end) +
                                    "<br><b>Fixed Path Cost: </b>" + str(path_node.fixed_path_cost))
                    self.tool_tip = pygame_gui.elements.UITooltip(tool_tip_str, (0, maze_square_size),
                                                                  ui_manager)
                    self.tool_tip.find_valid_position(position)
            pygame.draw.rect(window_surface, self.path_colour_3, open_node_rect)

        if self.current_path_node is not None:
            position = self.current_path_node.nav_node.position
            current_node_rect = pygame.Rect(0, 0, maze_square_size, maze_square_size)
            current_node_rect.center = position
            pygame.draw.rect(window_surface, self.path_colour_2, current_node_rect)

            label_text = "Current Path Total Cost Estimate: " + str(self.current_path_node.total_path_cost_estimate)
            if self.progress_label is not None:
                self.progress_label.set_text(label_text)
            else:
                self.progress_label = pygame_gui.elements.UILabel(pygame.Rect((10, 510), (600, 30)),
                                                                  label_text, ui_manager)

        if self.finished:
            start_node = self.start_path_node.nav_node
            for i in range(0, len(self.final_path)):
                end_node = self.final_path[i].nav_node
                pygame.draw.line(window_surface, self.path_colour,
                                 start_node.position, end_node.position, 4)
                start_node = end_node

            if self.progress_label is not None:
                self.progress_label.kill()

            if self.finished_path_info_label is None:
                end_path_node = self.final_path[-1]
                label_text = ("Search nodes explored: "
                              "" + str(self.search_size) + ", Total depth: "
                              "" + str(end_path_node.depth) + ", Total Path Cost: "
                              "" + str(end_path_node.fixed_path_cost))
                self.finished_path_info_label = pygame_gui.elements.UILabel(pygame.Rect((10, 560), (600, 30)),
                                                                            label_text, ui_manager)

        if not hovering_anything:
            if self.tool_tip is not None:
                self.tool_tip.kill()
                self.tool_tip = None
