import pygame
import pygame_gui

from queue import PriorityQueue

from ..nav_node import PathFinderNode


class DijkstraFinder:
    def __init__(self, start_nav_node, end_nav_node, nav_nodes, incremental=False):
        self.name = "Dijkstra's"
        self.start_path_node = PathFinderNode(start_nav_node, None, 0)
        self.end_nav_node = end_nav_node
        self.current_path_node = self.start_path_node

        self.open_node_list = PriorityQueue()

        self.distances = {nav_node: float('infinity') for nav_node in nav_nodes}
        self.distances[start_nav_node] = 0

        self.closed_node_list = []
        self.final_path = []

        self.expand_path_node(self.current_path_node)

        self.incremental = incremental
        self.time_to_increment = False
        self.finished = False

        self.search_size = 0

        self.tool_tip = None
        self.path_colour = pygame.Color("#FFAA00")
        self.path_colour_2 = pygame.Color("#882222AA")
        self.path_colour_3 = pygame.Color("#22AA22AA")
        self.path_colour_4 = pygame.Color("#444499AA")
        self.path_colour_5 = pygame.Color("#449999AA")
        self.font = pygame.font.Font(None, 12)
        self.finished_path_info_label = None
        self.progress_label = None

    def get_name(self):
        return self.name

    def shutdown(self):
        if self.tool_tip is not None:
            self.tool_tip.kill()
        if self.finished_path_info_label is not None:
            self.finished_path_info_label.kill()
        if self.progress_label is not None:
            self.progress_label.kill()

    def update(self):
        explored_every_node = self.open_node_list.empty() or self.finished
        need_to_wait_for_increment = self.incremental and not self.time_to_increment
        if not explored_every_node and not need_to_wait_for_increment:
            # expand the lowest cost node
            node_to_expand = self.open_node_list.get()[1]

            if node_to_expand is not None:
                self.closed_node_list.append(self.current_path_node)
                self.expand_path_node(node_to_expand)
                self.current_path_node = node_to_expand
            else:
                print("Unable to find path")

            if self.incremental:
                self.time_to_increment = False
            self.search_size += 1
        else:
            if explored_every_node and not self.finished:
                if self.current_path_node not in self.closed_node_list:
                    self.closed_node_list.append(self.current_path_node)
                self.finished = True
                end_path_node = None
                for path_node in self.closed_node_list:
                    x_match = path_node.nav_node.position[0] == self.end_nav_node.position[0]
                    y_match = path_node.nav_node.position[1] == self.end_nav_node.position[1]
                    if x_match and y_match:
                        end_path_node = path_node
                        break
                if end_path_node is not None:
                    while end_path_node is not None and end_path_node.parent_path_node is not None:
                        self.final_path.append(end_path_node)
                        end_path_node = end_path_node.parent_path_node
                    self.final_path.reverse()
                    self.current_path_node = None

    def expand_path_node(self, path_node):
        for neighbour_nav_node in path_node.nav_node.neighbours:
            if not self.is_nav_node_in_closed_list(neighbour_nav_node):
                x_diff = path_node.nav_node.position[0] - neighbour_nav_node.position[0]
                y_diff = path_node.nav_node.position[1] - neighbour_nav_node.position[1]
                distance_to_neighbour = (x_diff ** 2 + y_diff ** 2) ** 0.5

                fixed_cost = path_node.fixed_path_cost + distance_to_neighbour

                if fixed_cost < self.distances[neighbour_nav_node]:
                    self.distances[neighbour_nav_node] = fixed_cost
                    self.open_node_list.put((fixed_cost, PathFinderNode(neighbour_nav_node, path_node,
                                                                        path_node.depth + 1,
                                                                        fixed_cost, 0.0, fixed_cost)))

    def get_nav_node_in_open_list(self, nav_node):
        path_finder_node = None
        for path_node in self.open_node_list.queue:
            x_match = path_node[1].nav_node.position[0] == nav_node.position[0]
            y_match = path_node[1].nav_node.position[1] == nav_node.position[1]
            if x_match and y_match:
                path_finder_node = path_node
        return path_finder_node

    def is_nav_node_in_open_list(self, nav_node):
        is_in_open_list = False
        for path_node in self.open_node_list.queue:
            x_match = path_node[1].nav_node.position[0] == nav_node.position[0]
            y_match = path_node[1].nav_node.position[1] == nav_node.position[1]
            if x_match and y_match:
                is_in_open_list = True
        return is_in_open_list

    def is_nav_node_in_closed_list(self, nav_node):
        is_in_closed_list = False
        for path_node in self.closed_node_list:
            x_match = path_node.nav_node.position[0] == nav_node.position[0]
            y_match = path_node.nav_node.position[1] == nav_node.position[1]
            if x_match and y_match:
                is_in_closed_list = True
        return is_in_closed_list

    def increment_algorithm(self):
        self.time_to_increment = True

    def draw_information(self, window_surface, ui_manager, maze_square_size):
        mouse_position = pygame.mouse.get_pos()
        hovering_anything = False
        hovered_closed_node = None
        for path_node in self.closed_node_list:
            position = path_node.nav_node.position
            closed_node_rect = pygame.Rect(0, 0, maze_square_size, maze_square_size)
            closed_node_rect.center = position
            pygame.draw.rect(window_surface, self.path_colour_4, closed_node_rect)

            if closed_node_rect.collidepoint(mouse_position[0], mouse_position[1]):
                hovering_anything = True
                hovered_closed_node = path_node
                if self.tool_tip is None:
                    tool_tip_str = ("<b>Fixed Path Cost: </b>" + str(path_node.fixed_path_cost))
                    self.tool_tip = pygame_gui.elements.UITooltip(tool_tip_str, (0, 32),
                                                                  ui_manager)
                    self.tool_tip.find_valid_position(position)

        for path_node in self.open_node_list.queue:
            position = path_node[1].nav_node.position
            open_node_rect = pygame.Rect(0, 0, maze_square_size, maze_square_size)
            open_node_rect.center = position
            if open_node_rect.collidepoint(mouse_position[0], mouse_position[1]):
                hovering_anything = True
                if self.tool_tip is None:
                    tool_tip_str = ("<b>Fixed Path Cost: </b>" + str(path_node[1].fixed_path_cost))
                    self.tool_tip = pygame_gui.elements.UITooltip(tool_tip_str, (0, 32),
                                                                  ui_manager)
                    self.tool_tip.find_valid_position(position)
            pygame.draw.rect(window_surface, self.path_colour_3, open_node_rect)

        if self.current_path_node is not None:
            position = self.current_path_node.nav_node.position
            current_node_rect = pygame.Rect(0, 0, maze_square_size, maze_square_size)
            current_node_rect.center = position
            pygame.draw.rect(window_surface, self.path_colour_2, current_node_rect)

            label_text = "Current Node Path Cost: " + str(self.current_path_node.fixed_path_cost)
            if self.progress_label is not None:
                self.progress_label.set_text(label_text)
            else:
                self.progress_label = pygame_gui.elements.UILabel(pygame.Rect((10, 510), (600, 30)),
                                                                  label_text, ui_manager)

        if self.finished:
            start_node = self.start_path_node.nav_node
            end_path_node = self.final_path[-1]
            for i in range(0, len(self.final_path)):
                end_node = self.final_path[i].nav_node
                pygame.draw.line(window_surface, self.path_colour,
                                 start_node.position, end_node.position, 4)
                start_node = end_node

            if self.progress_label is not None:
                self.progress_label.kill()

            if self.finished_path_info_label is None:
                label_text = ("Search nodes explored: "
                              "" + str(self.search_size) + ", Total depth: "
                              "" + str(end_path_node.depth) + ", Total Path Cost: "
                              "" + str(end_path_node.fixed_path_cost))
                self.finished_path_info_label = pygame_gui.elements.UILabel(pygame.Rect((10, 560), (600, 30)),
                                                                            label_text, ui_manager)

        if hovered_closed_node is not None:
            hover_path_node = hovered_closed_node
            while hover_path_node is not None:
                new_hover_path_node = hover_path_node.parent_path_node
                if new_hover_path_node is not None:
                    pygame.draw.line(window_surface, self.path_colour_5,
                                     hover_path_node.nav_node.position,
                                     new_hover_path_node.nav_node.position, 4)
                hover_path_node = new_hover_path_node

        if not hovering_anything:
            if self.tool_tip is not None:
                self.tool_tip.kill()
                self.tool_tip = None
