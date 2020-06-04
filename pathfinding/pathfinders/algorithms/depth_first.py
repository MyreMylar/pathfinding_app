import pygame
import pygame_gui

from ..nav_node import PathFinderNode


class DepthFirstFinder:
    def __init__(self, start_nav_node, end_nav_node, incremental=False, allow_revisiting=True):
        self.name = "Depth First"
        self.start_path_node = PathFinderNode(start_nav_node, None, 0)
        self.end_nav_node = end_nav_node
        self.current_path_node = self.start_path_node
        self.allow_revisiting = allow_revisiting

        self.open_node_list = []
        self.closed_node_list = []
        self.final_path = []

        self.expand_path_node(self.current_path_node)

        self.incremental = incremental
        self.time_to_increment = False
        self.finished = False

        self.search_size = 0

        self.path_colour = pygame.Color("#FFAA00")
        self.path_colour_2 = pygame.Color("#882222AA")
        self.path_colour_3 = pygame.Color("#22AA22AA")
        self.path_colour_4 = pygame.Color("#444499AA")

        self.font = pygame.font.Font(None, 12)
        self.finished_path_info_label = None
        self.progress_label = None

    def get_name(self):
        return self.name

    def shutdown(self):
        if self.finished_path_info_label is not None:
            self.finished_path_info_label.kill()
        if self.progress_label is not None:
            self.progress_label.kill()

    def update(self):
        if self.current_path_node is not None:
            reached_end_of_path = self.current_path_node.nav_node == self.end_nav_node or self.finished
            need_to_wait_for_increment = self.incremental and not self.time_to_increment
            if not reached_end_of_path and not need_to_wait_for_increment:
                # try to expand the deepest available node
                node_to_expand = None
                highest_depth = -1
                for path_node in self.open_node_list:
                    if path_node.depth > highest_depth:
                        highest_depth = path_node.depth
                        node_to_expand = path_node

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
                if self.current_path_node.nav_node == self.end_nav_node:
                    self.finished = True
                    while self.current_path_node is not None and self.current_path_node.parent_path_node is not None:
                        self.final_path.append(self.current_path_node)
                        self.current_path_node = self.current_path_node.parent_path_node
                    self.final_path.reverse()
                    self.current_path_node = None

    def expand_path_node(self, path_node):
        for neighbour_nav_node in path_node.nav_node.neighbours:
            x_diff = path_node.nav_node.position[0] - neighbour_nav_node.position[0]
            y_diff = path_node.nav_node.position[1] - neighbour_nav_node.position[1]
            distance_to_neighbour = (x_diff ** 2 + y_diff ** 2) ** 0.5

            fixed_cost = path_node.fixed_path_cost + distance_to_neighbour

            if self.allow_revisiting:
                self.open_node_list.append(PathFinderNode(neighbour_nav_node, path_node, path_node.depth + 1,
                                                          fixed_cost, 0.0, fixed_cost))
            else:
                if not self.is_nav_node_in_open_list(neighbour_nav_node) and not self.is_nav_node_in_closed_list(neighbour_nav_node):
                    self.open_node_list.append(PathFinderNode(neighbour_nav_node, path_node, path_node.depth + 1,
                                                              fixed_cost, 0.0, fixed_cost))

        if path_node in self.open_node_list:
            self.open_node_list.remove(path_node)

    def is_nav_node_in_open_list(self, nav_node):
        is_in_open_list = False
        for path_node in self.open_node_list:
            x_match = path_node.nav_node.position[0] == nav_node.position[0]
            y_match = path_node.nav_node.position[1] == nav_node.position[1]
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

        for path_node in self.closed_node_list:
            position = path_node.nav_node.position
            closed_node_rect = pygame.Rect(0, 0, maze_square_size, maze_square_size)
            closed_node_rect.center = position
            pygame.draw.rect(window_surface, self.path_colour_4, closed_node_rect)

        for path_node in self.open_node_list:
            position = path_node.nav_node.position
            open_node_rect = pygame.Rect(0, 0, maze_square_size, maze_square_size)
            open_node_rect.center = position
            pygame.draw.rect(window_surface, self.path_colour_3, open_node_rect)
            text_num = self.font.render(str(path_node.depth), True, pygame.Color('#FFFFFF'))
            window_surface.blit(text_num, text_num.get_rect(center=position))

        if self.current_path_node is not None:
            position = self.current_path_node.nav_node.position
            current_node_rect = pygame.Rect(0, 0, maze_square_size, maze_square_size)
            current_node_rect.center = position
            pygame.draw.rect(window_surface, self.path_colour_2, current_node_rect)

            text_num = self.font.render(str(self.current_path_node.depth), True, pygame.Color('#FFFFFF'))
            window_surface.blit(text_num, text_num.get_rect(center=position))

            label_text = "Current Node Depth: " + str(self.current_path_node.depth)
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
