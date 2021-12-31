import random

import pygame
import pygame_gui
from pygame_gui.elements import UIDropDownMenu, UIButton, UIHorizontalSlider

from pathfinding.maze.maze_generation import create_maze
from pathfinding.pathfinders.nav_node import PathFinderNode
from pathfinding.pathfinders.algorithms.a_star import AStarFinder
from pathfinding.pathfinders.algorithms.breadth_first import BreadthFirstFinder
from pathfinding.pathfinders.algorithms.uniform_cost import UniformCostFinder
from pathfinding.pathfinders.algorithms.depth_first import DepthFirstFinder
from pathfinding.pathfinders.algorithms.dijkstra import DijkstraFinder


class PathfindingApp:
    def __init__(self):
        pygame.init()

        pygame.display.set_caption("Pathfinding Algorithms")
        self.window_surface = pygame.display.set_mode((800, 600))

        self.ui_manager = pygame_gui.UIManager((800, 600), "pathfinding/data/ui_theme.json")
        self.ui_manager.preload_fonts([{'name': 'fira_code', 'point_size': 14, 'style': 'bold'}])

        self.background_surface = pygame.Surface((800, 600))
        self.background_surface.fill(self.ui_manager.get_theme().get_colour('dark_bg'))

        self.playing_pathfinder = False
        self.play_speed = 0.5
        self.play_speed_acc = 0.0

        pathfinding_algorithms = ['A*', "Breadth First", "Depth First", "Dijkstra's", "Uniform Cost"]
        self.pathfinder_drop_down = UIDropDownMenu(pathfinding_algorithms, 'A*',
                                                   pygame.Rect((620, 50), (150, 25)), self.ui_manager)

        self.pathfinder_label = pygame_gui.elements.UILabel(pygame.Rect((520, 50), (100, 25)),
                                                            "Algorithm: ", self.ui_manager)

        map_sizes = ['20x20', "40x40", "80x80"]
        self.map_size_drop_down = UIDropDownMenu(map_sizes, '20x20',
                                                 pygame.Rect((620, 225), (150, 25)), self.ui_manager)

        self.map_size_label = pygame_gui.elements.UILabel(pygame.Rect((520, 225), (100, 25)),
                                                          "Map size: ", self.ui_manager)

        self.random_start_button = UIButton(pygame.Rect((620, 350), (150, 25)),
                                            "Random start", self.ui_manager)

        self.increment_pathfinder_button = UIButton(pygame.Rect((620, 385), (150, 25)),
                                                    "Increment", self.ui_manager)

        self.play_button = UIButton(pygame.Rect((620, 420), (150, 25)),
                                    "Play", self.ui_manager)

        self.speed_slider = UIHorizontalSlider(pygame.Rect((620, 455), (150, 25)),
                                               self.play_speed, (1.0, 0.017),
                                               self.ui_manager)

        self.speed_slider_label = pygame_gui.elements.UILabel(pygame.Rect((520, 455), (100, 25)),
                                                              "Play speed: ", self.ui_manager)

        self.tool_tip = None

        self.wall_colour = pygame.Color("#FFFFFF")

        self.available_maze_space = 450
        self.wall_size = 4
        self.maze_dimension = 20

        self.maze_square_size = int(self.available_maze_space/self.maze_dimension) + 1
        self.walls, self.junctions, self.entrance, self.exit = create_maze(top_left=(20, 20),
                                                                           square_size=self.maze_square_size,
                                                                           width=self.maze_dimension,
                                                                           height=self.maze_dimension)

        self.nav_node_graph = [junction.nav_node for junction in self.junctions]
        self.font = pygame.font.Font(None, 12)
        self.current_finder = AStarFinder(self.entrance.nav_node, self.exit.nav_node, incremental=True)

        self.clock = pygame.time.Clock()
        self.running = True

    def set_current_pathfinder(self, finder_name):
        if self.current_finder is not None:
            self.current_finder.shutdown()

        if finder_name == "Breadth First":
            self.current_finder.shutdown()
            self.current_finder = BreadthFirstFinder(self.entrance.nav_node,
                                                     self.exit.nav_node,
                                                     incremental=True,
                                                     allow_revisiting=False)
        elif finder_name == "A*":
            self.current_finder.shutdown()
            self.current_finder = AStarFinder(self.entrance.nav_node,
                                              self.exit.nav_node,
                                              incremental=True)
        elif finder_name == "Uniform Cost":
            self.current_finder.shutdown()
            self.current_finder = UniformCostFinder(self.entrance.nav_node,
                                                    self.exit.nav_node,
                                                    incremental=True,
                                                    allow_revisiting=False)
        elif finder_name == "Depth First":
            self.current_finder.shutdown()
            self.current_finder = DepthFirstFinder(self.entrance.nav_node,
                                                   self.exit.nav_node,
                                                   incremental=True,
                                                   allow_revisiting=False)
        elif finder_name == "Dijkstra's":
            self.current_finder.shutdown()
            self.current_finder = DijkstraFinder(self.entrance.nav_node,
                                                 self.exit.nav_node,
                                                 self.nav_node_graph,
                                                 incremental=True)

    def run(self):
        while self.running:
            time_delta = self.clock.tick(60)/1000.0  # time_delta is time between loops in seconds

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

                self.ui_manager.process_events(event)

                if event.type == pygame_gui.UI_BUTTON_PRESSED:
                    if event.ui_element == self.increment_pathfinder_button:
                        self.playing_pathfinder = False
                        self.current_finder.increment_algorithm()
                    if event.ui_element == self.play_button:
                        if self.playing_pathfinder:
                            self.playing_pathfinder = False
                            self.play_button.set_text('Play')
                        else:
                            self.playing_pathfinder = True
                            self.play_button.set_text('Stop')

                    if event.ui_element == self.random_start_button:
                        start_nav_node = random.choice(self.nav_node_graph)
                        while start_nav_node == self.exit.nav_node:
                            start_nav_node = random.choice(self.nav_node_graph)
                        self.entrance = PathFinderNode(start_nav_node, None, 0)
                        self.set_current_pathfinder(self.current_finder.get_name())

                if event.type == pygame_gui.UI_DROP_DOWN_MENU_CHANGED:
                    if event.ui_element == self.map_size_drop_down:
                        self.maze_dimension = int(event.text.split('x')[0])
                        self.maze_square_size = int(self.available_maze_space / self.maze_dimension) + 1
                        result = create_maze(top_left=(20, 20),
                                             square_size=self.maze_square_size,
                                             width=self.maze_dimension,
                                             height=self.maze_dimension)
                        self.walls = result[0]
                        self.junctions = result[1]
                        self.entrance = result[2]
                        self.exit = result[3]
                        self.nav_node_graph = [junction.nav_node for junction in self.junctions]

                        self.set_current_pathfinder(self.current_finder.get_name())

                    elif event.ui_element == self.pathfinder_drop_down:
                        self.set_current_pathfinder(event.text)

            if self.playing_pathfinder:
                self.play_speed = self.speed_slider.get_current_value()
                self.play_speed_acc += time_delta
                if self.play_speed_acc >= self.play_speed:
                    self.play_speed_acc = 0.0
                    self.current_finder.increment_algorithm()

            self.ui_manager.update(time_delta)
            self.current_finder.update()

            self.window_surface.blit(self.background_surface, (0, 0))

            for wall in self.walls:
                pygame.draw.line(self.window_surface, self.wall_colour,
                                 wall.start_pos, wall.end_pos, self.wall_size)

            self.current_finder.draw_information(self.window_surface, self.ui_manager, self.maze_square_size)

            if self.entrance is not None:
                entrance_rect = pygame.Rect(0, 0, self.maze_square_size, self.maze_square_size)
                entrance_rect.center = self.entrance.nav_node.position
                pygame.draw.rect(self.window_surface, pygame.Color('#FFCC00'), entrance_rect)

            if self.exit is not None:
                exit_rect = pygame.Rect(0, 0, self.maze_square_size, self.maze_square_size)
                exit_rect.center = self.exit.nav_node.position
                pygame.draw.rect(self.window_surface, pygame.Color('#FF7700'), exit_rect)

            self.ui_manager.draw_ui(self.window_surface)

            pygame.display.update()


if __name__ == "__main__":
    app = PathfindingApp()
    app.run()

"""
Task
----

Try and add another pathfinder that uses the Greedy Search algorithm.

"""
