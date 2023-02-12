import heapq
from typing import *
from point import Point
import numpy as np

T = TypeVar('T')


class PriorityQueue:
    def __init__(self):
        self.elements: List[Tuple[float, T]] = []

    def empty(self) -> bool:
        return not self.elements

    def put(self, item: T, priority: float):
        heapq.heappush(self.elements, (priority, item))

    def get(self) -> T:
        return heapq.heappop(self.elements)[1]


class AStar:

    @staticmethod
    def heuristic(a: Point, b: Point) -> float:
        return abs(a.x - b.x) + abs(a.y - b.y)

    @staticmethod
    def neighbors(map, current: Point) -> [Point]:
        directions = [Point(1, 0), Point(0, 1), Point(-1, 0), Point(0, -1)]
        neighbors_of_current = [current + direction for direction in directions]
        rows, columns = np.shape(map)
        filtered = filter(lambda neighbor: 0 <= neighbor.x < columns and 0 <= neighbor.y < rows and \
                                           map[neighbor.y][neighbor.x] == 0, neighbors_of_current)
        return filtered

    @staticmethod
    def search(map, start: Point, goal: Point):
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from: Dict[Point, Optional[Point]] = {}
        cost_so_far: Dict[Point, float] = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            current: Point = frontier.get()

            if current == goal:
                break

            for next_neighbor in AStar.neighbors(map, current):
                parent_of_current = came_from.get(current, None)
                # Favor straight lines
                if parent_of_current:
                    if next_neighbor.x == current.x == parent_of_current.x or \
                            next_neighbor.y == current.y == parent_of_current.y:
                        new_cost = cost_so_far[current] + 1
                    else:
                        new_cost = cost_so_far[current] + 5
                else:
                    new_cost = cost_so_far[current] + 1

                if next_neighbor not in cost_so_far or new_cost < cost_so_far[next_neighbor]:
                    cost_so_far[next_neighbor] = new_cost
                    priority = new_cost + AStar.heuristic(next_neighbor, goal)
                    frontier.put(next_neighbor, priority)
                    came_from[next_neighbor] = current

        return came_from, cost_so_far


if __name__ == '__main__':

    maze = np.array([[0, 1, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 1, 0, 0],
                     [0, 1, 0, 0, 1, 0],
                     [0, 0, 0, 0, 1, 0]])

    new_maze = np.full(np.shape(maze), -1)

    start = Point(0, 0)  # starting position
    end = Point(5, 4)  # ending position

    came_from, cost_so_far = AStar.search(maze, start, end)
    for point, cost in cost_so_far.items():
        new_maze[point.y][point.x] = cost
    print(new_maze)
    last_elm = end
    print(last_elm)
    while last_elm is not None:
        last_elm = came_from[last_elm]
        print(last_elm)
