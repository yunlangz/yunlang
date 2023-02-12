from enum import Enum
from point import Point
from matplotlib import pyplot as plt
from webpage import WepPage
from typing import *
from a_star_search import AStar
from recognize_objects import ObjectRecognition
import picar_4wd as fc
import numpy as np
import time
import sys
import threading


np.set_printoptions(threshold=sys.maxsize)

class Orientation(Enum):
    North = 0
    East = 1
    South = 2
    West = 3

# [y, x]
side_length = 400
world_map = np.zeros((side_length, side_length))
step = 10
current_servo_angle = 0
curr_position = None
curr_orientation = Orientation.North


class Ultrasonic:
    """
    Collection of functions to control and compute data from ultrasonic sensor
    """

    @staticmethod
    def find_objects():
        measurements = Ultrasonic.mapping_scan()
        points = [Ultrasonic.compute_point(dist=measurement[0], angle=measurement[1]) for measurement in measurements]

        global world_map
        last_point = None

        # Run through points found by sensor
        for point in points:

            # If point exists, mark as obstacle
            if point is not None:
                Ultrasonic.mark_point(point)

                # If last point was marked, mark points in between
                if last_point is not None:
                    points_in_between = Ultrasonic.interpolate_points(point, last_point)
                    for pnt in points_in_between:
                        Ultrasonic.mark_point(pnt)

            # Set this point as last checked point for interpolation
            last_point = point

    @staticmethod
    def mapping_scan() -> [float]:
        """
        Scans in front of the car
        :return: list of distance and angles (cm, degrees)
        """
        global step
        angle_range = 170
        max_angle: int = int(angle_range / 2)
        min_angle = max_angle * -1

        # (distance, angle)
        measurements: [(float, int)] = []

        # Sweep from min to max angle along step
        for angle in range(min_angle, max_angle, step):
            fc.servo.set_angle(angle)
            time.sleep(0.2)
            distance = Ultrasonic.get_distance()
            measurements.append((distance, angle))
            # print(f"Measurement ({distance}, {angle})")

        return measurements

    @staticmethod
    def mark_point(point: Point):
        global side_length

        if 0 <= point.x < side_length and 0 <= point.y < side_length:
            # Swapped in matrix
            world_map[point.y][point.x] = 1
#             print(f"Marking point {point}")

    @staticmethod
    def compute_point(dist: float, angle: int) -> Point:
        """
        Computes where the point is in space that is detected given the angle and curr position
        relative_point = (dist * sin(angle), dist * cos (angle))
        absolute_point = relative_point + curr_position
        :return: (x, y) coordinate
        """

        # filter out sensor limit readings
        if np.abs(100 - dist) <= 30 or dist < 0:
            # print(f"Filtering out {dist}")
            return None


        global curr_position, curr_orientation

        radians = np.deg2rad(angle)
        # x and y from car's reference frame
        x = dist * np.sin(radians)
        y = dist * np.cos(radians)

        """
        Below are the transformations done to the map reference
        For example, if the car is facing east and sees something 
        directly in front of it at 13cm, this translates to a +13cm
        in the x direction on the board (locked to the world)
        
        """
        if curr_orientation == Orientation.North:
            # (x, y)
            relative_point = Point(x, y)
        elif curr_orientation == Orientation.East:
            # (y, -x)
            relative_point = Point(-1 * y, x)
        elif curr_orientation == Orientation.South:
            # (-x, -y)
            relative_point = Point(-1 * x, -1 * y)
        elif curr_orientation == Orientation.West:
            # (-y, x)
            relative_point = Point(y, -1 * x)

        absolute_point = relative_point + curr_position

        return absolute_point

    @staticmethod
    def get_distance() -> int:
        """
        Gets distance from sensor
        :return: distance in cm
        """
        distance: int = fc.us.get_distance()  # cm
        # print(f"Distance: {distance}cm")

        return distance

    @staticmethod
    def interpolate_points(p1: Point, p2: Point) -> [Point]:
        # print(f'Interpolating {p1} and {p2}')
        if p2.x - p1.x == 0:
            # print(f'Infinite slope')
            return []
        slope = (p2.y - p1.y) / (p2.x - p1.x)
        y_intercept = p1.y - slope * p1.x
        # print (f"Slope {slope} and y-intercept {y_intercept}")
        points_to_fill_in = []
        sorted_x = sorted([p1.x, p2.x])

        # find all points between 2 points to fill in
        for x in range(sorted_x[0], sorted_x[1]):
            y = slope * x + y_intercept
            new_pnt = Point(x, y)
            # print(f"New Point {new_pnt}")
            points_to_fill_in.append(new_pnt)

        return points_to_fill_in

    @staticmethod
    def avoidance_scan() -> float:
        """
        Scans in front of the car and returns the distance
        :return: distance in cm
        """
        global current_servo_angle, step
        angle_range = 135
        max_angle = angle_range / 2
        min_angle = max_angle * -1

        current_servo_angle += step
        if current_servo_angle > max_angle:
            current_angle = max_angle
            step *= -1
        elif current_servo_angle < min_angle:
            current_angle = min_angle
            step *= -1

        fc.servo.set_angle(current_servo_angle)
        time.sleep(.2)
        distance = fc.us.get_distance()
        return distance

    @staticmethod
    def pad_world_map():
        '''
        Adds extra passing to obstacle found to help with car clearance
        :return:
        '''
        global world_map
        padded_map = np.copy(world_map)
        for x in range(12):
            temp_map = np.copy(padded_map)
            for row_i, row in enumerate(padded_map):
                for col_i, col in enumerate(row):
                    if col == 1:
                        neighbors = AStar.neighbors(temp_map, Point(col_i, row_i))
                        for neighbor in neighbors:
                            if neighbor != curr_position:
                                temp_map[neighbor.y][neighbor.x] = 1
            padded_map = temp_map

        return padded_map

class Movement:
    """
    Moves the car in each direction
    """

    class Direction(Enum):

        Left = 0
        Right = 1

        def turn(self):
            """
            Picks random direction and turns 90 degrees
            and turns in that direction
            :return: None
            """

            if self == Movement.Direction.Left:
                Movement.turn_left()
            else:
                Movement.turn_right()

    class Move:
        class Type(Enum):
            Forward = 0
            Left = 1
            Right = 2
            Backward = 3

        def __init__(self, type_of_move: Type, amount: int = None):
            self.type = type_of_move
            self.amount = amount

    turn_time = .55

    @staticmethod
    def turn_left(power: int = 50):
        fc.turn_left(power)
        time.sleep(Movement.turn_time)
        fc.stop()
        global curr_orientation
        curr_orientation = Location.update_orientation(Movement.Direction.Left)
        print(f"New orientation {curr_orientation}")

    @staticmethod
    def turn_right(power: int = 50):
        fc.turn_right(power)
        time.sleep(Movement.turn_time)
        fc.stop()
        global curr_orientation
        curr_orientation = Location.update_orientation(Movement.Direction.Right)
        print(f"New orientation {curr_orientation}")

    # 100 power over 1s is 1cm
    # distance (cm) = time * (power / 100 ) ?
    @staticmethod
    def move_backward(power: int = 10):
        fc.backward(power)

    @staticmethod
    def move_forward(power: int = 10):
        fc.forward(power)

    @staticmethod
    def compute_moves(path: [Point]) -> [Move]:
        global curr_orientation

        temp_orientation = curr_orientation

        last_point = None
        forward = None
        moves = []
        for next_point in path:
            if next_point is None or last_point is None:
                last_point = next_point
                continue
            # Still forward
            if temp_orientation in [Orientation.North, Orientation.South] and last_point.x == next_point.x \
                    or temp_orientation in [Orientation.East, Orientation.West] and last_point.y == next_point.y:
                if forward is None:
                    forward = Movement.Move(Movement.Move.Type.Forward, 0)
                forward.amount += 1
            elif next_point.x > last_point.x and temp_orientation == Orientation.North or \
                    next_point.x < last_point.x and temp_orientation == Orientation.South or \
                    next_point.y < last_point.y and temp_orientation == Orientation.West or \
                    next_point.y > last_point.y and temp_orientation == Orientation.East:

                if forward is not None:
                    print(f"Move forward for {forward.amount}")
                    moves.append(forward)
                print("Turn left")
                moves.append(Movement.Move(Movement.Move.Type.Left))
                temp_orientation = Location.update_orientation(Movement.Direction.Left, orientation=temp_orientation)
                forward = Movement.Move(Movement.Move.Type.Forward, 0)
            else:
                if forward is not None:
                    print(f"Move forward for {forward.amount}")
                    moves.append(forward)
                print("Turn right")
                moves.append(Movement.Move(Movement.Move.Type.Right))
                temp_orientation = Location.update_orientation(Movement.Direction.Right, orientation=temp_orientation)
                forward = Movement.Move(Movement.Move.Type.Forward, 0)

            last_point = next_point

        print(f"Move forward for {forward.amount}")
        moves.append(forward)

        return moves


found_stop_sign = False

class Location:
    """
    Maintains location of car in space
    """

    @staticmethod
    def monitor_location(stop_at: int):
        speeds = []
        start_time = time.perf_counter()
        while True:
            global found_stop_sign
            if found_stop_sign:
                fc.stop()
                continue
            Movement.move_forward()
            speeds.append(Location.speed())
            elapsed_time = time.perf_counter() - start_time
            distance = Location.distance_traveled(elapsed_time, speeds)
            # print(f"{distance}")

            if abs(distance - stop_at) < .5:
                global stopped_moving
                break

        fc.stop()
        fc.left_rear_speed.deinit()
        fc.right_rear_speed.deinit()

        Location.update_location(distance)

    @staticmethod
    def update_location(new_location: float):
        """

        :return:
        """

        global curr_orientation, curr_position
        if curr_orientation == Orientation.North:
            # (x, y)
            relative_point = Point(0, new_location)
        elif curr_orientation == Orientation.East:
            # (y, -x)
            relative_point = Point(-1 * new_location, 0)
        elif curr_orientation == Orientation.South:
            # (-x, -y)
            relative_point = Point(0, -1 * new_location)
        elif curr_orientation == Orientation.West:
            # (-y, x)
            relative_point = Point(new_location, 0)

        curr_position += relative_point

        print(f"New position {curr_position}")

    @staticmethod
    def update_orientation(turn_direction: Movement.Direction, orientation=None):
        """

        :return:
        """

        if orientation == None:
            global curr_orientation
            updated = curr_orientation
        else:
            updated = orientation

        if turn_direction == Movement.Direction.Left:
            value = -1
        else:
            value = 1
        updated = Orientation((updated.value + value) % 4)
        return updated

    @staticmethod
    def distance_traveled(time_elapsed, speed_intervals) -> int:
        """
        speed / time
        :return: distance in cm
        """
        if len(speed_intervals) == 0:
            return 0
        mean_speed = np.mean(speed_intervals)
        distance =  mean_speed * time_elapsed
        # print(f"Distance traveled {distance}cm")

        return distance

    @staticmethod
    def speed() -> float:
        """
        :return: speed in cm/s
        """
        speed_reading = fc.speed_val()
        # print(f"Current speed: {speed_reading} cm/s")
        return speed_reading

finished = False

def recognize_objects():
    with ObjectRecognition() as recognizer:
        for frame in recognizer.camera.capture_continuous(recognizer.rawCapture, format="bgr", use_video_port=True):

            global finished
            if finished:
                return

            image = frame.array
            # Resize
            rgb = recognizer.process_images(image)
            results = recognizer.detect_objects(rgb, 0.4)
            objects = [recognizer.label_from_class_id(recognized_object["class_id"]) for recognized_object in results]
            print(objects)
            recognizer.rawCapture.truncate(0)
            if "stop sign" in objects:
                global found_stop_sign
                found_stop_sign = True
                print("Stop sign")
            else:
                found_stop_sign = False

def main():
    # Process(target=WepPage.run).start()
    global curr_position
    # Starting point
    curr_position = Point(200, 0)
    end = Point(200, 250)

    done = False
    fc.start_speed_thread()
    i = 0
    thread = threading.Thread(target=recognize_objects)
    thread.start()
    while not done:
        # ================================
        # Scan 180 FOV, Update map, pad the objects
        print("Finding objects")
        Ultrasonic.find_objects()

        print("Padding world map for clearance")
        padded_map = Ultrasonic.pad_world_map()

        # ================================
        # Find best possible path
        print("Searching for best possible path")
        came_from, cost_so_far = AStar.search(padded_map, curr_position, end)

        new_map = np.full(np.shape(padded_map), -1)
        for point, cost in cost_so_far.items():
            new_map[point.y][point.x] = cost

        last_elm = end
        path_forward = [last_elm]
        while last_elm is not None:
            last_elm = came_from[last_elm]
            path_forward.append(last_elm)

        # Path is reversed
        path_forward.reverse()

        #cutoff part of path to rescan
        cutoff = 70
        if len(path_forward) > cutoff:
            path_forward = path_forward[0:cutoff]
        else:
            # Last leg of the journey
            print("To the finish")
            done = True


        # ================================
        # Save photo of path
        cmap = plt.cm.gray
        norm = plt.Normalize(padded_map.min(), padded_map.max())
        rgba = cmap(norm(padded_map))

        print("Saving map to png")
        for point in path_forward:
            if point is not None:
                # Set path pixels to blue
                rgba[point.y][point.x] = 0, 0, 1, 1

        # Set start and end pixel
        rgba[end.y][end.x] = 1, 0, 0, 1
        rgba[curr_position.y][curr_position.x] = 0, 1, 0, 1

        plt.imshow(rgba, interpolation='nearest')
        plt.savefig(f"/home/pi/Desktop/map_search_{i}.png")

        # ================================
        # Compute moves and move
        print("Computing moves to make")
        moves = Movement.compute_moves(path_forward)
        for move in moves:
            print(f"Make move {move.type} for {move.amount}")
            if move.type == Movement.Move.Type.Forward:
                Movement.move_forward()
                Location.monitor_location(stop_at=move.amount)
            elif move.type == Movement.Move.Type.Left:
                Movement.turn_left()
            else:
                Movement.turn_right()

        i += 1
        print(f"Ended at {curr_position}")

    finished = True
    thread.join()

if __name__ == "__main__":
    try:
        main()
    finally:
        finished = True
        fc.stop()
