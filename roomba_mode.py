import picar_4wd as fc
import random
from enum import Enum
import time

step = 20
current_angle = 0

class Direction(Enum):
    Left = 0
    Right = 1

    def turn(self):
        """
        Picks a random time in seconds between [.5, 2]
        and turns in that direction
        :return: None
        """
        seconds: float = random.uniform(.25, 1)
        if self == Direction.Left:
            turn_left()
        else:
            turn_right()
        time.sleep(seconds)


def get_distance() -> int:
    """
    Gets distance in front of car
    :return: distance in cm
    """
    distance: int = fc.us.get_distance() #cm
    print(f"Distance: {distance}cm")

    return distance


def scan() -> float:
    """
    Scans in front of the car and returns the distance
    :return: distance in cm
    """
    global current_angle, step
    angle_range = 140
    max_angle = angle_range / 2
    min_angle = max_angle * -1

    current_angle += step
    if current_angle > max_angle:
        current_angle = max_angle
        step *= -1
    elif current_angle < min_angle:
        current_angle = min_angle
        step *= -1

    fc.servo.set_angle(current_angle)
    time.sleep(.04)
    distance = get_distance()
    return distance


def turn_random_direction():
    """
    Picks a random direction and turns car that way
    :return: None
    """
    direction: Direction = random.choice(list(Direction))
    print(f"Turning {direction}")
    direction.turn()


def turn_left(power: int = 50):
    fc.turn_left(power)


def turn_right(power: int = 50):
    fc.turn_right(power)


def move_backward(power: int = 10):
    print("Backing up")
    fc.backward(power)


def move_forward(power: int = 10):
    print("Onward!")
    fc.forward(power)


def main():
    fc.servo.set_angle(current_angle)
    while True:
        distance = scan()
        move_forward()
        if distance < 15:
            move_backward()
            time.sleep(.75)
            turn_random_direction()
            move_forward()


if __name__ == "__main__":
    try:
        main()
    finally:
        fc.stop()
