class Point:
    """
    Used to denote x, y coordinated
    """
    x: int
    y: int
    obstacle: bool

    def __init__(self, x: float, y: float):
        self.x = int(x)
        self.y = int(y)

    def __add__(self, other):
        return Point(self.x + other.x,  self.y + other.y)

    def __lt__(self, other):
        return self.x < other.x and self.y < other.y

    def __str__(self):
        return f"({self.x},{self.y})"

    def __hash__(self):
        return hash((self.x, self.y))

    def __eq__(self, other):
        return other is not None and self.x == other.x and self.y == other.y
