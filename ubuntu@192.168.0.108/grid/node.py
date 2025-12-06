

class Node:
    """Class that handles storage of singular nodes.


    Attributes:
        walls: list of wall data in order of [N, E, S, W]
        visited: boolean wether a node has been visited
        start: posibility of storing where the start is
        exit: posibility of storing where the exit is
        row, col: location in the larger grid
    """
    def __init__(self, row, col):
        self.walls = [-1, -1, -1, -1]
        self.visited = False

        # currently unused meant for improved grid
        self.start = False
        self.exit = False

        self.row = row
        self.col = col
