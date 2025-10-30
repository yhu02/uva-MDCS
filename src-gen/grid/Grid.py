from grid.node import Node
from grid.visualiser import Visualizer

class Maze:
    """Class that handles the datastructure of the grid


    Attributes:
        grid_cols (int): ammount of columns in the grid
        grid_rows (int): ammount of rows in the grid
        grid: nested list of size grid_rows x grid_cols containing Nodes
        visualiser: class that can visualise the grid
    """
    # initiate main variables
    def __init__(self, cols, rows):
        self.grid_cols = cols
        self.grid_rows = rows
        self.create_grid()
        
    # Create grid based on the ammount of columns and rows
    def create_grid(self):
        self.grid = []
        for i in range(self.grid_rows):
            self.grid.append([])
            for j in range(self.grid_cols):
                self.grid[i].append(Node(i,j))

    # Draw the maze to an image
    def draw(self):
        self.visualiser = Visualizer(self, 2, "map")
        self.visualiser.show_maze()

if __name__ == "__main__":
    m = Maze(4, 4)
    print(m.grid[0][0].walls[0])
    #m.grid[0][0].walls[0] = 1
    #m.grid[0][0].walls[1] = 1
    print(m.grid[0][0].walls[0])

    m.grid[0][0].walls[0] = 1
    m.grid[0][0].walls[1] = 0 # overwritten by next cell
    m.grid[0][0].walls[2] = 1 # not overwritten, matches wall(1)
    m.grid[0][0].walls[3] = 1

    m.grid[1][0].walls[0] = 1 # wall(1)

    m.grid[2][2].walls[0] = 1
    m.grid[2][2].walls[1] = 0 # matches data in wall(2)
    m.grid[2][2].walls[2] = 1
    m.grid[2][2].walls[3] = 1

    m.grid[2][3].walls[3] = 0 # wall(2)

    m.draw()
