import matplotlib.pyplot as plt

# Code from: https://github.com/jostbr/pymaze edited to fit this datastructure

class Visualizer(object):
    """Class that handles all aspects of visualization.


    Attributes:
        maze: The maze that will be visualized
        cell_size (int): How large the cells will be in the plots
        height (int): The height of the maze
        width (int): The width of the maze
        ax: The axes for the plot
        lines:
        squares:
        media_filename (string): The name of the animations and images

    """
    def __init__(self, maze, cell_size, media_filename):
        self.maze = maze
        self.cell_size = cell_size
        self.height = maze.grid_rows * cell_size
        self.width = maze.grid_cols * cell_size
        self.ax = None
        self.lines = dict()
        self.squares = dict()
        self.media_filename = media_filename

    def set_media_filename(self, filename):
        """Sets the filename of the media
            Args:
                filename (string): The name of the media
        """
        self.media_filename = filename

    def show_maze(self):
        """Displays a plot of the maze without the solution path"""

        # Create the plot figure and style the axes
        fig = self.configure_plot()

        # Plot the walls on the figure
        self.plot_walls()

        fig.savefig("{}{}.png".format(self.media_filename, "_generation"), frameon=None)



    def plot_walls(self):
        """ Plots the walls of a maze. This is used when generating the maze image"""
        rev_i = list(range(self.maze.grid_rows))
        rev_i.reverse()
        # colors is indexed with wall status:
        # wall == 1     => black
        # unknown == -1 => '0.9' is very light gray
        # no_wall == 0  => white

        # If neighbouring cells have inconsistent data on their shared wall
        # the latter drawn cell will overwrite the former.
        # TODO: check for inconsistant data and mark it as red.
        colors = ['white', 'black', '0.7']
        transparency = [0.5, 0.5, 0.5]
        linestyle = ['solid', 'solid', 'dotted']
        for i in range(self.maze.grid_rows):
            for j in range(self.maze.grid_cols):
                self.ax.plot([j*self.cell_size, (j+1)*self.cell_size],
                             [(rev_i[i]+1)*self.cell_size, (rev_i[i]+1)*self.cell_size],
                             color=colors[self.maze.grid[i][j].walls[0]],
                             alpha=transparency[self.maze.grid[i][j].walls[0]],
                             linestyle=linestyle[self.maze.grid[i][j].walls[0]])
                self.ax.plot([(j+1)*self.cell_size, (j+1)*self.cell_size],
                             [rev_i[i]*self.cell_size, (rev_i[i]+1)*self.cell_size],
                             color=colors[self.maze.grid[i][j].walls[1]],
                             alpha=transparency[self.maze.grid[i][j].walls[1]],
                             linestyle=linestyle[self.maze.grid[i][j].walls[1]])
                self.ax.plot([(j+1)*self.cell_size, j*self.cell_size],
                             [(rev_i[i])*self.cell_size, (rev_i[i])*self.cell_size],
                             color=colors[self.maze.grid[i][j].walls[2]],
                             alpha=transparency[self.maze.grid[i][j].walls[2]],
                             linestyle=linestyle[self.maze.grid[i][j].walls[2]])
                self.ax.plot([j*self.cell_size, j*self.cell_size],
                             [(rev_i[i]+1)*self.cell_size, rev_i[i]*self.cell_size],
                             color=colors[self.maze.grid[i][j].walls[3]],
                             alpha=transparency[self.maze.grid[i][j].walls[3]],
                             linestyle=linestyle[self.maze.grid[i][j].walls[3]])

    def configure_plot(self):
        """Sets the initial properties of the maze plot. Also creates the plot and axes"""

        # Create the plot figure
        fig = plt.figure(figsize = (7, 7*self.maze.grid_rows/self.maze.grid_rows))

        # Create the axes
        self.ax = plt.axes()

        # Set an equal aspect ratio
        self.ax.set_aspect("equal")

        # Remove the axes from the figure
        self.ax.axes.get_xaxis().set_visible(False)
        self.ax.axes.get_yaxis().set_visible(False)

        # title_box = self.ax.text(0, self.maze.grid_rows + self.cell_size + 0.1, r"{}$\times${}".format(self.maze.grid_rows, self.maze.grid_cols), bbox={"facecolor": "gray", "alpha": 0.5, "pad": 4}, fontname="serif", fontsize=15)

        return fig
