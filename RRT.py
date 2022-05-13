# import pygame
from matplotlib import pyplot as plt
# from matplotlib.patches import Rectangle
from matplotlib.patches import Circle

from RRTbasePy import RRTGraph
from RRTbasePy import RRTMap


# import time


def main():
    dimensions = (500, 500)
    start = (50, 50)
    goal = (410, 410)
    obsdim = 20
    obsnum = 55
    iteration = 0
    # t1=0

    plt.ion()
    fig = plt.figure(num='RRT Path Planning', figsize=(10, 10))
    plt.xlim(0, 500)
    plt.ylim(0, 500)
    map = RRTMap(start, goal, dimensions, obsdim, obsnum)
    graph = RRTGraph(start, goal, dimensions, obsdim, obsnum)


    obstacles = graph.makeobs()
    map.drawMap(obstacles)

    # while (True):
    #     pygame.time.wait(0)
    #     x, y = graph.sample_envir()
    #     n = graph.number_of_nodes()
    #     graph.add_node(n, x, y)
    #     graph.add_edge(n - 1, n)
    #     x1, y1 = graph.x[n], graph.y[n]
    #     x2, y2 = graph.x[n - 1], graph.y[n - 1]
    #     if (graph.isFree()):
    #         pygame.draw.circle(map.map, map.R  ed, (graph.x[n], graph.y[n]), map.nodeRad, map.nodeThickness)
    #         if not graph.crossObstacle(x1, y1, x2, y2):
    #             pygame.draw.line(map.map, map.Blue, (x1, y1), (x2, y2), map.edgeThickness)
    #     pygame.display.update()

    # t1 = time.time()
    while (not graph.path_to_goal()):
        # elapsed = time.time() - 1
        # t1 = time.time()
        # if elapsed > 10:
        #     raise
        if iteration % 10 == 0:
            X, Y, Parent = graph.bias(goal)
            Circle1 = Circle((X[-1], Y[-1]), map.nodeRad + 2, facecolor='gray', fill = 0)
            fig.gca().add_patch(Circle1)
            plt.plot((X[-1], X[Parent[-1]]), (Y[-1], Y[Parent[-1]]), 'blue')
        else:
            X, Y, Parent = graph.expand()
            Circle1 = Circle((X[-1], Y[-1]), map.nodeRad + 2, facecolor='gray', fill=0)
            fig.gca().add_patch(Circle1)
            plt.plot((X[-1], X[Parent[-1]]), (Y[-1], Y[Parent[-1]]), 'blue')

        if iteration % 5 == 0:
            fig.canvas.draw()
        iteration += 1

    map.drawPath(graph.getPathCoords())


    # fig.canvas.draw()
    # fig.canvas.flush_events()
    plt.waitforbuttonpress()


if __name__ == "__main__":
    result = False
    while not result:
        try:
            main()
            result = True
        except:
            result = False
            plt.clf()
