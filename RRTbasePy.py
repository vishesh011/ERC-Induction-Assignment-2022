from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.patches import Circle
from shapely.geometry import Point, Polygon
# import numpy as np
import random
import math
# import pygame


class RRTMap:
    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        self.start = start
        self.goal = goal
        self.MapDimensions = MapDimensions
        self.Maph, self.Mapw = self.MapDimensions

        # window settings
        self.MapWindowName = 'RRT path planning'
        self.nodeRad = 1
        self.nodeThickness = 0
        self.edgeThickness = 1

        self.obstacles = []
        self.obsdim = obsdim
        self.obsNumber = obsnum

        # colors
        self.gray = (70, 70, 70)
        self.Blue = (0, 0, 255)
        self.Green = (0, 255, 0)
        self.Red = (255, 0, 0)
        self.white = (255, 255, 255)

    def drawMap(self, obstacles):
        fig = plt.gcf()
        circle1 = Circle((self.start), self.nodeRad + 5, facecolor='green', fill=1)
        fig.gca().add_patch(circle1)
        circle2 = Circle((self.goal), self.nodeRad + 20, edgecolor='green', fill=0)
        fig.gca().add_patch(circle2)
        self.drawObs(obstacles)

    def drawPath(self, path):
        for node in path:
            # pygame.draw.circle(self.map, self.Red, node, self.nodeRad + 3, 0)
            Circle1 = Circle(node, self.nodeRad + 3, facecolor='red', fill=1)
            plt.gca().add_patch(Circle1)
            Circle2 = Circle(node, self.nodeRad + 2, facecolor='gray', fill=0)
            plt.gca().add_patch(Circle2)

    def drawObs(self, obstacles):
        fig = plt.gcf()
        obstaclesList = obstacles.copy()
        while (len(obstaclesList) > 0):
            obstacle = obstaclesList.pop(0)
            # pygame.draw.rect(self.map, self.gray, obstacle)
            # Rectangle((obstacle), self.obsDim, self.obsDim)
            fig.gca().add_patch(obstacle)


class RRTGraph:
    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        (x, y) = start
        self.start = start
        self.goal = goal
        self.goalFlag = False
        self.maph, self.mapw = MapDimensions
        self.x = []
        self.y = []
        self.parent = []

        # initialize the tree
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)

        # the obstacles
        self.obstacles = []
        self.obsDim = obsdim
        self.obsNum = obsnum

        # path
        self.goalstate = None
        self.path = []

    def makeRandomRect(self):
        lower_left_x = int(random.uniform(0, self.mapw - self.obsDim))
        lower_left_y = int(random.uniform(0, self.maph - self.obsDim))

        return (lower_left_x, lower_left_y)

    def makeobs(self):
        obs = []
        fig = plt.gcf()
        for i in range(0, self.obsNum):
            rectang = None
            startgoalcol = True
            while startgoalcol:
                lower_left = self.makeRandomRect()
                rectang = Rectangle((lower_left), self.obsDim, self.obsDim, facecolor= 'gray')
                # fig.gca().add_patch(rectang)
                if rectang.contains_point(self.start) or rectang.contains_point(self.goal):
                    startgoalcol = True
                else:
                    startgoalcol = False
            obs.append(rectang)
        self.obstacles = obs.copy()
        return obs

    def add_node(self, n, x, y):
        self.x.insert(n, x)
        self.y.insert(n, y)

    def remove_node(self, n):
        self.x.pop(n)
        self.y.pop(n)

    def add_edge(self, parent, child):
        self.parent.insert(child, parent)

    def remove_edge(self, n):
        self.parent.pop(n)

    def number_of_nodes(self):
        return len(self.x)

    def distance(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        px = (float(x1) - float(x2)) ** 2
        py = (float(y1) - float(y2)) ** 2
        return (px + py) ** (0.5)

    def sample_envir(self):
        x = int(random.uniform(0, self.mapw))
        y = int(random.uniform(0, self.maph))
        return x, y

    def nearest(self, n):
        dmin = self.distance(0, n)
        nnear = 0
        for i in range(0, n):
            if self.distance(i, n) < dmin:
                dmin = self.distance(i, n)
                nnear = i
        return nnear

    def isFree(self):
        n = self.number_of_nodes() - 1
        (x, y) = (self.x[n], self.y[n])
        obs = self.obstacles.copy()
        while len(obs) > 0:
            rectang = obs.pop(0)
            pnt = Point(x, y)
            coords = rectang.get_patch_transform().transform(rectang.get_path().vertices[:-1])
            poly = Polygon(coords)

            if poly.contains(pnt):
                self.remove_node(n)
                return False
        return True

    def crossObstacle(self, x1, y1, x2, y2):
        obs = self.obstacles.copy()
        while (len(obs) > 0):
            rectang = obs.pop(0)
            for i in range(0, 101):
                u = i / 100
                x = x1 * u + x2 * (1 - u)
                y = y1 * u + y2 * (1 - u)
                pnt = Point(x, y)
                coords = rectang.get_patch_transform().transform(rectang.get_path().vertices[:-1])
                poly = Polygon(coords)
                if poly.contains(pnt):
                    return True
        return False

    def connect(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        if self.crossObstacle(x1, y1, x2, y2):
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1, n2)
            return True

    def step(self, nnear, nrand, dmax=25):
        d = self.distance(nnear, nrand)
        if d > dmax:
            u = dmax / d
            (xnear, ynear) = (self.x[nnear], self.y[nnear])
            (xrand, yrand) = (self.x[nrand], self.y[nrand])
            (px, py) = (xrand - xnear, yrand - ynear)
            theta = math.atan2(py, px)
            (x, y) = (int(xnear + dmax * math.cos(theta)),
                      int(ynear + dmax * math.sin(theta)))
            self.remove_node(nrand)
            if abs(x - self.goal[0]) < dmax and abs(y - self.goal[0]) < dmax:
                self.add_node(nrand, self.goal[0], self.goal[1])
                self.goalstate = nrand
                self.goalFlag = True
            else:
                self.add_node(nrand, x, y)

    def path_to_goal(self):
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalstate)
            newpos = self.parent[self.goalstate]
            while (newpos != 0):
                self.path.append(newpos)
                newpos = self.parent[newpos]
            self.path.append(0)
        return self.goalFlag

    def getPathCoords(self):
        pathCoords = []
        for node in self.path:
            x, y = (self.x[node], self.y[node])
            pathCoords.append((x, y))
        return pathCoords

    def bias(self, ngoal):
        n = self.number_of_nodes()
        self.add_node(n, ngoal[0], ngoal[1])
        nnear = self.nearest(n)
        self.step(nnear, n)
        self.connect(nnear, n)
        return self.x, self.y, self.parent

    def expand(self):
        n = self.number_of_nodes()
        x, y = self.sample_envir()
        self.add_node(n, x, y)
        if self.isFree():
            xnearest = self.nearest(n)
            self.step(xnearest, n)
            self.connect(xnearest, n)
        return self.x, self.y, self.parent

    def cost(self):
        pass
