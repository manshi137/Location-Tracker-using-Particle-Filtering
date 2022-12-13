'''
Licensing Information: Please do not distribute or publish solutions to this
project. You are free to use and extend Driverless Car for educational
purposes. The Driverless Car project was developed at Stanford, primarily by
Chris Piech (piech@cs.stanford.edu). It was inspired by the Pacman projects.

'''
import util
import math
import itertools
from turtle import Vec2D
from engine.const import Const
from engine.vector import Vec2d
from engine.model.car.car import Car
from engine.model.layout import Layout
from engine.model.block import Block
from engine.model.car.junior import Junior
from configparser import InterpolationMissingOptionError
import random
# Class: Graph
# -------------

class Graph(object):
    def __init__(self, nodes, edges):
        self.nodes = nodes
        self.edges = edges

# Class: IntelligentDriver
# ---------------------
# An intelligent driver that avoids collisions while visiting the given goal locations (or checkpoints) sequentially. 
class IntelligentDriver(Junior):
    MIN_PROB =0.04
    # Funciton: Init
    def __init__(self, layout: Layout):
        self.burnInIterations = 30
        self.layout = layout 
        # self.worldGraph = None
        self.worldGraph = self.createWorldGraph()
        self.checkPoints = self.layout.getCheckPoints() # a list of single tile locations corresponding to each checkpoint
        self.transProb = util.loadTransProb()
        
    # ONE POSSIBLE WAY OF REPRESENTING THE GRID WORLD. FEEL FREE TO CREATE YOUR OWN REPRESENTATION.
    # Function: Create World Graph
    # ---------------------
    # Using self.layout of IntelligentDriver, create a graph representing the given layout.
    def createWorldGraph(self):
        nodes = []
        edges = []
        # create self.worldGraph using self.layout
        numRows, numCols = self.layout.getBeliefRows(), self.layout.getBeliefCols()

        # NODES #
        ## each tile represents a node
        nodes = [(x, y) for x, y in itertools.product(range(numRows), range(numCols))]
        
        # EDGES #
        ## We create an edge between adjacent nodes (nodes at a distance of 1 tile)
        ## avoid the tiles representing walls or blocks#
        ## YOU MAY WANT DIFFERENT NODE CONNECTIONS FOR YOUR OWN IMPLEMENTATION,
        ## FEEL FREE TO MODIFY THE EDGES ACCORDINGLY.

        ## Get the tiles corresponding to the blocks (or obstacles):
        blocks = self.layout.getBlockData()
        blockTiles = []
        for block in blocks:
            row1, col1, row2, col2 = block[1], block[0], block[3], block[2] 
            # some padding to ensure the AutoCar doesn't crash into the blocks due to its size. (optional)
            row1, col1, row2, col2 = row1-1, col1-1, row2+1, col2+1
            blockWidth = col2-col1 
            blockHeight = row2-row1 

            for i in range(blockHeight):
                for j in range(blockWidth):
                    blockTile = (row1+i, col1+j)
                    blockTiles.append(blockTile)

        ## Remove blockTiles from 'nodes'
        nodes = [x for x in nodes if x not in blockTiles]

        for node in nodes:
            x, y = node[0], node[1]
            adjNodes = [(x, y-1), (x, y+1), (x-1, y), (x+1, y)]
            
            # only keep allowed (within boundary) adjacent nodes
            adjacentNodes = []
            for tile in adjNodes:
                if tile[0]>=0 and tile[1]>=0 and tile[0]<numRows and tile[1]<numCols:
                    if tile not in blockTiles:
                        adjacentNodes.append(tile)

            for tile in adjacentNodes:
                edges.append((node, tile))
                edges.append((tile, node))
        return Graph(nodes, edges)

    #######################################################################################
    # Function: Get Next Goal Position
    # ---------------------
    # Given the current belief about where other cars are and a graph of how
    # one can driver around the world, chose the next position.
    #######################################################################################
    def getNextGoalPos(self, beliefOfOtherCars: list, parkedCars:list , chkPtsSoFar: int):
        '''
        Input:
        - beliefOfOtherCars: list of beliefs corresponding to all cars
        - parkedCars: list of booleans representing which cars are parked
        - chkPtsSoFar: the number of checkpoints that have been visited so far 
                       Note that chkPtsSoFar will only be updated when the checkpoints are updated in sequential order!
        Output:
        - goalPos: The position of the next tile on the path to the next goal location.
        - moveForward: Unset this to make the AutoCar stop and wait.
        Notes:
        - You can explore some files "layout.py", "model.py", "controller.py", etc.
         to find some methods that might help in your implementation. 
        '''
        # print(f"checkedpoints before {chkPtsSoFar}")
        goalPos = (0, 0) # next tile 
        moveForward = True
        currPos = self.getPos() # the current 2D location of the AutoCar (refer util.py to convert it to tile (or grid cell) coordinate)
        # BEGIN_YOUR_CODE 
        checkpt = self.checkPoints
        nextcheck = checkpt[chkPtsSoFar]

        # currgrid = checkpt[chkPtsSoFar-1]
        currgrid = (util.yToRow(currPos[1]), util.xToCol(currPos[0]))

        graph = self.createWorldGraph()
        nodes = graph.nodes
        edges = graph.edges
        xytov = dict()
        i=0
        for n in nodes:
            xytov[n]= i
            i+=1
        adjacent = dict()
        # print(edges)
        for (e1, e2) in edges:
            if e1 not in adjacent.keys():
                adjacent[e1]=[e2]
            else:
                adjacent[e1].append(e2)
            if e2 not in adjacent.keys():
                adjacent[e2]=[e1]
            else:
                adjacent[e2].append(e1)
        # print(edges)
        visited =dict()
        dist = dict()
        parent = dict()
        for n in nodes:
            visited[n]=False
            dist[n]= 10000
        queue = []
        queue.append(currgrid)
        dist[currgrid] = 0
        visited[currgrid] = True
        parent[currgrid] = [None]
        paths = dict()
        neighbours=[]
        (currgridx, currgridy) = currgrid
        neighbours.append((currgridx-1, currgridy-1))
        neighbours.append((currgridx-1, currgridy+1))
        neighbours.append((currgridx+1, currgridy-1))
        neighbours.append((currgridx+1, currgridy+1))
        neighbours.append((currgridx-1, currgridy))
        neighbours.append((currgridx+1, currgridy))
        neighbours.append((currgridx, currgridy-1))
        neighbours.append((currgridx, currgridy+1))
        numRows, numCols = self.layout.getBeliefRows(), self.layout.getBeliefCols()
        # print(f"neighbours {neighbours}")
        dist = dict()
        l = len(neighbours)-1
        while(l>=0):
            (x, y) = neighbours[l]
            if neighbours[l] not in nodes:
                neighbours.pop(l)
                # print(neighbours)
                l-=1
            else:
                l-=1
                # k-=1
        # print(f"neighbours {neighbours}")
        nextcheckx, nextchecky = nextcheck
        # print(f"{nextcheckx}  {nextchecky}")
        for n in neighbours:
            nx, ny = n
            d = math.sqrt((nextcheckx-nx)**2 + (nextchecky-ny)**2)
            d_old = math.sqrt((nx-currgridx)**2 + (ny-currgridy)**2)
            dist[n] = d+d_old
        inv_dist = dict()
        for dd in dist:
            inv_dist[dist[dd]] = dd
        distkeys = list(dist.keys())

        sorted_list = sorted(inv_dist.keys())
        new_dict = dict()
        for d in sorted_list:
            new_dict[inv_dist[d]] = d

        sorted_dist = new_dict
        # print(f"dist : {dist}")
        # print(f"sorted dist : {sorted_dist}")

        distkeys = list(sorted_dist.keys())
        best_neighbour= distkeys[0]
        bestbelief=100
        moveForward=False
                
        # for m in range(len(beliefOfOtherCars)):
        #     x_best, y_best= best_neighbour
        #     bestbelief+=beliefOfOtherCars[m].getProb(x_best, y_best)

        for n in sorted_dist.keys():
            offset = self.dir.normalized() * 1.5 * Car.LENGTH
            my= util.rowToY(n[0])
            mx= util.colToX(n[1])
            mxy= (mx, my)
            newPos = mxy + offset
            
            
            row = util.yToRow(newPos.y)
            col = util.xToCol(newPos.x)
            
            # print(f"row {row} col {col}")
            if(row>=numRows or col>=numCols):
                newPos-=offset
                row = util.yToRow(newPos.y)
                col = util.xToCol(newPos.x)

            
            correct= True
            beliefsum=100
            maxbelief = -1
            # print("maxbelief intilaise")
            for k in range(len(beliefOfOtherCars)):
                # beliefsum+= beliefOfOtherCars[k].getProb(row , col)
                # print(f"row {row} col {col}")
                maxbelief= max(maxbelief, beliefOfOtherCars[k].getProb(row , col))

                # print(f"curr belief {k}   ..... {beliefOfOtherCars[k].getProb(row , col)}")
                # print(f"maxbelief {k}   ..... {maxbelief}")
                if(beliefOfOtherCars[k].getProb(row , col)>IntelligentDriver.MIN_PROB):
                    correct=False
                    break
            # if(beliefsum <IntelligentDriver.MIN_PROB*len(beliefOfOtherCars)/2 and correct==True):
            if(maxbelief <IntelligentDriver.MIN_PROB and correct==True):
                bestbelief=maxbelief
                best_neighbour=n
                moveForward=True
                break
            

        # if(bestbelief<IntelligentDriver.MIN_PROB*len(beliefOfOtherCars)/2):
        # moveForward=True
        x_best, y_best= best_neighbour
        # print("...........................................")
        # print(f"row {x_best} col {y_best}")
        ycord= util.rowToY(x_best)
        xcord= util.colToX(y_best)


        # else:
        #     moveForward=False
        #     ycord= util.rowToY(currgrid[0])
        #     xcord= util.colToX(currgrid[1])
        # (xcord, ycord) = 
        # print(f"self {self.pos}")
        # print(f"xcord {xcord} ycord {ycord}")
        goalPos=(xcord, ycord)
        
        # print(f"moveforward {moveForward}")
        # END_YOUR_CODE
        
        
        return goalPos, moveForward
 
    def getAutonomousActions(self, beliefOfOtherCars: list, parkedCars: list, chkPtsSoFar: int):
        # Don't start until after your burn in iterations have expired
        if self.burnInIterations > 0:
            self.burnInIterations -= 1
            return[]
       
        goalPos, df = self.getNextGoalPos(beliefOfOtherCars, parkedCars, chkPtsSoFar)
        vectorToGoal = goalPos - self.pos
        wheelAngle = -vectorToGoal.get_angle_between(self.dir)
        driveForward = df
        actions = {
            Car.TURN_WHEEL: wheelAngle
        }
        if driveForward:
            actions[Car.DRIVE_FORWARD] = 1.0
        return actions
    
    
