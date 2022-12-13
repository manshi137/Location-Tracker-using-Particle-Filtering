import util, collections 
import statistics
import random
from util import Belief, pdf 
from engine.const import Const
from engine.model.car.car import Car

from engine.containers import counter

def weightedsample(pdict):
    weight = []
    values = []
    for e in pdict.keys():
        weight.append(pdict[e])
        values.append(e)
    total = sum(weight)
    alpha = random.uniform(0, total)
    runningsum = 0.0
    ind = None
    for i in range(len(weight)):
        runningsum += weight[i]
        if runningsum> alpha:
            ind = i
            return values[ind]

# Class: Estimator
#----------------------
# Maintain and update a belief distribution over the probability of a car being in a tile.
class Estimator(object):
    def __init__(self, numRows: int, numCols: int):
        self.belief = util.Belief(numRows, numCols) 
        self.transProb = util.loadTransProb()
            
    ##################################################################################

    def estimate(self, posX: float, posY: float, observedDist: float, isParked: bool) -> None:
        particles = {}
        startpts = []
        for(s,_) in self.transProb:
            startpts.append(s)
        # print("running estimate function")
        col = util.Belief.getNumCols(self.belief)
        row = util.Belief.getNumRows(self.belief)
        N = col*row*10
        for i in range(N):
            
            randomind = int(random.randrange(0,len(startpts)-1))
            random_startpt = startpts[randomind]
            # if a particle already exists in the gird random_startpt, then incremenet the count of particles in that grid by 1
            if startpts[randomind] in particles.keys():
                particles[random_startpt] +=1
            # if there is no particle in the gird random_startpt, then assign count of particles in that grid by 1
            else:
                particles[random_startpt] = 1

        #update belief start
        nextbelief = util.Belief(row, col, 0)
        for p in particles.keys():
            # nextbelief.setProb(p[0], p[1], particles[p])
            # store number of particles at each grid in the belief
            nextbelief.setProb(p[0], p[1], particles[p])
            # print("------")
            # print(p)
            # print(p[0])
            # print(p[1])
            # print(particles[p])
        nextbelief.normalize()
        self.belief= nextbelief
        #update belief end
#-------------------------------------------S
        # resampling start
        for k in particles.keys():
            numpart = particles[k]
            yy = util.rowToY(k[0])
            xx = util.colToX(k[1])
            dist = pow((xx- posX)*(xx- posX) + (yy - posY)*(yy - posY), 0.5)
            # get probability of particles sitting in the grid at observedDist from Autocar
            prob = util.pdf(dist, Const.SONAR_STD, observedDist)
            weight = numpart*prob
            particles[k]= weight
            # particles.count(k) = 
        newparticles = dict()

        for _ in range(N):
            # do weighted sampling of particles
            sample = weightedsample(particles)
            if sample in newparticles.keys():
                newparticles[sample]+=1
            else:
                newparticles[sample]=1
        particles = newparticles
        # resampling end
        
        #update beleif start
        nextbelief = util.Belief(row, col, 0)
        for p in particles.keys():
            nextbelief.setProb(p[0], p[1], particles[p])
        nextbelief.normalize()
        self.belief= nextbelief
        #update belief end


    def getBelief(self) -> Belief:
        return self.belief
