# myTeam.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


from captureAgents import CaptureAgent
import random, time, util
import game
from game import Grid
import distanceCalculator
import random, time, util, sys
from game import Directions,Actions
from util import nearestPoint


#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first='DefenseAgent', second='OffenseAgent'):


    # The following line is an example only; feel free to change it.
    return [eval(first)(firstIndex), eval(second)(secondIndex)]


##########
# Agents #
##########

class BaseAgent(CaptureAgent):


    def registerInitialState(self, gameState):


        self.start = gameState.getAgentPosition(self.index)
        CaptureAgent.registerInitialState(self, gameState)


    def getSuccessor(self, gameState, action):
        """
    Finds the next successor which is a grid position (location tuple).
    """
        successor = gameState.generateSuccessor(self.index, action)
        pos = successor.getAgentState(self.index).getPosition()
        if pos != nearestPoint(pos):
            # Only half a grid position was covered
            return successor.generateSuccessor(self.index, action)
        else:
            return successor

    def __init__(self, index):
        CaptureAgent.__init__(self, index)
        # Variables used to verify if the agent os locked
        self.foodNum = 999
        self.trappedTime = 0
        self.defendingFood=[]
        self.flag=0
        self.lastPoint=()

    def evaluate(self, gameState, action):
        """
    Computes a linear combination of features and feature weights
    """
        features = self.getFeatures(gameState, action)
        weights = self.getWeights(gameState, action)
        return features * weights

    def getClosestGhosts(self, gameState):
        ghosts = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
        myPos = gameState.getAgentState(self.index).getPosition()
        distToGhost = 200
        for ghost in ghosts:
            if ghost.getPosition():
                dist = self.getMazeDistance(myPos, ghost.getPosition())
                if distToGhost > dist:
                    distToGhost = dist
        return distToGhost

    def chooseAction(self, gameState):
        """
    Picks among the actions with the highest Q(s,a).
    """
        actions = gameState.getLegalActions(self.index)
        actions.remove(Directions.STOP)
        # You can profile your evaluation time by uncommenting these lines
        # start = time.time()

        values = [self.evaluate(gameState, a) for a in actions]
        # print 'eval time for agent %d: %.4f' % (self.index, time.time() - start)

        maxValue = max(values)
        bestActions = [a for a, v in zip(actions, values) if v == maxValue]

        foodLeft = len(self.getFood(gameState).asList())

        if foodLeft <= 2:
            bestDist = 999
            values = []
            for action in actions:
                successor = self.getSuccessor(gameState, action)
                pos2 = successor.getAgentPosition(self.index)
                dist = self.aStarSearch(gameState, pos2)
                if dist < bestDist:
                    bestAction = action
                    bestDist = dist
                newState = gameState.generateSuccessor(self.index, a)
                value = 0
                for i in range(1, 30):
                    value += self.randomEvaluate(1, newState)
                values.append(value)
                bestValue = max(values)
                bestActions = []
                for (value, action) in zip(values, actions):
                    if value == bestValue:
                        bestActions.append(action)
            if bestAction in bestActions:
                return bestAction               

        return random.choice(bestActions)
        # return Directions.STOP

    def aStarSearch(self, gameState, po2):
        true_cost = 0
        po1 = gameState.getAgentPosition(self.index)
        heu_cost = 0 + self.getMazeDistance(po1, po2)
        closed_states = []
        result = util.PriorityQueue()
        actions = []
        initial_state = (po1, actions, true_cost, heu_cost)
        result.push(initial_state, heu_cost)
        while not (result.isEmpty()):
            (state, actions, true_cost, hue_cost) = result.pop()
            if state == po2:
                return len(actions)
            if not state in closed_states:
                closed_states.append(state)
                legalActions = gameState.getLegalActions(self.index)
                for action in legalActions:
                    next_action = actions + [action]
                    next_true_cost = true_cost + 1
                    successor = self.getSuccessor(gameState, action)
                    if successor.getAgentState(self.index).getPosition()!=None:
                      myPos = successor.getAgentState(self.index).getPosition()
                      next_hu_cost = next_true_cost + self.getMazeDistance(myPos, po2)
                      next_state = (myPos, next_action, next_true_cost, next_hu_cost)
                      result.push(next_state, next_hu_cost)
                    else:
                      return self.getMazeDistance(po1,po2)

    def randomEvaluate(self, step, gameState):
        newState = gameState.deepCopy()
        while step > 0:
            actions = newState.getLegalActions(self.index)
            actions.remove(Directions.STOP)
            a = random.choice(actions)
            newState = newState.generateSuccessor(self.index, a)
            step -= 1
        return self.evaluate(newState, Directions.STOP)


class DefenseAgent(BaseAgent):

    def getNearToCenterPoint(self, gameState):

        walls=gameState.getWalls().asList()
        centerX = gameState.getWalls().width / 2
        centerY = gameState.getWalls().height / 2
        if self.red:
            centerX = centerX - 1
        else:
            centerX = centerX + 1
            centerPoint=(centerX,centerY)
        if not centerPoint in walls:
            return centerPoint
        else:
            for i in range(1,gameState.getWalls().height-centerY):
                if not (centerX,centerY-1) in walls:
                    return (centerX,centerY-i)
                else:
                    if not (centerX,centerY+i) in walls:
                        return (centerX,centerY+i)


    def getFeatures(self, gameState, action):

        features = util.Counter()
        successor = self.getSuccessor(gameState, action)
        myState = successor.getAgentState(self.index)
        myPos = myState.getPosition()
        myFood=self.getFoodYouAreDefending(gameState).asList()

        centerPoint=self.getNearToCenterPoint(gameState)
        if self.flag==0:
            self.flag=1
            self.defendingFood=myFood
            self.lastPoint=centerPoint
        if len(self.defendingFood)>len(myFood):
            lastPoint=list(set(self.defendingFood)-set(myFood))[0]
            self.defendingFood=myFood
            self.lastPoint=lastPoint


        if myState.isPacman:
           features['isDefense'] = 0
        else:
           features['isDefense'] = 1
        opponents = [successor.getAgentState(i) for i in self.getOpponents(successor)]
        enemies = [i for i in opponents if i.isPacman and i.getPosition() != None]
        features['numEnemies'] = len(enemies)

        if len(enemies) > 0:
            dist = [self.getMazeDistance(myPos, a.getPosition()) for a in enemies]
            if gameState.getAgentState(self.index).scaredTimer > 0:
                features['distToEnemies'] = -min(dist)
            else:
                features['distToEnemies'] = min(dist)
            self.lastPoint=centerPoint

        if len(enemies) == 0:
            features['goCenter'] = self.getMazeDistance(myPos,self.lastPoint)
        else:
            features['goCenter'] = 0
        reverse = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
        if action == reverse:
            features['reverse'] = 1

        return features

    def getWeights(self, gameState, action):
        return { 'isDefense': 50, 'distToEnemies': -10, 'reverse': -10,
              'goCenter': -20,'numEnemies':-1000}


class OffenseAgent(BaseAgent):
    """
  A reflex agent that seeks food. This is an agent
  we give you to get an idea of what an offensive agent might look like,
  but it is by no means the best or only way to build an offensive agent.
  """

    def getFeatures(self, gameState, action):
        features = util.Counter()
        successor = self.getSuccessor(gameState, action)
        myState = successor.getAgentState(self.index)
        foodList = self.getFood(successor).asList()
        ghosts = [successor.getAgentState(i) for i in self.getOpponents(successor)]
        powerDots = self.getCapsules(gameState)
        features['successorScore'] = -len(foodList)  # self.getScore(successor)

        # Compute distance to the nearest food

        if len(foodList) > 0:  # This should always be True,  but better safe than sorry
            myPos = successor.getAgentState(self.index).getPosition()
            minDistance = min([self.getMazeDistance(myPos, food) for food in foodList])
            features['distanceToFood'] = minDistance

        # compute the distance to nearest ghost
        distToGhost = 50
        for ghost in ghosts:
            if ghost.getPosition():
                dist = self.getMazeDistance(myPos, ghost.getPosition())
                if distToGhost > dist:
                    distToGhost = dist
                    nearestGhost = ghost
        if myState.isPacman:
            features['DistToNearestGhost'] = distToGhost

        if len(powerDots) > 0:
            distToPower = min([self.getMazeDistance(myPos, dot) for dot in powerDots])
            
            features['distanceToPower'] = distToPower
            features['powerScore'] = -len(powerDots)

        if self.trappedTime > 10:
            features['goHome'] = 0
            if successor.getAgentState(self.index).isPacman:
                features['attack'] = 1
            else:
                features['attack'] = 0
        else:

            if distToGhost < 3 and nearestGhost.scaredTimer < 2 and not nearestGhost.isPacman:
                features['distanceToFood'] = 0
                features['successorScore'] = -len(powerDots) 
                features['DistToNearestGhost'] = distToGhost               
                features['goHome'] = self.getMazeDistance(myPos, self.start)

        rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
        if action == rev:
            features['reverse'] = 1
            
            

        return features

    def isBadMove(self, gameState, action, step):
        distToGhost = self.getClosestGhosts(gameState)
        successor = gameState.generateSuccessor(self.index, action)
        currentNumFood = len(self.getFood(gameState).asList())
        newNumFood = len(self.getFood(successor).asList())
        if step == 0:
            return False
        if currentNumFood > newNumFood and distToGhost >= 5:
            return False
        actions = successor.getLegalActions(self.index)
        actions.remove(Directions.STOP)
        return_direction = Directions.REVERSE[successor.getAgentState(self.index).configuration.direction]
        if return_direction in actions:
            actions.remove(return_direction)
        if len(actions) == 0:
            return True
        for action in actions:
            if not self.isBadMove(successor, action, step - 1):
                return False
        return True

    def getWeights(self, gameState, action):

        return {'successorScore': 500, 'distanceToFood': -20, 'DistToNearestGhost': 20,
                'goHome': -40, 'attack': 500,'reverse':-10}

    def chooseAction(self, gameState):
        """
    Picks among the actions with the highest Q(s,a).
    """
        currentNumFood = len(self.getFood(gameState).asList())
        if self.foodNum != currentNumFood:
            self.foodNum = currentNumFood
            self.trappedTime = 0
        else:
            self.trappedTime += 1
        if gameState.getInitialAgentPosition(self.index) == gameState.getAgentState(self.index).getPosition():
            self.trappedTime = 0

        actions = gameState.getLegalActions(self.index)
        actions.remove(Directions.STOP)
        goodAction = []
        for a in actions:
            if not self.isBadMove(gameState, a, 5):
                goodAction.append(a)
        if len(goodAction) == 0:
            goodAction = actions

        values = [self.evaluate(gameState, a) for a in goodAction]

        maxValue = max(values)
        bestActions = [a for a, v in zip(goodAction, values) if v == maxValue]

        foodLeft = len(self.getFood(gameState).asList())

        if foodLeft <= 2:
            bestDist = 9999
            for action in goodAction:
                successor = self.getSuccessor(gameState, action)
                pos2 = successor.getAgentPosition(self.index)
                dist = self.getMazeDistance(self.start, pos2)
                if dist < bestDist:
                    bestAction = action
                    bestDist = dist
            return bestAction


        return random.choice(bestActions)




