# multiAgents.py
# --------------
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


from util import manhattanDistance
from game import Directions
import random, util
import math

from game import Agent

class ReflexAgent(Agent):
    """
    A reflex agent chooses an action at each choice point by examining
    its alternatives via a state evaluation function.

    The code below is provided as a guide.  You are welcome to change
    it in any way you see fit, so long as you don't touch our method
    headers.
    """


    def getAction(self, gameState):
        """
        You do not need to change this method, but you're welcome to.

        getAction chooses among the best options according to the evaluation function.

        Just like in the previous project, getAction takes a GameState and returns
        some Directions.X for some X in the set {NORTH, SOUTH, WEST, EAST, STOP}
        """
        # Collect legal moves and successor states
        legalMoves = gameState.getLegalActions()

        # Choose one of the best actions
        scores = [self.evaluationFunction(gameState, action) for action in legalMoves]
        bestScore = max(scores)
        bestIndices = [index for index in range(len(scores)) if scores[index] == bestScore]
        chosenIndex = random.choice(bestIndices) # Pick randomly among the best

        "Add more of your code here if you want to"

        return legalMoves[chosenIndex]

    def evaluationFunction(self, currentGameState, action):
        """
        Design a better evaluation function here.

        The evaluation function takes in the current and proposed successor
        GameStates (pacman.py) and returns a number, where higher numbers are better.

        The code below extracts some useful information from the state, like the
        remaining food (newFood) and Pacman position after moving (newPos).
        newScaredTimes holds the number of moves that each ghost will remain
        scared because of Pacman having eaten a power pellet.

        Print out these variables to see what you're getting, then combine them
        to create a masterful evaluation function.
        """
        # Useful information you can extract from a GameState (pacman.py)
        successorGameState = currentGameState.generatePacmanSuccessor(action)
        newPos = successorGameState.getPacmanPosition() # tuple (+x, +y) from bottom left (1,1) ~ (0,0) = bottom left board corner
        newFood = successorGameState.getFood() # board -> T = food, F = no food
        newGhostStates = successorGameState.getGhostStates()
        newGhostPositions = successorGameState.getGhostPositions()

        walls = successorGameState.getWalls()

        "*** YOUR CODE HERE ***"
        adjustedScore = successorGameState.getScore()
        
        # food
        newFoodLocs = [[x, y] for x in range(newFood.width) for y in range(newFood.height)\
            if not walls[x][y] and newFood[x][y]]

        for newFoodLoc in newFoodLocs:
            manDist = manhattanDistance(newPos, newFoodLoc)
            if manDist:
                adjustedScore += 1.0 / manDist
            else:
                adjustedScore += 2.0
        
        # ghost
        for newGhostPosition in newGhostPositions:
            manDist = manhattanDistance(newPos, newGhostPosition)
            if manDist:
                adjustedScore -= 1.0 / manhattanDistance(newPos, newGhostPosition)

        return adjustedScore

def scoreEvaluationFunction(currentGameState):
    """
    This default evaluation function just returns the score of the state.
    The score is the same one displayed in the Pacman GUI.

    This evaluation function is meant for use with adversarial search agents
    (not reflex agents).
    """
    return currentGameState.getScore()

class MultiAgentSearchAgent(Agent):
    """
    This class provides some common elements to all of your
    multi-agent searchers.  Any methods defined here will be available
    to the MinimaxPacmanAgent, AlphaBetaPacmanAgent & ExpectimaxPacmanAgent.

    You *do not* need to make any changes here, but you can if you want to
    add functionality to all your adversarial search agents.  Please do not
    remove anything, however.

    Note: this is an abstract class: one that should not be instantiated.  It's
    only partially specified, and designed to be extended.  Agent (game.py)
    is another abstract class.
    """

    def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
        self.index = 0 # Pacman is always agent index 0
        self.evaluationFunction = util.lookup(evalFn, globals())
        self.depth = int(depth)

class MinimaxAgent(MultiAgentSearchAgent):
    """
    Your minimax agent (question 2)
    """

    def DFMiniMax(self, gameState, numAgents, depth=0, agentIndex=0):
        # terminal position
        if gameState.isWin() or gameState.isLose() or self.depth == depth:
            return self.evaluationFunction(gameState), None
        
        if (agentIndex + 1) % numAgents == 0: # iterated through all agents
            depth += 1

        if agentIndex == 0: # MAX
            value = -math.inf
        else: # MIN
            value = math.inf

        bestMove = None

        for action in gameState.getLegalActions(agentIndex):
            nextState = gameState.generateSuccessor(agentIndex, action)
            nextValue, _ = self.DFMiniMax(nextState, numAgents, depth=depth, agentIndex=((agentIndex + 1) % numAgents))

            if agentIndex == 0 and value < nextValue: # MAX
                value, bestMove = nextValue, action
            if agentIndex != 0 and value > nextValue: # MIN
                value, bestMove = nextValue, action
        
        return value, bestMove

    def getAction(self, gameState):
        """
        Returns the minimax action from the current gameState using self.depth
        and self.evaluationFunction.

        Here are some method calls that might be useful when implementing minimax.

        gameState.getLegalActions(agentIndex):
        Returns a list of legal actions for an agent
        agentIndex=0 means Pacman, ghosts are >= 1

        gameState.generateSuccessor(agentIndex, action):
        Returns the successor game state after an agent takes an action

        gameState.getNumAgents():
        Returns the total number of agents in the game

        gameState.isWin():
        Returns whether or not the game state is a winning state

        gameState.isLose():
        Returns whether or not the game state is a losing state
        """
        "*** YOUR CODE HERE ***"
        return self.DFMiniMax(gameState, gameState.getNumAgents())[1]

class AlphaBetaAgent(MultiAgentSearchAgent):
    """
    Your minimax agent with alpha-beta pruning (question 3)
    """
    def AlphaBeta(self, gameState, numAgents, alpha=-math.inf, beta=math.inf, depth=0, agentIndex=0):
        # terminal position
        if gameState.isWin() or gameState.isLose() or self.depth == depth:
            return self.evaluationFunction(gameState), None
        
        if (agentIndex + 1) % numAgents == 0: # iterated through all agents
            depth += 1

        if agentIndex == 0: # MAX
            value = -math.inf
        else: # MIN
            value = math.inf

        bestMove = None

        for action in gameState.getLegalActions(agentIndex):
            nextState = gameState.generateSuccessor(agentIndex, action)
            nextValue, _ = self.AlphaBeta(nextState, numAgents, alpha, beta, depth=depth, agentIndex=((agentIndex + 1) % numAgents))

            if agentIndex == 0: # MAX 
                if value < nextValue: value, bestMove = nextValue, action
                if value >= beta: return value, bestMove
                alpha = max(alpha, value)
            if agentIndex != 0: # MIN
                if value > nextValue: value, bestMove = nextValue, action
                if value <= alpha: return value, bestMove
                beta = min(beta, value)
        
        return value, bestMove

    def getAction(self, gameState):
        """
        Returns the minimax action using self.depth and self.evaluationFunction
        """
        "*** YOUR CODE HERE ***"
        return self.AlphaBeta(gameState, gameState.getNumAgents())[1]

class ExpectimaxAgent(MultiAgentSearchAgent):
    """
      Your expectimax agent (question 4)
    """
    def Expectimax(self, gameState, numAgents, depth=0, agentIndex=0):
        # terminal position
        if gameState.isWin() or gameState.isLose() or self.depth == depth:
            return self.evaluationFunction(gameState), None
        
        if (agentIndex + 1) % numAgents == 0: # iterated through all agents
            depth += 1

        if agentIndex == 0: # MAX
            value = -math.inf
        else: # MIN
            value = 0

        bestMove = None
        actions = gameState.getLegalActions(agentIndex)

        for action in actions:
            nextState = gameState.generateSuccessor(agentIndex, action)
            nextValue, _ = self.Expectimax(nextState, numAgents, depth=depth, agentIndex=((agentIndex + 1) % numAgents))

            if agentIndex == 0 and value < nextValue: # MAX
                value, bestMove = nextValue, action
            if agentIndex != 0: # MIN
                value += nextValue * (1.0 / len(actions))
        
        return value, bestMove

    def getAction(self, gameState):
        """
        Returns the expectimax action using self.depth and self.evaluationFunction

        All ghosts should be modeled as choosing uniformly at random from their
        legal moves.
        """
        "*** YOUR CODE HERE ***"
        return self.Expectimax(gameState, gameState.getNumAgents())[1]

def betterEvaluationFunction(currentGameState):
    """
    Your extreme ghost-hunting, pellet-nabbing, food-gobbling, unstoppable
    evaluation function (question 5).

    DESCRIPTION: <write something here so we know what you did>
    """
    "*** YOUR CODE HERE ***"
    newPos = currentGameState.getPacmanPosition() # tuple (+x, +y) from bottom left (1,1) ~ (0,0) = bottom left board corner
    newFood = currentGameState.getFood() # board -> T = food, F = no food
    newGhostStates = currentGameState.getGhostStates()
    newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]
    newGhostPositions = currentGameState.getGhostPositions()
    newCapsulePositions = currentGameState.getCapsules()

    # print(newScaredTimes)

    walls = currentGameState.getWalls()

    adjustedScore = currentGameState.getScore()
    
    # food
    newFoodLocs = [[x, y] for x in range(newFood.width) for y in range(newFood.height)\
        if not walls[x][y] and newFood[x][y]]

    for newFoodLoc in newFoodLocs:
        manDist = manhattanDistance(newPos, newFoodLoc)
        if manDist:
            adjustedScore += 1.0 / manDist
        else:
            adjustedScore += 10.0

    # for newCapsulePosition in newCapsulePositions:
    #     manDist = manhattanDistance(newPos, newCapsulePosition)
    #     if manDist:
    #         adjustedScore += 0.1 / manDist
    #     else:
    #         adjustedScore += 1.0

    for newGhostPosition in newGhostPositions:
        manDist = manhattanDistance(newPos, newGhostPosition)
        if manDist:
            incr = 1.0 / manhattanDistance(newPos, newGhostPosition)
            if newScaredTimes:
                adjustedScore += incr
            else:
                adjustedScore -= incr

    return adjustedScore

# Abbreviation
better = betterEvaluationFunction
