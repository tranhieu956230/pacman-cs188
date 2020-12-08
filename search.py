# search.py
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


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util
from util import Stack
from util import Queue
from game import Directions


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    path = []
    states = []
    visitedNodes = []
    startState = (problem.getStartState(), "", 1)
    visitedNodes.append(startState[0])
    states.append(startState)
    while len(states):
        top = states[-1]
        if path and len(path) and path[-1] == top:
            path.pop()
            states.pop()
            continue
        if top != startState:
            path.append(top)
        successors = problem.getSuccessors(top[0])
        if not len(successors):
            path.pop()
            states.pop()
        for successor in successors:
            if problem.isGoalState(top[0]):
                path.append(successor)
                return [p[1] for p in path]
            if not isVisited(successor, visitedNodes):
                states.append(successor)
                visitedNodes.append(successor)


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    states = []
    path = []
    startState = (problem.getStartState(), "", 1)
    states.append(startState)
    pathMap = {}
    while len(states):
        top = states.pop()
        path.append(top)
        successors = problem.getSuccessors(top[0])
        pathMap[top] = []
        for successor in successors:
            if problem.isGoalState(successor[0]):
                path.append(successor)
                path = tracePath(pathMap, path[0]) + path
                return [p[1] for p in path if p[1] != '']
            if not isVisited(successor, states):
                pathMap[top].append(successor)
                states.insert(0, successor)
        path.pop()


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    startState = (problem.getStartState(), "", 0)
    paths = [{"path": [startState], "cost": startState[2]}]
    visitedNodes = [startState]
    while len(paths):
        minPath = paths[-1]
        currentPosition = minPath["path"][-1][0]
        if problem.isGoalState(currentPosition):
            return [p[1] for p in minPath["path"][1:]]
        paths.pop()
        successors = problem.getSuccessors(currentPosition)
        for successor in successors:
            if not isVisited(successor, visitedNodes):
                newPath = {"path": minPath["path"] + [successor],
                           "cost": minPath["cost"] + successor[2]}
                insertionSort(paths, newPath)
                visitedNodes.append(successor)


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    startNode = (problem.getStartState(), "", 0)
    visitedNodes = [startNode]
    paths = [{"path": [startNode], "cost": startNode[2],
              "heuristic": startNode[2] + heuristic(startNode[0], problem)}]
    while len(paths):
        minPath = paths[-1]
        currentPosition = minPath["path"][-1][0]
        if problem.isGoalState(currentPosition):
            return [p[1] for p in minPath["path"][1:]]
        paths.pop()
        successors = problem.getSuccessors(currentPosition)
        for successor in successors:
            newPath = {"path": minPath["path"] + [successor],
                       "cost": minPath["cost"] + successor[2],
                       "heuristic": heuristic(successor[0], problem)}
            if not isVisited(successor, visitedNodes):
                visitedNodes.append(successor)
                insertionSort(paths, newPath)
            else:
                for i in range(len(paths)):
                    path = paths[i]
                    lastElement = path["path"][-1]
                    if lastElement[0] == successor[0] and path["cost"] > newPath["cost"]:
                        path[i] = newPath


def tracePath(pathMap, state):
    path = []
    currentState = state
    listKeys = list(pathMap.keys())
    listKeys.reverse()
    for key in listKeys:
        if currentState in pathMap[key]:
            if key != listKeys[-1]:
                path.append(key)
            currentState = key
    path.reverse()
    return path


def isVisited(state, path):
    for p in path:
        if state[0] == p[0]:
            return True
    return False


def insertionSort(paths, newPath):
    isInserted = False
    for i in range(len(paths)):
        condition = newPath["cost"] + newPath["heuristic"] >= paths[i]["cost"] + \
            paths[i]["heuristic"] if "heuristic" in newPath.keys(
        ) else newPath["cost"] >= paths[i]["cost"]
        if condition:
            paths.insert(i, newPath)
            isInserted = True
            return
    if not isInserted:
        paths.append(newPath)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
