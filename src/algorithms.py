import random
import time
import config
from typing import Tuple, List, Set
import heapq
from collections import deque

State = Tuple[int]


class Node:  # class with default comparisonFunction that can be overwritten
    def __init__(self, state: State, path: List[int], algorithm):
        self.state = state
        self.path = path
        self.algorithm = algorithm

    def __lt__(self, other):
        return self.algorithm.compareNodes(self, other)


class Algorithm:
    def __init__(self, heuristic=None):
        self.heuristic = heuristic
        self.nodes_evaluated = 0
        self.nodes_generated = 0
        self.visitedNodes: Set[State] = set()

    # returns list of indexes of tiles that can be swapped with 0
    def get_legal_actions(self, state):
        self.nodes_evaluated += 1
        max_index = len(state)
        zero_tile_ind = state.index(0)
        legal_actions = []
        if 0 <= (up_ind := (zero_tile_ind - config.N)) < max_index:
            legal_actions.append(up_ind)
        if 0 <= (right_ind := (zero_tile_ind + 1)) < max_index and right_ind % config.N:
            legal_actions.append(right_ind)
        if 0 <= (down_ind := (zero_tile_ind + config.N)) < max_index:
            legal_actions.append(down_ind)
        if 0 <= (left_ind := (zero_tile_ind - 1)) < max_index and (left_ind + 1) % config.N:
            legal_actions.append(left_ind)
        return legal_actions

    def apply_action(self, state, action) -> tuple:
        self.nodes_generated += 1
        copy_state = list(state)
        zero_tile_ind = state.index(0)
        copy_state[action], copy_state[zero_tile_ind] = copy_state[zero_tile_ind], copy_state[action]
        return tuple(copy_state)

    # returns list of actions(indexes of numbers that are swapped with 0, because game is basically
    # swapping tiles with empty(zero) tile until we get to goal state)
    def get_steps(self, initial_state, goal_state):
        pass

    # method called from main
    def get_solution_steps(self, initial_state, goal_state):
        begin_time = time.time()
        solution_actions = self.get_steps(initial_state, goal_state)
        print(f'Execution time in seconds: {(time.time() - begin_time):.2f} | '
              f'Nodes generated: {self.nodes_generated} | '
              f'Nodes evaluated: {self.nodes_evaluated}')
        return solution_actions

    def compareNodes(self, node1: Node, node2: Node):
        return 0


class ExampleAlgorithm(Algorithm):  # random choice action
    def get_steps(self, initial_state, goal_state):
        state = initial_state
        solution_actions = []
        while state != goal_state:
            legal_actions = self.get_legal_actions(state)
            action = legal_actions[random.randint(0, len(legal_actions) - 1)]
            solution_actions.append(action)
            state = self.apply_action(state, action)
        return solution_actions


class BreadthFirstSearch(Algorithm):
    def get_steps(self, initialState: State, goalState: State):
        bfsQueue = deque()
        bfsQueue.append(Node(initialState, [], self))
        while len(bfsQueue) > 0:
            node: Node = bfsQueue.popleft()
            self.visitedNodes.add(node.state)
            if node.state == goalState:
                return node.path
            res = self.addLegalActions(bfsQueue, node, goalState)
            if res is not None:
                return res

        return None  # didn't find a solution

    # adds legal actions to the priority queue
    def addLegalActions(self, bfsQueue, node: Node, goalState: State):
        legalActions: List[int] = self.get_legal_actions(node.state)
        newNodes = []  # heapq of new nodes for heuristic

        for action in legalActions:  # if goalState is reachable return path to goalState
            if self.apply_action(node.state, action) == goalState:
                return node.path + [action]

        for action in legalActions:
            newNode: Node = Node(self.apply_action(node.state, action), node.path + [action], self)
            if newNode.state in self.visitedNodes:
                continue
            heapq.heappush(newNodes, newNode)

        bfsQueue.extend(newNodes)
        return None

    def compareNodes(self, node1: Node, node2: Node):
        return self.heuristic.get_evaluation(node1.state) < self.heuristic.get_evaluation(node2.state)


class BestFirstSearch(Algorithm):
    def get_steps(self, initialState: State, goalState: State):
        self.visitedNodes = set()
        bfsQueue = []
        heapq.heappush(bfsQueue, Node(initialState, [], self))
        while bfsQueue:
            node: Node = heapq.heappop(bfsQueue)
            if node.state in self.visitedNodes:
                continue
            self.visitedNodes.add(node.state)

            if node.state == goalState:
                return node.path
            self.addLegalActions(bfsQueue, node, goalState)

        return None  # didn't find a solution

    # adds legal actions to the priority queue
    def addLegalActions(self, bfsQueue, node: Node, goalState: State):
        legalActions: List[int] = self.get_legal_actions(node.state)

        for action in legalActions:
            newNode: Node = Node(self.apply_action(node.state, action), node.path + [action], self)
            if newNode.state in self.visitedNodes:
                continue
            heapq.heappush(bfsQueue, newNode)


    def compareNodes(self, node1: Node, node2: Node):  # check heuristic and if same check node ID
        result: bool = self.heuristic.get_evaluation(node1.state) < self.heuristic.get_evaluation(node2.state)
        if not result:
            for tile1, tile2 in zip(node1.state, node2.state):
                if tile1 != tile2:
                    return tile1 < tile2
        return result


class AStar(Algorithm):
    def get_steps(self, initialState: State, goalState: State):
        self.visitedNodes = set()
        bfsQueue = []
        heapq.heappush(bfsQueue, Node(initialState, [], self))
        while len(bfsQueue) > 0:
            node: Node = heapq.heappop(bfsQueue)
            if node.state in self.visitedNodes:
                continue
            self.visitedNodes.add(node.state)

            if node.state == goalState:
                return node.path
            self.addLegalActions(bfsQueue, node, goalState)

        return None  # didn't find a solution

    # adds legal actions to the priority queue
    def addLegalActions(self, bfsQueue, node: Node, goalState: State):
        legalActions: List[int] = self.get_legal_actions(node.state)

        for action in legalActions:
            newNode: Node = Node(self.apply_action(node.state, action), node.path + [action], self)
            if newNode.state in self.visitedNodes:
                continue
            heapq.heappush(bfsQueue, newNode)


    def compareNodes(self, node1: Node, node2: Node):  # check heuristic and if same check node ID
        result: bool = len(node1.path) + self.heuristic.get_evaluation(node1.state) \
                       < len(node2.path) + self.heuristic.get_evaluation(node2.state)
        if not result:
            for tile1, tile2 in zip(node1.state, node2.state):
                if tile1 != tile2:
                    return tile1 < tile2
        return result
