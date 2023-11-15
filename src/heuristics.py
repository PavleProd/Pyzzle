from typing import Tuple
from config import N
import config
import time

State = Tuple[int]


class Heuristic:
    def get_evaluation(self, state):
        pass


class ExampleHeuristic(Heuristic):
    def get_evaluation(self, state):
        return 0


class Hamming(Heuristic):  # Heuristic is number of tiles not in place
    def get_evaluation(self, state: State):
        heuristic = 0
        for i in range(len(state)):
            if state[i] == 0:
                continue
            if state[i] != (i + 1) % (N ** 2):
                heuristic += 1
        return heuristic


# heuristic is sum of manhattan distances, or minimum number of moves to get tile to the target tile
class Manhattan(Heuristic):
    def __init__(self):
        self.distances = [[0 for _ in range(N**2)] for _ in range(N**2)]

        for i in range(N**2):
            for j in range(N**2):
                rowI, colI = divmod(i, N)
                rowJ, colJ = divmod(j, N)
                self.distances[i][j] = abs(rowI - rowJ) + abs(colI - colJ)

    def get_evaluation(self, state):
        heuristic = 0
        for i, val in enumerate(state):
            if val == 0:
                continue
            target = (val - 1) % (N ** 2)
            heuristic += self.distances[i][target]
        return heuristic


