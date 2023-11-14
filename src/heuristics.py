from typing import Tuple
from config import N

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
    def get_evaluation(self, state):
        heuristic = 0
        for i in range(len(state)):
            if state[i] == 0:
                continue
            rowI = i // N
            colI = i % N

            target = (state[i] - 1) % (N ** 2)
            rowTarget = target // N
            colTarget = target % N

            heuristic += abs(rowI - rowTarget) + abs(colI - colTarget)

        return heuristic
