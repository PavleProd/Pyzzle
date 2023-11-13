import numpy as np
import config


def get_pos_2d(index_1d):
    return index_1d // config.N, index_1d % config.N


def get_inversion_count(state):
    inversion_count = 0
    last_tile_val = config.N ** 2
    for i in range(last_tile_val - 1):
        for j in range(i + 1, last_tile_val):
            if state[i] and state[j] and state[i] > state[j]:
                inversion_count += 1
    return inversion_count


def is_solvable(state):
    inversion_count = get_inversion_count(state)
    if config.N & 1:
        return not (inversion_count & 1)
    else:
        return (inversion_count + (state.index(0) // config.N)) & 1


def get_init_and_goal_states(seed=123):
    np.random.seed(seed)
    initial_state = [ident for ident in range(config.N ** 2)]
    goal_state = tuple(initial_state[1:] + [initial_state[0]])
    while True:
        np.random.shuffle(initial_state)
        if is_solvable(initial_state) and tuple(initial_state) != goal_state:
            break
    initial_state = tuple(initial_state)
    return initial_state, goal_state
