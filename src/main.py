import os.path

from screeninfo import get_monitors
import sys
import traceback
import pygame

import config
from game import Game
from state import get_init_and_goal_states

try:
    for m in get_monitors():
        if m.is_primary:
            config.SCREEN_WIDTH, config.SCREEN_HEIGHT = m.width, m.height
            break
    config.SCREEN_WIDTH = int(config.SCREEN_WIDTH * 0.9)
    config.SCREEN_HEIGHT = int(config.SCREEN_HEIGHT * 0.9)

    config.N = max(
        min(
            int(sys.argv[1]),
            int(min(config.SCREEN_WIDTH, config.SCREEN_HEIGHT) / config.TILE_SIZE)),
        2
    ) if len(sys.argv) > 1 else config.N
    if len(sys.argv) > 1 and config.N != int(sys.argv[1]):
        print(f'N changed from {sys.argv[1]} to {config.N} due to minimum tile size.')
    image_name = sys.argv[2] if len(sys.argv) > 2 else 'example.png'
    if not os.path.exists(os.path.join(config.IMG_FOLDER, image_name)):
        raise Exception('Image file not found.')
    module_algorithm = __import__('algorithms')
    class_algorithm = getattr(module_algorithm, sys.argv[3] if len(sys.argv) > 3 else 'ExampleAlgorithm')
    module_heuristic = __import__('heuristics')
    class_heuristic = getattr(module_heuristic, sys.argv[4] if len(sys.argv) > 4 else 'ExampleHeuristic')
    algorithm = class_algorithm(class_heuristic())

    initial_state, goal_state = get_init_and_goal_states()
    print(f'ALGORITHM: {class_algorithm.__name__} | Waiting for solution\'s actions ...')
    solution_steps = algorithm.get_solution_steps(initial_state, goal_state)
    if solution_steps:
        print(f'Solution length comprises {len(solution_steps)} actions.')
        pygame.init()
        g = Game(initial_state, goal_state, solution_steps, image_name)
        g.run()
    else:
        print(f'Solution not found.')
except (Exception,):
    traceback.print_exc()
    input()
finally:
    pygame.display.quit()
    pygame.quit()
