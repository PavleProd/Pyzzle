import os

# parameters
SCREEN_WIDTH = 1366
SCREEN_HEIGHT = 768
TILE_SIZE = 128
N = 3
INFO_HEIGHT = 30
FRAMES_PER_SEC_TILE_DRAW = 2 * TILE_SIZE
FRAMES_PER_SEC = 120
GAME_FONT = None
INFO_FONT = None

# define colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (192, 0, 0)
GREEN = (0, 255, 0)
DARK_GREEN = (0, 128, 0)
YELLOW = (255, 255, 0)

# paths
GAME_FOLDER = os.path.dirname(__file__)
IMG_FOLDER = os.path.join(GAME_FOLDER, 'img')
FONT_FOLDER = os.path.join(GAME_FOLDER, 'fonts')
