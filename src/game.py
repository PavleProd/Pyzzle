import os
import pygame

import config
from sprites import Tile
from state import get_pos_2d


class EndGame(Exception):
    pass


class Quit(Exception):
    pass


class Game:

    def load_tiles(self):
        try:
            tiles_sprites = pygame.sprite.Group()
            tiles_dict = {}
            for ident in range(config.N ** 2):
                goal_place_1d = ident - 1 if ident else config.N ** 2 - 1
                current_place_1d = self.initial_state.index(ident)
                tiles_dict[ident] = Tile(self.image, ident, get_pos_2d(current_place_1d), get_pos_2d(goal_place_1d))
                tiles_dict[ident].add(tiles_sprites)
            return tiles_dict, tiles_sprites
        except Exception as e:
            raise e

    def __init__(self, initial_state, goal_state, solution_steps, image_name):
        self.initial_state = initial_state
        self.goal_state = goal_state
        self.solution_steps = solution_steps
        pygame.display.set_caption('Pyzzle')
        pygame.font.init()
        config.GAME_FONT = pygame.font.Font(os.path.join(config.FONT_FOLDER, 'game_font.ttf'), 40)
        config.INFO_FONT = pygame.font.Font(os.path.join(config.FONT_FOLDER, 'info_font.ttf'), 22)
        self.DIM = config.N * config.TILE_SIZE
        self.screen = pygame.display.set_mode((self.DIM, self.DIM + config.INFO_HEIGHT))
        self.image = pygame.image.load(os.path.join(config.IMG_FOLDER, image_name)).convert()
        self.image = pygame.transform.scale(self.image, (self.DIM, self.DIM))
        self.tiles_dict, self.tiles_sprites = self.load_tiles()
        self.clock = pygame.time.Clock()
        self.running = True
        self.playing = False
        self.game_over = False
        self.step_width = len(str(len(self.solution_steps)))
        self.tile_ident_width = len(str(config.N ** 2 - 1))
        self.state = None
        self.step_cnt = None
        self.travelling = None
        self.tile_place = None
        self.tile_place_2d = None
        self.new_tile_place = None
        self.new_tile_place_2d = None
        self.move_to = None
        self.success = None

    def check_goal_reached(self):
        self.success = True
        if self.state != self.goal_state:
            self.success = False
            print(f'[ERROR] Goal state not reached!')
        else:
            print(f'Goal state reached.')

    def apply_step(self):
        if not self.travelling:
            self.tile_place = self.solution_steps[self.step_cnt]
            self.tile_place_2d = get_pos_2d(self.tile_place)
            self.new_tile_place = self.state.index(0)
            self.new_tile_place_2d = get_pos_2d(self.new_tile_place)
            diff = (self.new_tile_place_2d[0] - self.tile_place_2d[0],
                    self.new_tile_place_2d[1] - self.tile_place_2d[1])
            if diff not in [(-1, 0), (0, -1), (0, 1), (1, 0)]:
                print(f'[ERROR] Illegal tile {self.state[self.tile_place]:{self.tile_ident_width}} movement '
                      f'from {self.tile_place_2d} to {self.new_tile_place_2d}')
                raise EndGame()
            print(f'[STEP {(self.step_cnt + 1):0{self.step_width}}] | '
                  f'Moving tile {self.state[self.tile_place]:{self.tile_ident_width}} '
                  f'from {self.tile_place_2d} to {self.new_tile_place_2d}')
            self.move_to = (self.new_tile_place_2d[1] * config.TILE_SIZE,
                            self.new_tile_place_2d[0] * config.TILE_SIZE)
            self.travelling = True
        else:
            if self.move_to == self.tiles_dict[self.state[self.tile_place]].rect.topleft:
                self.tiles_dict[self.state[self.tile_place]].set_position(self.new_tile_place_2d)
                self.tiles_dict[self.state[self.new_tile_place]].set_position(self.tile_place_2d)
                copy_state = list(self.state)
                copy_state[self.tile_place], copy_state[self.new_tile_place] = \
                    (copy_state[self.new_tile_place], copy_state[self.tile_place])
                self.state = tuple(copy_state)
                self.step_cnt += 1
                self.travelling = False
            self.tiles_dict[self.state[self.tile_place]].translate(*self.move_to)

    def run(self):
        self.state = self.initial_state
        self.step_cnt = 0
        self.travelling = None
        while self.running:
            try:
                if self.playing:
                    try:
                        self.apply_step()
                    except IndexError:
                        raise EndGame()
                self.events()
                self.draw()
                self.clock.tick(config.FRAMES_PER_SEC)
            except EndGame:
                if self.step_cnt < len(self.solution_steps) and self.game_over:
                    self.state = self.initial_state
                    self.step_cnt = 0
                    self.travelling = False
                    try:
                        print('*' * 55)
                        while True:
                            self.apply_step()
                            self.events()
                    except (IndexError, EndGame, Quit):
                        pass
                self.check_goal_reached()
                self.game_over = True
                self.playing = False
            except Quit:
                self.game_over = True
                self.playing = False
            except Exception as e:
                raise e

    def draw_info_text(self):
        if self.game_over:
            if self.success:
                text_str = 'END'
            else:
                text_str = 'ERROR'
        elif not self.playing:
            text_str = 'PAUSED'
        else:
            text_str = ''
        text_width, text_height = config.INFO_FONT.size(text_str)
        text = config.INFO_FONT.render(f'{text_str}', True, config.GREEN if text_str == 'END' else config.RED)
        self.screen.blit(text, (self.DIM - text_width - 5, self.DIM))

    def draw(self):
        self.tiles_sprites.draw(self.screen)
        if not self.game_over or not self.success:
            for tile_sprite in self.tiles_sprites:
                tile_sprite.draw_text(self.screen)
        text = config.INFO_FONT.render(f'Steps: {self.step_cnt}', True, config.GREEN)
        self.screen.fill(config.BLACK, rect=(0, self.DIM, self.DIM, config.INFO_HEIGHT))
        self.screen.blit(text, (5, self.DIM))
        self.draw_info_text()
        pygame.display.flip()

    def events(self):
        # catch all events here
        for event in pygame.event.get():
            if event.type == pygame.QUIT or event.type == pygame.WINDOWCLOSE or \
                    event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                self.running = False
                raise Quit()
            if self.game_over:
                return
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                self.playing = not self.playing
            if event.type == pygame.KEYDOWN and event.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                self.game_over = True
                raise EndGame()
