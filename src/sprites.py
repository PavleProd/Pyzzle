import pygame
import config


class Tile(pygame.sprite.Sprite):
    offset = 1

    def __init__(self, image, ident, position, goal_position):
        super().__init__()
        self.ident = ident
        self.position = position
        self.goal_position = goal_position
        self.image = pygame.Surface((config.TILE_SIZE, config.TILE_SIZE))
        self.rect = self.image.get_rect()
        self.rect.topleft = (position[1] * config.TILE_SIZE, position[0] * config.TILE_SIZE)
        if self.ident:
            self.image.blit(image, (Tile.offset, Tile.offset),
                            (goal_position[1] * config.TILE_SIZE + Tile.offset,
                             goal_position[0] * config.TILE_SIZE + Tile.offset,
                             config.TILE_SIZE - 2 * Tile.offset,
                             config.TILE_SIZE - 2 * Tile.offset))

    def translate(self, x, y):
        if self.rect.x < x:
            self.rect.x += 1
        if self.rect.x > x:
            self.rect.x -= 1
        if self.rect.y < y:
            self.rect.y += 1
        if self.rect.y > y:
            self.rect.y -= 1

    def set_position(self, position):
        self.position = position
        self.rect.topleft = (position[1] * config.TILE_SIZE, position[0] * config.TILE_SIZE)

    def draw_text(self, screen):
        if not self.ident:
            return
        text = config.GAME_FONT.render(f'{self.ident}', True, config.RED)
        text_rect = text.get_rect(center=self.rect.center)
        screen.blit(text, text_rect)
