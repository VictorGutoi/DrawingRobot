from dataclasses import dataclass, field
from typing import Callable, Optional

import pygame


COLOR_BG_PANEL = (32, 34, 40)
COLOR_TRACK = (70, 74, 84)
COLOR_HANDLE = (200, 205, 215)
COLOR_HANDLE_ACTIVE = (240, 245, 255)
COLOR_TEXT = (220, 225, 235)
COLOR_TEXT_DIM = (150, 155, 165)
COLOR_BTN = (60, 110, 180)
COLOR_BTN_HOVER = (80, 140, 220)
COLOR_BTN_ACTIVE = (50, 90, 150)


@dataclass
class Slider:
    rect: pygame.Rect
    label: str
    min_value: float
    max_value: float
    value: float
    fmt: str = "{:.2f}"
    _dragging: bool = field(default=False, init=False)

    def _value_to_x(self) -> int:
        t = (self.value - self.min_value) / (self.max_value - self.min_value)
        return int(self.rect.left + t * self.rect.width)

    def _x_to_value(self, x: int) -> float:
        t = (x - self.rect.left) / self.rect.width
        t = max(0.0, min(1.0, t))
        return self.min_value + t * (self.max_value - self.min_value)

    def handle_event(self, event: pygame.event.Event) -> bool:
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            hit = self.rect.inflate(0, 16).collidepoint(event.pos)
            if hit:
                self._dragging = True
                self.value = self._x_to_value(event.pos[0])
                return True
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            if self._dragging:
                self._dragging = False
                return True
        elif event.type == pygame.MOUSEMOTION and self._dragging:
            self.value = self._x_to_value(event.pos[0])
            return True
        return False

    def draw(self, surface: pygame.Surface, font: pygame.font.Font) -> None:
        label_text = f"{self.label}: {self.fmt.format(self.value)}"
        label = font.render(label_text, True, COLOR_TEXT)
        surface.blit(label, (self.rect.left, self.rect.top - 22))

        track = pygame.Rect(self.rect.left, self.rect.centery - 3, self.rect.width, 6)
        pygame.draw.rect(surface, COLOR_TRACK, track, border_radius=3)

        handle_x = self._value_to_x()
        handle_color = COLOR_HANDLE_ACTIVE if self._dragging else COLOR_HANDLE
        pygame.draw.circle(surface, handle_color, (handle_x, self.rect.centery), 9)


@dataclass
class Button:
    rect: pygame.Rect
    label: str
    on_click: Callable[[], None]
    _hover: bool = field(default=False, init=False)
    _pressed: bool = field(default=False, init=False)

    def handle_event(self, event: pygame.event.Event) -> bool:
        if event.type == pygame.MOUSEMOTION:
            self._hover = self.rect.collidepoint(event.pos)
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.rect.collidepoint(event.pos):
                self._pressed = True
                return True
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            was_pressed = self._pressed
            self._pressed = False
            if was_pressed and self.rect.collidepoint(event.pos):
                self.on_click()
                return True
        return False

    def draw(self, surface: pygame.Surface, font: pygame.font.Font) -> None:
        if self._pressed:
            color = COLOR_BTN_ACTIVE
        elif self._hover:
            color = COLOR_BTN_HOVER
        else:
            color = COLOR_BTN
        pygame.draw.rect(surface, color, self.rect, border_radius=6)
        text = font.render(self.label, True, COLOR_TEXT)
        surface.blit(text, text.get_rect(center=self.rect.center))


@dataclass
class TextInput:
    rect: pygame.Rect
    placeholder: str = ""
    text: str = ""
    focused: bool = True
    on_submit: Callable[[str], None] = lambda s: None
    _frame: int = field(default=0, init=False)

    def handle_event(self, event: pygame.event.Event) -> bool:
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            self.focused = self.rect.collidepoint(event.pos)
            return self.focused
        if event.type == pygame.KEYDOWN and self.focused:
            if event.key == pygame.K_RETURN:
                if self.text.strip():
                    submitted = self.text
                    self.text = ""
                    self.on_submit(submitted)
                return True
            if event.key == pygame.K_BACKSPACE:
                self.text = self.text[:-1]
                return True
            if event.key == pygame.K_ESCAPE:
                self.focused = False
                return True
            if event.unicode and event.unicode.isprintable():
                self.text += event.unicode
                return True
        return False

    def draw(self, surface: pygame.Surface, font: pygame.font.Font) -> None:
        bg = (50, 54, 64) if self.focused else (40, 44, 50)
        border = COLOR_HANDLE if self.focused else COLOR_TRACK
        pygame.draw.rect(surface, bg, self.rect, border_radius=4)
        pygame.draw.rect(surface, border, self.rect, width=1, border_radius=4)

        prompt = font.render("> ", True, COLOR_TEXT_DIM)
        prompt_x = self.rect.left + 8
        prompt_y = self.rect.centery - prompt.get_height() // 2
        surface.blit(prompt, (prompt_x, prompt_y))

        text_x = prompt_x + prompt.get_width()
        if self.text:
            ts = font.render(self.text, True, COLOR_TEXT)
        else:
            ts = font.render(self.placeholder, True, COLOR_TEXT_DIM)
        surface.blit(ts, (text_x, self.rect.centery - ts.get_height() // 2))

        if self.focused and (self._frame // 30) % 2 == 0:
            cx = text_x + (font.size(self.text)[0] if self.text else 0)
            pygame.draw.line(surface, COLOR_TEXT,
                             (cx, self.rect.top + 6), (cx, self.rect.bottom - 6), 1)
        self._frame += 1


@dataclass
class Cycler:
    """Prev/next selector that cycles through a list of string options."""
    rect: pygame.Rect
    options: list[str]
    on_change: Callable[[str], None] = lambda s: None
    index: int = 0
    _prev_hover: bool = field(default=False, init=False)
    _next_hover: bool = field(default=False, init=False)

    @property
    def value(self) -> Optional[str]:
        if not self.options:
            return None
        return self.options[self.index]

    def _prev_rect(self) -> pygame.Rect:
        return pygame.Rect(self.rect.left, self.rect.top, 32, self.rect.height)

    def _next_rect(self) -> pygame.Rect:
        return pygame.Rect(self.rect.right - 32, self.rect.top, 32, self.rect.height)

    def handle_event(self, event: pygame.event.Event) -> bool:
        if not self.options:
            return False
        if event.type == pygame.MOUSEMOTION:
            self._prev_hover = self._prev_rect().collidepoint(event.pos)
            self._next_hover = self._next_rect().collidepoint(event.pos)
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self._prev_rect().collidepoint(event.pos):
                self.index = (self.index - 1) % len(self.options)
                self.on_change(self.value)
                return True
            if self._next_rect().collidepoint(event.pos):
                self.index = (self.index + 1) % len(self.options)
                self.on_change(self.value)
                return True
        return False

    def draw(self, surface: pygame.Surface, font: pygame.font.Font) -> None:
        for r, hover, glyph in (
            (self._prev_rect(), self._prev_hover, "<"),
            (self._next_rect(), self._next_hover, ">"),
        ):
            color = COLOR_BTN_HOVER if hover else COLOR_BTN
            pygame.draw.rect(surface, color, r, border_radius=4)
            label = font.render(glyph, True, COLOR_TEXT)
            surface.blit(label, label.get_rect(center=r.center))

        name = self.value or "(none)"
        ns = font.render(name, True, COLOR_TEXT)
        center_x = (self._prev_rect().right + self._next_rect().left) // 2
        surface.blit(ns, ns.get_rect(center=(center_x, self.rect.centery)))


def draw_panel_background(surface: pygame.Surface, rect: pygame.Rect) -> None:
    pygame.draw.rect(surface, COLOR_BG_PANEL, rect)


def draw_divider(surface: pygame.Surface, x: int, y: int, width: int) -> None:
    pygame.draw.line(surface, COLOR_TRACK, (x, y), (x + width, y), 1)


def draw_text(surface: pygame.Surface, font: pygame.font.Font, text: str,
              pos: tuple[int, int], color: Optional[tuple[int, int, int]] = None) -> None:
    surface.blit(font.render(text, True, color or COLOR_TEXT), pos)
