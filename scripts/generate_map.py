#!/usr/bin/env python3
"""Genera un mapa occupancy grid (PGM+YAML) a partir de obstacles.world."""

import math
import os
from PIL import Image, ImageDraw

# Configuración del mapa
RESOLUTION = 0.05  # m / pixel
WIDTH_M = 12.0     # m
HEIGHT_M = 12.0    # m
ORIGIN_X = -6.0    # m
ORIGIN_Y = -6.0    # m

WIDTH_PX = int(WIDTH_M / RESOLUTION)
HEIGHT_PX = int(HEIGHT_M / RESOLUTION)

# Colores PGM estándar ROS: 0 = libre (blanco), 254 = ocupado (negro)
FREE = 0
OCCUPIED = 254


def world_to_px(x, y):
    """Convierte coordenadas mundo -> píxeles (imagen).
    Row-major para ROS: y=origin_y -> py=0 (parte superior de la imagen PIL),
    de modo que al aplanar la imagen la fila 0 corresponda a y=origin_y en el mundo.
    """
    px = int((x - ORIGIN_X) / RESOLUTION)
    py = int((y - ORIGIN_Y) / RESOLUTION)
    return px, py


def draw_box(draw, cx, cy, w, h, angle):
    """Dibuja un rectángulo rotado en el mapa."""
    corners = [
        (-w / 2, -h / 2),
        (w / 2, -h / 2),
        (w / 2, h / 2),
        (-w / 2, h / 2),
    ]
    rot_corners = []
    for dx, dy in corners:
        rx = cx + (dx * math.cos(angle) - dy * math.sin(angle))
        ry = cy + (dx * math.sin(angle) + dy * math.cos(angle))
        rot_corners.append(world_to_px(rx, ry))
    draw.polygon(rot_corners, fill=OCCUPIED)


def draw_circle(draw, cx, cy, radius):
    """Dibuja un círculo en el mapa."""
    px, py = world_to_px(cx, cy)
    pr = int(radius / RESOLUTION)
    draw.ellipse([px - pr, py - pr, px + pr, py + pr], fill=OCCUPIED)


def main():
    img = Image.new('L', (WIDTH_PX, HEIGHT_PX), FREE)
    draw = ImageDraw.Draw(img)

    # 1. Paredes del cuarto 10x10
    # Norte: centro (0, 5), size (10, 0.2)
    draw_box(draw, 0.0, 5.0, 10.0, 0.2, 0.0)
    # Sur: centro (0, -5), size (10, 0.2)
    draw_box(draw, 0.0, -5.0, 10.0, 0.2, 0.0)
    # Este: centro (5, 0), size (10, 0.2), rot 90°
    draw_box(draw, 5.0, 0.0, 10.0, 0.2, math.pi / 2)
    # Oeste: centro (-5, 0), size (10, 0.2), rot 90°
    draw_box(draw, -5.0, 0.0, 10.0, 0.2, math.pi / 2)

    # 2. Caja 1: (2, 2), 0.5x0.5
    draw_box(draw, 2.0, 2.0, 0.5, 0.5, 0.0)

    # 3. Caja 2: (-2, -1), 0.6x0.6, rot 45°
    draw_box(draw, -2.0, -1.0, 0.6, 0.6, 0.7854)

    # 4. Cilindro: (-1, 3), radio 0.3
    draw_circle(draw, -1.0, 3.0, 0.3)

    # 5. Pared interna: (1, -2), 3x0.15, rot 0.3
    draw_box(draw, 1.0, -2.0, 3.0, 0.15, 0.3)

    # Guardar PGM
    script_dir = os.path.dirname(os.path.abspath(__file__))
    maps_dir = os.path.join(script_dir, '..', 'maps')
    os.makedirs(maps_dir, exist_ok=True)
    pgm_path = os.path.join(maps_dir, 'obstacles.pgm')
    yaml_path = os.path.join(maps_dir, 'obstacles.yaml')

    img.save(pgm_path)

    yaml_content = f"""image: obstacles.pgm
resolution: {RESOLUTION}
origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
"""
    with open(yaml_path, 'w') as f:
        f.write(yaml_content)

    print(f"Mapa guardado en:\n  {pgm_path}\n  {yaml_path}")


if __name__ == '__main__':
    main()
