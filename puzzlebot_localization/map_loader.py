#!/usr/bin/env python3
"""Carga un mapa occupancy grid (PGM+YAML) y proporciona utilidades de conversión."""

import math
import yaml
import numpy as np
from PIL import Image


class MapLoader:
    def __init__(self, yaml_path):
        with open(yaml_path, 'r') as f:
            self.meta = yaml.safe_load(f)

        image_path = self.meta['image']
        if not image_path.startswith('/'):
            import os
            yaml_dir = os.path.dirname(yaml_path)
            image_path = os.path.join(yaml_dir, image_path)

        img = Image.open(image_path).convert('L')
        self.map_data = np.array(img, dtype=np.uint8)
        self.height, self.width = self.map_data.shape
        self.resolution = float(self.meta['resolution'])
        origin = self.meta['origin']
        self.origin_x = float(origin[0])
        self.origin_y = float(origin[1])
        self.origin_yaw = float(origin[2])

        # Umbral ocupado/libre (en escala 0-1 para occupancy, aquí usamos 0-255)
        occupied_thresh = float(self.meta.get('occupied_thresh', 0.65))
        free_thresh = float(self.meta.get('free_thresh', 0.196))
        negate = int(self.meta.get('negate', 0))

        # Convertir a matriz de ocupación: -1 = unknown, 0 = free, 100 = occupied
        self.occupancy = np.full((self.height, self.width), -1, dtype=np.int8)
        for i in range(self.height):
            for j in range(self.width):
                val = self.map_data[i, j]
                if negate:
                    val = 255 - val
                p = val / 255.0  # convención de este PGM: 0=libre(p bajo), 254=ocupado(p alto)
                if p > occupied_thresh:
                    self.occupancy[i, j] = 100
                elif p < free_thresh:
                    self.occupancy[i, j] = 0
                else:
                    self.occupancy[i, j] = -1

    def world_to_map(self, x, y):
        """Convierte coordenadas mundo -> índices de celda (fila, col).
        Row 0 corresponde a y = origin_y (parte inferior del mundo).
        """
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        return row, col

    def map_to_world(self, row, col):
        """Convierte índices de celda -> coordenadas mundo (centro de la celda)."""
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return x, y

    def is_free(self, x, y):
        row, col = self.world_to_map(x, y)
        if row < 0 or row >= self.height or col < 0 or col >= self.width:
            return False
        return self.occupancy[row, col] == 0

    def is_occupied(self, x, y):
        row, col = self.world_to_map(x, y)
        if row < 0 or row >= self.height or col < 0 or col >= self.width:
            return True
        return self.occupancy[row, col] == 100

    @staticmethod
    def bresenham_line(x0, y0, x1, y1):
        """Genera puntos (col, row) en una línea de Bresenham."""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return points
