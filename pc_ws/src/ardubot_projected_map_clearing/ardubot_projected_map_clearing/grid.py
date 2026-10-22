"""Map/OctoMap fusion for ArduBot, with free-space history but CURRENT obstacles.

This module deliberately has no ROS dependency so its behavior can be tested
independently. IMPORTANT: the FOV clearing is a visualization-only estimate,
not a measured safety-certified free-space map.
"""

import math


class PersistentGrid:
    """An extensible free-space history overlaid by a non-persistent obstacle layer.

    - self.data holds only UNKNOWN (-1) and FREE (0). It NEVER stores obstacles.
    - self.current_obstacles is replaced with EVERY valid incoming source snapshot.
    - the output is composed as persistent free history + current source obstacles.

    Obstacles are stored with world-aligned cell coordinates, so expansion never
    shifts or accumulates them. All layers share the same grid resolution.
    """

    UNKNOWN = -1
    FREE = 0
    OCCUPIED = 100

    def __init__(self, resolution=0.05, initial_size_m=16.0, expand_step_m=2.0):
        if resolution <= 0 or initial_size_m <= 0 or expand_step_m <= 0:
            raise ValueError('Grid dimensions must be positive')
        self.resolution = float(resolution)
        self.initial_cells = math.ceil(initial_size_m / self.resolution)
        self.expand_cells = math.ceil(expand_step_m / self.resolution)
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.width = 0
        self.height = 0
        self.data = []  # Persistent free/unknown cells ONLY.
        self.current_obstacles = set()  # World grid cells from latest /projected_map.

    @property
    def initialized(self):
        return self.width > 0 and self.height > 0

    def initialize(self, origin_x, origin_y, width_cells, height_cells, src_resolution):
        """Start centered on the source bounds, with at least a 16 m square."""
        if self.initialized:
            return
        src_width = width_cells * src_resolution
        src_height = height_cells * src_resolution
        mid_x = origin_x + src_width * 0.5
        mid_y = origin_y + src_height * 0.5
        target_width = max(self.initial_cells, math.ceil(src_width / self.resolution))
        target_height = max(self.initial_cells, math.ceil(src_height / self.resolution))
        self.width = math.ceil(target_width / self.expand_cells) * self.expand_cells
        self.height = math.ceil(target_height / self.expand_cells) * self.expand_cells
        self.origin_x = (math.floor((mid_x - 0.5 * self.width * self.resolution) /
                                    self.resolution) * self.resolution)
        self.origin_y = (math.floor((mid_y - 0.5 * self.height * self.resolution) /
                                    self.resolution) * self.resolution)
        self.data = [self.UNKNOWN] * (self.width * self.height)

    def ensure_bounds(self, min_x, min_y, max_x, max_y):
        """Extend by blocks of 2 m. Preserve every persistent free cell."""
        if not self.initialized:
            return False
        block_m = self.expand_cells * self.resolution
        end_x = self.origin_x + self.width * self.resolution
        end_y = self.origin_y + self.height * self.resolution
        left = max(0, math.ceil((self.origin_x - min_x) / block_m)) * self.expand_cells
        right = max(0, math.ceil((max_x - end_x) / block_m)) * self.expand_cells
        bottom = max(0, math.ceil((self.origin_y - min_y) / block_m)) * self.expand_cells
        top = max(0, math.ceil((max_y - end_y) / block_m)) * self.expand_cells
        if not (left or right or bottom or top):
            return False
        new_w = self.width + left + right
        new_h = self.height + bottom + top
        updated = [self.UNKNOWN] * (new_w * new_h)
        for y in range(self.height):
            old = y * self.width
            new = (y + bottom) * new_w + left
            updated[new:new + self.width] = self.data[old:old + self.width]
        self.origin_x -= left * self.resolution
        self.origin_y -= bottom * self.resolution
        self.width = new_w
        self.height = new_h
        self.data = updated
        return True

    def index(self, wx, wy):
        if not self.initialized:
            return None
        col = math.floor((wx - self.origin_x) / self.resolution)
        row = math.floor((wy - self.origin_y) / self.resolution)
        if 0 <= col < self.width and 0 <= row < self.height:
            return row * self.width + col
        return None

    def world_cell(self, wx, wy):
        return (math.floor(wx / self.resolution),
                math.floor(wy / self.resolution))

    def key_to_index(self, key):
        # Take the CENTER of each world cell to avoid floating-point edge effects.
        x = (key[0] + 0.5) * self.resolution
        y = (key[1] + 0.5) * self.resolution
        return self.index(x, y)

    def merge_source(self, origin_x, origin_y, width_cells, height_cells, src_resolution,
                     values, obstacle_threshold=99, free_max_value=0):
        """Atomically replace current obstacles with ONE new valid source snapshot.

        Source UNKNOWN cells do not erase historical FREE cells. In contrast,
        obstacles are NOT memorized: absent in the latest map => absent in output.
        """
        if width_cells <= 0 or height_cells <= 0 or src_resolution <= 0:
            return
        if len(values) != width_cells * height_cells:
            raise ValueError('Source OccupancyGrid data length mismatch')
        self.initialize(origin_x, origin_y, width_cells, height_cells, src_resolution)
        self.ensure_bounds(origin_x, origin_y,
                           origin_x + width_cells * src_resolution,
                           origin_y + height_cells * src_resolution)

        next_obstacles = set()
        for sy in range(height_cells):
            wy = origin_y + (sy + 0.5) * src_resolution
            row = sy * width_cells
            for sx in range(width_cells):
                value = int(values[row + sx])
                if value < 0:
                    continue  # Do not clear free-space history on UNKNOWN.
                wx = origin_x + (sx + 0.5) * src_resolution
                if value >= obstacle_threshold:
                    next_obstacles.add(self.world_cell(wx, wy))
                elif value <= free_max_value:
                    idx = self.index(wx, wy)
                    if idx is not None:
                        self.data[idx] = self.FREE
        # Swap only once all source cells have been processed.
        self.current_obstacles = next_obstacles

    def clear_ray(self, origin_x, origin_y, angle, length, start_distance=0.0):
        """Virtually clear UNKNOWN cells, stopping at the FIRST current obstacle."""
        if length <= 0 or not self.initialized:
            return
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        step = max(0.01, self.resolution * 0.5)
        previous_key = None
        distance = max(0.0, start_distance)
        while distance <= length + 1e-9:
            wx = origin_x + distance * cos_a
            wy = origin_y + distance * sin_a
            key = self.world_cell(wx, wy)
            if key != previous_key:
                if key in self.current_obstacles:
                    break  # Do not carve through any CURRENT 3D obstacle.
                idx = self.key_to_index(key)
                if idx is not None and self.data[idx] == self.UNKNOWN:
                    self.data[idx] = self.FREE
                previous_key = key
            distance += step

    def clear_fov(self, origin_x, origin_y, robot_yaw, fov_degrees, max_range):
        """Persistently clear the unknown cells in a forward virtual FOV.

        No distance sensing is used here: this is ONLY for map visualization.
        """
        if not self.initialized or max_range <= 0 or fov_degrees <= 0:
            return
        aperture = math.radians(fov_degrees)
        ray_count = max(2, int(math.ceil(aperture * max_range /
                                         (0.5 * self.resolution))))
        start = robot_yaw - aperture / 2.0
        for i in range(ray_count + 1):
            angle = start + aperture * i / ray_count
            self.clear_ray(origin_x, origin_y, angle, max_range)

    def compose(self):
        """Return current obstacle snapshot on top of persistent free history."""
        if not self.initialized:
            return []
        out = self.data.copy()
        for key in self.current_obstacles:
            idx = self.key_to_index(key)
            if idx is not None:
                out[idx] = self.OCCUPIED
        return out
