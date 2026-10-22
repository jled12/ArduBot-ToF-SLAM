import math
import unittest

from ardubot_projected_map_clearing.grid import PersistentGrid


class GridTests(unittest.TestCase):
    def cell(self, grid, x, y):
        return grid.compose()[grid.index(x, y)]

    def test_initial_16m_and_2m_blocks(self):
        g = PersistentGrid(resolution=0.05, initial_size_m=16.0, expand_step_m=2.0)
        g.merge_source(0, 0, 2, 2, 0.05, [100, -1, 0, -1])
        self.assertEqual((g.width, g.height), (320, 320))
        self.assertEqual(g.expand_cells, 40)
        self.assertEqual(self.cell(g, 0.025, 0.025), 100)
        self.assertEqual(self.cell(g, 0.025, 0.075), 0)

    def test_current_obstacles_not_accumulated(self):
        g = PersistentGrid()
        g.merge_source(0, 0, 3, 1, .05, [100, -1, -1])
        self.assertEqual(self.cell(g, .025, .025), 100)
        # Move the obstacle 10cm inside the same projected map snapshot.
        g.merge_source(0, 0, 3, 1, .05, [-1, -1, 100])
        self.assertEqual(self.cell(g, .025, .025), -1)
        self.assertEqual(self.cell(g, .125, .025), 100)
        self.assertEqual(len(g.current_obstacles), 1)

    def test_fov_free_history_persists_and_old_obstacle_disappears(self):
        g = PersistentGrid()
        g.merge_source(0, 0, 1, 1, 0.05, [-1])
        g.clear_fov(0, 0, 0, 62.8, 1.5)
        self.assertEqual(self.cell(g, .5, 0), 0)
        g.merge_source(.5, 0, 1, 1, .05, [100])
        self.assertEqual(self.cell(g, .525, .025), 100)
        # Source becomes unknown again; FOV history remains free underneath.
        g.merge_source(.5, 0, 1, 1, .05, [-1])
        self.assertEqual(self.cell(g, .525, .025), 0)

    def test_obstacle_prioritized_over_source_free_and_fov(self):
        g = PersistentGrid()
        g.merge_source(0, 0, 3, 1, .05, [0, 100, 0])
        g.clear_ray(-.15, .025, 0, .5)
        self.assertEqual(self.cell(g, .025, .025), 0)
        self.assertEqual(self.cell(g, .075, .025), 100)
        self.assertEqual(self.cell(g, .125, .025), 0)

    def test_ray_stops_at_current_obstacle_not_removed_one(self):
        g = PersistentGrid()
        g.merge_source(.5, -.1, 1, 4, .05, [100] * 4)
        g.clear_fov(0, 0, 0, 20, 1.2)
        self.assertEqual(self.cell(g, .525, .025), 100)
        self.assertEqual(self.cell(g, .25, .025), 0)
        self.assertEqual(self.cell(g, .85, .025), -1)
        # Latest projected map removes the fence; FOV can now go through it.
        g.merge_source(.5, -.1, 1, 4, .05, [-1] * 4)
        g.clear_fov(0, 0, 0, 20, 1.2)
        self.assertEqual(self.cell(g, .525, .025), 0)
        self.assertEqual(self.cell(g, .85, .025), 0)

    def test_expansion_preserves_free_and_obstacles_world_positions(self):
        g = PersistentGrid()
        g.merge_source(0, 0, 2, 1, .05, [100, 0])
        before = (g.width, g.height)
        g.ensure_bounds(-12, -11, 13, 13)
        self.assertGreater(g.width, before[0])
        self.assertGreater(g.height, before[1])
        self.assertEqual(g.width % 40, 0)
        self.assertEqual(g.height % 40, 0)
        self.assertEqual(self.cell(g, .025, .025), 100)
        self.assertEqual(self.cell(g, .075, .025), 0)
        g.merge_source(0, 0, 2, 1, .05, [-1, -1])
        self.assertEqual(self.cell(g, .025, .025), -1)
        self.assertEqual(self.cell(g, .075, .025), 0)

    def test_source_map_can_expand_over_16m(self):
        g = PersistentGrid()
        g.merge_source(0, 0, 1, 1, .05, [100])
        g.merge_source(-10, -8, 1, 1, .05, [100])
        self.assertEqual(self.cell(g, -9.975, -7.975), 100)
        self.assertEqual(self.cell(g, .025, .025), -1)
        self.assertGreater(g.width, 320)

    def test_unknown_does_not_erase_free_history(self):
        g = PersistentGrid()
        g.merge_source(0, 0, 2, 1, .05, [0, -1])
        g.merge_source(0, 0, 2, 1, .05, [-1, 100])
        self.assertEqual(self.cell(g, .025, .025), 0)
        self.assertEqual(self.cell(g, .075, .025), 100)

    def test_invalid_message_does_not_wipe_current_obstacles(self):
        g = PersistentGrid()
        g.merge_source(0, 0, 1, 1, .05, [100])
        with self.assertRaises(ValueError):
            g.merge_source(0, 0, 2, 2, .05, [100])
        self.assertEqual(self.cell(g, .025, .025), 100)

    def test_fov_rotates_and_does_not_clear_behind(self):
        g = PersistentGrid()
        g.merge_source(0, 0, 1, 1, .05, [-1])
        g.clear_fov(0, 0, math.pi / 2, 62.8, 2.5)
        self.assertEqual(self.cell(g, 0, 1.0), 0)
        self.assertEqual(self.cell(g, 1.0, 0), -1)
        self.assertEqual(self.cell(g, 0, -1.0), -1)

    def test_source_free_and_fov_history_no_thickening(self):
        g = PersistentGrid()
        # One 5cm-wide occupied column moving sideways in subsequent snapshots.
        for col in range(6):
            cells = [-1] * 6
            cells[col] = 100
            g.merge_source(0, 0, 6, 1, .05, cells)
            g.clear_fov(-.1, .025, 0, 20.0, .5)
        row = g.compose()
        self.assertEqual(sum(v == 100 for v in row), 1)
        self.assertEqual(self.cell(g, .275, .025), 100)


if __name__ == '__main__':
    unittest.main()
