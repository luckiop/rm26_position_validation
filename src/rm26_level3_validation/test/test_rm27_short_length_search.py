#!/usr/bin/env python3
import json
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'src'))
import arm_variants as arms
import rm27_short_length_search as search

SOURCE = PACKAGE.parent / 'rm27_heavy_arm_demo/model/mjcf/rm27_heavy_arm_demo.xml'
EXECUTABLE = Path(sys.argv.pop(1)).resolve() if len(sys.argv) > 1 else None


class ShortLengthTest(unittest.TestCase):
    def test_bounds_projection(self):
        original = np.array(list(arms.rm27_lengths(SOURCE, True).values()))
        self.assertAlmostEqual(sum(original) + search.DELTA, .8090030581, places=6)
        for stage in ('priority', 'joint'):
            low, high = search.bounds(original, stage)
            for seed in range(30):
                values = search.project(np.random.default_rng(seed).uniform(low, high), sum(original)+search.DELTA, low, high)
                self.assertTrue(search.feasible(values, original, stage))
        self.assertFalse(search.feasible(original, original, 'joint'))

    def test_terminal_moves_with_tool_and_models_load(self):
        original = arms.rm27_lengths(SOURCE, True)
        src = ET.parse(SOURCE).getroot()
        old_site = np.fromstring(src.find(".//site[@name='tool0']").get('pos'), sep=' ')
        for change in (-.03, .03):
            with self.subTest(change=change), tempfile.TemporaryDirectory() as temp:
                spec = {'model_family': 'rm27', 'stage': 'joint', 'include_j6': True,
                        'total_delta_m': -.147, 'changes': [
                            {'link': 'link_2', 'method': 'set_length', 'target_length_m': original['link_2']-.147-change},
                            {'link': 'link_7', 'method': 'set_length', 'target_length_m': original['link_7']+change}]}
                identity = arms.generate_arm(SOURCE, spec, Path(temp)/'model')
                manifest = arms.verify_arm(identity)
                self.assertEqual(identity['attach_link'], 'variant_energy_mount')
                root = ET.parse(identity['model_path']).getroot()
                mount = root.find(".//body[@name='variant_energy_mount']")
                np.testing.assert_allclose(np.fromstring(mount.get('pos'), sep=' '), [change, 0, 0], atol=1e-9)
                new_site = np.fromstring(root.find(".//site[@name='tool0']").get('pos'), sep=' ')
                np.testing.assert_allclose(new_site-old_site, [change, 0, 0], atol=1e-9)
                self.assertAlmostEqual(sum(manifest['rod_lengths_after_m'].values())-sum(original.values()), -.147)
                self.assertTrue(any(x['link'] == 'link_7' and x['collision_pieces'] for x in manifest['modifications']))
                if EXECUTABLE:
                    subprocess.run([str(EXECUTABLE), '--check-resources', '--arm-family=rm27',
                                    '--arm-model-path='+identity['model_path'],
                                    '--arm-attach-link='+identity['attach_link']], check=True, capture_output=True)


if __name__ == '__main__':
    unittest.main()
