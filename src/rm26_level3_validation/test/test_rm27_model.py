#!/usr/bin/env python3
"""Check imported geometry, family isolation, and the real RM27 solver/replay path."""
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
IMPORTED = PACKAGE.parent / 'rm27_heavy_arm_demo'
SOURCE = IMPORTED / 'model/mjcf/rm27_heavy_arm_demo.xml'
sys.path.insert(0, str(PACKAGE / 'src'))
import arm_variants as arms
import level3_batch as batch

EXECUTABLE = Path(sys.argv.pop()).absolute() if len(sys.argv) > 1 else None


class RM27ModelTest(unittest.TestCase):
    def test_imported_geometry_matches_upstream(self):
        manifest = json.loads((IMPORTED / 'source_manifest.json').read_text())
        self.assertEqual(manifest['source_commit'], '9a08859c33508e890363e767eaa14e440b882d0c')
        resources = {path: digest for path, digest in manifest['files_sha256'].items()
                     if path.startswith('model/')}
        self.assertGreater(len(resources), 50)
        for path, digest in resources.items():
            self.assertEqual(hashlib.sha256((IMPORTED / path).read_bytes()).hexdigest(), digest, path)

    def test_family_is_bound_to_snapshot(self):
        spec = json.loads((PACKAGE / 'config/arm_rm27.json').read_text())
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            identity = arms.generate_arm(SOURCE, spec, directory / 'model')
            manifest = arms.verify_arm(identity)
            self.assertEqual(identity['model_family'], 'rm27')
            self.assertEqual(identity['attach_link'], 'link_7')
            self.assertIn('RM27', identity['label'])
            self.assertEqual(manifest['modifications'], [])
            with self.assertRaisesRegex(ValueError, 'family'):
                arms.verify_arm({**identity, 'model_family': 'rm26'})
            with self.assertRaisesRegex(ValueError, 'model_family'):
                arms.generate_arm(SOURCE, {'changes': []}, directory / 'wrong-family')
            self.assertFalse((directory / 'wrong-family').exists())

    def test_rm26_length_changes_are_rejected_for_rm27(self):
        spec = json.loads((PACKAGE / 'config/arm_j1_short_5cm.json').read_text())
        spec['model_family'] = 'rm27'
        with self.assertRaisesRegex(ValueError, 'RM27'):
            arms.variant_label(spec)
        with self.assertRaises(ValueError):
            arms.variant_label({'model_family': 'unsupported', 'changes': []})

    def test_rm27_collision_names_are_classified(self):
        self.assertEqual(batch.failure_reason({
            'message': 'Collision at link_5_plate|tech_core_planner_unit/unit_cap'}),
            'unit_arm_collision')
        self.assertEqual(batch.failure_reason({
            'message': 'Collision at tech_core_planner_core/core_peg|link_7_finger_0'}),
            'arm_core_collision')

    @unittest.skipUnless(EXECUTABLE, 'requires level3_case')
    def test_real_solver_and_replay_use_rm27_parameters(self):
        info = json.loads(subprocess.run([str(EXECUTABLE), '--print-arm-config', '--arm-family=rm27'],
                          check=True, text=True, capture_output=True).stdout)
        self.assertEqual(info['planning_joints'], [f'joint_{index}' for index in range(1, 8)])
        self.assertEqual(info['start_joints_rad'], [0.0, 1.39, -1.3, 0.0, 0.0, 0.3, 0.0])
        tightened = json.loads(subprocess.run(
            [str(EXECUTABLE), '--print-arm-config', '--arm-family=rm27',
             '--j1-max-from-vertical-rad=1.0472'],
            check=True, text=True, capture_output=True).stdout)
        self.assertAlmostEqual(tightened['start_joints_rad'][1], 1.0189637841025003)
        info = tightened
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            identity = arms.generate_arm(Path(info['model_path']),
                {'model_family': 'rm27', 'changes': []}, directory / 'snapshot')
            case = {'id': 'rm27', 'arm_model': identity,
                    'j1_max_from_vertical_rad': 1.0472,
                    'start_joints_rad': info['start_joints_rad'],
                    'core_in_base': {'position_m': [0.45, 0.0, 0.4],
                                     'quaternion_wxyz': [0.0, 0.0, 0.0, 1.0]}}
            for name in ('first', 'replay'):
                result = batch.call_case(EXECUTABLE, case, 0.15, directory / name)
                self.assertTrue(result['arm_model_loaded'], result)
                self.assertEqual(result['arm_model_family'], 'rm27')
                self.assertEqual(result['arm_attach_link'], 'link_7')
                self.assertEqual(result['start_joints_rad'], info['start_joints_rad'])
                self.assertEqual(result['elevation_joint'], 'joint_2')
                self.assertEqual(len(result['j1_joint_limits_rad']), 2)
                self.assertNotIn(result['status'], ('ModelError', 'InvalidRequest', 'ProcessError', 'StartCollision'))
                corrupted = copy.deepcopy(result)
                corrupted['arm_model_family'] = 'rm26'
                with self.assertRaisesRegex(ValueError, 'family'):
                    batch.annotate_arm_result(corrupted, case)
                corrupted = copy.deepcopy(result)
                corrupted['start_joints_rad'] = [0.0] * 7
                with self.assertRaisesRegex(ValueError, 'initial joint'):
                    batch.annotate_arm_result(corrupted, case)

    @unittest.skipUnless(EXECUTABLE, 'requires level3_case')
    def test_rm26_default_and_family_specific_limit(self):
        info = json.loads(subprocess.run([str(EXECUTABLE), '--print-arm-config'],
                          check=True, text=True, capture_output=True).stdout)
        self.assertEqual(info['model_family'], 'rm26')
        self.assertEqual(info['attach_link'], 'j6_link')
        self.assertEqual(info['start_joints_rad'], [0.0] * 7)
        self.assertEqual(info['planning_joints'][0], 'j0')
        check = subprocess.run([str(EXECUTABLE), '--check-j1-limits', '--arm-family=rm27'],
                               text=True, capture_output=True)
        self.assertEqual(check.returncode, 0, check.stderr)
        self.assertEqual(len(json.loads(check.stdout)), 2)


if __name__ == '__main__':
    unittest.main()
