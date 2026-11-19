"""Configuration and no-replan batch orchestration regressions."""
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import sys
from types import SimpleNamespace

SOURCE = Path(__file__).resolve().parents[1] / 'src'
sys.path.insert(0, str(SOURCE))
import camera_visibility as camera


class CameraTests(unittest.TestCase):
    def test_configuration(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'camera.json'
            path.write_text(json.dumps({'position_base_m': [.1, .2, .3]}))
            config = camera.camera_config(path)
            self.assertEqual(config['samples'], {'Approach': 10, 'Lift': 10})
            self.assertAlmostEqual(config['diagonal_fov_rad'], math.radians(139))
            self.assertIn('TECH_CORE_CAMERA', camera.camera_environment(config))
            for value in ([float('nan'), 0, 0], [True, 0, 0], [0, 0]):
                path.write_text(json.dumps({'position_base_m': value}))
                with self.assertRaises(ValueError):
                    camera.camera_config(path)

    def test_saved_input_no_planning(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / 'batch'
            source.mkdir()
            identity = {'fingerprint_sha256': 'frozen'}
            case = {'id': 'case_0000', 'arm_model': identity}
            (source / 'cases.json').write_text(json.dumps([case]))
            (source / 'config.json').write_text('{}')
            trajectory = source / 'trajectories' / case['id']
            trajectory.mkdir(parents=True)
            (trajectory / 'candidate.csv').write_text('candidate bytes')
            config = root / 'camera.json'
            config.write_text(json.dumps({'position_base_m': [0, 0, .3]}))
            args = SimpleNamespace(batch=source, case=None, trajectory_dir=None, gui=False,
                                   allow_legacy_model=False, output=root / 'output',
                                   camera_config=config, images=False)
            commands = []
            def fake_run(command, **kwargs):
                commands.append(command)
                destination = Path(command[1])
                (destination / 'visibility.json').write_text(json.dumps({
                    'source': 'candidate', 'complete': False, 'available_mean': .25,
                    'case_occlusion': None, 'valid_frames': 10}))
            def save(path, value):
                path.write_text(json.dumps(value))
            verified = []
            with patch.object(camera.subprocess, 'run', fake_run):
                camera.evaluate_batch(args, Path('/bin/level3_case'),
                    lambda executable, case, timeout, output, **kwargs: [str(executable), str(output)],
                    verified.append, save)
            self.assertEqual(verified, [identity])
            self.assertTrue(commands[0][-1].startswith('--visibility-trajectory='))
            self.assertTrue(commands[0][-1].endswith('candidate.csv'))
            row = json.loads((args.output / 'results.json').read_text())[0]
            self.assertEqual(row['available_mean'], .25)
            self.assertIsNone(row['case_occlusion'])
            self.assertEqual(row['trajectory_sha256'], camera.file_digest(trajectory / 'candidate.csv'))
            with self.assertRaises(FileExistsError):
                camera.evaluate_batch(args, Path('/bin/level3_case'), lambda *a, **k: [], verified.append, save)

    def test_paired_comparison(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            directories = [root / 'a', root / 'b']
            for index, path in enumerate(directories):
                path.mkdir()
                (path / 'camera_config.json').write_text(json.dumps({'position_base_m': [0, 0, .3]}))
                rows = [{'case_id': 'case', 'sample_id': 'sample', 'parking_id': 'p00',
                         'model_verified': True, 'complete': True, 'case_occlusion': .1 + index*.2}]
                (path / 'results.json').write_text(json.dumps(rows))
                (path / 'case').mkdir()
                (path / 'case' / 'inputs.json').write_text(json.dumps({'case': {
                    'core_in_world': {'position_m': [0, 0, 0]},
                    'parking_offset_world_m': [0, 0, 0], 'parking_yaw_world_rad': 0}}))
            def save(path, value):
                path.write_text(json.dumps(value))
            camera.compare_evaluations(directories, root / 'comparison', save)
            result = json.loads((root / 'comparison' / 'comparison.json').read_text())
            self.assertEqual(result['common_complete_cases'], 1)
            self.assertAlmostEqual(result['configurations'][1]['paired_mean'], .3)
            (directories[1] / 'camera_config.json').write_text(json.dumps({'position_base_m': [1, 0, .3]}))
            with self.assertRaises(ValueError):
                camera.compare_evaluations(directories, root / 'invalid', save)


if __name__ == '__main__':
    unittest.main()
