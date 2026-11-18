#!/usr/bin/env python3
"""Independent sampling, coordinate and reporting contracts."""
import csv
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('level3_batch', ROOT / 'src/level3_batch.py')
batch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(batch)


class BatchTest(unittest.TestCase):
    def test_start_joint_comparison_tolerates_only_roundoff(self):
        identity = {'model_path': '/tmp/arm.xml', 'model_family': 'rm27', 'attach_link': 'link_7'}
        case = {'arm_model': identity, 'start_joints_rad': [0, 1.0189637841025003]}
        result = {'arm_model_path': '/tmp/arm.xml', 'arm_model_family': 'rm27',
                  'arm_attach_link': 'link_7', 'start_joints_rad': [0, 1.0189637841024972]}
        batch.annotate_arm_result(result.copy(), case)
        for invalid in ([0, 1.019], [0], [0, float('nan')], None):
            with self.assertRaises(ValueError):
                batch.annotate_arm_result({**result, 'start_joints_rad': invalid}, case)

    def setUp(self):
        self.config = json.loads((ROOT / 'config/default.json').read_text())
        # Sampling contracts must be independent of the user's run size.
        self.config['adaptive_y_parking'] = False
        self.config.pop('parking_grid', None)
        self.config['samples'] = 100
        self.config['quick_samples'] = 10
        self.config.pop('parking_yaw_world_rad', None)
        self.config['check_self_collision'] = True
        self.config['check_gripper_pillar_collision'] = True

    def test_adaptive_y_with_parking_yaw_and_grid(self):
        self.config['assembly_in_base']['rpy_rad'] = [0.2, -0.3, 1.1]
        self.config['parking_yaw_world_rad'] = -math.pi / 2
        for count in (1, 4, 9):
            self.config['parking_grid'] = {
                'positions': count, 'step_x_m': 0.1, 'step_y_m': 0.2}
            self.config['adaptive_y_parking'] = False
            original = batch.sample_cases(self.config)
            self.config['adaptive_y_parking'] = True
            adaptive = batch.sample_cases(self.config)
            for before, after in zip(original, adaptive):
                self.assertEqual(before['core_in_world'], after['core_in_world'])
                parking = after['parking_offset_world_m']
                self.assertAlmostEqual(parking[1], before['parking_offset_world_m'][1] +
                                       after['core_in_world']['position_m'][1])
                for axis in (0, 2):
                    self.assertEqual(parking[axis], before['parking_offset_world_m'][axis])
                restored = batch.rotate(batch.rotation_rpy(0, 0, -math.pi / 2),
                                        after['core_in_base']['position_m'])
                for axis in range(3):
                    self.assertAlmostEqual(restored[axis] + parking[axis],
                                           after['core_in_world']['position_m'][axis])
        self.config['adaptive_y_parking'] = 'true'
        with self.assertRaises(ValueError):
            batch.sample_cases(self.config)

    def test_completed_progress_scores(self):
        for stage, score in [('Request', 0), ('Start', 0), ('TaskSetup', 0),
                             ('PostureSearch', 0), ('Approach', 0), ('Preinsert', 0),
                             ('Insert', 0), ('Lift', 1), ('Flip', 2), ('Twist', 3),
                             ('Timing', 3), ('Validation', 3)]:
            self.assertEqual(batch.case_score({'success': False, 'failure_stage': stage}), score)
        self.assertEqual(batch.case_score({'success': True}), 4)
        self.assertIsNone(batch.case_score({'success': False, 'candidate_last_phase': 'Twist'}))
        self.assertIsNone(batch.case_score(None))

    def test_progress_statistics(self):
        cases = [{'id': f'{s}_{p}', 'sample_id': str(s), 'parking_id': str(p)}
                 for s in range(3) for p in range(2)]
        results = {'0_0': {'success': True}, '0_1': {'failure_stage': 'Twist'},
                   '1_0': {'failure_stage': 'Flip'}, '1_1': {'failure_stage': 'Insert'},
                   '2_0': {'status': 'Timeout'}}
        stats = batch.progress_statistics(cases, results)
        self.assertEqual([g['mean_score'] for g in stats['parking_results']], [3, 1.5])
        self.assertEqual(stats['mean_parking_score'], 2.25)
        self.assertEqual(stats['qualifying_parking_histogram'], {'0': 1, '1': 0, '2': 1})
        self.assertEqual(stats['mean_qualifying_parking_count'], 1)
        self.assertEqual(stats['incomplete_task_count'], 1)
        self.assertEqual(stats['parking_results'][0]['unknown'], 1)
        self.assertEqual(stats['parking_results'][1]['pending'], 1)

    def test_failure_reason_categories(self):
        examples = [
            ({'message': 'IK iteration limit reached'}, 'unreachable'),
            ({'message': 'Approach extension budget exhausted'}, 'unreachable'),
            ({'status': 'Timeout', 'last_failure_message': 'Collision at j2_link_collision_000|tech_core_planner_unit/unit_cap'}, 'unreachable'),
            ({'message': 'Collision at j2_link_collision_000|tech_core_planner_unit/unit_cap'}, 'unit_arm_collision'),
            ({'message': 'Collision at tech_core_planner_core/core_peg|j6_link_collision_003'}, 'arm_core_collision'),
            ({'message': 'Collision at tech_core_planner_unit/unit_cap|tech_core_planner_core/core_peg'}, 'other_collision'),
            ({'status': 'ProcessError'}, 'other'),
            ({'success': True}, None), (None, None)]
        for result, category in examples:
            self.assertEqual(batch.failure_reason(result), category)
        result = {'failure_stage': 'Twist', 'message': 'IK iteration limit reached'}
        self.assertEqual(batch.case_score(result), 3)

    def test_j1_limit(self):
        self.config['j1_max_from_vertical_rad'] = 0.8
        case = batch.sample_cases(self.config)[0]
        self.assertEqual(case['j1_max_from_vertical_rad'], 0.8)
        self.assertIn('--j1-max-from-vertical-rad=0.8', batch.case_command(
            Path('/solver'), case, 1, Path('/output')))
        for invalid in (True, 0, -1, 1.6, float('nan'), float('inf'), '1'):
            self.config['j1_max_from_vertical_rad'] = invalid
            with self.assertRaises(ValueError):
                batch.sample_cases(self.config)

    def test_latin_hypercube_and_quick_subset(self):
        cases = batch.sample_cases(self.config)
        self.assertEqual(cases, batch.sample_cases(self.config))
        self.assertEqual(len(cases), 100)
        self.assertEqual(sum(c['quick'] for c in cases), 10)
        for field in batch.FIELDS:
            lo, hi = self.config['core_bounds'][field]
            bins = {int(100 * (case['core_in_assembly'][field] - lo) / (hi - lo)) for case in cases}
            self.assertEqual(bins, set(range(100)))
        self.config['seed'] += 1
        self.assertNotEqual(cases, batch.sample_cases(self.config))

    def test_known_frame_transform(self):
        self.config['assembly_in_base'] = {'position_m': [0.30, 0.20, -0.25],
                                           'rpy_rad': [0.0, 0.0, math.pi]}
        # Known reference point maps to (0.39, 0, 0.35) with yaw pi.
        aq = batch.rotation_rpy(*self.config['assembly_in_base']['rpy_rad'])
        offset = batch.rotate(aq, [-0.09, 0.2, 0.6])
        pos = [offset[i] + self.config['assembly_in_base']['position_m'][i] for i in range(3)]
        for actual, expected in zip(pos, [0.39, 0.0, 0.35]):
            self.assertAlmostEqual(actual, expected, places=12)
        axis = batch.rotate(batch.qmul(aq, batch.rotation_rpy(0, 0, 0)), [1, 0, 0])
        self.assertAlmostEqual(axis[0], -1, places=12)
        self.assertAlmostEqual(axis[1], 0, places=12)

    def test_parking_keeps_world_core_fixed(self):
        self.config.pop('parking_offset_world_m', None)
        original = batch.sample_cases(self.config)
        self.config['parking_offset_world_m'] = [0.3, -0.2, 0.1]
        parked = batch.sample_cases(self.config)
        for before, after in zip(original, parked):
            self.assertEqual(before['core_in_world'], after['core_in_world'])
            self.assertEqual(before['core_in_assembly'], after['core_in_assembly'])
            self.assertEqual(before['core_in_base']['quaternion_wxyz'], after['core_in_base']['quaternion_wxyz'])
            for axis, offset in enumerate(self.config['parking_offset_world_m']):
                self.assertAlmostEqual(after['core_in_base']['position_m'][axis],
                                       before['core_in_base']['position_m'][axis] - offset)
        for invalid in ([1, 2], [0, float('nan'), 0], [True, 0, 0]):
            self.config['parking_offset_world_m'] = invalid
            with self.assertRaises(ValueError):
                batch.sample_cases(self.config)

    def test_self_collision_option_is_snapshotted_and_forwarded(self):
        self.config['check_self_collision'] = False
        case = batch.sample_cases(self.config)[0]
        self.assertFalse(case['check_self_collision'])
        for gui in (False, True):
            cmd = batch.case_command(Path('/solver'), case, 1, Path('/output'),
                                     gui=gui, assembly=self.config['assembly_in_base'])
            self.assertEqual(cmd[-1], '--disable-self-collision')
        self.assertNotIn('--disable-self-collision', batch.case_command(
            Path('/solver'), {'core_in_base': case['core_in_base']}, 1, Path('/output')))
        self.config['check_self_collision'] = 'false'
        with self.assertRaises(ValueError):
            batch.sample_cases(self.config)

    def test_gripper_pillar_option_and_combined_flags(self):
        self.config['check_gripper_pillar_collision'] = False
        self.config['check_self_collision'] = False
        case = batch.sample_cases(self.config)[0]
        self.assertFalse(case['check_gripper_pillar_collision'])
        for gui in (False, True):
            command = batch.case_command(Path('/solver'), case, 1, Path('/output'), gui=gui,
                                         assembly=self.config['assembly_in_base'])
            self.assertEqual(command[-2:], ['--disable-self-collision', '--disable-gripper-pillar-collision'])
        self.config['check_gripper_pillar_collision'] = 'false'
        with self.assertRaises(ValueError):
            batch.sample_cases(self.config)

    def test_rotated_base_keeps_field_pose_and_passes_yaw(self):
        self.config['parking_yaw_world_rad'] = -math.pi / 2
        self.config['parking_offset_world_m'] = [0.3, -0.2, 0.1]
        case = batch.sample_cases(self.config)[0]
        base = case['core_in_base']
        field = case['core_in_world']
        field_from_base = batch.axis_quaternion(2, -math.pi / 2)
        restored = batch.rotate(field_from_base, base['position_m'])
        for i in range(3):
            self.assertAlmostEqual(restored[i] + case['parking_offset_world_m'][i], field['position_m'][i])
        restored_q = batch.qmul(field_from_base, base['quaternion_wxyz'])
        for a, b in zip(restored_q, field['quaternion_wxyz']):
            self.assertAlmostEqual(a, b)
        cmd = batch.case_command(Path('/solver'), case, 1, Path('/output'), gui=True,
                                 assembly=self.config['assembly_in_base'])
        self.assertEqual(len(cmd), 22)
        self.assertAlmostEqual(float(cmd[-2]), -math.pi / 2)
        with self.assertRaises(ValueError):
            batch.sample_cases({**self.config, 'parking_yaw_world_rad': float('nan')})

    def test_parking_grids_repeat_identical_world_samples(self):
        self.config.pop('parking_grid', None)
        original = batch.sample_cases(self.config)
        for count, axis_indices in ((4, (0, 1)), (9, (0, 1, 2))):
            self.config['parking_grid'] = {'positions': count, 'step_x_m': 0.2, 'step_y_m': -0.3}
            positions = batch.parking_positions(self.config)
            cases = batch.sample_cases(self.config)
            self.assertEqual(len(cases), count * len(original))
            self.assertEqual(len({c['id'] for c in cases}), len(cases))
            self.assertEqual(sum(c['quick'] for c in cases), count * self.config['quick_samples'])
            self.assertEqual({tuple(p['grid_index']) for p in positions},
                             {(x, y) for x in axis_indices for y in axis_indices})
            for p in positions:
                parked = [c for c in cases if c['parking_id'] == p['id']]
                for before, after in zip(original, parked):
                    self.assertEqual(before['id'], after['sample_id'])
                    self.assertEqual(before['core_in_world'], after['core_in_world'])
                    self.assertEqual(before['core_in_assembly'], after['core_in_assembly'])
                    for i in range(3):
                        self.assertAlmostEqual(after['core_in_base']['position_m'][i] +
                                               after['parking_offset_world_m'][i],
                                               before['core_in_world']['position_m'][i])
                x, y = p['grid_index']
                self.assertEqual(p['offset_world_m'],
                                 [self.config['parking_offset_world_m'][0] + x * 0.2,
                                  self.config['parking_offset_world_m'][1] - y * 0.3,
                                  self.config['parking_offset_world_m'][2]])

    def test_invalid_parking_grid(self):
        for grid in ([], {'positions': True}, {'positions': 3}, {'positions': 4.0},
                     {'positions': 4, 'step_x_m': 0}, {'positions': 9, 'step_y_m': float('inf')},
                     {'step_x_m': True}):
            with self.subTest(grid=grid):
                self.config['parking_grid'] = grid
                with self.assertRaises(ValueError):
                    batch.sample_cases(self.config)

    def test_grid_report_and_solver_parking_arguments(self):
        self.config['samples'] = self.config['quick_samples'] = 1
        self.config['parking_grid'] = {'positions': 4, 'step_x_m': 0.2, 'step_y_m': 0.3}
        cases = batch.sample_cases(self.config)
        results = {c['id']: {'success': i == 2, 'status': 'Ok' if i == 2 else 'StartCollision',
                             'planning_s': 0.1} for i, c in enumerate(cases)}
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            batch.save_report(directory, cases, self.config, results)
            summary = json.loads((directory / 'summary.json').read_text())
            self.assertEqual([p['successes'] for p in summary['parking_results']], [0, 0, 1, 0])
            with (directory / 'results.csv').open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual([r['parking_id'] for r in rows], ['p00', 'p01', 'p02', 'p03'])
            self.assertEqual(float(rows[3]['parking_x_m']), cases[3]['parking_offset_world_m'][0])
            executable = directory / 'solver'
            executable.write_text('#!/bin/sh\necho \'{"success":true,"status":"Ok"}\'\n')
            executable.chmod(0o755)
            output = directory / 'case'
            batch.call_case(executable, cases[3], 1, output,
                            assembly=self.config['assembly_in_base'], config=self.config)
            command = json.loads((output / 'request.json').read_text())['command']
            self.assertEqual([float(x) for x in command[-4:-1]], cases[3]['parking_offset_world_m'])
            for i in range(3):
                self.assertAlmostEqual(float(command[11+i]),
                                       self.config['assembly_in_base']['position_m'][i] -
                                       cases[3]['parking_offset_world_m'][i])

    def test_failure_pie_denominator_and_unknown(self):
        rows = [None, {'success': True}, {'success': False, 'failure_stage': 'Insert'},
                {'success': False, 'failure_stage': 'Twist'},
                {'success': False, 'message': 'Required Q coverage incomplete'},
                {'success': False, 'candidate_last_phase': 'Twist'}]
        distribution = batch.failure_distribution(rows)
        self.assertEqual({row[0]: row[2] for row in distribution}, {'Insert': 1, 'Twist': 2, 'Unknown': 1})
        self.assertAlmostEqual(sum(row[3] for row in distribution), 1.0)
        self.assertEqual(batch.failure_distribution([None, {'success': True}]), [])

    def test_parking_pies_include_success_and_keep_groups_separate(self):
        cases = [{'id': str(i), 'parking_id': point,
                  'parking_offset_world_m': [-1 if point == 'p00' else 2, 0, 0]}
                 for i, point in enumerate(['p00', 'p00', 'p00', 'p01', 'p02'])]
        results = {'0': {'success': True},
                   '1': {'success': False, 'failure_stage': 'Flip'},
                   '3': {'success': True}}
        groups = batch.parking_distributions(cases, results)
        self.assertEqual([g['id'] for g in groups], ['p00', 'p01', 'p02'])
        self.assertEqual(groups[0]['offset_world_m'], [-1, 0, 0])
        self.assertEqual((groups[0]['successes'], groups[0]['failures'], groups[0]['pending']), (1, 1, 1))
        self.assertEqual({r[0]: r[3] for r in groups[0]['distribution']}, {'Success': 0.5, 'score2': 0.5})
        self.assertEqual(groups[1]['distribution'][0][3], 1.0)
        self.assertEqual(groups[2]['distribution'], [])
        self.assertEqual(groups[2]['pending'], 1)

    def test_failure_denominator_and_csv_fields(self):
        cases = batch.sample_cases(self.config)[:5]
        kinds = ['Ok', 'ValidationFailed', 'StartCollision', 'IKFailed', 'Timeout']
        results = {case['id']: {'success': k == 'Ok', 'status': k, 'planning_s': float(i),
                                'q_coverage_rad': [-math.pi/2, math.pi/2] if i == 0 else None}
                   for i, (case, k) in enumerate(zip(cases, kinds))}
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            batch.save_report(directory, cases, self.config, results)
            summary = json.loads((directory / 'summary.json').read_text())
            with (directory / 'results.csv').open() as stream:
                rows = list(csv.DictReader(stream))
        self.assertEqual(summary['requests'], 5)
        self.assertEqual(summary['successes'], 1)
        self.assertEqual(summary['success_rate'], 0.2)
        self.assertEqual(summary['status_counts']['Timeout'], 1)
        self.assertEqual(len(rows), 5)
        self.assertEqual(rows[0]['status'], 'Ok')
        self.assertAlmostEqual(float(rows[0]['q_min_rad']), -math.pi/2)
        self.assertEqual(rows[1]['q_min_rad'], '')

    def test_invalid_config_and_incomplete_solver_result(self):
        self.config['timeout_s'] = float('nan')
        with self.assertRaises(ValueError):
            batch.sample_cases(self.config)
        with tempfile.TemporaryDirectory() as tmp:
            executable = Path(tmp) / 'bad_solver'
            executable.write_text('#!/bin/sh\necho failure >&2\nexit 7\n')
            executable.chmod(0o755)
            self.config['timeout_s'] = 1
            case = batch.sample_cases(self.config)[0]
            result = batch.call_case(executable, case, 1, Path(tmp), config=self.config)
            self.assertFalse(result['success'])
            self.assertEqual(result['status'], 'ProcessError')
            self.assertEqual(result['returncode'], 7)
            saved_config = json.loads((Path(tmp) / 'config.json').read_text())
            self.assertEqual(saved_config, self.config)
            self.assertEqual(json.loads((Path(tmp) / 'case.json').read_text()), case)
            self.assertEqual(json.loads((Path(tmp) / 'result.json').read_text()), result)
            self.assertEqual(json.loads((Path(tmp) / 'request.json').read_text())['case_id'], case['id'])
            self.config['timeout_s'] = 99
            batch.save_case_snapshot(Path(tmp), case, self.config)
            self.assertEqual(json.loads((Path(tmp) / 'config.json').read_text()), saved_config)

    def test_failure_stage_labels(self):
        self.assertIn('Q 旋转', batch.describe_result({'success': False, 'failure_stage': 'Twist'})[0])
        self.assertIn('未知', batch.describe_result({'success': False, 'candidate_last_phase': 'Flip'})[0])
        self.assertIn('根据原始消息判断', batch.describe_result({
            'success': False, 'message': 'Required Q coverage incomplete'})[0])
        self.assertEqual(batch.describe_result({'success': True})[0], '成功')

    def test_failure_hints_do_not_replace_status(self):
        self.assertEqual(batch.failure_hint({'status': 'PlanningFailed', 'message': 'Collision at link'}), 'collision')
        self.assertEqual(batch.failure_hint({'status': 'PlanningFailed', 'message': 'Required Q coverage incomplete: Collision'}), 'q_incomplete')
        self.assertEqual(batch.failure_hint({'status': 'IKFailed', 'message': 'iteration limit'}), 'ik_failed')
        self.assertEqual(batch.failure_hint({'status': 'StartCollision'}), 'start_collision_or_limits')


if __name__ == '__main__':
    unittest.main()
