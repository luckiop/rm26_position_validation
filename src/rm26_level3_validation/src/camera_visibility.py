"""Saved-state optical evaluation orchestration; planning is never requested."""
import copy
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess


def camera_config(path):
    """Validate the shared camera configuration, in base-frame metres and radians."""
    value = json.loads(Path(path).read_text())
    position = value['position_base_m']
    if len(position) != 3 or any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in position):
        raise ValueError('position_base_m must contain three finite numbers')
    for name, default in [('width', 640), ('height', 480)]:
        value.setdefault(name, default)
        if type(value[name]) is not int or not 16 <= value[name] <= 4096:
            raise ValueError(name + ' must be an integer in 16..4096')
    value.setdefault('diagonal_fov_rad', math.radians(139))
    fov = value['diagonal_fov_rad']
    if isinstance(fov, bool) or not isinstance(fov, (int, float)) or not math.isfinite(fov) or not 0 < fov < math.pi:
        raise ValueError('diagonal_fov_rad must be in (0, pi)')
    value.setdefault('samples', {'Approach': 10, 'Lift': 10})
    for phase in ('Approach', 'Lift'):
        count = value['samples'].get(phase, 10)
        if type(count) is not int or not 2 <= count <= 1000:
            raise ValueError('phase sample counts must be integers in 2..1000')
        value['samples'][phase] = count
    return value


def camera_environment(camera):
    """Return serializable viewer/evaluator environment values."""
    return {'TECH_CORE_CAMERA': ','.join(map(str, [*camera['position_base_m'], camera['width'], camera['height'], camera['diagonal_fov_rad']])),
            'TECH_CORE_APPROACH_FRAMES': str(camera['samples']['Approach']),
            'TECH_CORE_LIFT_FRAMES': str(camera['samples']['Lift'])}


def file_digest(path):
    """Bind evaluation to the exact input bytes."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def evaluate_batch(args, executable, case_command, verify_arm, write_json):
    """Evaluate frozen batch trajectories in independent subprocesses, without planning."""
    batch = args.batch.resolve()
    camera = camera_config(args.camera_config)
    config = json.loads((batch / 'config.json').read_text())
    cases = json.loads((batch / 'cases.json').read_text())
    if args.case:
        cases = [case for case in cases if case['id'] == args.case]
        if not cases:
            raise ValueError('case not found: ' + args.case)
    if args.trajectory_dir and not args.case:
        raise ValueError('--trajectory-dir requires --case')
    if args.gui and not args.case:
        raise ValueError('--gui requires --case')
    # Validate identities before creating output. Never silently substitute installed models.
    for case in cases:
        identity = case.get('arm_model')
        if identity:
            verify_arm(identity)
        elif not args.allow_legacy_model:
            raise ValueError('No frozen model identity; use --allow-legacy-model for explicitly unverified evaluation')
    args.output.mkdir(parents=True, exist_ok=False)
    write_json(args.output / 'camera_config.json', camera)
    results = []
    environment = {**os.environ, **camera_environment(camera)}
    if args.images:
        environment['TECH_CORE_CAMERA_IMAGES'] = '1'
    else:
        environment.pop('TECH_CORE_CAMERA_IMAGES', None)
    for original in cases:
        case = copy.deepcopy(original)
        source = args.trajectory_dir.resolve() if args.trajectory_dir else batch / 'trajectories' / case['id']
        destination = args.output / case['id']
        destination.mkdir()
        if args.trajectory_dir:
            pose_path = source / 'core_pose.json'
            if not pose_path.is_file():
                raise ValueError('Explicit trajectory directory must contain core_pose.json')
            case['core_in_base'] = json.loads(pose_path.read_text())
            source_case = source / 'case.json'
            if not source_case.exists():
                source_case = source / 'source_case.json'
            if source_case.exists():
                saved = json.loads(source_case.read_text())
                if saved.get('arm_model') != case.get('arm_model'):
                    raise ValueError('Trajectory model identity differs from batch')
        # A success prefix takes precedence; never concatenate Q branch files.
        trajectory = next((source / name for name in ('prefix.csv', 'candidate.csv', 'collision_diagnostic.csv')
                           if (source / name).is_file()), None)
        row = {'case_id': case['id'], 'sample_id': case.get('sample_id', case['id']),
               'parking_id': case.get('parking_id', 'p00'), 'arm_model': case.get('arm_model'),
               'model_verified': bool(case.get('arm_model')), 'trajectory': str(trajectory) if trajectory else None,
               'trajectory_sha256': file_digest(trajectory) if trajectory else None,
               'case_sha256': hashlib.sha256(json.dumps(case, sort_keys=True).encode()).hexdigest(),
               'parking_offset_world_m': case.get('parking_offset_world_m'),
               'parking_yaw_world_rad': case.get('parking_yaw_world_rad', 0)}
        if 'start_joints_rad' in case:
            environment['TECH_CORE_CAMERA_START'] = ' '.join(map(str, case['start_joints_rad']))
        else:
            environment.pop('TECH_CORE_CAMERA_START', None)
        row['executable_sha256'] = file_digest(executable) if Path(executable).is_file() else None
        command = case_command(executable, case, config.get('timeout_s', 30), destination,
                               gui=args.gui, assembly=config.get('assembly_in_base'))
        if not case.get('arm_model'):
            family = config.get('arm_variant', {}).get('model_family', 'rm26')
            command.append('--arm-family=' + family)
        command.append('--visibility-trajectory=' + (str(trajectory) if trajectory else ''))
        write_json(destination / 'inputs.json', {**row, 'case': case, 'camera': camera, 'command': command})
        try:
            with (destination / 'evaluation.log').open('w') as log:
                subprocess.run(command, check=True, env=environment, stdout=log, stderr=subprocess.STDOUT)
            row.update(json.loads((destination / 'visibility.json').read_text()))
            row['status'] = 'ok'
        except (subprocess.SubprocessError, OSError, ValueError) as error:
            row.update(status='error', error=str(error), complete=False, case_occlusion=None)
        results.append(row)
        write_json(args.output / 'results.json', results)
        print(f"{case['id']}: {row['status']}, mean={row.get('available_mean')}", flush=True)
    summaries = {}
    for row in results:
        key = row.get('source', 'error')
        group = summaries.setdefault(key, {'cases': 0, 'complete_cases': 0, 'means': []})
        group['cases'] += 1
        if row.get('complete'):
            group['complete_cases'] += 1
            group['means'].append(row['case_occlusion'])
    for group in summaries.values():
        values = group.pop('means')
        group['complete_case_mean'] = sum(values)/len(values) if values else None
    write_json(args.output / 'summary.json', {'sources': summaries, 'cases': len(results),
               'errors': sum(row['status'] == 'error' for row in results)})
    with (args.output / 'results.csv').open('w') as stream:
        fields = ['case_id', 'sample_id', 'parking_id', 'status', 'source', 'model_verified', 'complete', 'valid_frames', 'case_occlusion', 'available_mean']
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(results)
    if any(row['status'] == 'error' for row in results):
        raise RuntimeError('Some optical evaluations failed; inspect per-case evaluation.log')


def compare_evaluations(directories, output, write_json):
    """Compare verified complete cases on the same task/parking intersection."""
    if len(directories) < 2:
        raise ValueError('Comparison requires at least two evaluation directories')
    cameras = [camera_config(directory / 'camera_config.json') for directory in directories]
    if any(value != cameras[0] for value in cameras[1:]):
        raise ValueError('Camera settings differ between evaluations')
    groups = []
    for directory in directories:
        rows = json.loads((directory / 'results.json').read_text())
        keyed = {}
        for row in rows:
            if row.get('model_verified') and row.get('complete') and row.get('case_occlusion') is not None:
                key = (row['sample_id'], row['parking_id'])
                if key in keyed:
                    raise ValueError('Duplicate task/parking identity')
                keyed[key] = row
        groups.append(keyed)
    common = sorted(set.intersection(*(set(group) for group in groups)))
    output.mkdir(parents=True, exist_ok=False)
    # Check actual input poses, since matching case numbers alone does not prove equal tasks.
    for key in common:
        cases = [json.loads((directory / group[key]['case_id'] / 'inputs.json').read_text())['case']
                 for directory, group in zip(directories, groups)]
        for field in ('core_in_world', 'parking_offset_world_m', 'parking_yaw_world_rad'):
            if any(case.get(field) != cases[0].get(field) for case in cases[1:]):
                raise ValueError('Paired cases have different task/parking poses')
    with (output / 'paired_cases.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['sample_id', 'parking_id', *map(str, directories)])
        for key in common:
            writer.writerow([*key, *(group[key]['case_occlusion'] for group in groups)])
    write_json(output / 'comparison.json', {
        'common_complete_cases': len(common),
        'configurations': [{'directory': str(directory), 'verified_complete_cases': len(group),
                            'paired_mean': sum(group[key]['case_occlusion'] for key in common)/len(common) if common else None}
                           for directory, group in zip(directories, groups)]})
