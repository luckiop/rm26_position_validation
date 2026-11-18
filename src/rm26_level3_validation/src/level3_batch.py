#!/usr/bin/env python3
"""固定基座三级任务的确定性分层采样、批量评测及案例重算查看。"""
import argparse
import concurrent.futures
import csv
import json
import math
import os
from pathlib import Path
import random
import re
import signal
import queue
import tempfile
import threading
import statistics
import subprocess
import sys
import time

# Also works when this script is installed as the extensionless level3_batch.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from arm_variants import generate_arm, verify_arm, variant_label

FIELDS = ('x_m', 'y_m', 'z_m', 'theta_rad', 'phi_rad', 'alpha_rad')


def write_json(path, value):
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    os.replace(temporary, path)


def save_case_snapshot(directory, case, config=None, command=None):
    """Persist independent inputs before solving; retain any existing original snapshots."""
    directory.mkdir(parents=True, exist_ok=True)
    snapshots = {'case.json': case}
    if config is not None:
        snapshots['config.json'] = config
    if command is not None:
        snapshots['request.json'] = {'case_id': case['id'], 'command': command,
                                     'arm_model': case.get('arm_model')}
    for name, value in snapshots.items():
        if not (directory / name).exists():
            write_json(directory / name, value)


def finite_vector(value, length, name):
    if not isinstance(value, list) or len(value) != length or any(
            isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in value):
        raise ValueError(f'{name} requires {length} finite numbers')
    return [float(x) for x in value]


def check_config(config):
    for field in ('samples', 'quick_samples', 'seed'):
        value = config[field]
        if isinstance(value, bool) or not isinstance(value, int) or (field != 'seed' and value <= 0):
            raise ValueError(f'{field} must be an integer' + (' greater than zero' if field != 'seed' else ''))
    if config['quick_samples'] > config['samples']:
        raise ValueError('quick_samples must not exceed samples')
    if isinstance(config['timeout_s'], bool) or not isinstance(config['timeout_s'], (int, float)) or not math.isfinite(config['timeout_s']) or config['timeout_s'] <= 0:
        raise ValueError('timeout_s must be finite and positive')
    for field in FIELDS:
        lower, upper = finite_vector(config['core_bounds'][field], 2, field)
        if lower > upper:
            raise ValueError(f'{field}: lower bound exceeds upper bound')
    finite_vector(config.get('parking_offset_world_m', [0.0, 0.0, 0.0]), 3, 'parking_offset_world_m')
    if not isinstance(config.get('adaptive_y_parking', False), bool):
        raise ValueError('adaptive_y_parking must be a boolean')
    for option in ('check_self_collision', 'check_gripper_pillar_collision'):
        if not isinstance(config.get(option, True), bool):
            raise ValueError(f'{option} must be a boolean')
    limit = config.get('j1_max_from_vertical_rad', math.pi / 2)
    if isinstance(limit, bool) or not isinstance(limit, (int, float)) or not math.isfinite(limit) or not 0 < limit <= math.pi / 2:
        raise ValueError('j1_max_from_vertical_rad must be finite and in (0, pi/2] rad')
    yaw = config.get('parking_yaw_world_rad', 0.0)
    if isinstance(yaw, bool) or not isinstance(yaw, (int, float)) or not math.isfinite(yaw):
        raise ValueError('parking_yaw_world_rad must be finite')
    parking_positions(config)
    finite_vector(config['assembly_in_base']['position_m'], 3, 'position_m')
    finite_vector(config['assembly_in_base']['rpy_rad'], 3, 'rpy_rad')


def parking_positions(config):
    """Return world-frame parking positions; steps are signed X/Y metres."""
    origin = finite_vector(config.get('parking_offset_world_m', [0.0, 0.0, 0.0]), 3,
                           'parking_offset_world_m')
    grid = config.get('parking_grid', {})
    if not isinstance(grid, dict):
        raise ValueError('parking_grid must be an object')
    count = grid.get('positions', 1)
    if isinstance(count, bool) or not isinstance(count, int) or count not in (1, 4, 9):
        raise ValueError('parking_grid.positions must be 1, 4 or 9')
    steps = finite_vector([grid.get('step_x_m', 0.1), grid.get('step_y_m', 0.1)], 2,
                          'parking_grid steps')
    if count != 1 and any(step == 0 for step in steps):
        raise ValueError('parking_grid steps must be nonzero for multiple positions')
    indices = {1: (0,), 4: (0, 1), 9: (0, 1, 2)}[count]
    positions = []
    for y in indices:
        for x in indices:
            offset = [origin[0] + x * steps[0], origin[1] + y * steps[1], origin[2]]
            finite_vector(offset, 3, 'generated parking offset')
            positions.append({'id': f'p{len(positions):02d}', 'grid_index': [x, y],
                              'offset_world_m': offset})
    if len({tuple(p['offset_world_m']) for p in positions}) != count:
        raise ValueError('parking_grid steps are too small to produce distinct positions')
    return positions


def qmul(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return [w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v]


def axis_quaternion(axis, angle):
    vector = [0.0, 0.0, 0.0]
    vector[axis] = math.sin(angle / 2)
    return [math.cos(angle / 2), *vector]


def rotation_rpy(r, p, y):
    return qmul(qmul(axis_quaternion(2, y), axis_quaternion(1, p)), axis_quaternion(0, r))


def rotate(q, v):
    w, x, y, z = q
    r = qmul(qmul(q, [0.0, *v]), [w, -x, -y, -z])
    return r[1:]


def core_in_base(core_in_world, parking, yaw):
    """Transform a fixed field pose into the rotated, translated arm base frame."""
    inverse = axis_quaternion(2, -yaw)
    return {'position_m': rotate(inverse, [core_in_world['position_m'][i] - parking[i] for i in range(3)]),
            'quaternion_wxyz': qmul(inverse, core_in_world['quaternion_wxyz'])}


def case_command(executable, case, timeout, directory, gui=False, assembly=None):
    """Build the same frame-aware command for batch, replay, and browser."""
    pose = case['core_in_base']
    command = [str(executable), *map(str, pose['position_m']), *map(str, pose['quaternion_wxyz']),
               str(timeout), str(directory), '1' if gui else '0']
    parking = case.get('parking_offset_world_m', [0.0, 0.0, 0.0])
    yaw = case.get('parking_yaw_world_rad', 0.0)
    if assembly is not None:
        origin = rotate(axis_quaternion(2, -yaw),
                        [assembly['position_m'][i] - parking[i] for i in range(3)])
        r, p, y = assembly['rpy_rad']
        command.extend(map(str, [*origin, r, p, y - yaw, *parking]))
    elif any(parking) or yaw:
        command.extend(map(str, parking))
    if yaw:
        command.append(str(yaw))
    command.append('--j1-max-from-vertical-rad=' + str(case.get('j1_max_from_vertical_rad', math.pi / 2)))
    if not case.get('check_self_collision', True):
        command.append('--disable-self-collision')
    if not case.get('check_gripper_pillar_collision', True):
        command.append('--disable-gripper-pillar-collision')
    if case.get('arm_model'):
        command.append('--arm-model-path=' + case['arm_model']['model_path'])
        if case['arm_model'].get('attach_link', 'j6_link') != 'j6_link':
            command.append('--arm-attach-link=' + case['arm_model']['attach_link'])
        if case['arm_model'].get('model_family', 'rm26') != 'rm26':
            command.append('--arm-family=' + case['arm_model']['model_family'])
    return command


def sample_cases(config):
    """Generate one fixed LHS: each dimension visits each stratum exactly once."""
    check_config(config)
    count = config['samples']
    rng = random.Random(config['seed'])
    columns = []
    for field in FIELDS:
        lower, upper = config['core_bounds'][field]
        bins = list(range(count))
        rng.shuffle(bins)
        columns.append([lower + (upper-lower) * (index + rng.random()) / count for index in bins])
    assembly = config['assembly_in_base']
    origin = assembly['position_m']
    parking = config.get('parking_offset_world_m', [0.0, 0.0, 0.0])
    aq = rotation_rpy(*assembly['rpy_rad'])
    yaw = config.get('parking_yaw_world_rad', 0.0)
    quick = {i * count // config['quick_samples'] for i in range(config['quick_samples'])}
    cases = []
    for index in range(count):
        values = [col[index] for col in columns]
        x, y, z, theta, phi, alpha = values
        offset = rotate(aq, [x, y, z])
        # 规则方位定义：Rz(theta) Ry(phi - pi/2) Rx(alpha)。
        core_q = rotation_rpy(alpha, phi - math.pi / 2, theta)
        field_pose = {'position_m': [origin[i] + offset[i] for i in range(3)],
                      'quaternion_wxyz': qmul(aq, core_q)}
        cases.append({'id': f'case_{index:04d}', 'quick': index in quick,
                      'core_in_assembly': dict(zip(FIELDS, values)),
                      **({'arm_model': config['arm_model']} if config.get('arm_model') else {}),
                      **({'start_joints_rad': config['start_joints_rad']} if 'start_joints_rad' in config else {}),
                      'parking_offset_world_m': list(parking), 'parking_yaw_world_rad': yaw,
                      'j1_max_from_vertical_rad': config.get('j1_max_from_vertical_rad', math.pi / 2),
                      'check_self_collision': config.get('check_self_collision', True),
                      'check_gripper_pillar_collision': config.get('check_gripper_pillar_collision', True),
                      'core_in_world': field_pose,
                      'core_in_base': core_in_base(field_pose, parking, yaw)})
    positions = parking_positions(config)
    if len(positions) == 1:
        return adapt_y_parking(config, cases)
    expanded = []
    for position in positions:
        for case in cases:
            parking = position['offset_world_m']
            expanded.append({**case, 'id': f"{case['id']}_{position['id']}",
                             'sample_id': case['id'], 'parking_id': position['id'],
                             'parking_grid_index': position['grid_index'],
                             'parking_offset_world_m': list(parking),
                             'core_in_base': core_in_base(case['core_in_world'], parking, yaw)})
    return adapt_y_parking(config, expanded)


def adapt_y_parking(config, cases):
    """Follow core field-world Y in metres, retaining X/Z and parking yaw."""
    if config.get('adaptive_y_parking', False):
        for case in cases:
            parking = case['parking_offset_world_m']
            parking[1] += case['core_in_world']['position_m'][1]
            finite_vector(parking, 3, 'adaptive parking offset')
            case['core_in_base'] = core_in_base(
                case['core_in_world'], parking, case['parking_yaw_world_rad'])
    return cases


def annotate_arm_result(result, case):
    identity = case.get('arm_model')
    if identity:
        reported = result.get('arm_model_path')
        loaded = reported is not None and Path(reported).resolve() == Path(identity['model_path']).resolve()
        if result.get('success') and not loaded:
            raise ValueError('Solver did not confirm requested arm model path')
        if reported is not None and not loaded:
            raise ValueError('Solver loaded a different arm model')
        if reported is not None and result.get('arm_model_family', 'rm26') != identity.get('model_family', 'rm26'):
            raise ValueError('Solver used a different arm model family')
        if reported is not None and 'start_joints_rad' in case:
            expected = case['start_joints_rad']
            actual = result.get('start_joints_rad')
            if (not isinstance(actual, list) or len(actual) != len(expected) or
                    any(not isinstance(a, (int, float)) or not math.isfinite(a) or
                        not math.isclose(a, b, rel_tol=0.0, abs_tol=1e-10)
                        for a, b in zip(actual, expected))):
                raise ValueError('Solver used a different initial joint configuration')
        if identity.get('attach_link', 'j6_link') != 'j6_link' and reported is not None:
            if result.get('arm_attach_link') != identity['attach_link']:
                raise ValueError('Solver used a different terminal mounting body')
        result['arm_model'] = identity
        result['arm_model_loaded'] = loaded
    return result


def call_case(executable, case, timeout, directory, gui=False, assembly=None, config=None):
    if case.get('arm_model'):
        verify_arm(case['arm_model'])
    command = case_command(executable, case, timeout, directory, gui=gui, assembly=assembly)
    save_case_snapshot(directory, case, config, command)
    try:
        start = time.monotonic()
        run = subprocess.run(command, text=True, capture_output=True,
                             timeout=None if gui else timeout + 120, check=False)
        if run.returncode:
            result = {'success': False, 'status': 'ProcessError', 'message': run.stderr.strip(),
                    'last_failure': 'NotRun', 'candidate_last_phase': 'unknown',
                    'process_s': time.monotonic()-start, 'returncode': run.returncode}
        else:
            result = json.loads(run.stdout.partition('\n')[0])
            result['process_s'] = time.monotonic()-start
            result['stderr'] = run.stderr
    except subprocess.TimeoutExpired:
        result = {'success': False, 'status': 'ProcessTimeout', 'message': 'case process exceeded external timeout',
                'last_failure': 'NotRun', 'candidate_last_phase': 'unknown', 'process_s': timeout + 120}
    except (OSError, ValueError) as error:
        result = {'success': False, 'status': 'ProcessError', 'message': str(error),
                'last_failure': 'NotRun', 'candidate_last_phase': 'unknown'}

    annotate_arm_result(result, case)
    write_json(directory / 'result.json', result)
    return result


def summarize(rows):
    counts = {}
    failure_hints = {}
    times = []
    for row in rows:
        result = row['result']
        category = result.get('status', 'ProcessError')
        counts[category] = counts.get(category, 0) + 1
        if not result.get('success'):
            hint = failure_hint(result)
            failure_hints[hint] = failure_hints.get(hint, 0) + 1
        if isinstance(result.get('planning_s'), (int, float)):
            times.append(result['planning_s'])
    successes = sum(bool(row['result'].get('success')) for row in rows)
    return {'requests': len(rows), 'successes': successes,
            'success_rate': successes / len(rows) if rows else None,
            'status_counts': counts,
            'failure_hints': failure_hints,
            'planning_time_s': {'count': len(times), 'mean': statistics.mean(times) if times else None,
                                'median': statistics.median(times) if times else None,
                                'max': max(times) if times else None}}


def failure_hint(result):
    """Messages are unstructured; these categories are only diagnostic hints."""
    status = result.get('status', 'ProcessError')
    message = (result.get('last_failure_message') or result.get('message') or '').lower()
    if status == 'StartCollision':
        return 'start_collision_or_limits'
    if status in ('Timeout', 'ProcessTimeout', 'ProcessError', 'InvalidRequest', 'ModelError'):
        return status
    if 'required q coverage incomplete' in message:
        return 'q_incomplete'
    if 'collision' in message:
        return 'collision'
    if status == 'IKFailed' or 'ik ' in message:
        return 'ik_failed'
    return status


def save_report(directory, cases, config, results):
    rows = [{'case': case, 'result': results[case['id']]} for case in cases if case['id'] in results]
    write_json(directory / 'results.json', rows)
    summary = summarize(rows)
    if len(parking_positions(config)) > 1:
        summary['parking_results'] = [
            {**position, **summarize([row for row in rows
                                     if row['case'].get('parking_id') == position['id']])}
            for position in parking_positions(config)]
    summary['arm_model'] = config.get('arm_model', {'label': '原始（历史批次未记录模型指纹）'})
    summary['progress'] = progress_statistics(cases, results)
    write_json(directory / 'summary.json', summary)
    write_json(directory / 'progress_statistics.json', summary['progress'])
    (directory / 'progress_statistics.log').write_text('Arm: ' + json.dumps(summary['arm_model'], ensure_ascii=False) + '\n' + progress_log(summary['progress']), encoding='utf-8')
    with (directory / 'results.csv').open('w', encoding='utf-8', newline='') as stream:
        keys = ['id', 'quick', *FIELDS, 'base_x_m', 'base_y_m', 'base_z_m', 'base_qw', 'base_qx', 'base_qy', 'base_qz',
                'success', 'status', 'message', 'last_failure', 'last_failure_message', 'candidate_last_phase',
                'planning_s', 'q_min_rad', 'q_max_rad', 'failure_stage',
                'q_negative_reached_rad', 'q_negative_failed_target_rad', 'q_negative_failure_status', 'q_negative_failure_message',
                'q_positive_reached_rad', 'q_positive_failed_target_rad', 'q_positive_failure_status', 'q_positive_failure_message',
                'sample_id', 'parking_id', 'parking_x_m', 'parking_y_m', 'parking_z_m',
                'arm_id', 'arm_label', 'arm_fingerprint_sha256']
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        for row in rows:
            case, result = row['case'], row['result']
            pose = case['core_in_base']
            q = result.get('q_coverage_rad') or [None, None]
            record = {'id': case['id'], 'quick': case['quick'], **case['core_in_assembly'],
                      **dict(zip(keys[8:11], pose['position_m'])), **dict(zip(keys[11:15], pose['quaternion_wxyz'])),
                      **{key: result.get(key) for key in keys[15:23]}, 'q_min_rad': q[0], 'q_max_rad': q[1]}
            record['failure_stage'] = result.get('failure_stage')
            diagnostic = result.get('q_diagnostic') or {}
            for side in ('negative', 'positive'):
                for field in ('reached_rad', 'failed_target_rad', 'failure_status', 'failure_message'):
                    record[f'q_{side}_{field}'] = (diagnostic.get(side) or {}).get(field)
            record.update(sample_id=case.get('sample_id', case['id']), parking_id=case.get('parking_id', 'p00'))
            record.update(zip(('parking_x_m', 'parking_y_m', 'parking_z_m'),
                              case.get('parking_offset_world_m', [0.0, 0.0, 0.0])))
            identity = case.get('arm_model', {})
            record.update(arm_id=identity.get('id', 'original-legacy'), arm_label=identity.get('label', '原始（历史未记录）'),
                          arm_fingerprint_sha256=identity.get('fingerprint_sha256', ''))
            writer.writerow(record)


STAGES = {'Request': '请求检查', 'Start': '起点检查', 'TaskSetup': '任务生成',
          'PostureSearch': '插入', 'Approach': '插入', 'Preinsert': '插入',
          'Insert': '插入', 'Lift': '平移', 'Flip': 'P 上翻', 'Twist': 'Q 旋转',
          'Timing': '轨迹时间化', 'Validation': '最终验收'}


# Scores describe completed milestones, never merely an attempted phase.
CASE_STYLES = {
    'pending': ('未运行', '#89939e'),
    'score0': ('0分 · 未完成插入', '#d73027'),
    'score1': ('1分 · 已完成插入', '#f28e2b'),
    'score2': ('2分 · 已完成平移', '#f2cf43'),
    'score3': ('3分 · 已完成P上翻', '#8bc34a'),
    'success': ('4分 · 全部完成', '#168b45'),
    'unknown': ('进度未知', '#555e68'),
}


def case_score(result):
    """Infer completed milestones from an explicit failure stage; unknown stays unknown."""
    if result is None:
        return None
    if result.get('success'):
        return 4
    return {'Request': 0, 'Start': 0, 'TaskSetup': 0, 'PostureSearch': 0,
            'Approach': 0, 'Preinsert': 0, 'Insert': 0, 'Lift': 1,
            'Flip': 2, 'Twist': 3, 'Timing': 3, 'Validation': 3}.get(failure_stage(result))


def case_style(result):
    if result is None:
        return CASE_STYLES['pending']
    score = case_score(result)
    key = 'success' if score == 4 else ('unknown' if score is None else f'score{score}')
    return CASE_STYLES[key]


FAILURE_REASONS = {
    'unreachable': '不可达（迭代超时/预算不足）',
    'unit_arm_collision': '能量单元与机械臂碰撞',
    'arm_core_collision': '机械臂与科技核心碰撞',
    'other_collision': '其他碰撞',
    'other': '其他失败',
}


def failure_reason(result):
    """Classify reported failure only; timeout/budget takes precedence over stale candidates."""
    if result is None or result.get('success'):
        return None
    primary = str(result.get('message') or '').lower()
    message = str(result.get('last_failure_message') or result.get('message') or '').lower()
    def exhausted(text):
        return any(word in text for word in ('timeout', 'timed out', 'time budget',
            'iteration limit', 'iteration budget', 'maximum iterations', 'max iterations',
            'budget exhausted', 'budget exceeded', 'insufficient budget', '预算不足', '迭代超时'))
    if result.get('status') in ('Timeout', 'ProcessTimeout') or exhausted(primary) or exhausted(message):
        return 'unreachable'
    pairs = re.findall(r'([a-z0-9_./-]+)\s*\|\s*([a-z0-9_./-]+)', message)
    def role(name):
        if name.startswith('tech_core_planner_unit/'):
            return 'unit'
        if name.startswith('tech_core_planner_core/'):
            return 'core'
        if re.match(r'(?:.*/)?(?:j[0-6]_link|link_[1-7])(?:_|$)', name):
            return 'arm'
        return 'other'
    for a, b in pairs:
        roles = {role(a), role(b)}
        if roles == {'unit', 'arm'}:
            return 'unit_arm_collision'
        if roles == {'arm', 'core'}:
            return 'arm_core_collision'
    if 'collision' in message or result.get('status') == 'StartCollision':
        return 'other_collision'
    return 'other'


def failure_reason_statistics(cases, results):
    def counts(items):
        output = {key: 0 for key in FAILURE_REASONS}
        for case in items:
            reason = failure_reason(results.get(case['id']))
            if reason:
                output[reason] += 1
        return output
    return {'counts': counts(cases), 'by_parking': {
        pid: counts([c for c in cases if c.get('parking_id', 'p00') == pid])
        for pid in sorted({c.get('parking_id', 'p00') for c in cases})}}


def progress_statistics(cases, results):
    """Score observed progress and count parking alternatives per fixed world task."""
    parking, tasks = {}, {}
    for case in cases:
        pid = case.get('parking_id', 'p00')
        sid = case.get('sample_id', case['id'])
        group = parking.setdefault(pid, {'parking_id': pid,
            'position_m': case.get('parking_offset_world_m', [0, 0, 0]),
            'scores': [], 'pending': 0, 'unknown': 0})
        task = tasks.setdefault(sid, {'sample_id': sid, 'qualifying_parking_count': 0,
                                     'scored_parking_count': 0, 'pending': 0, 'unknown': 0})
        result = results.get(case['id'])
        score = case_score(result)
        if score is None:
            key = 'pending' if result is None else 'unknown'
            group[key] += 1
            task[key] += 1
        else:
            group['scores'].append(score)
            task['scored_parking_count'] += 1
            task['qualifying_parking_count'] += int(score >= 3)
    for group in parking.values():
        scores = group.pop('scores')
        group['scored_count'] = len(scores)
        group['mean_score'] = statistics.mean(scores) if scores else None
        group['score_counts'] = {str(i): scores.count(i) for i in range(5)}
    histogram = {str(i): 0 for i in range(len(parking) + 1)}
    for task in tasks.values():
        task['complete'] = task['scored_parking_count'] == len(parking)
        if task['complete']:
            histogram[str(task['qualifying_parking_count'])] += 1
    complete = [t for t in tasks.values() if t['complete']]
    return {'score_definition': '0 before insertion; 1 inserted; 2 translated; 3 P flip complete; 4 accepted',
            'failure_reasons': failure_reason_statistics(cases, results),
            'qualifying_scores': [3, 4], 'parking_results': list(parking.values()),
            'mean_parking_score': statistics.mean([g['mean_score'] for g in parking.values()
                if g['mean_score'] is not None]) if any(g['mean_score'] is not None for g in parking.values()) else None,
            'tasks': list(tasks.values()), 'complete_task_count': len(complete),
            'incomplete_task_count': len(tasks) - len(complete),
            'qualifying_parking_histogram': histogram,
            'mean_qualifying_parking_count': statistics.mean([t['qualifying_parking_count']
                for t in complete]) if complete else None}


def progress_log(stats):
    lines = ['Progress scores: 0..4; qualifying parking positions: score >= 3',
             f"Task score (parking means): {stats['mean_parking_score']}"]
    for group in stats['parking_results']:
        lines.append(f"{group['parking_id']} {group['position_m']}: mean={group['mean_score']}, "
                     f"scored={group['scored_count']}, pending={group['pending']}, unknown={group['unknown']}")
    lines.append(f"Adaptability score (complete tasks): {stats['mean_qualifying_parking_count']}")
    lines.append(f"Qualifying parking count histogram: {stats['qualifying_parking_histogram']}")
    for task in stats['tasks']:
        lines.append(f"{task['sample_id']}: qualifying={task['qualifying_parking_count']}, "
                     f"scored={task['scored_parking_count']}, complete={task['complete']}")
    lines.append('Failure reasons: ' + json.dumps(stats['failure_reasons'], ensure_ascii=False))
    return '\n'.join(lines) + '\n'


def failure_stage(result):
    """Identify the recorded failure stage, preserving unknowns instead of guessing completion."""
    stage = result.get('failure_stage')
    if stage in STAGES:
        return stage
    if not stage:
        message = result.get('last_failure_message') or result.get('message') or ''
        if 'Required Q coverage incomplete' in message:
            return 'Twist'
        if result.get('status') == 'StartCollision':
            return 'Start'
    return 'Unknown'


def failure_distribution(results):
    """Count failed cases by stage; successful and unrun cases are excluded from percentages."""
    counts = {}
    for result in results:
        if result is not None and not result.get('success'):
            stage = failure_stage(result)
            if stage in ('PostureSearch', 'Approach', 'Preinsert'):
                stage = 'Insert'
            counts[stage] = counts.get(stage, 0) + 1
    total = sum(counts.values())
    colors = ['#8e2424', '#b43c3c', '#c05e46', '#a94d1a', '#b56820', '#cc8735',
              '#967414', '#167e88', '#3769b0', '#8053b0', '#ad427e', '#cb638f', '#555e68']
    return [(stage, STAGES.get(stage, '阶段未知'), counts[stage], counts[stage] / total, color)
            for stage, color in zip([*STAGES, 'Unknown'], colors) if stage in counts]


def parking_distributions(cases, results):
    """Group saved cases by parking point; fractions include successful completed cases."""
    groups = {}
    for case in cases:
        key = case.get('parking_id', 'p00')
        group = groups.setdefault(key, {'id': key, 'offset_world_m': case.get(
            'parking_offset_world_m', [0.0, 0.0, 0.0]), 'results': []})
        group['results'].append(results.get(case['id']))
    output = []
    for group in groups.values():
        rows = group.pop('results')
        successful = sum(bool(row and row.get('success')) for row in rows)
        pending = sum(row is None for row in rows)
        failed = len(rows) - pending - successful
        completed = successful + failed
        distribution = []
        counts = {}
        for row in rows:
            if row is not None:
                style = case_style(row)
                counts[style] = counts.get(style, 0) + 1
        for key, (label, color) in CASE_STYLES.items():
            count = counts.get((label, color), 0)
            if count:
                distribution.append(('Success' if key == 'success' else key,
                                     label, count, count / completed, color))
        output.append({**group, 'successes': successful, 'failures': failed,
                       'pending': pending, 'distribution': distribution})
    return output


def describe_result(result):
    """Describe only recorded or explicitly identifiable failure stages."""
    if result is None:
        return '尚无批次结果', '尚未运行'
    if result.get('success'):
        return '成功', '完整任务已通过最终验收'
    stage = result.get('failure_stage')
    message = result.get('last_failure_message') or result.get('message') or ''
    if stage:
        label = STAGES.get(stage, stage)
    elif 'Required Q coverage incomplete' in message:
        label = 'Q 旋转（根据原始消息判断）'
    elif result.get('status') == 'StartCollision':
        label = '起点检查（根据状态判断）'
    else:
        label = '未知（旧结果未记录失败步骤）'
    details = f"状态：{result.get('status', '未知')}\n原因：{message or '未提供'}"
    details += '\n失败分类：' + FAILURE_REASONS[failure_reason(result)]
    collision = result.get('collision_diagnostic')
    if collision:
        found = '已找到忽略碰撞的解' if collision.get('solution_found') else '忽略碰撞后仍未完成任务，显示已有候选'
        details += f"\n碰撞诊断（未验收）：{found}\n三维窗口 C 切换碰撞体，红色为违规块；←/→ 逐点，N/B 跳转碰撞点。"
    diagnostic = result.get('q_diagnostic') or {}
    for side, name in (('negative', '负向'), ('positive', '正向')):
        value = diagnostic.get(side)
        if value:
            target = value.get('failed_target_rad')
            details += f"\nQ {name}：到达 {value['reached_rad']:.4f} rad"
            if target is not None:
                details += f"；失败目标 {target:.4f} rad，{value.get('failure_message', '')}"
    return '失败步骤：' + label, details


def browse_batch(batch, executable):
    """Show all batch cases and launch a separate replanning viewer on click."""
    import tkinter as tk
    from tkinter import ttk
    batch = batch.resolve()
    config = json.loads((batch / 'config.json').read_text(encoding='utf-8'))
    cases = json.loads((batch / 'cases.json').read_text(encoding='utf-8'))
    rows = json.loads((batch / 'results.json').read_text(encoding='utf-8'))
    results = {row['case']['id']: row['result'] for row in rows}
    events = queue.Queue()
    identity = config.get('arm_model')
    if identity:
        verify_arm(identity)
        if any(c.get('arm_model') != identity for c in cases):
            raise ValueError('Batch case arm identity mismatch')
    arm_label = identity['label'] if identity else '原始（历史批次未记录模型指纹）'
    root = tk.Tk()
    root.title('三级任务案例浏览 · ' + arm_label)
    window_height = int(root.winfo_screenheight() * 0.8)
    root.geometry(f'1100x{window_height}')
    root.minsize(850, min(550, window_height))
    root.columnconfigure(0, weight=1, uniform='halves')
    root.columnconfigure(1, weight=1, uniform='halves')
    root.rowconfigure(2, weight=1)
    heading = tk.Label(root, text='三级任务案例  |  颜色表示已完成进度：0 → 4分',
                       font=('Sans', 15, 'bold'), anchor='w')
    heading.grid(row=0, column=0, columnspan=2, sticky='ew', padx=14, pady=10)
    legend = ttk.Frame(root)
    legend.grid(row=1, column=0, columnspan=2, sticky='ew', padx=14, pady=(0, 8))
    for index, (label, color) in enumerate(CASE_STYLES.values()):
        tk.Label(legend, text='  ' + label + '  ', bg=color, fg='#17202a', font=('Sans', 10)).grid(
            row=index // 5, column=index % 5, sticky='ew', padx=2, pady=2)
    for index in range(5):
        legend.columnconfigure(index, weight=1, uniform='legend')
    left = ttk.Frame(root)
    left.grid(row=2, column=0, sticky='nsew', padx=(12, 6), pady=4)
    canvas = tk.Canvas(left, width=1, highlightthickness=0)
    scroll = ttk.Scrollbar(left, orient='vertical', command=canvas.yview)
    canvas.configure(yscrollcommand=scroll.set)
    canvas.pack(side='left', fill='both', expand=True)
    scroll.pack(side='right', fill='y')
    case_list = ttk.Frame(canvas)
    grid_window = canvas.create_window((0, 0), window=case_list, anchor='nw')
    case_list.bind('<Configure>', lambda event: canvas.configure(scrollregion=canvas.bbox('all')))
    def wheel(event):
        target = detail_canvas if event.x_root >= detail_canvas.winfo_rootx() else canvas
        target.yview_scroll(int(-event.delta / 120), 'units')

    root.bind('<MouseWheel>', wheel)
    detail_container = ttk.Frame(root)
    detail_container.grid(row=2, column=1, sticky='nsew')
    toolbar = ttk.Frame(detail_container, padding=(12, 6))
    toolbar.pack(side='top', fill='x')
    copy_button = ttk.Button(toolbar, text='复制案例编号和路径', state='disabled', command=lambda: copy_case())
    copy_button.pack(side='left')
    copy_status = tk.Label(toolbar, text='', anchor='w', fg='#555555')
    copy_status.pack(side='left', padx=8)
    detail_canvas = tk.Canvas(detail_container, width=1, highlightthickness=0)
    detail_scroll = ttk.Scrollbar(detail_container, orient='vertical', command=detail_canvas.yview)
    detail_canvas.configure(yscrollcommand=detail_scroll.set)
    detail_scroll.pack(side='right', fill='y')
    detail_canvas.pack(side='left', fill='both', expand=True)
    panel = ttk.Frame(detail_canvas, padding=14)
    detail_window = detail_canvas.create_window((0, 0), window=panel, anchor='nw')
    panel.bind('<Configure>', lambda event: detail_canvas.configure(scrollregion=detail_canvas.bbox('all')))
    detail_canvas.bind('<Configure>', lambda event: detail_canvas.itemconfigure(detail_window, width=event.width))
    detail_canvas.bind('<Button-4>', lambda event: detail_canvas.yview_scroll(-1, 'units'))
    detail_canvas.bind('<Button-5>', lambda event: detail_canvas.yview_scroll(1, 'units'))
    groups = parking_distributions(cases, results)
    successful = sum(group['successes'] for group in groups)
    failed = sum(group['failures'] for group in groups)
    pending = sum(group['pending'] for group in groups)
    stats_heading = tk.Label(panel, text=f'原批次统计：成功 {successful} / 失败 {failed} / 未运行 {pending}',
                             font=('Sans', 12, 'bold'), anchor='w')
    stats_heading.pack(fill='x')

    tk.Label(panel, text='机械臂：' + arm_label, font=('Sans', 20, 'bold'),
             fg='#173b57', anchor='w', wraplength=650).pack(fill='x', pady=8)
    if identity:
        tk.Label(panel, text='模型指纹：' + identity['fingerprint_sha256'][:16], anchor='w').pack(fill='x')
    progress = progress_statistics(cases, results)
    def number(value):
        return '暂无数据' if value is None else f'{value:.2f}'
    metrics = tk.Frame(panel, bg='#eef4f8', padx=14, pady=12)
    metrics.pack(fill='x', pady=10)
    tk.Label(metrics, text=f"任务分  {number(progress['mean_parking_score'])} / 4",
             font=('Sans', 24, 'bold'), fg='#173b57', bg='#eef4f8', anchor='w').pack(fill='x')
    tk.Label(metrics, text=f"适应分  {number(progress['mean_qualifying_parking_count'])} / {len(groups)}",
             font=('Sans', 24, 'bold'), fg='#087c36', bg='#eef4f8', anchor='w').pack(fill='x', pady=(8, 0))
    tk.Label(panel, text=f"任务分：各停车点等权平均；适应分：满足3/4分的停车点数均值\n"
             f"完整任务 {progress['complete_task_count']}；缺测/未知任务 {progress['incomplete_task_count']}\n"
             f"满足停车点数 → 任务数：{progress['qualifying_parking_histogram']}",
             anchor='w', justify='left', wraplength=650).pack(fill='x', pady=8)
    heat = tk.Canvas(panel, height=220, highlightthickness=0, background='#f7f8fa')
    heat.pack(fill='x', pady=8)
    def draw_heat(event):
        heat.delete('all')
        data = progress['parking_results']
        xs = sorted({g['position_m'][0] for g in data})
        ys = sorted({g['position_m'][1] for g in data})
        width = max(40, (event.width - 70) / len(xs))
        height = 160 / len(ys)
        heat.create_text(8, 12, anchor='w', text='停车位置任务分热区图（世界系 X/Y，单位m；色标0红→4绿）')
        palette = [CASE_STYLES[k][1] for k in ('score0', 'score1', 'score2', 'score3', 'success')]
        for g in data:
            x = 65 + xs.index(g['position_m'][0]) * width
            y = 30 + (len(ys) - 1 - ys.index(g['position_m'][1])) * height
            mean = g['mean_score']
            color = '#89939e'
            if mean is not None:
                lo = min(3, int(mean))
                weight = mean - lo
                left, right = palette[lo], palette[lo + 1]
                channels = [round(int(left[j:j+2], 16) * (1-weight) + int(right[j:j+2], 16) * weight)
                            for j in (1, 3, 5)]
                color = '#' + ''.join(f'{v:02x}' for v in channels)
            heat.create_rectangle(x, y, x+width, y+height, fill=color, outline='white')
            heat.create_text(x+width/2, y+height/2, text=f"{g['parking_id']}  {number(mean)}\n"
                             f"n={g['scored_count']}  Z={g['position_m'][2]:g}")
        for i, x in enumerate(xs):
            heat.create_text(65+(i+.5)*width, 205, text=f'X={x:g}')
        for i, y in enumerate(ys):
            heat.create_text(30, 30+(len(ys)-i-.5)*height, text=f'Y={y:g}')
    heat.bind('<Configure>', draw_heat)
    reasons = progress['failure_reasons']
    reason_colors = ['#7c6bb3', '#e68a36', '#3d82b8', '#ca697b', '#8b949e']
    reason_labels = ['不可达（超时/预算）', '能量单元与机械臂', '机械臂与科技核心', '其他碰撞', '其他失败']
    reason_chart = tk.Canvas(panel, height=310, highlightthickness=0, bg='#f7f8fa')
    reason_chart.pack(fill='x', pady=10)
    def draw_reason_totals(event):
        reason_chart.delete('all')
        reason_chart.create_text(12, 18, text='失败原因 · 总统计', anchor='w', font=('Sans', 13, 'bold'))
        values = list(reasons['counts'].values())
        maximum = max(max(values), 1)
        left, width = 165, max(30, event.width - 215)
        for i, (label, value, color) in enumerate(zip(reason_labels, values, reason_colors)):
            y = 55 + i * 44
            reason_chart.create_text(left - 10, y + 11, text=label, anchor='e', font=('Sans', 10))
            right = left + width * value / maximum
            if value:
                reason_chart.create_rectangle(left, y, right, y + 23, fill=color, outline='', tags=('reason_bar',))
            reason_chart.create_text(right + 6, y + 11, text=str(value), anchor='w', font=('Sans', 10, 'bold'))
        reason_chart.create_text(12, 292, text='不可达按本次搜索超时/预算耗尽归类；进度分数保留。', anchor='w', font=('Sans', 9))
    reason_chart.bind('<Configure>', draw_reason_totals)
    parking_reason_chart = tk.Canvas(panel, height=90 + 38 * len(reasons['by_parking']),
                                     highlightthickness=0, bg='#f7f8fa')
    parking_reason_chart.pack(fill='x', pady=8)
    def draw_parking_reasons(event):
        chart = parking_reason_chart
        chart.delete('all')
        chart.create_text(12, 18, text='失败原因 · 各停车点（同色对应上图）', anchor='w', font=('Sans', 12, 'bold'))
        maximum = max([sum(c.values()) for c in reasons['by_parking'].values()] + [1])
        width = max(30, event.width - 120)
        for i, (pid, counts) in enumerate(reasons['by_parking'].items()):
            y = 48 + 38 * i
            chart.create_text(12, y + 12, text=pid, anchor='w')
            x = 60
            for value, color in zip(counts.values(), reason_colors):
                span = width * value / maximum
                if value:
                    chart.create_rectangle(x, y, x + span, y + 25, fill=color, outline='white', tags=('parking_reason_bar',))
                    chart.create_text(x + span/2, y + 12, text=str(value), fill='#17202a', font=('Sans', 9, 'bold'))
                x += span
            chart.create_text(x + 6, y + 12, text=str(sum(counts.values())), anchor='w')
    parking_reason_chart.bind('<Configure>', draw_parking_reasons)
    task_text = '\n'.join(f"{t['sample_id']}: {t['qualifying_parking_count']}/{len(groups)}"
                          + ('' if t['complete'] else '（缺测/未知，暂计）') for t in progress['tasks'])
    task_box = tk.Text(panel, height=5, wrap='word')
    task_box.insert('1.0', '逐任务适应分（满足3/4分的停车点数）：\n' + task_text)
    task_box.config(state='disabled')
    task_box.pack(fill='x', pady=6)

    def add_parking_pie(group):
        distribution = group['distribution']
        xyz = ', '.join(f'{x:g}' for x in group['offset_world_m'])
        tk.Label(panel, text=f"{group['id']} ({xyz}) m：成功 {group['successes']} / "
                 f"失败 {group['failures']} / 未运行 {group['pending']}"
                 f" / 任务分 {number(next(g for g in progress['parking_results'] if g['parking_id'] == group['id'])['mean_score'])}",
                 anchor='w', justify='left', font=('Sans', 10)).pack(fill='x', pady=6)
        pie = tk.Canvas(panel, width=1, height=max(190, len(distribution) * 23 + 35),
                        highlightthickness=0, background='#f7f8fa')
        pie.pack(fill='x', pady=(6, 14))

        def draw_pie(event):
            pie.delete('all')
            pie.create_text(10, 12, text='已完成进度占比（分母：本停车点已运行案例）',
                            anchor='w', font=('Sans', 10, 'bold'))
            diameter = min(150, max(90, event.width * 0.4))
            bounds = (8, 32, 8 + diameter, 32 + diameter)
            start = 90.0
            for index, (stage, label, count, fraction, color) in enumerate(distribution):
                tags = ('result_slice', 'success_slice' if stage == 'Success' else 'failure_slice', stage)
                if len(distribution) == 1:
                    pie.create_oval(*bounds, fill=color, outline='white', tags=tags)
                else:
                    pie.create_arc(*bounds, start=start, extent=-360 * fraction, fill=color,
                                   outline='white', tags=tags)
                start -= 360 * fraction
                x, y = diameter + 23, 38 + index * 23
                pie.create_rectangle(x, y - 5, x + 10, y + 5, fill=color, outline='')
                pie.create_text(x + 16, y, text=f'{label}  {count}例 / {fraction:.1%}',
                                anchor='w', font=('Sans', 9), tags=('result_legend',))
            if not distribution:
                pie.create_oval(*bounds, fill='#e5e9ed', outline='white')
                pie.create_text(8 + diameter / 2, 32 + diameter / 2, text='暂无已运行案例',
                                font=('Sans', 10), tags=('no_results',))

        pie.bind('<Configure>', draw_pie)

    for group in groups:
        add_parking_pie(group)
    title = tk.Label(panel, text='点击左侧案例，启动重算与三维预览', font=('Sans', 17, 'bold'), anchor='w')
    title.pack(fill='x', pady=(0, 12))
    original_title = tk.Label(panel, text='原批次结果', font=('Sans', 13, 'bold'), anchor='w')
    original_title.pack(fill='x')
    original_stage = tk.Label(panel, text='请选择案例', font=('Sans', 19, 'bold'), fg='#087c36', anchor='w', wraplength=700, justify='left')
    original_stage.pack(fill='x', pady=5)
    original_details = tk.Label(panel, text='', font=('Sans', 11), anchor='w', justify='left', wraplength=700)
    original_details.pack(fill='x', pady=(0, 22))
    tk.Label(panel, text='本次重算结果', font=('Sans', 13, 'bold'), anchor='w').pack(fill='x')
    current_stage = tk.Label(panel, text='尚未重算', font=('Sans', 19, 'bold'), fg='#087c36', anchor='w', wraplength=700, justify='left')
    current_stage.pack(fill='x', pady=5)
    current_details = tk.Label(panel, text='', font=('Sans', 11), anchor='w', justify='left', wraplength=700)
    current_details.pack(fill='x')
    footer = tk.Label(panel, text='三维窗口为重新规划的预览；结果可能与原批次不同。\nI/K、J/L、U/O 移动核心，Shift 配合旋转；P 重算。\n进度按已完成阶段计分；失败详情保留原始失败步骤。',
                      font=('Sans', 10), fg='#555555', anchor='w', justify='left', wraplength=700)
    footer.pack(side='bottom', fill='x')
    state = {'active': False, 'case': None, 'preview': None, 'pending': None}

    def copy_case():
        if state['case'] is None:
            return
        case_path = (batch / 'trajectories' / state['case']).resolve()
        try:
            root.clipboard_clear()
            root.clipboard_append(f"case_id: {state['case']}\ncase_path: {case_path}\n")
            root.update_idletasks()
            copy_status.config(text='已复制', fg='#168b45')
        except tk.TclError as error:
            copy_status.config(text='复制失败', fg='#b43c3c')
            current_details.config(text=str(error))

    def show(stage_widget, detail_widget, result):
        stage, details = describe_result(result)
        stage_widget.config(text=stage, fg=case_style(result)[1])
        detail_widget.config(text=details)

    def signal_preview(preview, sig):
        process = preview.get('process') if preview is not None else None
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                pass

    def stop_preview():
        preview = state['preview']
        if preview is not None:
            preview['cancel'].set()
            signal_preview(preview, signal.SIGTERM)
            timer = threading.Timer(1.5, signal_preview, args=(preview, signal.SIGKILL))
            timer.daemon = True
            timer.start()

    def close():
        state['pending'] = None
        stop_preview()
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', close)

    def worker(case, output, preview):
        cmd = case_command(executable, case, config['timeout_s'], output, gui=True,
                           assembly=config['assembly_in_base'])
        try:
            if case.get('arm_model'):
                verify_arm(case['arm_model'])
            save_case_snapshot(output, case, config, cmd)
            with tempfile.TemporaryFile(mode='w+t', encoding='utf-8') as errors:
                process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=errors, text=True,
                                           start_new_session=True)
                preview['process'] = process
                if preview['cancel'].is_set():
                    signal_preview(preview, signal.SIGTERM)
                    try:
                        process.wait(timeout=1.5)
                    except subprocess.TimeoutExpired:
                        signal_preview(preview, signal.SIGKILL)
                        process.wait()
                    return
                line = ''
                for line in process.stdout:
                    result = json.loads(line)
                    annotate_arm_result(result, case)
                    if not result.get('revision', 0) and not result.get('collision_diagnostic'):
                        write_json(output / 'result.json', result)
                    events.put(('result', result, preview))
                code = process.wait()
                if (not line or code) and not preview['cancel'].is_set():
                    errors.seek(0)
                    events.put(('error', errors.read().strip() or f'进程退出：{code}', preview))
        except (OSError, ValueError) as error:
            if not preview['cancel'].is_set():
                events.put(('error', str(error), preview))
        finally:
            process = preview.get('process')
            if process is not None:
                if process.poll() is None:
                    signal_preview(preview, signal.SIGTERM)
                    try:
                        process.wait(timeout=1.5)
                    except subprocess.TimeoutExpired:
                        signal_preview(preview, signal.SIGKILL)
                        process.wait()
                if process.stdout is not None:
                    process.stdout.close()
            events.put(('closed', None, preview))

    def select(case):
        if state['active']:
            state['pending'] = case
            stop_preview()
            copy_button.config(state='disabled')
            copy_status.config(text='正在切换')
            current_stage.config(text='正在切换案例…', fg='#555555')
            current_details.config(text=f"正在关闭上一预览，即将打开 {case['id']}")
            return
        parking = case.get('parking_offset_world_m', [0.0, 0.0, 0.0])
        caption = f"案例 {case['id']}"
        if 'parking_id' in case:
            caption += '\n停车 XYZ (m): ' + ', '.join(f'{x:g}' for x in parking)
        title.config(text=caption)
        show(original_stage, original_details, results.get(case['id']))
        current_stage.config(text='正在规划…', fg='#555555')
        current_details.config(text='结果将在规划完成后显示，三维窗口可保持打开。')
        try:
            original_path = batch / 'trajectories' / case['id']
            save_case_snapshot(original_path, case, config)
            if case['id'] in results and not (original_path / 'result.json').exists():
                write_json(original_path / 'result.json', results[case['id']])
            state['case'] = case['id']
            copy_button.config(state='normal')
            copy_status.config(text='')
            output = batch / 'replays' / f"{case['id']}_{time.time_ns()}"
            output.mkdir(parents=True, exist_ok=False)
        except OSError as error:
            current_stage.config(text='无法启动', fg='#087c36')
            current_details.config(text=str(error))
            return
        state['active'] = True
        preview = {'cancel': threading.Event(), 'process': None}
        state['preview'] = preview
        threading.Thread(target=worker, args=(case, output, preview), daemon=True).start()

    tiles = []
    for case in cases:
        result = results.get(case['id'])
        label, color = case_style(result)
        identifier = case['id'].replace('case_', '')
        button = tk.Button(case_list, text=f"{identifier}\n{label}", bg=color, fg='#17202a',
                           activebackground=color, activeforeground='#17202a', font=('Sans', 10, 'bold'),
                           wraplength=85, relief='flat', borderwidth=2,
                           command=lambda c=case: select(c))
        tiles.append(button)

    def layout_tiles(event):
        width = max(1, event.width)
        columns = max(1, width // 100)
        cell = width // columns
        for index, button in enumerate(tiles):
            button.place(x=(index % columns) * cell + 3, y=(index // columns) * cell + 3,
                         width=max(1, cell - 6), height=max(1, cell - 6))
            button.config(wraplength=max(1, cell - 12))
        height = max(1, math.ceil(len(tiles) / columns) * cell)
        canvas.itemconfigure(grid_window, width=width, height=height)
        canvas.configure(scrollregion=(0, 0, width, height))

    canvas.bind('<Configure>', layout_tiles)
    canvas.bind('<Button-4>', lambda event: canvas.yview_scroll(-1, 'units'))
    canvas.bind('<Button-5>', lambda event: canvas.yview_scroll(1, 'units'))

    def resize_details(event):
        for widget in (stats_heading, title, original_stage, original_details, current_stage, current_details, footer):
            widget.config(wraplength=max(100, event.width - 32))

    panel.bind('<Configure>', resize_details, add='+')

    def poll():
        try:
            while True:
                kind, value, preview = events.get_nowait()
                if preview is not state['preview']:
                    continue
                if preview['cancel'].is_set() and kind != 'closed':
                    continue
                if kind == 'result':
                    show(current_stage, current_details, value)
                    if value.get('revision', 0):
                        current_details.config(text=current_details.cget('text') +
                                               f"\n核心调整 #{value['revision']}\n保存至：{value['output_directory']}")
                elif kind == 'error':
                    current_stage.config(text='预览启动失败', fg='#087c36')
                    current_details.config(text=value)
                else:
                    state['active'] = False
                    state['preview'] = None
                    pending = state['pending']
                    state['pending'] = None
                    if pending is not None:
                        select(pending)
        except queue.Empty:
            pass
        root.after(100, poll)
    poll()
    root.mainloop()


def installed_config():
    return Path(__file__).absolute().parent.parent / 'share' / 'rm26_level3_validation' / 'config' / 'default.json'


def latest_batch():
    """Find the most recently updated browseable batch in workspace artifacts."""
    roots = {Path.cwd() / 'artifacts'}
    for parent in Path(__file__).absolute().parents:
        if (parent / 'artifacts').is_dir():
            roots.add(parent / 'artifacts')
    candidates = {result.parent.resolve()
                  for root in roots if root.is_dir()
                  for result in root.glob('*/results.json')
                  if all((result.parent / name).is_file()
                         for name in ('config.json', 'cases.json'))}
    if not candidates:
        raise ValueError('未找到历史批次，请使用 --batch 指定包含 config.json、cases.json 和 results.json 的目录')
    return max(candidates, key=lambda path: ((path / 'results.json').stat().st_mtime_ns, str(path)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    run = sub.add_parser('run', help='批量规划')
    run.add_argument('--config', type=Path, default=installed_config())
    run.add_argument('--arm-variant', type=Path, help='机械臂选择/变体JSON，覆盖配置中的arm_variant')
    run.add_argument('--output', type=Path, required=True, help='必须不存在的新目录')
    run.add_argument('--set', choices=['quick', 'full'], default='full')
    run.add_argument('--workers', type=int, default=1)
    replay = sub.add_parser('replay', help='按案例编号重算并打开查看器')
    replay.add_argument('--batch', type=Path, required=True)
    replay.add_argument('--case', required=True)
    browser = sub.add_parser('browse', help='打开批次案例浏览窗口')
    browser.add_argument('--batch', type=Path, help='批次目录；默认打开 artifacts 中最近更新的批次')
    args = parser.parse_args()
    executable = Path(__file__).absolute().parent / 'level3_case'
    if args.command == 'browse':
        selected_batch = args.batch if args.batch is not None else latest_batch()
        print(f'打开批次：{selected_batch.resolve()}', flush=True)
        browse_batch(selected_batch, executable)
    elif args.command == 'run':
        config = json.loads(args.config.read_text(encoding='utf-8'))
        check_config(config)
        variant = json.loads(args.arm_variant.read_text(encoding='utf-8')) if args.arm_variant else config.get('arm_variant', {'id': 'original', 'changes': []})
        config['arm_variant'] = variant
        variant_label(variant)
        if args.workers < 1:
            raise ValueError('workers must be positive')
        family = variant.get('model_family', 'rm26')
        family_option = '--arm-family=' + family
        arm_config = json.loads(subprocess.run([str(executable), '--print-arm-config', family_option,
                                '--j1-max-from-vertical-rad=' + str(config.get('j1_max_from_vertical_rad', math.pi / 2))],
                               capture_output=True, text=True, check=True).stdout)
        source = arm_config['model_path']
        config['start_joints_rad'] = arm_config['start_joints_rad']
        args.output = args.output.resolve()
        args.output.mkdir(parents=True, exist_ok=False)
        config['arm_model'] = generate_arm(Path(source), variant, args.output / 'arm_model')
        verify_arm(config['arm_model'])
        subprocess.run([str(executable), '--check-j1-limits', family_option,
                        '--arm-model-path=' + config['arm_model']['model_path'],
                        '--j1-max-from-vertical-rad=' + str(config.get('j1_max_from_vertical_rad', math.pi / 2))],
                       capture_output=True, text=True, check=True)
        print('机械臂：' + json.dumps(config['arm_model'], ensure_ascii=False), flush=True)
        cases = sample_cases(config)
        cases = [case for case in cases if args.set == 'full' or case['quick']]
        if args.workers < 1:
            raise ValueError('workers must be positive')
        check = subprocess.run([str(executable), '--check-resources', family_option,
                                '--arm-model-path=' + config['arm_model']['model_path']],
                               capture_output=True, text=True, check=False)
        if check.returncode:
            raise RuntimeError(check.stderr.strip())
        write_json(args.output / 'config.json', config)
        write_json(args.output / 'cases.json', cases)
        write_json(args.output / 'metadata.json', {'set': args.set, 'workers': args.workers, 'seed': config['seed'],
                   'arm_model': config['arm_model'],
                   'parking_offset_world_m': config.get('parking_offset_world_m', [0.0, 0.0, 0.0]),
                   'parking_yaw_world_rad': config.get('parking_yaw_world_rad', 0.0),
                   'j1_max_from_vertical_rad': config.get('j1_max_from_vertical_rad', math.pi / 2),
                      'check_self_collision': config.get('check_self_collision', True),
                   'check_gripper_pillar_collision': config.get('check_gripper_pillar_collision', True),
                   'parking_positions': parking_positions(config),
                   'start_joints_rad': config['start_joints_rad'], 'twist_mode': 'RequiredRange',
                   'twist_range_rad': [-math.pi/2, math.pi/2], 'station_front_plane_filter': False,
                   'velocity_rad_s': 0.5, 'acceleration_rad_s2': 1.0,
                   'disclaimer': 'fixed grasp and mount are simulation assumptions; finite-budget failure is not proof of unreachable'})
        results = {}
        with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as pool:
            futures = {pool.submit(call_case, executable, case, config['timeout_s'], args.output / 'trajectories' / case['id'],
                                   assembly=config['assembly_in_base'], config=config): case for case in cases}
            for future in concurrent.futures.as_completed(futures):
                case = futures[future]
                results[case['id']] = future.result()
                save_report(args.output, cases, config, results)
                print(f"{len(results)}/{len(cases)} {case['id']} {results[case['id']]['status']}", flush=True)
        print(json.dumps(summarize([{'result': results[c['id']]} for c in cases]), ensure_ascii=False))
    else:
        config = json.loads((args.batch / 'config.json').read_text(encoding='utf-8'))
        cases = json.loads((args.batch / 'cases.json').read_text(encoding='utf-8'))
        results = json.loads((args.batch / 'results.json').read_text(encoding='utf-8'))
        case = next((entry for entry in cases if entry['id'] == args.case), None)
        original = next((entry['result'] for entry in results if entry['case']['id'] == args.case), None)
        if case is None or original is None:
            raise ValueError('case not found in completed batch')
        if config.get('arm_model') and case.get('arm_model') != config['arm_model']:
            raise ValueError('Replay arm identity mismatch')
        print('原批次结果：' + json.dumps(original, ensure_ascii=False), flush=True)
        output = args.batch / 'replays' / f"{args.case}_{time.time_ns()}"
        output.mkdir(parents=True, exist_ok=False)
        result = call_case(executable, case, config['timeout_s'], output, gui=True, assembly=config['assembly_in_base'], config=config)
        write_json(output / 'result.json', result)
        print('重新规划结果：' + json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    try:
        main()
    except (KeyError, ValueError, OSError, RuntimeError, subprocess.SubprocessError, json.JSONDecodeError) as error:
        print(f'level3_batch: {error}', file=sys.stderr)
        sys.exit(2)
