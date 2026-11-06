#!/usr/bin/env python3
"""Real-model level-3 integration check with both accepted Q endpoints."""
import csv
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('batch', ROOT / 'src/level3_batch.py')
batch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(batch)
config = json.loads((ROOT / 'config/default.json').read_text())
# Keep this deterministic regression independent of user-edited batch settings.
config['parking_yaw_world_rad'] = 0.0
config['check_self_collision'] = True
config['check_gripper_pillar_collision'] = True
config['samples'] = 100
config['quick_samples'] = 10
config['core_bounds']['x_m'] = [-0.1, 0.0]
config['core_bounds']['y_m'] = [0.1, 0.3]
config['core_bounds']['z_m'] = [0.5, 0.7]
config['assembly_in_base']['position_m'] = [0.23762249574065208, 0.29782700538635254, 0.095]
case = batch.sample_cases(config)[10]
with tempfile.TemporaryDirectory() as tmp:
    output = Path(tmp)
    result = batch.call_case(Path(sys.argv[1]), case, config['timeout_s'], output)
    assert result['success'], result
    assert result['status'] == result['validation_status'] == 'Ok', result
    assert result['required_range_covered']
    assert result['q_coverage_rad'][0] <= -math.pi/2
    assert result['q_coverage_rad'][1] >= math.pi/2
    for filename, endpoint in [('negative.csv', -math.pi/2), ('positive.csv', math.pi/2)]:
        with (output / filename).open() as stream:
            knots = list(csv.DictReader(stream))
        assert len(knots) > 2
        assert float(knots[0]['time_s']) == 0
        assert float(knots[-1]['twist_rad']) == endpoint
        assert knots[-1]['phase'] == 'Twist'
        assert all(abs(float(knots[0][name])) < 1e-12 for name in (
            'j0', 'j1', 'j2', 'j3', 'j4', 'end_differential_controller/j5',
            'end_differential_controller/j6'))
print('RM26 required Q endpoints accepted')
