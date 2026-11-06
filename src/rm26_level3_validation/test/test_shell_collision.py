#!/usr/bin/env python3
"""Regression for case_0093: the shell bounding box filled empty visual space."""
import importlib.util
from pathlib import Path
import sys
import tempfile


def main():
    """Require the original case pose to pass the actual-start collision check."""
    root = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location('batch', root / 'src/level3_batch.py')
    batch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(batch)
    case = {
        'id': 'case_0093',
        'parking_offset_world_m': [0.0, 0.0, 0.1],
        'core_in_base': {
            # The original regression pose was expressed in the former chassis frame.
            'position_m': [0.006141661284872542 + 0.23762249574065208,
                           -0.17559023973414492 + 0.29782700538635254,
                           0.5696312175897237 + 0.095],
            'quaternion_wxyz': [-0.140933972666844, 0.5047429361242826,
                               0.11424280783279658, 0.8439909742642147],
        },
    }
    with tempfile.TemporaryDirectory() as directory:
        result = batch.call_case(Path(sys.argv[1]), case, 30.0, Path(directory))
        assert result['status'] not in ('StartCollision', 'ProcessError', 'ProcessTimeout'), result
        assert result['attempts'] > 0 and result['ik_calls'] > 0, result
    print('case_0093 passes actual-start collision check')


if __name__ == '__main__':
    main()
