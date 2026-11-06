#!/usr/bin/env python3
"""Exercise the batch browser click and asynchronous result under a virtual display."""
import importlib.util
import json
from pathlib import Path
import tempfile
import tkinter as tk
from tkinter import ttk

root_dir = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('level3_batch', root_dir / 'src/level3_batch.py')
batch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(batch)

with tempfile.TemporaryDirectory() as tmp:
    directory = Path(tmp)
    cases = batch.sample_cases(json.loads((root_dir / 'config/default.json').read_text()))
    case = cases[0]
    config = json.loads((root_dir / 'config/default.json').read_text())
    # The grid fixture expects 100 cases regardless of the user's batch size.
    config['parking_yaw_world_rad'] = -1.5707963267948966
    config['check_self_collision'] = True
    config['check_gripper_pillar_collision'] = True
    config.pop('parking_grid', None)
    config['samples'] = 100
    config['quick_samples'] = 10
    config['timeout_s'] = 2
    (directory / 'config.json').write_text(json.dumps(config))
    config['parking_offset_world_m'] = [0.2, -0.1, 0.05]
    cases = batch.sample_cases(config)
    case = cases[0]
    (directory / 'config.json').write_text(json.dumps(config))
    (directory / 'cases.json').write_text(json.dumps(cases))
    rows = [{'case': case, 'result': {
        'success': False, 'status': 'PlanningFailed', 'message': 'Required Q coverage incomplete'}}]
    rows.append({'case': cases[1], 'result': {'success': True, 'status': 'Ok'}})
    for sample, stage in zip(cases[2:], ['Start', 'Approach', 'Insert', 'Lift', 'Flip', 'Timing', 'Validation']):
        rows.append({'case': sample, 'result': {'success': False, 'failure_stage': stage}})
    (directory / 'results.json').write_text(json.dumps(rows))
    assert batch.case_style(rows[1]['result'])[1] == '#168b45'
    assert len({batch.case_style(row['result'])[1] for row in rows[:8]}) == 5
    assert batch.case_style({'success': False, 'candidate_last_phase': 'Twist'}) == batch.CASE_STYLES['unknown']
    assert batch.case_style(None) == batch.CASE_STYLES['pending']
    solver = directory / 'solver'
    solver.write_text('#!/usr/bin/env python3\nimport json,time,sys,os,signal\nfrom pathlib import Path\nassert len(sys.argv) == 22\nassert [float(x) for x in sys.argv[17:20]] == [0.2, -0.1, 0.05]\nassert abs(float(sys.argv[20]) + 1.5707963267948966) < 1e-12\nout = Path(sys.argv[9])\n(out / \'pid.txt\').write_text(str(os.getpid()))\ndef stopped(sig, frame):\n    (out / \'terminated.txt\').write_text(\'stopped\')\n    sys.exit(0)\nsignal.signal(signal.SIGTERM, stopped)\nprint(json.dumps({"success":False,"status":"IKFailed","failure_stage":"Twist","message":"test"}),flush=True)\nprint(json.dumps({"success":False,"status":"IKFailed","failure_stage":"Lift","message":"edited","revision":1,"output_directory":str(out / "interactive/edit_1")}),flush=True)\ntime.sleep(5)\n')
    solver.chmod(0o755)
    real_tk = tk.Tk
    errors = []
    def factory():
        window = real_tk()
        window.report_callback_exception = lambda kind, value, trace: errors.append(str(value))
        def all_widgets(widget):
            yield widget
            for child in widget.winfo_children():
                yield from all_widgets(child)
        def copy_control():
            return next(w for w in all_widgets(window) if isinstance(w, ttk.Button)
                        and w.cget('text') == '复制案例编号和路径')

        def click():
            window.update_idletasks()
            if str(copy_control().cget('state')) != 'disabled':
                errors.append('copy button enabled before selecting a case')
            buttons = [w for w in all_widgets(window) if isinstance(w, tk.Button)]
            if len(buttons) != 100 or buttons[0].cget('bg') != batch.CASE_STYLES['score3'][1]:
                errors.append('case grid count or Q stage color incorrect')
            else:
                if buttons[1].cget('bg') != '#168b45':
                    errors.append('successful case is not green')
                if any(abs(w.winfo_width() - w.winfo_height()) > 1 for w in buttons):
                    errors.append('case tiles are not square')
                if buttons[0].winfo_y() != buttons[1].winfo_y() or buttons[1].winfo_x() <= buttons[0].winfo_x():
                    errors.append(f'cases do not form a multi-column grid: {[w.winfo_geometry() for w in buttons[:3]]}, panels={window.grid_bbox(0,2)},{window.grid_bbox(1,2)}, parent={buttons[0].master.winfo_geometry()}')
                left_width = window.grid_bbox(0, 2)[2]
                right_width = window.grid_bbox(1, 2)[2]
                if abs(left_width - right_width) > 2:
                    errors.append(f'browser panels do not split equally: {left_width}/{right_width}')
                buttons[0].invoke()
                window.geometry('900x650')
        def check():
            buttons = [w for w in all_widgets(window) if isinstance(w, tk.Button)]
            if any(abs(w.winfo_width() - w.winfo_height()) > 1 for w in buttons):
                errors.append('resized tiles are not square')
            pie_canvases = [w for w in all_widgets(window) if isinstance(w, tk.Canvas)
                            and w.find_withtag('failure_slice')]
            if len(pie_canvases) != 1 or len(pie_canvases[0].find_withtag('failure_slice')) != 4:
                errors.append('failure pie stage slices missing')
            elif len(pie_canvases[0].find_withtag('success_slice')) != 1:
                errors.append('success pie slice missing')
            elif not any('33.3%' in pie_canvases[0].itemcget(item, 'text')
                         for item in pie_canvases[0].find_withtag('result_legend')):
                errors.append('failure pie percentages use the wrong denominator')
            labels = [w.cget('text') for w in all_widgets(window) if isinstance(w, tk.Label)]
            if not any('Q 旋转（根据原始消息判断）' in value for value in labels):
                errors.append('original failure stage missing')
            if not any(value == '失败步骤：平移' for value in labels):
                errors.append('latest interactive failure stage missing')
            if not any('核心调整 #1' in value for value in labels):
                errors.append('second streamed result not displayed')
            if not list((directory / 'replays').glob('*/result.json')):
                errors.append('result not persisted while preview is open')
            copy_control().invoke()
            expected_path = (directory / 'trajectories' / case['id']).resolve()
            if window.clipboard_get() != f"case_id: {case['id']}\ncase_path: {expected_path}\n":
                errors.append('clipboard does not contain original case id and absolute path')
            if json.loads((expected_path / 'config.json').read_text()) != config:
                errors.append('original case parameter snapshot missing')
            replay = next((directory / 'replays').glob('*/result.json')).parent
            if json.loads((replay / 'result.json').read_text()).get('revision', 0):
                errors.append('interactive result overwrote initial replay result')
            if json.loads((replay / 'config.json').read_text()) != config:
                errors.append('replay parameter snapshot missing')
            buttons[1].invoke()
            buttons[2].invoke()
            window.after(400, finish)

        def finish():
            labels = [w.cget('text') for w in all_widgets(window) if isinstance(w, tk.Label)]
            if '案例 case_0002' not in labels:
                errors.append('rapid switching did not select the last case')
            if list((directory / 'replays').glob('case_0001_*')):
                errors.append('intermediate pending case was launched')
            original_replay = next((directory / 'replays').glob('case_0000_*'))
            if not (original_replay / 'terminated.txt').exists():
                errors.append('previous preview was not closed automatically')
            copy_control().invoke()
            expected_path = (directory / 'trajectories' / 'case_0002').resolve()
            if window.clipboard_get() != f"case_id: case_0002\ncase_path: {expected_path}\n":
                errors.append('clipboard retained previous selection')
            # Exercise the real close handler so the current child is stopped as well.
            window.tk.call(window.protocol('WM_DELETE_WINDOW'))

        window.after(100, click)
        window.after(8000, lambda: (errors.append('browser test timed out'), window.destroy()))
        window.after(1800, check)
        return window
    tk.Tk = factory
    batch.browse_batch(directory, solver)
    if errors:
        raise AssertionError('; '.join(errors))
