#!/usr/bin/env python3
"""Deterministic, resumable RM27 constant-length experiment (one worker)."""
import argparse
import copy
import concurrent.futures
import csv
import hashlib
import itertools
import json
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parent))
# Installed batch script has no .py suffix; load it without executing main.
import importlib.machinery
import importlib.util
_batch = Path(__file__).resolve().parent / 'level3_batch.py'
if not _batch.exists():
    _batch = _batch.with_name('level3_batch')
_loader = importlib.machinery.SourceFileLoader('level3_batch', str(_batch))
_spec = importlib.util.spec_from_loader(_loader.name, _loader)
batch = importlib.util.module_from_spec(_spec)
_loader.exec_module(batch)
import arm_variants as arms


def read(path):
    return json.loads(path.read_text())


def identifier(lengths):
    return hashlib.sha256(json.dumps([round(x, 10) for x in lengths]).encode()).hexdigest()[:16]


def bounds(original, stage):
    import numpy as np
    if stage == 'pair':
        total = sum(original[:2])
        low, high = np.array(original), np.array(original)
        low[:2], high[:2] = .1 * total, .9 * total
    else:
        low, high = .3 * original, 2 * original
    guard = np.array([2*p['end_clearance_m'] + 2e-6 for p in arms.RM27_PROFILES.values()])
    return np.maximum(low, guard), high


def feasible(values, original, stage):
    import numpy as np
    low, high = bounds(original, stage)
    return bool(abs(sum(values)-sum(original)) <= 1e-8 and np.all(values >= low-1e-10) and np.all(values <= high+1e-10))


def project(values, total, low, high):
    """Project onto a bounded fixed-sum simplex, without rejection bias at bounds."""
    import numpy as np
    left, right = float(np.min(values-high))-1, float(np.max(values-low))+1
    for _ in range(80):
        middle = (left+right)/2
        if np.clip(values-middle, low, high).sum() > total:
            left = middle
        else:
            right = middle
    return np.clip(values-(left+right)/2, low, high)


def metric(cases, results, values, original):
    stats = batch.progress_statistics(cases, results)
    # Unknown progress cannot improve rank by disappearing from the denominator.
    scored = {c['id']: (batch.case_score(results[c['id']]) or 0) for c in cases}
    groups = {}
    for c in cases:
        groups.setdefault(c.get('parking_id', 'p00'), []).append(scored[c['id']])
    return {'mean_score': sum(sum(v)/len(v) for v in groups.values())/len(groups),
            'success_rate': sum(bool(r.get('success')) for r in results.values())/len(cases),
            'adaptability': sum(sum(scored[c['id']] >= 3 for c in cases if c.get('sample_id', c['id']) == sid)
                                for sid in {c.get('sample_id', c['id']) for c in cases}) / len({c.get('sample_id', c['id']) for c in cases}),
            'change_m': sum(abs(a-b) for a,b in zip(values, original)),
            'unknown_count': sum(p['unknown'] for p in stats['parking_results'])}


def rank(row):
    m = row['metrics']
    return (-m['mean_score'], -m['success_rate'], -m['adaptability'], m['change_m'], row['id'])


class Search:
    def __init__(self, args):
        import numpy as np
        self.args, self.root = args, args.output.resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.config = read(args.config)
        batch.check_config(self.config)
        query = [str(args.executable), '--print-arm-config', '--arm-family=rm27',
                 '--j1-max-from-vertical-rad='+str(self.config.get('j1_max_from_vertical_rad', math.pi/2))]
        info = json.loads(subprocess.run(query, check=True, capture_output=True, text=True).stdout)
        self.source = Path(info['model_path'])
        self.original = np.array(list(arms.rm27_lengths(self.source).values()))
        self.config['start_joints_rad'] = info['start_joints_rad']
        self.config.pop('arm_model', None)
        self.config['arm_variant'] = {'id': 'rm27-heavy', 'model_family': 'rm27', 'changes': []}
        self.cases = batch.sample_cases(self.config)
        samples = list(dict.fromkeys(c.get('sample_id', c['id']) for c in self.cases))
        selected = {samples[i*len(samples)//min(10, len(samples))] for i in range(min(10, len(samples)))}
        self.screen = [c for c in self.cases if c.get('sample_id', c['id']) in selected]
        # Bind resume to source assets, executable, scripts, config and exact cases.
        xml = ET.parse(self.source).getroot()
        meshdir = self.source.parent / xml.find('compiler').get('meshdir', '')
        resources = {str(self.source): arms.file_hash(self.source)}
        resources.update({str(meshdir/m.get('file')): arms.file_hash(meshdir/m.get('file')) for m in xml.find('asset')})
        contract = {'config': self.config, 'source_hashes': resources, 'cases': self.cases,
                    'screen_ids': [c['id'] for c in self.screen], 'workers': getattr(args, 'workers', 1),
                    'executable_sha256': arms.file_hash(args.executable),
                    'implementation_sha256': {str(p): arms.file_hash(p) for p in (Path(__file__).resolve(), Path(arms.__file__), _batch)},
                    'original_lengths_m': self.original.tolist(), 'seed': self.config['seed']}
        frozen = self.root/'experiment.json'
        if frozen.exists() and read(frozen) != contract:
            raise ValueError('Resume inputs differ from frozen experiment; use a new output directory')
        if not frozen.exists():
            batch.write_json(frozen, contract)
        self.rows = []
        self.seen = set()

    def evaluate(self, values, stage, phase='screen'):
        values = list(map(float, values))
        key = identifier(values)
        directory = self.root/phase/key
        result_path = directory/'evaluation.json'
        if (directory/'invalid.json').exists():
            self.seen.add(key)
            return None
        if result_path.exists():
            row = read(result_path)
            arms.verify_arm(read(directory/'config.json')['arm_model'])
        else:
            directory.mkdir(parents=True, exist_ok=True)
            spec = {'id': 'rm27-'+key, 'model_family': 'rm27', 'stage': stage,
                    'changes': [{'link': link, 'method': 'set_length', 'target_length_m': length}
                                for link, length, old in zip(arms.RM27_PROFILES, values, self.original)
                                if abs(length-old) > 1e-10]}
            cfgpath = directory/'config.json'
            try:
                if cfgpath.exists():
                    config = read(cfgpath)
                    arms.verify_arm(config['arm_model'])
                else:
                    config = copy.deepcopy(self.config)
                    config['arm_variant'] = spec
                    modeldir = directory/'arm_model'
                    # A partially generated snapshot is never silently reused.
                    if modeldir.exists():
                        raise RuntimeError(f'Incomplete model snapshot: {modeldir}; inspect and remove this candidate directory before resuming')
                    config['arm_model'] = arms.generate_arm(self.source, spec, modeldir)
                    arms.verify_arm(config['arm_model'])
                    check = subprocess.run([str(self.args.executable), '--check-resources', '--arm-family=rm27',
                                            '--arm-model-path='+config['arm_model']['model_path']], capture_output=True, text=True)
                    if check.returncode:
                        raise ValueError(check.stderr or check.stdout)
                    subprocess.run([str(self.args.executable), '--check-j1-limits', '--arm-family=rm27',
                                    '--arm-model-path='+config['arm_model']['model_path'],
                                    '--j1-max-from-vertical-rad='+str(config.get('j1_max_from_vertical_rad', math.pi/2))],
                                   capture_output=True, text=True, check=True)
                    batch.write_json(cfgpath, config)
            except (ValueError, subprocess.CalledProcessError) as error:
                row = {'id': key, 'stage': stage, 'phase': phase, 'lengths_m': values, 'invalid': str(error)}
                batch.write_json(directory/'invalid.json', row)
                self.seen.add(key)
                print('INVALID', key, str(error), flush=True)
                return None
            cases = copy.deepcopy(self.screen if phase == 'screen' else self.cases)
            for case in cases:
                case['arm_model'] = config['arm_model']
            batch.write_json(directory/'cases.json', cases)
            batch.write_json(directory/'metadata.json', {'workers': getattr(self.args, 'workers', 1), 'set': phase, 'arm_model': config['arm_model'], 'seed': config['seed']})
            results = {}
            pending = []
            for case in cases:
                saved = directory/'trajectories'/case['id']/'result.json'
                if saved.exists():
                    snapshot = read(saved.parent/'case.json')
                    if snapshot != case:
                        raise ValueError('Saved case differs from frozen inputs')
                    results[case['id']] = batch.annotate_arm_result(read(saved), case)
                else:
                    pending.append(case)
            with concurrent.futures.ThreadPoolExecutor(max_workers=getattr(self.args, 'workers', 1)) as pool:
                futures = {pool.submit(batch.call_case, self.args.executable, case, config['timeout_s'],
                                       directory/'trajectories'/case['id'],
                                       assembly=config['assembly_in_base'], config=config): case for case in pending}
                for future in concurrent.futures.as_completed(futures):
                    case = futures[future]
                    results[case['id']] = future.result()
                    batch.save_report(directory, cases, config, results)
                    print(phase, key, f'{len(results)}/{len(cases)}', results[case['id']]['status'], flush=True)
            row = {'id': key, 'stage': stage, 'phase': phase, 'lengths_m': values,
                   'metrics': metric(cases, results, values, self.original), 'batch': str(directory)}
            batch.write_json(result_path, row)
        if phase == 'screen' and key not in self.seen:
            self.rows.append(row)
            self.seen.add(key)
        self.report()
        return row

    def report(self):
        rows = sorted(self.rows, key=rank)
        batch.write_json(self.root/'screen_ranking.json', rows)
        if rows:
            with (self.root/'screen_ranking.csv').open('w') as stream:
                writer = csv.writer(stream)
                writer.writerow(['id', 'stage', 'mean_score', 'success_rate', 'adaptability', 'unknown_count', *arms.RM27_PROFILES])
                for r in rows:
                    m = r['metrics']
                    writer.writerow([r['id'], r['stage'], m['mean_score'], m['success_rate'], m['adaptability'], m['unknown_count'], *r['lengths_m']])

    def run(self):
        import numpy as np
        self.evaluate(self.original, 'joint')
        total = sum(self.original[:2])
        for i in range(2, 19):
            values = self.original.copy()
            values[:2] = [i*.05*total, (1-i*.05)*total]
            if feasible(values, self.original, 'pair'):
                self.evaluate(values, 'pair')
            else:
                batch.write_json(self.root/f'invalid-pair-{i:02}.json', {'ratio': i*.05, 'reason': 'protected joint-end zones'})
        rng = np.random.default_rng(self.config['seed'])
        low, high = bounds(self.original, 'joint')
        centers = [np.array(r['lengths_m']) for r in sorted(self.rows, key=rank)
                   if feasible(r['lengths_m'], self.original, 'joint')][:3] or [self.original]
        generated = set(self.seen)
        n = 0
        attempts = 0
        while n < 128:
            attempts += 1
            if attempts > 100000:
                raise RuntimeError('Could not construct enough unique joint candidates')
            raw = rng.uniform(low, high) if n < 64 else centers[(n-64)%len(centers)] + rng.normal(0, .12*self.original)
            values = project(raw, sum(self.original), low, high)
            if identifier(values) in generated:
                continue
            generated.add(identifier(values))
            if self.evaluate(values, 'joint') is not None:
                n += 1
        for i in range(len(low)):
            for bound in (max(low[i], sum(self.original)-sum(high)+high[i]),
                          min(high[i], sum(self.original)-sum(low)+low[i])):
                values = self.original.copy()
                values[i] = bound
                rest = np.arange(len(low)) != i
                values[rest] = project(values[rest], sum(self.original)-bound, low[rest], high[rest])
                self.evaluate(values, 'joint')
        # Reserve 20 evaluations per resolution per search line: 120 total.
        # Step order and seed order are reconstructed identically on resume.
        for stage in ('pair', 'joint'):
            seeds = sorted([r for r in self.rows if r['stage'] == stage], key=rank)[:5]
            currents = list(seeds)
            for step in (.020, .010, .005):
                fine_count = 0
                for seed_index, current in enumerate(currents):
                    for _ in range(2):
                        choices = [current]
                        indices = range(2 if stage == 'pair' else len(low))
                        for i,j in itertools.permutations(indices, 2):
                            values = np.array(current['lengths_m'])
                            values[i] += step
                            values[j] -= step
                            existing = next((r for r in self.rows if r['id'] == identifier(values)), None)
                            if existing and feasible(values, self.original, stage):
                                choices.append(existing)
                            if (fine_count < 20 and feasible(values, self.original, stage)
                                    and identifier(values) not in self.seen):
                                candidate = self.evaluate(values, stage)
                                fine_count += 1
                                if candidate:
                                    choices.append(candidate)
                        best = min(choices, key=rank)
                        if best['id'] == current['id']:
                            break
                        current = best
                    currents[seed_index] = current
        finalists = sorted(self.rows, key=rank)[:5]
        if identifier(self.original) not in {r['id'] for r in finalists}:
            finalists.append(next(r for r in self.rows if r['id'] == identifier(self.original)))
        full = [self.evaluate(r['lengths_m'], r['stage'], 'full') for r in finalists]
        full = sorted([r for r in full if r], key=rank)
        repeated = []
        for row in full[:2]:
            second = self.evaluate(row['lengths_m'], row['stage'], 'repeat')
            avg = copy.deepcopy(row)
            avg['metrics'] = {k: (row['metrics'][k]+second['metrics'][k])/2 for k in row['metrics']}
            avg['runs'] = [row, second]
            repeated.append(avg)
        batch.write_json(self.root/'final_ranking.json', {'full': full, 'repeated': sorted(repeated, key=rank)})
        self.final_report(full, sorted(repeated, key=rank))

    def final_report(self, full, repeated):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        winner = repeated[0]
        baseline = next(r for r in full if r['id'] == identifier(self.original))
        a = {r['case']['id']: r['result'] for r in read(Path(baseline['batch'])/'results.json')}
        b = {r['case']['id']: r['result'] for r in read(Path(winner['batch'])/'results.json')}
        changes = [{'case_id': k, 'before': batch.case_score(a[k]), 'after': batch.case_score(b[k]),
                    'before_status': a[k]['status'], 'after_status': b[k]['status']} for k in a]
        batch.write_json(self.root/'case_comparison.json', changes)
        with (self.root/'full_ranking.csv').open('w') as stream:
            writer = csv.writer(stream)
            writer.writerow(['id', 'mean_score', 'success_rate', 'adaptability', 'unknown_count', *arms.RM27_PROFILES])
            for r in full:
                m = r['metrics']
                writer.writerow([r['id'], m['mean_score'], m['success_rate'], m['adaptability'], m['unknown_count'], *r['lengths_m']])
        fig, axes = plt.subplots(1, 5, figsize=(18, 3), sharey=True)
        for i, ax in enumerate(axes):
            ax.scatter([r['lengths_m'][i]*1000 for r in self.rows],
                       [r['metrics']['mean_score'] for r in self.rows], s=8, alpha=.5)
            ax.set(xlabel=f'J{i+1} length (mm)', ylim=(0, 4.1))
        axes[0].set_ylabel('Mean task score')
        fig.tight_layout(); fig.savefig(self.root/'length_score_scatter.png'); plt.close(fig)
        pair = [r for r in self.rows if r['stage'] == 'pair']
        pair.sort(key=lambda r:r['lengths_m'][0])
        fig, ax = plt.subplots()
        ax.plot([r['lengths_m'][0]/sum(self.original[:2]) for r in pair], [r['metrics']['mean_score'] for r in pair], 'o-')
        ax.set(xlabel='J1 / original (J1 + J2)', ylabel='Mean task score', ylim=(0,4.1))
        fig.savefig(self.root/'j1_j2_scores.png'); plt.close(fig)
        fig, axes = plt.subplots(1,2,figsize=(10,4))
        for ax, row, title in zip(axes, (baseline,winner), ('Original', 'Recommended')):
            stats = read(Path(row['batch'])/'progress_statistics.json')['parking_results']
            values = [p['mean_score'] for p in stats]
            side = math.isqrt(len(values))
            if side*side == len(values):
                import numpy as np
                im=ax.imshow(np.array(values).reshape(side,side),vmin=0,vmax=4,origin='lower')
                for i,v in enumerate(values): ax.text(i%side,i//side,f"p{i:02}\n{v:.3f}",ha='center')
                fig.colorbar(im,ax=ax)
            else:
                ax.bar(range(len(values)),values); ax.set_ylim(0,4)
            ax.set_title(title)
        fig.savefig(self.root/'parking_scores.png'); plt.close(fig)
        lengths = '\n'.join(f"| J{i+1} ({link}) | {old*1000:.3f} | {new*1000:.3f} |" for i,(link,old,new) in enumerate(zip(arms.RM27_PROFILES,self.original,winner['lengths_m'])))
        text = f"""# RM27 length search result

Recommended: `{winner['id']}`. Two-run mean score: {winner['metrics']['mean_score']:.6f}; success rate: {winner['metrics']['success_rate']:.6f}.
Baseline score: {baseline['metrics']['mean_score']:.6f}. See final_ranking.json for run-to-run variation and unknown counts.

| Segment | Original mm | Recommended mm |
|---|---:|---:|
{lengths}

Total: {sum(self.original)*1000:.6f} -> {sum(winner['lengths_m'])*1000:.6f} mm. J0 and gripper unchanged.

Browse: `level3_batch browse --batch {winner['batch']}`

See case_comparison.json, j1_j2_scores.png, parking_scores.png, and screen_ranking.csv.
Results apply only to the sampled tasks and tested search range. Guard zones are exploration margins; original mass/inertia retained.
"""
        reasons = {r['id']: read(Path(r['batch'])/'progress_statistics.json')['failure_reasons'] for r in full}
        batch.write_json(self.root/'failure_reason_comparison.json', reasons)
        batch.write_json(self.root/'recommended.json', winner)
        (self.root/'REPORT.md').write_text(text)
        print(text, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--executable', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--workers', type=int, default=1)
    args = parser.parse_args()
    if args.workers < 1:
        parser.error('--workers must be positive')
    args.executable = args.executable.resolve(strict=True)
    # Prevent two resumptions from writing into the same experiment.
    args.output.mkdir(parents=True, exist_ok=True)
    import fcntl
    with (args.output/'.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        Search(args).run()


if __name__ == '__main__':
    main()
