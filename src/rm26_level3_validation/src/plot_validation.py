#!/usr/bin/env python3
"""Plot base-frame projections and paired parking/sample results from a batch."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.patches import Patch
import numpy as np

from level3_batch import case_style, CASE_STYLES, progress_statistics

# Reuse the browser's stage classification and colors; use portable English labels.
LABELS = ['Not run', '0: Before insertion', '1: Inserted', '2: Translated',
          '3: P flip complete', '4: All complete', 'Unknown progress']
COLORS = [value[1] for value in CASE_STYLES.values()]


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8'))


def plot_batch(directory, output):
    rows = read_json(directory / 'results.json')
    cases_path = directory / 'cases.json'
    cases = read_json(cases_path) if cases_path.exists() else [row['case'] for row in rows]
    if not cases:
        raise ValueError('Batch contains no cases')
    results = {}
    for row in rows:
        identifier = row['case']['id']
        if identifier in results:
            raise ValueError(f'Duplicate result: {identifier}')
        results[identifier] = row['result']
    parks = sorted({case.get('parking_id', 'p00') for case in cases})
    samples = sorted({case.get('sample_id', case['id']) for case in cases})
    pi, si = {p: i for i, p in enumerate(parks)}, {s: i for i, s in enumerate(samples)}
    matrix = np.zeros((len(parks), len(samples)), dtype=int)
    present = set()
    for case in cases:
        cell = (pi[case.get('parking_id', 'p00')], si[case.get('sample_id', case['id'])])
        if cell in present:
            raise ValueError(f'Duplicate parking/sample pair: {cell}')
        present.add(cell)
        matrix[cell] = COLORS.index(case_style(results.get(case['id']))[1])
    config_path = directory / 'config.json'
    config = read_json(config_path) if config_path.exists() else {}
    arm_label = config.get('arm_model', {}).get('id', 'original-legacy')
    flags = ', '.join(f'{name}={config.get(name, "unknown")}' for name in
                      ('check_self_collision', 'check_gripper_pillar_collision'))
    legend = [Patch(facecolor=c, label=l) for c, l in zip(COLORS, LABELS)]
    output.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(len(parks), 3, figsize=(12, 3 * len(parks) + 1.4), squeeze=False)
    xyz = np.asarray([case['core_in_base']['position_m'] for case in cases], dtype=float)
    limits = []
    for dim in range(3):
        lo, hi = xyz[:, dim].min(), xyz[:, dim].max()
        pad = max((hi - lo) * .06, .005)
        limits.append((lo - pad, hi + pad))
    for p, row_axes in zip(parks, axes):
        indices = [i for i, c in enumerate(cases) if c.get('parking_id', 'p00') == p]
        colors = [case_style(results.get(cases[i]['id']))[1] for i in indices]
        succeeded = int(np.count_nonzero(matrix[pi[p]] == COLORS.index(CASE_STYLES['success'][1])))
        tested = int(np.count_nonzero(matrix[pi[p]]))
        for ax, (a, b) in zip(row_axes, [(0, 1), (0, 2), (1, 2)]):
            ax.scatter(xyz[indices, a], xyz[indices, b], c=colors, s=28, edgecolors='white', linewidths=.35)
            ax.set(xlabel=f'{"XYZ"[a]} (m)', ylabel=f'{"XYZ"[b]} (m)',
                   xlim=limits[a], ylim=limits[b], title=f'{p}: {succeeded}/{tested} succeeded')
            ax.set_aspect('equal', adjustable='box')
            ax.grid(alpha=.2)
    fig.suptitle(f'{directory.name} | {arm_label}\nCore position in arm base frame\n{flags}', fontsize=10)
    fig.legend(handles=legend, loc='lower center', ncol=5, fontsize=8)
    fig.tight_layout(rect=(0, .045, 1, 1 - .6 / fig.get_figheight()))
    save(fig, output / 'xyz_projections')

    fig, ax = plt.subplots(figsize=(max(10, len(samples) * .23), max(4, len(parks) * .48 + 2)))
    ax.imshow(matrix, cmap=ListedColormap(COLORS), norm=BoundaryNorm(np.arange(-.5, len(COLORS)), len(COLORS)),
              aspect='auto', interpolation='nearest')
    ax.set_yticks(range(len(parks)), labels=parks)
    step = max(1, (len(samples) + 49) // 50)
    ax.set_xticks(range(0, len(samples), step), labels=samples[::step], rotation=90, fontsize=7)
    ax.set(xlabel='Core sample (same world pose across parking positions)', ylabel='Parking position')
    success = matrix == COLORS.index(CASE_STYLES['success'][1])
    complete = (matrix != 0).all(axis=0)
    covered = success.any(axis=0)
    union = f'{covered[complete].sum()}/{complete.sum()}'
    ax.set_title(f'{directory.name} | {arm_label}\nParking x sample progress | any-parking coverage (complete samples): {union}\n{flags}', fontsize=10)
    for i in range(len(parks)):
        tested = np.count_nonzero(matrix[i])
        ax.text(1.01, i, f'{success[i].sum()}/{tested}', transform=ax.get_yaxis_transform(), va='center', fontsize=9)
    fig.legend(handles=legend, loc='lower center', ncol=5, fontsize=8)
    fig.tight_layout(rect=(0, .13, .95, 1))
    save(fig, output / 'parking_sample_matrix')

    stats = progress_statistics(cases, results)
    groups = stats['parking_results']
    xs = sorted({g['position_m'][0] for g in groups})
    ys = sorted({g['position_m'][1] for g in groups})
    means = np.full((len(ys), len(xs)), np.nan)
    for g in groups:
        if g['mean_score'] is not None:
            means[ys.index(g['position_m'][1]), xs.index(g['position_m'][0])] = g['mean_score']
    from matplotlib.colors import LinearSegmentedColormap
    cmap = LinearSegmentedColormap.from_list('progress',
        [CASE_STYLES[k][1] for k in ('score0', 'score1', 'score2', 'score3', 'success')])
    cmap.set_bad('#89939e')
    fig, ax = plt.subplots(figsize=(7, 5))
    im = ax.imshow(means, origin='lower', cmap=cmap, vmin=0, vmax=4)
    ax.set_xticks(range(len(xs)), labels=[f'{x:g}' for x in xs])
    ax.set_yticks(range(len(ys)), labels=[f'{y:g}' for y in ys])
    ax.set(xlabel='Parking world X (m)', ylabel='Parking world Y (m)', title='Task score by parking position')
    for g in groups:
        value = 'N/A' if g['mean_score'] is None else f"{g['mean_score']:.2f}"
        ax.text(xs.index(g['position_m'][0]), ys.index(g['position_m'][1]),
                f"{g['parking_id']}\n{value} / 4\nn={g['scored_count']}", ha='center', va='center')
    fig.colorbar(im, ax=ax, ticks=range(5), label='Task score')
    fig.tight_layout()
    save(fig, output / 'parking_mean_score_heatmap')


def save(fig, path):
    for extension in ('png', 'pdf'):
        target = path.with_suffix('.' + extension)
        fig.savefig(target, dpi=180)
        print(target)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batch', type=Path, help='Batch directory containing results.json and optional cases.json/config.json')
    parser.add_argument('--out', type=Path, help='Output directory (default: BATCH/plots)')
    args = parser.parse_args()
    try:
        plot_batch(args.batch, args.out or args.batch / 'plots')
    except (ValueError, KeyError, OSError) as error:
        parser.exit(1, f'Error: {error}\n')


if __name__ == '__main__':
    main()
