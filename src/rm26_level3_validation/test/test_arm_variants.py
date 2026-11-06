#!/usr/bin/env python3
"""Check middle removal, immutable provenance, and solver model selection."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'src'))
import arm_variants as arms
import level3_batch as batch

EXECUTABLE = Path(sys.argv.pop()).resolve() if len(sys.argv) > 1 else None
SOURCE = PACKAGE.parents[1] / 'rm_tech_core_trajectory_solve/src/rm26_engineering_arm_demo/models/rm26_arm_only/rm26_arm_only.xml'


class ArmVariantTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory()
        cls.folder = Path(cls.temporary.name)
        cls.identity = arms.generate_arm(SOURCE,
            json.loads((PACKAGE / 'config/arm_j1_short_10cm.json').read_text()), cls.folder / 'short')
        cls.j5 = {percent: arms.generate_arm(SOURCE, json.loads(
            (PACKAGE / f'config/arm_j5_short_{percent}pct.json').read_text()),
            cls.folder / f'j5-{percent}') for percent in (20, 40)}
        cls.extra = {(joint, percent): arms.generate_arm(SOURCE, json.loads(
            (PACKAGE / f'config/arm_j{joint}_short_{percent}pct.json').read_text()),
            cls.folder / f'j{joint}-{percent}') for joint in (4, 6) for percent in (20, 40)}

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def test_shortening_and_downstream_mounts(self):
        original = ET.parse(SOURCE)
        modified = ET.parse(self.identity['model_path'])
        change = arms.verify_arm(self.identity)['modifications'][0]
        before = np.array(change['child_position_before_m'])
        after = np.array(change['child_position_after_m'])
        np.testing.assert_allclose(before-after, np.array(change['axis_in_link']) * .1, atol=1e-12)
        self.assertAlmostEqual(change['axial_length_before_m']-change['axial_length_after_m'], .1)
        self.assertEqual(before[2], after[2])
        # A downstream mount must follow J2 without being shortened itself.
        for link in ('j3_link', 'j4_link', 'j5_link', 'j6_link'):
            self.assertEqual(original.find(f".//body[@name='{link}']").attrib,
                             modified.find(f".//body[@name='{link}']").attrib)
        self.assertEqual(self.identity['label'], 'J1缩短10cm')
        body = modified.find(".//body[@name='j1_link']")
        self.assertTrue(body.findall("geom[@class='collision']"))
        # Both ends meet at the former near cut plane; the removed slice cannot remain.
        axis = np.array(change['axis_in_link'])
        plane = change['cut_interval_m'][0]
        for geom in body.findall('geom'):
            mesh = modified.find(f".//mesh[@name='{geom.get('mesh')}']")
            triangles = arms.load_triangles(Path(self.identity['model_path']).parent / mesh.get('file'))
            projections = triangles @ axis
            if geom.get('name').endswith('_prox'):
                self.assertLessEqual(float(projections.max()), plane + 1e-8)
            else:
                self.assertGreaterEqual(float(projections.min()), plane - 1e-8)

    def test_j5_percentage_shortening(self):
        original = ET.parse(SOURCE)
        for percent, identity in self.j5.items():
            with self.subTest(percent=percent):
                manifest = arms.verify_arm(identity)
                change = manifest['modifications'][0]
                expected_length = .0755 * percent / 100
                self.assertAlmostEqual(change['shorten_m'], expected_length)
                self.assertAlmostEqual(change['axial_length_after_m'], .0755 - expected_length)
                self.assertAlmostEqual(change['cut_center_m'], .0755 / 2)
                np.testing.assert_allclose(change['axis_in_link'], [0, 0, -1])
                np.testing.assert_allclose(change['child_position_after_m'], [.012369, 0, -.0755 + expected_length])
                self.assertEqual(identity['label'], f'J5缩短{percent}%')
                modified = ET.parse(identity['model_path'])
                for link in ('j0_link', 'j1_link', 'j2_link', 'j3_link', 'j4_link', 'j5_link'):
                    self.assertEqual(original.find(f".//body[@name='{link}']").attrib,
                                     modified.find(f".//body[@name='{link}']").attrib)
                self.assertEqual(original.find(".//body[@name='j6_link']").get('quat'),
                                 modified.find(".//body[@name='j6_link']").get('quat'))
                body = modified.find(".//body[@name='j5_link']")
                for category in ('visual', 'collision'):
                    geoms = body.findall(f"geom[@class='{category}']")
                    self.assertTrue(geoms)
                    for geom in geoms:
                        asset = modified.find(f".//mesh[@name='{geom.get('mesh')}']")
                        triangles = arms.load_triangles(Path(identity['model_path']).parent / asset.get('file'))
                        projections = -triangles[:, :, 2]
                        seam = change['cut_interval_m'][0]
                        if geom.get('name').endswith('_prox'):
                            self.assertLessEqual(float(projections.max()), seam + 1e-8)
                        else:
                            self.assertGreaterEqual(float(projections.min()), seam - 1e-8)

    def test_j4_and_j6_independent_shortening(self):
        original = ET.parse(SOURCE)
        for (joint, percent), identity in self.extra.items():
            with self.subTest(joint=joint, percent=percent):
                change = arms.verify_arm(identity)['modifications'][0]
                reference = .1211 if joint == 4 else .173794121
                length = reference * percent / 100
                self.assertAlmostEqual(change['shorten_m'], length)
                self.assertAlmostEqual(change['axial_length_after_m'], reference - length)
                self.assertEqual(identity['label'], f'J{joint}缩短{percent}%')
                modified = ET.parse(identity['model_path'])
                for link in ('j0_link', 'j1_link', 'j2_link', 'j3_link', 'j4_link', 'j5_link', 'j6_link'):
                    if joint == 4 and link == 'j5_link':
                        continue
                    self.assertEqual(original.find(f".//body[@name='{link}']").attrib,
                                     modified.find(f".//body[@name='{link}']").attrib)
                if joint == 4:
                    np.testing.assert_allclose(change['child_position_after_m'], [.1211-length, 0, .0245])
                    self.assertEqual(identity['attach_link'], 'j6_link')
                else:
                    self.assertEqual(identity['attach_link'], 'variant_energy_mount')
                    mount = modified.find(".//body[@name='j6_link']/body[@name='variant_energy_mount']")
                    np.testing.assert_allclose(np.fromstring(mount.get('pos'), sep=' '), [0, 0, -length])
                    np.testing.assert_allclose(change['terminal_reference_after_m'], [0, 0, reference-length])
                    with self.assertRaises(ValueError):
                        batch.annotate_arm_result({'arm_model_path': identity['model_path'],
                            'arm_attach_link': 'j6_link'}, {'arm_model': identity})
                    batch.annotate_arm_result({'arm_model_path': identity['model_path'],
                        'arm_attach_link': 'variant_energy_mount'}, {'arm_model': identity})
                body = modified.find(f".//body[@name='j{joint}_link']")
                for geom in body.findall('geom'):
                    asset = modified.find(f".//mesh[@name='{geom.get('mesh')}']")
                    triangles = arms.load_triangles(Path(identity['model_path']).parent / asset.get('file'))
                    projections = triangles @ np.asarray(change['axis_in_link'])
                    seam = change['cut_interval_m'][0]
                    if geom.get('name').endswith('_prox'):
                        self.assertLessEqual(float(projections.max()), seam + 1e-8)
                    else:
                        self.assertGreaterEqual(float(projections.min()), seam - 1e-8)
                config = json.loads((PACKAGE / 'config/default.json').read_text())
                config['arm_model'] = identity
                command = batch.case_command(Path('/solver'), batch.sample_cases(config)[0], 1, self.folder / 'case')
                self.assertEqual('--arm-attach-link=variant_energy_mount' in command, joint == 6)

    def test_fingerprint_rejects_changed_mesh(self):
        identity = self.identity
        manifest = arms.verify_arm(identity)
        relative = next(name for name in manifest['files_sha256'] if name.endswith('.obj'))
        path = Path(identity['manifest_path']).parent / relative
        contents = path.read_bytes()
        try:
            path.write_bytes(contents + b'\n# changed\n')
            with self.assertRaisesRegex(ValueError, 'fingerprint mismatch'):
                arms.verify_arm(identity)
        finally:
            path.write_bytes(contents)
        arms.verify_arm(identity)

    def test_original_preserves_meshes_and_joints(self):
        identity = arms.generate_arm(SOURCE, {'id': 'original', 'changes': []}, self.folder / 'original')
        manifest = arms.verify_arm(identity)
        self.assertEqual(identity['label'], '原始')
        before, after = ET.parse(SOURCE), ET.parse(identity['model_path'])
        self.assertEqual([b.attrib for b in before.iter('body')], [b.attrib for b in after.iter('body')])
        self.assertEqual([j.attrib for j in before.iter('joint')], [j.attrib for j in after.iter('joint')])
        self.assertEqual(set(manifest['files_sha256'].values()) - {arms.file_hash(Path(identity['model_path'])),
            arms.file_hash(Path(identity['model_path']).parent / 'variant_config.json')},
            set(manifest['source_files_sha256'].values()) - {arms.file_hash(SOURCE)})

    def test_invalid_variant(self):
        for spec in ({'changes': [{'link': 'j2_link', 'method': 'remove_middle', 'shorten_m': .1, 'cut_center_m': .2}]},
                     {'changes': [{'link': 'j1_link', 'method': 'remove_middle', 'shorten_m': float('nan'), 'cut_center_m': .2}]},
                     {'changes': [{'link': 'j5_link', 'method': 'remove_middle', 'shorten_fraction': 1}]},
                     {'changes': [{'link': 'j5_link', 'method': 'remove_middle', 'shorten_fraction': True}]},
                     {'changes': [{'link': 'j5_link', 'method': 'remove_middle', 'shorten_fraction': .2, 'shorten_m': .01}]}):
            with self.assertRaises(ValueError):
                arms.validate_variant(spec)

    def test_reported_solver_model_must_match(self):
        case = {'arm_model': self.identity}
        good = {'success': True, 'arm_model_path': self.identity['model_path']}
        self.assertTrue(batch.annotate_arm_result(good, case)['arm_model_loaded'])
        with self.assertRaises(ValueError):
            batch.annotate_arm_result({'success': True}, case)
        with self.assertRaises(ValueError):
            batch.annotate_arm_result({'success': False, 'arm_model_path': '/different/model.xml'}, case)

    @unittest.skipUnless(EXECUTABLE, 'Pass level3_case to check the real model loader')
    def test_solver_loads_frozen_variant(self):
        for identity in [*self.j5.values(), *self.extra.values()]:
            subprocess.run([str(EXECUTABLE), '--check-j1-limits', '--arm-model-path=' + identity['model_path']],
                           capture_output=True, text=True, check=True)
        result = subprocess.run([str(EXECUTABLE), '--check-j1-limits',
            '--arm-model-path=' + self.identity['model_path']], capture_output=True, text=True, check=True)
        limits = json.loads(result.stdout)
        self.assertEqual(len(limits), 2)
        self.assertGreater(limits[1], limits[0])
        config = json.loads((PACKAGE / 'config/default.json').read_text())
        config['arm_model'] = self.identity
        case = batch.sample_cases(config)[0]
        command = batch.case_command(EXECUTABLE, case, 1, self.folder / 'case')
        self.assertIn('--arm-model-path=' + self.identity['model_path'], command)
        self.assertEqual(case['arm_model'], self.identity)


if __name__ == '__main__':
    unittest.main()
