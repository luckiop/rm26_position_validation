#!/usr/bin/env python3
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'src'))
import arm_variants as arms
import rm27_length_search as search
SOURCE = PACKAGE.parent/'rm27_heavy_arm_demo/model/mjcf/rm27_heavy_arm_demo.xml'


class LengthSearchTest(unittest.TestCase):
    def setUp(self):
        self.original = np.array(list(arms.rm27_lengths(SOURCE).values()))

    def test_projection_and_stage_boundaries(self):
        low, high = search.bounds(self.original, 'joint')
        for seed in range(30):
            v = search.project(np.random.default_rng(seed).uniform(-1, 1, 5), sum(self.original), low, high)
            self.assertTrue(search.feasible(v, self.original, 'joint'))
        v = self.original.copy()
        v[:2] = [.2*sum(v[:2]), .8*sum(v[:2])]
        self.assertTrue(search.feasible(v, self.original, 'pair'))
        self.assertFalse(search.feasible(v, self.original, 'joint'))

    def test_rigid_guard_zones(self):
        axis = np.array([1.,0,0])
        vertices = np.array([[[.01,2,3],[.5,2,3],[.99,2,3]]])
        actual = arms.protected_resize(vertices,axis,1,.6,.1)
        np.testing.assert_allclose(actual[0,:,0], [.01,.3,.59])
        np.testing.assert_allclose(actual[0,:,1:],vertices[0,:,1:])

    def test_unknown_progress_stays_in_denominator(self):
        cases = [{'id':'a','sample_id':'s','parking_id':'p0'}, {'id':'b','sample_id':'s','parking_id':'p1'}]
        results = {'a':{'success':True,'status':'Ok'},'b':{'success':False,'status':'ProcessError'}}
        m = search.metric(cases, results, self.original,self.original)
        self.assertEqual(m['mean_score'],2)
        self.assertEqual(m['unknown_count'],1)

    def test_staged_search_is_deterministic_and_bounded(self):
        from types import MethodType
        sequences = []
        for repeat in range(2):
            with tempfile.TemporaryDirectory() as tmp:
                experiment = search.Search.__new__(search.Search)
                experiment.original = self.original
                experiment.config = {'seed': 20261030}
                experiment.root = Path(tmp)
                experiment.rows, experiment.seen = [], set()
                sequence = []
                def evaluate(obj, values, stage, phase='screen'):
                    self.assertTrue(search.feasible(values, self.original, stage))
                    key = search.identifier(values)
                    sequence.append((key, stage, phase))
                    row = {'id': key, 'stage': stage, 'phase': phase, 'lengths_m': list(values),
                           'metrics': {'mean_score': float(4-np.linalg.norm(values-self.original)),
                                       'success_rate': .9, 'adaptability': 8, 'change_m': float(sum(abs(values-self.original))),
                                       'unknown_count': 0}}
                    if phase == 'screen' and key not in obj.seen:
                        obj.seen.add(key); obj.rows.append(row)
                    return row
                experiment.evaluate = MethodType(evaluate, experiment)
                experiment.final_report = lambda *args: None
                experiment.run()
                self.assertTrue((Path(tmp)/'final_ranking.json').exists())
                self.assertLessEqual(len([r for r in sequence if r[2] == 'screen']), 1+17+128+10+120)
                self.assertEqual(len([r for r in sequence if r[2] == 'repeat']), 2)
                sequences.append(sequence)
        self.assertEqual(sequences[0], sequences[1])

    def test_completed_candidate_resume_and_contract(self):
        from types import SimpleNamespace
        from unittest.mock import patch
        executable = PACKAGE.parents[1]/'install-integrated/rm26_level3_validation/bin/level3_case'
        if not executable.exists():
            self.skipTest('requires installed level3_case')
        with tempfile.TemporaryDirectory() as tmp:
            args = SimpleNamespace(config=PACKAGE/'config/default.json', executable=executable,
                                   output=Path(tmp)/'experiment')
            first = search.Search(args)
            first.screen = first.screen[:1]
            row = first.evaluate(first.original, 'joint')
            self.assertEqual(row['metrics']['change_m'], 0)
            second = search.Search(args)
            second.screen = second.screen[:1]
            with patch.object(search.batch, 'call_case', side_effect=AssertionError('must reuse completed results')):
                self.assertEqual(row, second.evaluate(second.original, 'joint'))
            changed = json.loads(args.config.read_text())
            changed['seed'] += 1
            changed_path = Path(tmp)/'changed.json'
            changed_path.write_text(json.dumps(changed))
            args.config = changed_path
            with self.assertRaisesRegex(ValueError, 'Resume inputs differ'):
                search.Search(args)

    def test_constant_sum_and_snapshot_geometry(self):
        values = self.original.copy()
        values[0] -= .01
        values[1] += .01
        spec = {'model_family':'rm27','stage':'joint','changes':[
            {'link':link,'method':'set_length','target_length_m':float(v)}
            for link,v in zip(arms.RM27_PROFILES,values)][:2]}
        with tempfile.TemporaryDirectory() as tmp:
            target = Path(tmp)/'model'
            identity = arms.generate_arm(SOURCE,spec,target)
            manifest = arms.verify_arm(identity)
            self.assertAlmostEqual(sum(manifest['rod_lengths_before_m'].values()),sum(manifest['rod_lengths_after_m'].values()))
            old,new = ET.parse(SOURCE).getroot(),ET.parse(target/'model.xml').getroot()
            for name in ('link_1','link_2','link_5','link_6','link_7'):
                self.assertEqual(old.find(f".//body[@name='{name}']").get('pos'),new.find(f".//body[@name='{name}']").get('pos'))
            for link in ('link_2','link_3'):
                before = old.find(f".//body[@name='{link}']")
                after = new.find(f".//body[@name='{link}']")
                self.assertEqual(before.find('inertial').attrib,after.find('inertial').attrib)
                self.assertTrue(all(g.get('mesh').endswith(('_prox','_middle','_dist')) for g in after.findall('geom')))
            bad = copy.deepcopy(spec)
            bad['changes'][0]['target_length_m'] += .01
            with self.assertRaisesRegex(ValueError,'total rod length'):
                arms.generate_arm(SOURCE,bad,Path(tmp)/'bad')
            self.assertFalse((Path(tmp)/'bad').exists())
            executable = PACKAGE.parents[1]/'install-integrated/rm26_level3_validation/bin/level3_case'
            if executable.exists():
                import subprocess
                result = subprocess.run([str(executable),'--check-resources','--arm-family=rm27','--arm-model-path='+identity['model_path']],capture_output=True,text=True)
                self.assertEqual(result.returncode,0,result.stderr)


if __name__ == '__main__':
    unittest.main()
