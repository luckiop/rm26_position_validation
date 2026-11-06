#!/usr/bin/env python3
"""Verify planner limits against independent source-model link elevation geometry."""
import json
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[3]
model = ET.parse(root / 'rm_tech_core_trajectory_solve/src/rm26_engineering_arm_demo/models/rm26_arm_only/rm26_arm_only.xml')
j1 = model.find('.//body[@name="j1_link"]')
j2 = j1.find('body[@name="j2_link"]')
w, x, y, z = map(float, j1.get('quat').split())
n = math.sqrt(w*w+x*x+y*y+z*z)
w, x, y, z = (v/n for v in (w,x,y,z))
row = (2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y))
dx, dy, dz = map(float, j2.get('pos').split())

def height(q):
    return row[0]*(dx*math.cos(q)-dy*math.sin(q)) + row[1]*(dx*math.sin(q)+dy*math.cos(q)) + row[2]*dz

for maximum in (math.pi/2, 1.2):
    result = subprocess.run([sys.argv[1], '--check-j1-limits',
                             '--j1-max-from-vertical-rad='+str(maximum)],
                            capture_output=True, text=True, check=True)
    lo, hi = json.loads(result.stdout)
    assert lo < hi
    for i in range(1001):
        assert height(lo+(hi-lo)*i/1000) >= -1e-12
    center = (lo+hi)/2
    assert abs(center-math.pi/3) < 1e-5
    assert height(center) > height(center-0.01)
    assert height(center) > height(center+0.01)
    if maximum == math.pi/2:
        assert abs(height(lo)) < 1e-12 and abs(height(hi)) < 1e-12
        assert height(lo-0.001) < 0 and height(hi+0.001) < 0
    else:
        assert abs((hi-lo)/2-maximum) < 1e-12
for invalid in ('0', '1.6', 'nan'):
    result = subprocess.run([sys.argv[1], '--check-j1-limits',
                             '--j1-max-from-vertical-rad='+invalid], capture_output=True)
    assert result.returncode != 0
print('J1 horizontal boundaries and tightened limits verified')
