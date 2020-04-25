import json
import sys
import numpy as np
from numpy.polynomial.polynomial import polyfit
import matplotlib.pyplot as plt

with open(sys.argv[1]) as json_file:
    data = json.load(json_file)

polydata = []
for pts in data['label']:
    x = []
    y = []
    for pt in pts['points']:
        x.append(pt[0])
        y.append(pt[1])
    plt.plot(x, y, 'x')
    b, m = polyfit(np.array(x), np.array(y), 1)
    polydata.append((b, m))

l = len(polydata) - 1
for i in range(l):
    prevp = polydata[i]
    nextp = polydata[i + 1]
    polydata.append((0.5 * (prevp[0] + nextp[0]), 0.5 * (prevp[1] + nextp[1])))

y = [1210., 750.]
data_out = dict()
lines = []
for poly in polydata:
    x = []
    for yi in y:
        x.append((yi - poly[0]) / poly[1])
    ang = np.arctan(poly[1])
    step = 1. * np.cos(ang)
    x = np.arange(x[0], x[1], step)
    y_eval = poly[0] + poly[1] * x
    plt.plot(x, y_eval, '.-')
    line = []
    for xi, yi in zip(x ,y_eval):
        pt = dict()
        pt['x'] = xi
        pt['y'] = yi
        line.append(pt)
    lines.append(line)

plt.show()

data_out['lines'] = lines
with open('/tmp/lines.json', 'w') as jf:
    json.dump(data_out, jf, indent=4)
