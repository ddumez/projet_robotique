import paving
from matplotlib import pyplot
from scipy.sparse.csgraph import connected_components
import numpy
from scipy.optimize import least_squares
import robot
from matplotlib import colors as mcolors

"""
p = paving.Paving()
p.load_mnf("simple.mnf")
m = p.adjacency_matrix()
nc,l = connected_components(m, directed=False)

fig1,ax1 = pyplot.subplots()
p.draw2D(ax1,1,2)
ax1.axis(p.hull([1,2]))
sp0 = p.subpaving([i for i in range(len(p.boxes)) if l[i]==0])
sp0.draw2D(ax1,1,2,ec=None,fc='yellow')
sp1 = p.subpaving([i for i in range(len(p.boxes)) if l[i]==1])
sp1.draw2D(ax1,1,2,ec=None,fc='red')
fig1.show()

fig2,ax2 = pyplot.subplots()
p.draw2D(ax2,2,3)
ax2.axis(p.hull([2,3]))
sp2 = p.subpaving([i for i in range(len(p.boxes)) if l[i]==0])
sp2.draw2D(ax2,2,3,ec=None,fc='yellow')
sp3 = p.subpaving([i for i in range(len(p.boxes)) if l[i]==1])
sp3.draw2D(ax2,2,3,ec=None,fc='red')
fig2.show()
"""

No = 27;
o = [ [-12, -6, 1.5] , [-12, 0, 1.1] , [-12, 6, 1.8] , [-6, -18, 2.8] , [-6, -12, 2.9] , [-6, -6, 0.6] , [-6, 0, 1.6] , [-6, 6, 0.4] , [-6, 12, 1.9] , [-6, 18, 0.4] , [0, -24, 2.4] , [0, -12, 1.9] , [0, -6, 0.5] , [0, 0, 1.8] , [0, 6, 2.9] , [0, 12, 1.7] , [0, 24, 0.5] , [6, -18, 1.9] , [6, -12, 2.8] , [6, -6, 0.4] , [6, 0, 1.9] , [6, 6, 0.3] , [6, 12, 2.3] , [6, 18, 1.2] , [12, -6, 2.8] , [12, 0, 1.3] , [12, 6, 2.1] ]

color = ['blue','green','red','cyan','magenta','yellow','black']

p = paving.Paving()
p.load_mnf("5R.mnf")
m = p.adjacency_matrix()
nc,l = connected_components(m, directed=False)

fig1,ax1 = pyplot.subplots()
p.draw2D(ax1,1,2)
ax1.axis(p.hull([1,2]))
for i in range(1,1,1):
	sp = p.subpaving([j for j in range(len(p.boxes)) if l[j]==i])
	sp.draw2D(ax1,1,2,ec=None)
for i in range(0,No,1):
	ax1.add_artist(pyplot.Circle((o[i][0],o[i][1]),o[i][2],color='.5',fill=False))
fig1.show()

"""
fig2,ax2 = pyplot.subplots()
p.draw2D(ax2,2,3)
ax2.axis(p.hull([2,3]))
for i in range(0,nc-1,1):
	sp = p.subpaving([j for j in range(len(p.boxes)) if l[j]==i])
	sp.draw2D(ax1,2,3,ec=None)
fig2.show()
"""
input("Press <ENTER> to continue...")