import paving
from matplotlib import pyplot
from scipy.sparse.csgraph import connected_components
import robot


#getting started partie 1
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

#getting started partie 2
"""
pi=3.141592653590

def convert(r):
	return 180 * (r/3.141592653590)

r=robot.FiveBars([-22.5, 0, 22.5, 0, 17.8, 17.8, 17.8,17.8],0,1)
r.pen_up()
r.go_home()
print(r.measure_pose())
r.pen_down()

#r.actuate([convert(-0.328197775093003) , convert(4.721570472915456) ])
#r.actuate([convert(-0.328197775093003) , convert(3.265547488610673) ])
#r.actuate([convert(-0.9293947977378646) , convert(4.721570472915456) ])
r.actuate([convert(-0.9293947977378646) , convert(3.265547488610673) ])

#r.actuate([convert(0.9293947977378629),convert(3.017637818568912)])
#r.actuate([convert(0.9293947977378629),convert(1.561614834264128)])
#r.actuate([convert(0.3281977750930011),convert(3.017637818568912)])
#r.actuate([convert(0.3281977750930011),convert(1.561614834264128)])

print(r.measure_pose())
r.ax.plot([5],[-20],color='black',marker='*')
r.ax.plot([5],[20],color='black',marker='*')
r.pen_down()
input("Press <ENTER> to continue...")
"""

#calibration