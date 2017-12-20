import paving
from matplotlib import pyplot
from scipy.sparse.csgraph import connected_components
import numpy
from scipy.optimize import least_squares
import robot
from matplotlib import colors as mcolors

from scipy.sparse.csgraph import dijkstra

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

"""
fig1,ax1 = pyplot.subplots()
p.draw2D(ax1,1,2)
ax1.axis(p.hull([1,2]))
for i in range(1,1,1):
	sp = p.subpaving([j for j in range(len(p.boxes)) if l[j]==i])
	sp.draw2D(ax1,1,2,ec=None)
for i in range(0,No,1):
	ax1.add_artist(pyplot.Circle((o[i][0],o[i][1]),o[i][2],color='.5',fill=False))
fig1.show()

fig2,ax2 = pyplot.subplots()
p.draw2D(ax2,2,3)
ax2.axis(p.hull([2,3]))
for i in range(0,nc-1,1):
	sp = p.subpaving([j for j in range(len(p.boxes)) if l[j]==i])
	sp.draw2D(ax1,2,3,ec=None)
fig2.show()
"""
def path_planner(pav,nei,Bori,Bdes):
    print('Building shortest path from ',Bori,' to ',Bdes)
    # construction of the shortest path
    path=[]
    dist,pred = dijkstra(nei,return_predecessors=True,indices=Bori)
    # checking existence of a compatible destination box
    if dist[Bdes]!=numpy.inf:
        print('   shortest path found!')
        # collecting the path from the predecessors tree
        path=[Bdes]
        while path[0]!=Bori:
            path.insert(0,pred[path[0]])
    else:
        print('   impossible to connect boxes!')
    return path

r=robot.FiveBars([-22.5, 0, 22.5, 0, 17.8, 17.8, 17.8,17.8],0,1)
calibrated_architecture = [-22.46174947,   0.03725057,  22.60218071,  -0.19941578, 17.79576115,  17.7979901 ,  17.89124913,  17.50228139]
Xori,Xdes = [0,-15], [0,15]
r.ax.plot([Xori[0],Xdes[0]],[Xori[1],Xdes[1]],linestyle=':',marker='*',color='.3')
r.refresh()

paving_5R = paving.Paving()
paving_5R.load_mnf('5R.mnf') #Manque ajout des singularités parallèles
neighborhood = paving_5R.adjacency_matrix()

ori = list(paving_5R.boxes_intersecting(Xori,d=[1,2]))
des = list(paving_5R.boxes_intersecting(Xdes,d=[1,2]))
shortest_path=path_planner(paving_5R,neighborhood,ori[0],des[0])

from scipy.sparse.csgraph import connected_components

nc,l = connected_components(neighborhood, directed=False)
for b in [i for i in range(len(paving_5R.boxes)) if l[i]==l[ori[0]]]:
    paving_5R.boxes[b].draw2D(r.ax,1,2)
for b in [i for i in range(len(paving_5R.boxes)) if l[i]!=l[ori[0]]]:
    paving_5R.boxes[b].draw2D(r.ax,1,2,ec='magenta')
r.refresh()
del r
r=robot.FiveBars([-22.5, 0, 22.5, 0, 17.8, 17.8, 17.8,17.8],0,1)
shortest_path=path_planner(paving_5R,neighborhood,ori[0],des[0])
#obstacles = put_obstacles(r)

def display_path(rob,pav,spath,bcol='yellow',rcol='cyan'):
    def midq(b):
        return numpy.degrees([(b.vec[4]+b.vec[5])/2,(b.vec[6]+b.vec[7])/2])
    if spath!=[]:
        rob.pen_up()
        rob.actuate(midq(pav.boxes[spath[0]]))
        rob.pen_down(rcol)
        for b in spath:
            if bcol!=None:
                pav.boxes[b].draw2D(rob.ax,1,2,ec=bcol,fc=bcol)
            rob.actuate(midq(pav.boxes[b]))
        rob.pen_up()
        rob.go_home()

display_path(r,paving_5R,shortest_path)
neighborhood = paving_5R.adjacency_matrix(weight=lambda x,y : paving.projected_centers_distance(x,y,[1,2]))

input("Press <ENTER> to continue...")
