import paving
from matplotlib import pyplot
from scipy.sparse.csgraph import connected_components
import numpy
from scipy.optimize import least_squares
import robot
from matplotlib import colors as mcolors

from scipy.sparse.csgraph import dijkstra


#affichage de l'espace de travail et de l'espace des configurations

No = 27;
o = [ [-12, -6, 1.5] , [-12, 0, 1.1] , [-12, 6, 1.8] , [-6, -18, 2.8] , [-6, -12, 2.9] , [-6, -6, 0.6] , [-6, 0, 1.6] , [-6, 6, 0.4] , [-6, 12, 1.9] , [-6, 18, 0.4] , [0, -24, 2.4] , [0, -12, 1.9] , [0, -6, 0.5] , [0, 0, 1.8] , [0, 6, 2.9] , [0, 12, 1.7] , [0, 24, 0.5] , [6, -18, 1.9] , [6, -12, 2.8] , [6, -6, 0.4] , [6, 0, 1.9] , [6, 6, 0.3] , [6, 12, 2.3] , [6, 18, 1.2] , [12, -6, 2.8] , [12, 0, 1.3] , [12, 6, 2.1] ]

"""
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

def midq(b):
    return numpy.degrees([(b.vec[4]+b.vec[5])/2,(b.vec[6]+b.vec[7])/2])

def display_path(rob,pav,spath,bcol='yellow',rcol='cyan'):
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

#defintion du probleme
r=robot.FiveBars([-22.5, 0, 22.5, 0, 17.8, 17.8, 17.8,17.8],0,1)
Xori,Xdes = [0,-15], [0,15]
r.ax.plot([Xori[0],Xdes[0]],[Xori[1],Xdes[1]],linestyle=':',marker='*',color='.3')
r.refresh()

#recuperation des calculs fait avec ibex
paving_5R = paving.Paving()
paving_5R.load_mnf('5R.mnf')
neighborhood = paving_5R.adjacency_matrix()

#recherche des boites qui contiennent l'origine et la destination
ori = list(paving_5R.boxes_intersecting(Xori,d=[1,2]))
des = list(paving_5R.boxes_intersecting(Xdes,d=[1,2]))

#test de chacun des couple depart/arrive et du mode d'assemblage associe pour trouver un chemin valide
i = 0
shortest_path = []
while (i < len(ori)) and not(shortest_path):
    r.actuate(midq(paving_5R.boxes[ori[i]]))
    if (abs(r.measure_pose()[0] - Xori[0]) + abs(r.measure_pose()[0] - Xori[0]) <= 1): #si on est asse proche alors on est dans le meme mode d'assemblage
        j = 0
        while (j < len(des)) and not(shortest_path):
            shortest_path = path_planner(paving_5R,neighborhood,ori[i],des[j])
            j = j+1
    r.go_home()
    i = i+1
print(shortest_path)

#affichage des obstacles
for i in range(0,No,1):
    r.ax.add_artist(pyplot.Circle((o[i][0],o[i][1]),o[i][2],color='.5',fill=False))

#suivis du chemin trouve
display_path(r,paving_5R,shortest_path)
neighborhood = paving_5R.adjacency_matrix(weight=lambda x,y : paving.projected_centers_distance(x,y,[1,2]))

#fin
input("Press <ENTER> to continue...")
