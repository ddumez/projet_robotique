import paving
from matplotlib import pyplot
from scipy.sparse.csgraph import connected_components
import numpy
from scipy.optimize import least_squares
import robot
from scipy.optimize import root

pi=3.141592653590

def convert(r):
	return 180 * (r/3.141592653590)


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
"""

#calibration

def make_measurements(r2,commands):
    r2.actuate(commands[0])
    r2.pen_down()
    measures=[]
    print('   Taking measures ...')
    for q in commands:
        r2.actuate(q)
        x = r2.measure_pose()
        r2.ax.plot([x[0]],[x[1]],color='black',marker='*')
        measures.append((x,q))
    r2.pen_up()
    r2.go_home()
    return measures

def calibrate(kinematic_functions,nominal_architecture,measures):
    # error function
    def errors(a):
        err=[]
        for (x,q) in measures:
            for fi in kinematic_functions(a,x,q):
                err.append(fi)
        return err
    print('   Calibration processing ...')
    sol = least_squares(errors,nominal_architecture)
    print('   status : ',sol.message)
    print('   error : ',sol.cost)
    print('   result : ',sol.x)
    return sol.x

def f_5R(architecture,pose,command):
    [o11,o12,o21,o22,l1,l2,l3,l4] = architecture
    [x1,x2] = pose
    [q1,q2] = numpy.radians(command)
    f1 = numpy.square((o11+numpy.cos(q1)*l1-x1)) + numpy.square((o12+numpy.sin(q1)*l1-x2)) - numpy.square(l2)
    f2 = numpy.square((o21+numpy.cos(q2)*l4-x1)) + numpy.square((o22+numpy.sin(q2)*l4-x2)) - numpy.square(l3)
    return [f1,f2]

r=robot.FiveBars([-22.5, 0, 22.5, 0, 17.8, 17.8, 17.8,17.8],0,1)
nominal_architecture = [-22.5, 0, 22.5, 0, 17.8, 17.8, 17.8,17.8]
r.pen_up()
r.go_home()
"""
commands = []
commands = commands + [[0,q] for q in range(80,220,20)] + [[q,180] for q in range(-40,110,20)]
commands = commands + [[40,q] for q in range(80,220,20)] + [[q,140] for q in range(-40,110,20)]
commands = commands + [[q,180-q] for q in range(-40,80,10)]

commands2 = []
commands2 = commands2 + [[10,q] for q in range(80,220,10)] + [[q,190] for q in range(-40,100,10)]
commands2 = commands2 + [[q,180+q] for q in range(-40,40,10)]

#calcul de l'architecture reele
measures = make_measurements(r,commands)
real_architechture = calibrate(f_5R,[-22.5, 0, 22.5, 0, 17.8, 17.8, 17.8,17.8],measures)


#test de la diference avec une pop test
measures2 = make_measurements(r,commands2)
defaut = numpy.max([ numpy.max(f_5R(real_architechture, measures2[i][0], measures2[i][1])) for i in range(0,36,1)])
"""
defaut = 0.73943572916198264
real_architecture  = [-22.46174947,   0.03725057,  22.60218071,  -0.19941578, 17.79576115,  17.7979901 ,  17.89124913,  17.50228139]

#affichage
print("architecture : ",real_architecture)
print("erreur max : ", defaut)

# Path following
def lemniscate(t):
	r = 5
	return (r*numpy.cos(t), -20+r*numpy.sin(t))

def discretize(r2,trajectory,tmin,tmax,steps):
	target_path = [trajectory(t) for t in numpy.linspace(tmin,tmax,steps)]
	r2.ax.plot([x[0] for x in target_path],[x[1] for x in target_path],color='blue',linestyle=':',marker='+')
	r2.refresh()
	return target_path

target_path = discretize(r,lemniscate,0,2*numpy.pi,50)
x0 = target_path[0]

#quatre solutions possibles
q0 = numpy.degrees([0.1184982872423866, 3.009659697680965]) #ok coude en haut + coude en haut => c'est celle là la bonne
#q0 = numpy.degrees([0.1184982872423866, -1.850133485777623]) # coude en haut + coude en bas
#q0 = numpy.degrees([-1.29836667963693675, 3.009659697680965]) #ok coude en bas + coude en haut
#q0 = numpy.degrees([-1.29836667963693675,-1.850133485777623]) #coude en bas + coude en bas
r.actuate(q0)

def continuation(function_xq,target_path,q0):
    qs=[]
    qk=q0
    for xk in target_path:
        res = root(lambda q: function_xq(xk,q), qk)
        if not res.success:
            print(res.message)
            break
        qk = res.x
        qs.append(qk)
    return qs

commands = continuation(lambda x,q:f_5R(nominal_architecture,x,q),target_path,q0)
commands2 = continuation(lambda x,q:f_5R(real_architecture,x,q),target_path,q0)

def draw_path(r,commands,col='blue'):
    r.pen_up()
    r.go_home()
    r.actuate(commands[0])
    r.pen_down(col)
    real_path = []
    for q in commands:
        r.actuate(q)
        real_path.append(numpy.array(r.measure_pose()))
    r.pen_up()
    r.go_home()
    return real_path

real_path = draw_path(r,commands,col='red')
real_path2 = draw_path(r,commands2,col='green')



input("Press <ENTER> to continue...")
