#######################################################
# TwoBars robot demo
import numpy

#~ import demo-calibration

##################################################
# Definition of a RR robot with approximate architecture
import robot

# robot creation routine
def create_2R(nominal_architecture,manufacture_tolerance=0,measurement_precision=0):
    r2 = robot.TwoBars(nominal_architecture,seed=1,man_var=manufacture_tolerance/2,mes_var=measurement_precision/2,eps_cmd=10)
    return r2

#~ nominal_architecture = [0,0,3,2]
#~ r2 = demo.create_2R(nominal_architecture,0.2,0.02)

##################################################
# CALIBRATION

# Actuation of the robot in order to generate measures for calibration
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

#~ commands = [[q,q] for q in range(0,1,1)]
#~ measures = demo.make_measurements(r2,commands)

# RR kinematic functions
def f_RR(architecture,pose,command):
    [a1,a2,a3,a4] = architecture
    [x1,x2] = pose
    [q1,q2] = numpy.radians(command)
    f1 = a1+a3*numpy.cos(q1)+a4*numpy.cos(q2) - x1
    f2 = a2+a3*numpy.sin(q1)+a4*numpy.sin(q2) - x2
    return [f1,f2]

# calibration from measurements
from scipy.optimize import least_squares

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

#~ demo.calibrate(demo.f_RR,nominal_architecture,measures)
