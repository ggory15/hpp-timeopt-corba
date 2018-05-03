#Importing helper class for RBPRM
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer

#calling script darpa_hyq_path to compute root path
import config as tp
from hpp.corbaserver import Client


#packageName = "hyq_description"
packageName = "hpp-timeopt-corba"
meshPackageName = "red_description"
rootJointType = "freeflyer"

#  Information to retrieve urdf and srdf files.
urdfName = "red_robot"
#urdfName = "romeo_new"
urdfSuffix = ""
srdfSuffix = ""

#  This time we load the full body model of HyQ
fullBody = FullBody ()
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [-2,5, -1, 1, 0.3, 4])


#  Setting a number of sample configurations used
nbSamples = 10000

ps = tp.ProblemSolver(fullBody)
r = tp.Viewer (ps, viewerClient=tp.r.client)
r.loadObstacleModel ('hpp-timeopt-corba', "darpa", "contact")

rootName = 'base_joint_xyz'

cType = "_6_DOF"
rLegId = 'red_rleg'
rLeg = 'HipRoll_R_Joint'
#rLeg = 'RHipYaw'
#rfoot = 'AnkleRoll_R_Link'
rfoot =''
offset = [0, 0.0, 0.0]
normal = [0, 1.0, 0.0]
legx = 0.09; legy = 0.05

def addLimbDb(limbId, heuristicName, loadValues = True, disableEffectorCollision = False):
	fullBody.addLimbDatabase(str(db_dir+limbId+'.db'), limbId, heuristicName,loadValues, disableEffectorCollision)

fullBody.addLimb(rLegId,rLeg,rfoot,offset, normal, legx, legy, nbSamples, "manipulability", 0.03)

lLegId = 'red_lleg'
lLeg = 'HipRoll_L_Joint'
#lLeg = 'LHipYaw'
lfoot = ''
fullBody.addLimb(lLegId,lLeg, lfoot,offset, normal, legx, legy, nbSamples, "manipulability", 0.03)

#fullBody.runLimbSampleAnalysis(rLegId, "jointLimitsDistance", True)
#fullBody.runLimbSampleAnalysis(lLegId, "jointLimitsDistance", True)


q_0 = fullBody.getCurrentConfig();
q_init = fullBody.getCurrentConfig(); q_init[0:7] = tp.q_init[0:7]
q_goal = fullBody.getCurrentConfig(); q_goal[0:7] = tp.q_goal[0:7]

#q_init = q_goal;
q_init[0:7] = tp.q_init[0:7]
q_init[22:30] = [0.1754, 0.4363, 1.2453, -1.3963, -2.6180, 0, 0, 0];
q_init[32:40] = [-0.1754, -0.4363, -1.7453, 1.3963, 2.6180, 0, 0, 0];


q_goal[22:30] = [-0.9, 0.6, 0.6, 0.5, 1.5, 0, 0, 0];
q_goal[32:40] = [-0.9, 0.6, -0.6, 0.5, 1.5, 0, 0, 0];

#q_init[13:19] = [0, 0, -1.2, 1.3, -0.1, 0];


# Randomly generating a contact configuration at q_init
fullBody.setCurrentConfig (q_init)
q_init = fullBody.generateContacts(q_init, [0,0,1])

# Randomly generating a contact configuration at q_end
fullBody.setCurrentConfig (q_goal)
q_goal = fullBody.generateContacts(q_goal, [0,0,1])

# specifying the full body configurations as start and goal state of the problem
fullBody.setStartState(q_init,[])
fullBody.setEndState(q_goal,[rLegId,lLegId])

r(q_init)
configs = []


from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)

from hpp.corbaserver.timeopt.timeOptSolver import TimeOpt
from os import environ
import numpy as np

solver = TimeOpt()
ins_dir = environ['DEVEL_DIR']
db_dir = ins_dir+"/install/share/hpp-timeopt-corba/config/"

'''Initialize solver'''
#solver.setTimeOptPlanner(db_dir+"cfg_momTr_demo01.yaml", db_dir+"default_solver_setting.yaml");
#solver.setTimeOptPlanner(db_dir+"cfg_momSc_demo01.yaml", db_dir+"default_solver_setting.yaml");
solver.setTimeOptPlanner(db_dir+"cfg_timeopt_demo01.yaml", db_dir+"default_solver_setting.yaml");

'''Draw COM'''
solver.DrawCOMSphere(r)

import time
#DEMO METHODS

def initConfig():
	r.client.gui.setVisibility("red_robot", "ON")
	tp.cl.problem.selectProblem("default")
	tp.r.client.gui.setVisibility("toto", "OFF")
	tp.r.client.gui.setVisibility("red_trunk", "OFF")
	r(q_init)

def endConfig():
	r.client.gui.setVisibility("red_robot", "ON")
	tp.cl.problem.selectProblem("default")
	tp.r.client.gui.setVisibility("toto", "OFF")
	tp.r.client.gui.setVisibility("red_trunk", "OFF")
	r(q_goal)

def rootPath():
	r.client.gui.setVisibility("red_robot", "OFF")
	tp.cl.problem.selectProblem("rbprm_path")
	tp.r.client.gui.setVisibility("toto", "OFF")
	r.client.gui.setVisibility("red_robot", "OFF")
	tp.r.client.gui.setVisibility("red_trunk", "ON")
	tp.pp(0)
	tp.r.client.gui.setVisibility("red_trunk", "OFF")
	r.client.gui.setVisibility("red_robot", "ON")
	tp.cl.problem.selectProblem("default")

def genPlan():
	tp.cl.problem.selectProblem("default")
	r.client.gui.setVisibility("red_robot", "ON")
	tp.r.client.gui.setVisibility("toto", "OFF")
	tp.r.client.gui.setVisibility("red_trunk", "OFF")
	global configs
	start = time.clock()
	configs = fullBody.interpolate(0.1)
	#configs = fullBody.interpolate(0.12, 10, 10, True)
	end = time.clock()
	print "Contact plan generated in " + str(end-start) + "seconds"
def setInitConfigure():
    fullBody.setCurrentConfig(configs[0])

def contactPlan():
	r.client.gui.setVisibility("red_robot", "ON")
	tp.cl.problem.selectProblem("default")
	tp.r.client.gui.setVisibility("toto", "OFF")
	tp.r.client.gui.setVisibility("red_trunk", "OFF")
	for i in range(0,5):
         r(configs[i]);
         time.sleep(0.5);

def calculateUsingTimeOpt():
    setInitConfigure()
    solver.setInitialBodyState()
    solver.setInitialLimbState("AnkleRoll_R_Joint", True, 0, [0, 0, 0.5]);
    solver.setInitialLimbState("AnkleRoll_L_Joint", True, 1, [0, 0, 0.5]);
    solver.setInitialLimbState("J_rwrist2", False, 2, [0, 0, 0.0]);
    solver.setInitialLimbState("J_lwrist2", False, 3, [0, 0, 0.0]);

    ''' At this time, this demo try to generate COM trajectory using only three sequences'''
    ## to do
    s = (3, 10)
    global foot_rf
    global foot_lf
    foot_rf = np.zeros(s)
    foot_lf = np.zeros(s)

    foot_rf[0][0:2] = [0.0, 1.0]; #start time, end time
    foot_rf[1][0:2] = [2.0, 4.5];
    foot_rf[2][0:2] = [6.0, 9.9];

    foot_lf[0][0:2] = [0.0, 2.5];
    foot_lf[1][0:2] = [4.0, 6.5];
    foot_lf[2][0:2] = [8.5, 9.9];

    for i in range(0, 3):
        foot_rf[i][9] = 1 # contact type
        foot_lf[i][9] = 1
        [R, t] = solver.getEndeffectorTransform("AnkleRoll_R_Joint", configs[i*2])
        foot_rf[i][2:5] = t
        foot_rf[i][5:9] = solver.rot2quat(R)
        [R, t] = solver.getEndeffectorTransform("AnkleRoll_L_Joint", configs[i*2+1])
        foot_lf[i][2:5] = t
        foot_lf[i][5:9] = solver.rot2quat(R)

    solver.addContactSequence(0, foot_rf.tolist());
    solver.addContactSequence(1, foot_lf.tolist());
    solver.DrawDesiredContactSequence(r, 0, foot_rf)
    solver.DrawDesiredContactSequence(r, 1, foot_lf)
    '''Set Final state'''
    solver.setFinalBodyStatfromConfig(configs[5])
    '''Solve'''
    solver.calculate();

def drawPath():
	# if you want to show the result,
	# [time, com, linearmomentum, angularmomentum] = solver.getResultantBodyDynamics(cnt);
	# [ee's force, ee's torque, ee's cop] = solver.getResultantLimbDynamics(cnt, ee's id);
	current_time = 0.0;
	for i in range(0, solver.getNumSeqeunce()):
		(tick, com, lm, am) = solver.getResultantBodyDynamics(i)
		solver.UpdateCOMDisplay(com, r)
		current_time = current_time+ tick
		#print('time', round(current_time, 1), 'com', com)
		time.sleep(tick / 2.0) # for draw


def a():
	print "initial configuration"
	initConfig()

def b():
	print "end configuration"
	endConfig()

def c():
	print "displaying root path"
	rootPath()

def d():
	print "computing contact plan"
	genPlan()

def e():
	print "displaying contact plan"
	contactPlan()
def f():
	print "computing COM trajectory using time optimization"
	calculateUsingTimeOpt()

def g():
	print "draw COM"
	drawPath()

print "Root path generated in " + str(tp.t) + " ms."
d()
f()