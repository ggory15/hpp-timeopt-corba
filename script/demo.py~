from hpp.corbaserver.timeopt import Client
from hpp.corbaserver.robot import Robot as Parent
from hpp.corbaserver import ProblemSolver

class Robot (Parent):
	rootJointType = 'freeflyer'
	packageName = 'hpp-timeopt-corba'
	urdfName = 'red_robot'
	urdfSuffix = ""
	srdfSuffix = ""
	def __init__ (self, robotName, load = True):
		Parent.__init__ (self, robotName, self.rootJointType, load)
		self.tf_root = "base_footprint"
		self.client.basic = Client ()

c1 = Client()
robot = Robot ("red");
ps = ProblemSolver (robot)

from hpp.corbaserver.timeopt.timeOptSolver import TimeOpt
q_init = robot.getCurrentConfig ()

q_init =  [-2.0, 0, 0.8, 1, 0, 0, 0, #Quaternion Notation is w, x, y, z
		   -0.18625990960985309, 0.0653641030070116, -0.23641850893545993, 1.25616964830234, -1.0197204334654386, -0.06533846873335888,
		   -0.07570903906743214, 0.17745077316021962, -0.9548127675314133, 1.3070203284158541,-0.35224411037341646,-0.17741381491935357,
           0, 0, 0,
           0.1754, 0.4363, 1.2453, -1.3963, -2.618, 0, 0, 0,
           0, 0,
		   -0.1754, -0.4363, -0.5453, 1.3963, 2.618, 0, 0, 0,
          ];

robot.setCurrentConfig (q_init)

print("Import Viewer")
from hpp.gepetto import Viewer
r = Viewer (ps)
r (q_init)

print("Create TimeOptimization Solver")
solver = TimeOpt()

from os import environ
ins_dir = environ['DEVEL_DIR']
db_dir = ins_dir+"/install/share/hpp-timeopt-corba/config/"

'''Initialize solver'''
solver.setTimeOptPlanner(db_dir+"cfg_momTr_demo01.yaml", db_dir+"default_solver_setting.yaml");
#solver.setTimeOptPlanner(db_dir+"cfg_momSc_demo01.yaml", db_dir+"default_solver_setting.yaml");
#solver.setTimeOptPlanner(db_dir+"cfg_timeopt_demo01.yaml", db_dir+"default_solver_setting.yaml");

'''Set Initial State of Robot'''
solver.setInitialBodyState()
#id num -> rf:0, lf:1, rh:2, lh:3
solver.setInitialLimbState("AnkleRoll_R_Joint", True, 0, [0,0,0.5]);
solver.setInitialLimbState("AnkleRoll_L_Joint", True, 1, [0,0,0.5]);
solver.setInitialLimbState("J_rwrist2", False, 2, [0,0,0.0]);
solver.setInitialLimbState("J_lwrist2", False, 3, [0,0,0.0]);

'''Set desired contact Sequence'''
foot_rf = [(0.0, 1.0, -1.82, -0.021, 0.15, 1.0, 0.0, 0.0, 0.0, 1),
		   (2.0, 4.5, -1.55, -0.500, 0.3, 1.0, 0.0, 0.0, 0.0, 1),
		   (6.0, 9.9, -1.02, -0.450, 0.8, 1.0, 0.0, 0.0, 0.0, 1)]
foot_lf = [(0.0, 2.5, -2.2, 0.17, 0.15, 1.0, 0.0, 0.0, 0.0, 1),
		   (4.0, 6.5, -1.5, 0.17, 0.5, 1.0, 0.0, 0.0, 0.0, 1),
		   (8.5, 9.9, -0.9, 0.17, 0.8, 1.0, 0.0, 0.0, 0.0, 1)]

solver.addContactSequence(0, foot_rf);
solver.addContactSequence(1, foot_lf);

'''Set Final state'''
solver.setFianlBodyState([-0.94, -0.2, 1.4]);

'''Solve'''
solver.calculate();

# if you want to show the result,
# [time, com, linearmomentum, angularmomentum] = solver.getResultantBodyDynamics(cnt);
# [ee's force, ee's torque, ee's cop] = solver.getResultantLimbDynamics(cnt, ee's id);
'''Draw COM'''
solver.DrawCOMSphere(r)
import time
def drawPath():
	current_time = 0.0;
	for i in range(0, solver.getNumSeqeunce()):
		(tick, com, lm, am) = solver.getResultantBodyDynamics(i)
		solver.UpdateCOMDisplay(com, r)
		current_time = current_time+ tick
		print('time', round(current_time, 1), 'com', com)
		time.sleep(tick / 2.0) # for draw

def a():
	print "Draw Com Path"
	drawPath()

a()
