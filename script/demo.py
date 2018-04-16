from hpp.corbaserver.timeopt import Client
from hpp.corbaserver.robot import Robot as Parent
from hpp.corbaserver import ProblemSolver

class Robot (Parent):
	rootJointType = 'freeflyer'
	packageName = 'hpp-timeopt-corba'
	# URDF file describing the trunk of the robot HyQ
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

from hpp.corbaserver.timeopt.ComPath import COMPathTool
q_init = robot.getCurrentConfig ()

q_init =  [-2.0, 0, 0.8, 0, 0, 0, 1,
		   -0.18625990960985309, 0.0653641030070116, -0.23641850893545993, 1.25616964830234, -1.0197204334654386, -0.06533846873335888,
		   -0.07570903906743214, 0.17745077316021962, -0.9548127675314133, 1.3070203284158541,-0.35224411037341646,-0.17741381491935357,
           0, 0, 0,
           0.1754, 0.4363, 1.2453, -1.3963, -2.618, 0, 0, 0,
           0, 0,
		   -0.1754, -0.4363, -0.5453, 1.3963, 2.618, 0, 0, 0,
          ];

robot.setCurrentConfig (q_init)

from hpp.gepetto import Viewer
r = Viewer (ps)
r (q_init)

com = COMPathTool ()
from os import environ
ins_dir = environ['DEVEL_HPP_DIR']
db_dir = ins_dir+"/install/share/hpp-timeopt-corba/config/"

com.setPlanner(db_dir+"cfg_momTr_demo01.yaml", db_dir+"default_solver_setting.yaml");
#com.setPlanner(db_dir+"cfg_momSc_demo01.yaml", db_dir+"default_solver_setting.yaml");
#com.setPlanner(db_dir+"cfg_timeopt_demo01.yaml", db_dir+"default_solver_setting.yaml");
com.setInitialBodyState()
#id num -> rf:0, lf:1, rh:2, lh:3
com.setInitialLimbState("AnkleRoll_R_Joint", True, 0, [0,0,0.5]);
com.setInitialLimbState("AnkleRoll_L_Joint", True, 1, [0,0,0.5]);
com.setInitialLimbState("J_rwrist2", True, 2, [0,0,0.0]);
com.setInitialLimbState("J_lwrist2", True, 3, [0,0,0.0]);

#setdesiredfoot
com.setDesiredContactSequence();

#calculate and save COM path
com.saveCOMPath();

#draw COM Path
com.DrawDesiredContactSequence(r);
com.DrawCOMSphere(r);

import time
for i in range(0, 95):
    com.UpdateCOMDisplay(com.getresultCOMPos(i), r)
    print(com.getresultCOMPos(i))
    time.sleep(0.05)


