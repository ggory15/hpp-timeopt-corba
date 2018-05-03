#!/usr/bin/env python
# Copyright (c) 2016 CNRS
# Author: Anna Seppala
#
# This file is part of hpp-affordance-corba.
# hpp-affordance-corba is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-affordance-corba is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-affordance-corba.  If not, see
# <http://www.gnu.org/licenses/>.

from hpp.corbaserver.timeopt import Client as WPGClient
from hpp.corbaserver import Client as BasicClient
import numpy as np
from math import sqrt
## Corba clients to the various servers
#
class CorbaClient:
    """
    Container for corba clients to various interfaces.
    """
    def __init__ (self):
        self.basic = BasicClient ()
        self.timeopt = WPGClient ()

## \brief Load and handle an AffordanceTool for analysis of the environment.
#
#  AffordanceTool offers a set of helper functions that combines the affordance
# analysis on the C++ side with visualisation tools. The objective is to make the
# analysis tools more intuitive for the user.

class TimeOpt (object):
    ## Constructor
    def __init__ (self):
        self.client = CorbaClient ()
    def setInitialBodyState (self):
        self.client.timeopt.problem.setInitialBodyState()
    def setTimeOptPlanner (self, cfg_path, default_path):
        self.client.timeopt.problem.setTimeOptPlanner(cfg_path, default_path)
    def setInitialLimbState (self, limb_name, activation, ID, force_ration):
        self.client.timeopt.problem.setInitialLimbState(limb_name, activation, ID, force_ration)
    def addContactSequence(self, id, footstep):
        self.client.timeopt.problem.addContactSequence(id, footstep)
    def calculate(self):
        self.client.timeopt.problem.calculate()
    def setFinalBodyState(self, com):
        self.client.timeopt.problem.setFinalBodyState(com)
    def setFinalBodyStatfromConfig(self, config):
        self.client.timeopt.problem.setFinalBodyStatfromConfig(config)    
    def getEndeffectorTransform(self, limb_name, config):
        A = self.client.timeopt.problem.getEndeffectorTransform(limb_name, config) 
        R = np.array ([ [A[0], A[1], A[2] ],
                        [A[3], A[4], A[5] ],
                        [A[6], A[7], A[8] ]]);
        t = np.array([A[9], A[10], A[11]])
        return (R, t)
    def loadObstacleModel(self, package, filename, prefix, Viewer, meshPackageName=None, guiOnly=False):
        Viewer.loadObstacleModel(package, filename, prefix, meshPackageName, guiOnly)
    def DrawFootStep(self, name, position, Viewer):
        Viewer.client.gui.addBox(name, 0.05, 0.05, 0.01, Viewer.color.blue)
        Viewer.client.gui.applyConfiguration(name, position)
        Viewer.client.gui.refresh()
    def DrawDesiredContactSequence(self, viewer, id, footstep):
        for cnt_num in range(0, len(footstep)):
            A = footstep[cnt_num].tolist()[2:9];
            A[2] = 0.05;
            A[3] = 1
            A[4] = 0
            A[5] = 0
            A[6] = 0
            #A[3] = footstep[cnt_num].tolist()[8];
            #for i in range(0, 3):
            #    A[4 + i] = footstep[cnt_num].tolist()[7 - i]
            self.DrawFootStep("0_scene_hpp_/" + "foot" + str(id) + str(cnt_num), A, viewer)

    def getNumSeqeunce(self):
        return self.client.timeopt.problem.getNumSeqeunce()
    
    def getResultantBodyDynamics(self, cnt):
        A = self.client.timeopt.problem.getResultantBodyDynamics(cnt)
        return (A[0], A[1:4], A[4:7], A[7:10])
    def getResultantLimbDynamics(self, cnt):
        A = self.client.timeopt.problem.getResultantLimbDynamics(cnt)
        return (A[0:3], A[3:6], A[6:9])       
    def DrawCOMSphere(self, Viewer):
        Viewer.client.gui.addSphere("0_scene_hpp_/COM", 0.03, Viewer.color.green)
        Viewer.client.gui.applyConfiguration("0_scene_hpp_/COM", [0, 0, 0, 1, 0, 0, 0])
        Viewer.client.gui.refresh()
    def UpdateCOMDisplay(self, pos, Viewer):
        Viewer.client.gui.applyConfiguration("0_scene_hpp_/COM", [pos[0], pos[1], pos[2], 1, 0, 0, 0])
        Viewer.client.gui.refresh()
    def getComFromRobot(self, configure):
        return self.client.timeopt.problem.getComFromRobot(configure)
    def rot2quat(self, R):
        T = R.transpose()
        den = np.array([1.0 + T[0, 0] - T[1, 1] - T[2, 2],
                        1.0 - T[0, 0] + T[1, 1] - T[2, 2],
                        1.0 - T[0, 0] - T[1, 1] + T[2, 2],
                        1.0 + T[0, 0] + T[1, 1] + T[2, 2]])

        max_idx = np.flatnonzero(den == max(den))[0]

        q = np.zeros(4)
        q[max_idx] = 0.5 * sqrt(max(den))
        denom = 4.0 * q[max_idx]
        if (max_idx == 0):
            q[1] = (T[1, 0] + T[0, 1]) / denom
            q[2] = (T[2, 0] + T[0, 2]) / denom
            q[3] = -(T[2, 1] - T[1, 2]) / denom
        if (max_idx == 1):
            q[0] = (T[1, 0] + T[0, 1]) / denom
            q[2] = (T[2, 1] + T[1, 2]) / denom
            q[3] = -(T[0, 2] - T[2, 0]) / denom
        if (max_idx == 2):
            q[0] = (T[2, 0] + T[0, 2]) / denom
            q[1] = (T[2, 1] + T[1, 2]) / denom
            q[3] = -(T[1, 0] - T[0, 1]) / denom
        if (max_idx == 3):
            q[0] = -(T[2, 1] - T[1, 2]) / denom
            q[1] = -(T[0, 2] - T[2, 0]) / denom
            q[2] = -(T[1, 0] - T[0, 1]) / denom
        q_res = np.zeros(4)
        q_res[0] = q[3]
        q_res[1] = q[0]
        q_res[2] = q[1]
        q_res[3] = q[2]
        return q_res    




