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
    def setFianlBodyState(self, com):
        self.client.timeopt.problem.setFianlBodyState(com)
    def loadObstacleModel(self, package, filename, prefix, Viewer, meshPackageName=None, guiOnly=False):
        Viewer.loadObstacleModel(package, filename, prefix, meshPackageName, guiOnly)
    def DrawFootStep(self, name, position, Viewer):
        Viewer.client.gui.addBox(name, 0.05, 0.05, 0.01, Viewer.color.blue)
        Viewer.client.gui.applyConfiguration(name, position)
        Viewer.client.gui.refresh()
    def DrawDesiredContactSequence(self, viewer):
        for id_num in range(0, 4):
            for cnt_num in range(0, self.getNumContact()[id_num]):
                self.DrawFootStep("0_scene_hpp_/" + "foot" + str(id_num) + str(cnt_num), self.getDesiredFootPosture(id_num, cnt_num), viewer)
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




