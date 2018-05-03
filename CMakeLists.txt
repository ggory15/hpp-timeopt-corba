# Copyright (c) 2011 CNRS
# Author: Florent Lamiraux
#
# This file is part of hpp-wholebody-step-corba.
# hpp-wholebody-step-corba is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-wholebody-step-corba is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-wholebody-step-corba.  If not, see
# <http://www.gnu.org/licenses/>.

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CXX_DISABLE_WERROR true)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/idl.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAME hpp-timeopt-corba)
SET(PROJECT_DESCRIPTION "Corba server and python client for hpp-timeopt")
SET(PROJECT_URL "http://github.com/laas/hpp-wholebody-step-corba")

SET(CLIENT_ONLY FALSE CACHE BOOL "Set to true to install the client only")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -g -Wall -Wwrite-strings -pedantic -O3 -funroll-loops -fPIC")

SET(${PROJECT_NAME}_HEADERS
  include/hpp/corbaserver/timeopt/server.hh
  include/hpp/corbaserver/timeopt/fwd.hh
  include/hpp/corbaserver/timeopt/conversion.hh
)



SET(CUSTOM_HEADER_DIR hpp/corbaserver/timeopt)

SETUP_PROJECT()

# Activate hpp-util logging if requested
SET (HPP_DEBUG FALSE CACHE BOOL "trigger hpp-util debug output")
IF (HPP_DEBUG)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_DEBUG")
ENDIF()

# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("omniORB4 >= 4.1.4")
ADD_REQUIRED_DEPENDENCY("hpp-corbaserver >= 3")
ADD_REQUIRED_DEPENDENCY("eigen3 >= 3.0")
ADD_REQUIRED_DEPENDENCY("yaml-cpp >= 0.5")

IF (NOT CLIENT_ONLY)
  ADD_DOC_DEPENDENCY ("hpp-core >= 3")
  ADD_REQUIRED_DEPENDENCY("hpp-timeopt")
  ADD_REQUIRED_DEPENDENCY("hpp-template-corba >= 1.1")
  ADD_REQUIRED_DEPENDENCY("hpp-util >= 3")
ENDIF (NOT CLIENT_ONLY)

# Make sure that python submodule is installed in the same prefix as module.
IF (NOT ${HPP_CORBASERVER_PREFIX} STREQUAL ${CMAKE_INSTALL_PREFIX})
  MESSAGE(WARNING
    "  THIS PACKAGE SHOULD BE INSTALLED IN THE SAME PREFIX AS HPP-CORBASERVER,\n"
    "  SINCE IT INSTALLS A PYTHON SUBMODULE OF HPP.CORBASERVER.")
  MESSAGE("CMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}")
  MESSAGE("HPP_CORBASERVER_PREFIX=${HPP_CORBASERVER_PREFIX}")
ENDIF()

ADD_SUBDIRECTORY(src)

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

SET(CATKIN_PACKAGE_SHARE_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})

install(FILES
  data/package.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
install(FILES
  data/urdf/red_robot.urdf
  data/urdf/darpa.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  )
install(FILES
  data/srdf/red_robot.srdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srdf
  )
install(FILES
	data/meshes/darpareduced.stl
	data/meshes/AnkleCenter_L_Link.stl
	data/meshes/AnkleCenter_R_Link.stl
	data/meshes/AnkleRoll_L_Link.stl
	data/meshes/AnkleRoll_R_Link.stl
	data/meshes/head.stl
	data/meshes/HipCenter_L_Link.stl
	data/meshes/HipCenter_R_Link.stl
	data/meshes/HipRoll_L_Link.stl
	data/meshes/HipRoll_R_link.stl
	data/meshes/Knee_L_Link.stl
	data/meshes/Knee_R_Link.stl
	data/meshes/L_arm.stl
	data/meshes/L_elbow.stl
	data/meshes/L_forearm.stl
	data/meshes/L_shoulder1.stl
	data/meshes/L_shoulder2.stl
	data/meshes/L_shoulder3.stl
	data/meshes/L_wrist1.stl
	data/meshes/L_wrist2.stl
	data/meshes/neck.stl
	data/meshes/Pelvis_link.stl
	data/meshes/R_arm.stl
	data/meshes/R_elbow.stl
	data/meshes/R_forearm.stl
	data/meshes/R_shoulder1.stl
	data/meshes/R_shoulder2.stl
	data/meshes/R_shoulder3.stl
	data/meshes/R_wrist1.stl
	data/meshes/R_wrist2.stl
	data/meshes/Thigh_L_Link.stl
        data/meshes/Thigh_R_Link.stl
        data/meshes/upperbody.stl
	data/meshes/waist1.stl
	data/meshes/waist2.stl
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes	
  )
install(FILES
  config/cfg_momSc_demo01.yaml
  config/cfg_momTr_demo01.yaml
  config/cfg_timeopt_demo01.yaml
  config/default_solver_setting.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  )


SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
