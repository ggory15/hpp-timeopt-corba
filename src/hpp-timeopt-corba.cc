// Copyright (C) 2013 CNRS-LAAS
// Author: Florent Lamiraux
//
// This file is part of the hpp-wholebody-step-corba.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#include <iostream>
#include <hpp/util/debug.hh>
#include <hpp/core/problem-solver.hh>
#include <hpp/corbaserver/server.hh>
#include <hpp/corbaserver/timeopt/server.hh>
//#include <hpp/wholebody-step/small-steps.hh>

typedef hpp::timeopt::Server timeoptServer;
typedef hpp::corbaServer::Server CorbaServer;
typedef hpp::timeopt::ProblemSolverPtr_t ProblemSolverPtr_t;
typedef hpp::timeopt::ProblemSolver ProblemSolver;

using namespace hpp::timeopt;

int main (int argc, const char* argv[])
{
  ProblemSolverPtr_t problemSolver = ProblemSolver::create ();
  
  CorbaServer corbaServer (problemSolver, argc, argv, true);
  timeoptServer wpgServer (argc, argv, true);
  wpgServer.setProblemSolverMap (corbaServer.problemSolverMap());

  try {
    corbaServer.startCorbaServer ();
    hppDout (info, "successfully start hpp-corbaserver");
  } catch (const std::exception& exc) {
    hppDout (error, "Faile to start hpp-corbaserver");
  }
  try {
    wpgServer.startCorbaServer ("hpp", "corbaserver",  "timeopt", "problem");
    hppDout (info, "Successfully started corba server for timeopt");
  } catch (const std::exception& exc) {
    hppDout (error, "failed to start corba server for timeopt");
  }
  corbaServer.processRequest(true);
}
