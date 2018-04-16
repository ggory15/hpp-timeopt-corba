// Copyright (C) 2011,2012,2013,2014 CNRS-LAAS
// Author: Florent Lamiraux.
//
// This file is part of the hpp-wholebody-step-corba.
//
// hpp-wholebody-step-corba is free software: you can redistribute
// it and/or modify it under the terms of the GNU Lesser General
// Public License as published by the Free Software Foundation, either
// version 3 of the License, or (at your option) any later version.
//
// hpp-wholebody-step-corba is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with hpp-wholebody-step-corba.  If not, see
// <http://www.gnu.org/licenses/>.

#include "problem.impl.hh"

#include <cassert>
#include <hpp/util/debug.hh>

#include <hpp/core/config-projector.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/problem-solver.hh>

#include <hpp/corbaserver/conversions.hh>
#include <hpp/corbaserver/timeopt/server.hh>

#include <yaml-cpp/yaml.h>
#include <hpp/pinocchio/center-of-mass-computation.hh>

using hpp::pinocchio::CenterOfMassComputation;

namespace hpp {
  namespace timeopt {
      namespace impl {

      Problem::Problem () : server_ (0x0) {}
      ProblemSolverPtr_t Problem::problemSolver ()
      {
        return server_->problemSolver ();
      }

      void Problem::setPlanner(const char* cfg_path, const char* default_path) throw(hpp::Error){
          std::string cfg_string(cfg_path);
          std::string default_string(default_path);
          planner_setting_.initialize(cfg_string, default_string);
      }

      void Problem::setInitialBodyState() throw(hpp::Error){
          const DevicePtr_t& robot (problemSolver()->robot ());
          dynamic_state_.fillInitialBodyState(robot);
      }
      void Problem::setInitialLimbState(const char* limb_name, CORBA::Boolean activation, CORBA::UShort ID, const hpp::floatSeq& force) throw(hpp::Error){
          std::string name_string(limb_name);
          dynamic_state_.fillInitialLimbState(problemSolver()->robot ()->getJointByName(name_string), activation, ID, hpp::corbaServer::floatSeqToVector3(force));
       }
      void Problem::setDesiredContactSequence() throw(hpp::Error){
          contact_plan_.initialize(planner_setting_);
          contact_plan_.optimize(dynamic_state_);
      }
      void Problem::saveCOMPath() throw(hpp::Error){          
          DynamicsSequence ref_sequence;
          ref_sequence.resize(planner_setting_.get(PlannerIntParam_NumTimesteps));

          dyn_optimizer_.initialize(planner_setting_, dynamic_state_, &contact_plan_);
          dyn_optimizer_.optimize(ref_sequence);
      }
      hpp::intSeq* Problem::getNumContact() throw(hpp::Error){
          hpp::intSeq* dofArray = new hpp::intSeq();
          dofArray->length(4);
          for (int i=0; i<4; i++)
            (*dofArray)[i] = contact_plan_.endeffectorContacts()(i);
          return dofArray;
      }
      hpp::floatSeq* Problem::getDesiredFootPos(CORBA::UShort id, CORBA::UShort cnt) throw(hpp::Error){
          hpp::floatSeq* dofArray = new hpp::floatSeq();
          dofArray->length(7);
          //for (int i=0; i<3; i++)
          //    (*dofArray)[i] = contact_plan_.contactSequence().endeffectorContacts(id)[cnt].contactPosition()[i];

          (*dofArray)[0] = contact_plan_.contactSequence().endeffectorContacts(id)[cnt].contactPosition()[0];
          (*dofArray)[1] = contact_plan_.contactSequence().endeffectorContacts(id)[cnt].contactPosition()[1];
          (*dofArray)[2] = contact_plan_.contactSequence().endeffectorContacts(id)[cnt].contactPosition()[2];
          (*dofArray)[3] = contact_plan_.contactSequence().endeffectorContacts(id)[cnt].contactOrientation().x();
          (*dofArray)[4] = contact_plan_.contactSequence().endeffectorContacts(id)[cnt].contactOrientation().y();
          (*dofArray)[5] = contact_plan_.contactSequence().endeffectorContacts(id)[cnt].contactOrientation().z();
          (*dofArray)[6] = contact_plan_.contactSequence().endeffectorContacts(id)[cnt].contactOrientation().w();

          return dofArray;
      }
      hpp::floatSeq* Problem::getCOMPos(CORBA::UShort cnt) throw(hpp::Error){
          hpp::floatSeq* dofArray = new hpp::floatSeq();
          dofArray->length(3);

          for (int i=0; i<3; i++)
              (*dofArray)[i] = dyn_optimizer_.dynamicsSequence().dynamicsState(cnt).centerOfMass()(i);

          return dofArray;
      }

    } // namespace impl
  } // namespace wholebodyStep
} // namespace hpp
