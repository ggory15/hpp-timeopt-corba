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

#ifndef HPP_TIMEOPT_CORBA_PROBLEM_HH
# define HPP_TIMEOPT_CORBA_PROBLEM_HH

# include <omniORB4/CORBA.h>
# include <hpp/corbaserver/timeopt/fwd.hh>
# include <hpp/corbaserver/timeopt/problem.hh>
# include <hpp/timeopt/momentumopt/dynopt/DynamicsOptimizer.hpp>
# include <hpp/timeopt/momentumopt/cntopt/ContactPlanFromFile.hpp>

namespace hpp {
  namespace timeopt {
    namespace impl {
      class Problem : public virtual
      POA_hpp::corbaserver::timeopt::Problem
      {
      public:
            Problem ();
            void setPlanner(const char* cfg_path, const char* file_name) throw(hpp::Error);
            virtual void setInitialBodyState() throw (hpp::Error);
            virtual void setInitialLimbState(const char* limb_name, CORBA::Boolean activation, CORBA::UShort ID, const hpp::floatSeq& force) throw(hpp::Error);
            virtual void setDesiredContactSequence() throw (hpp::Error);
            virtual void saveCOMPath() throw (hpp::Error);
            hpp::intSeq* getNumContact() throw (hpp::Error);
            hpp::floatSeq* getDesiredFootPos(CORBA::UShort id, CORBA::UShort cnt) throw(hpp::Error);
            hpp::floatSeq* getCOMPos(CORBA::UShort cnt) throw(hpp::Error);
        void setServer (Server* server)
        {
          server_ = server;
        }
      private:
        core::ProblemSolverPtr_t problemSolver();
        PlannerSetting planner_setting_;
        DynamicsState dynamic_state_;
        ContactPlanFromFile contact_plan_;
        DynamicsOptimizer dyn_optimizer_;
        Server* server_;
      };
    } // namespace impl
  } // namespace timeopt
} // namespace hpp

#endif //HPP_WHOLEBODY_STEP_CORBA_WHOLEBODY_STEP_HH
