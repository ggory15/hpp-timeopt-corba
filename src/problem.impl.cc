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
#include <hpp/corbaserver/timeopt/conversion.hh>

#include <hpp/corbaserver/timeopt/server.hh>

#include <hpp/model/joint.hh>
#include <hpp/model/device.hh>
#include <hpp/model/urdf/util.hh>
#include <hpp/model/center-of-mass-computation.hh>

using hpp::model::DevicePtr_t;
using hpp::model::JointPtr_t;

#include <yaml-cpp/yaml.h>
using namespace std;

namespace hpp {
  namespace timeopt {
      namespace impl {

        Problem::Problem () : server_ (0x0) {}
        ProblemSolverPtr_t Problem::problemSolver (){
            return server_->problemSolver ();
        }

        void Problem::setTimeOptPlanner(const char* cfg_path, const char* default_path) throw(hpp::Error){
            std::string cfg_string(cfg_path);
            std::string default_string(default_path);
            planner_setting_.initialize(cfg_string, default_string);
            contact_plan_.initialize(planner_setting_);
            ref_sequence_.resize(planner_setting_.get(PlannerIntParam_NumTimesteps));
        }        
        void Problem::setInitialBodyState() throw(hpp::Error){
            const DevicePtr_t& robot (problemSolver()->robot ());
            dynamic_state_.fillInitialBodyState(robot);
        }
        hpp::floatSeq* Problem::getEndeffectorTransform(const char* limb_name, const floatSeq& configuration) throw(hpp::Error){
            std::string name_string(limb_name);
            std::size_t Dim = (std::size_t)configuration.length();
            vector_t posture(Dim);
            posture = hpp::corbaServer::floatSeqToVector(configuration, Dim);

            const DevicePtr_t& robot (problemSolver()->robot ());
            robot->currentConfiguration(posture);
            robot->computeForwardKinematics();
            Transform3f tf = robot->getJointByName(name_string)->currentTransformation();

            hpp::floatSeq* dofArray = new hpp::floatSeq();
            dofArray->length(12);

            for (int i=0; i<3; i++)
                (*dofArray)[i] = tf.getRotation()(0, i);
            for (int i=0; i<3; i++)
                (*dofArray)[i+3] = tf.getRotation()(1, i);
            for (int i=0; i<3; i++)
                (*dofArray)[i+6] = tf.getRotation()(2, i);     
            for (int i=0; i<3; i++)
                (*dofArray)[i+9] = tf.getTranslation()(i);                                

            return dofArray;
        }

        void Problem::setInitialLimbState(const char* limb_name, CORBA::Boolean activation, CORBA::UShort ID, const hpp::floatSeq& force) throw(hpp::Error){
            std::string name_string(limb_name);
            dynamic_state_.fillInitialLimbState(problemSolver()->robot ()->getJointByName(name_string), ID, activation, hpp::corbaServer::floatSeqToVector3(force));
        }
        void Problem::addContactSequence(CORBA::UShort id, const hpp::floatSeqSeq& footstep) throw(hpp::Error){
            std::size_t configsDim = (std::size_t)footstep.length();

            FootPrints_t footPrints;
            vector_t foot(10);

            for (_CORBA_ULong i = 0; i < configsDim; i++){
                foot = hpp::corbaServer::floatSeqToVector(footstep[i], 10);
                footPrints.push_back(FootPrint(foot(0), foot(1), vector3_t(foot(2), foot(3),  foot(4)), quaternion_t(foot(5), foot(6), foot(7), foot(8)), foot(9)));
            }

            ContactSet con(id, footPrints);
            contact_plan_.addContact(con, dynamic_state_);
        }
        void Problem::setFinalBodyState(const hpp::floatSeq& com) throw(hpp::Error) {
            dynamic_state_.setFinalcom(hpp::corbaServer::floatSeqToVector3(com)); 
        }
        void Problem::setFinalBodyStatfromConfig(const hpp::floatSeq& configuration) throw(hpp::Error){
            std::size_t Dim = (std::size_t)configuration.length();
            vector_t posture(Dim);
            posture = hpp::corbaServer::floatSeqToVector(configuration, Dim);

            vector3_t com;
            const DevicePtr_t& robot (problemSolver()->robot ());
            com = robot->positionCenterOfMass();            
            dynamic_state_.setFinalcom(com);
        }
        void Problem::calculate() throw(hpp::Error){          
            dyn_optimizer_.initialize(planner_setting_, dynamic_state_, &contact_plan_);
            dyn_optimizer_.optimize(ref_sequence_);
        }
        CORBA::UShort Problem::getNumSeqeunce() throw(hpp::Error){
            return planner_setting_.get(PlannerIntParam_NumTimesteps);
        }
        hpp::floatSeq* Problem::getComFromRobot(const hpp::floatSeq& configuration) throw(hpp::Error){
            std::size_t Dim = (std::size_t)configuration.length();
            vector_t posture(Dim);
            posture = hpp::corbaServer::floatSeqToVector(configuration, Dim);

            vector3_t com;
            const DevicePtr_t& robot (problemSolver()->robot ());
            com = robot->positionCenterOfMass();            

            hpp::floatSeq* dofArray = new hpp::floatSeq();
            dofArray->length(3);
            for (int i=0; i<3; i++)
                (*dofArray)[i] = com(i);

            return dofArray;         
        }
        hpp::floatSeq* Problem::getResultantBodyDynamics(CORBA::UShort cnt) throw(hpp::Error){
            // time(1), reference com(3), linear(3), angular momentum(3);
            if (cnt > this->getNumSeqeunce())
                throw std::runtime_error ("The total sequence is smaller than time_id");
            
            hpp::floatSeq* dofArray = new hpp::floatSeq();
            dofArray->length(10);

            (*dofArray)[0] = dyn_optimizer_.dynamicsSequence().dynamicsState(cnt).time();
            for (int i=0; i<3; i++)
                (*dofArray)[i+1] = dyn_optimizer_.dynamicsSequence().dynamicsState(cnt).centerOfMass()(i);
            for (int i=0; i<3; i++)
                (*dofArray)[i+4] = dyn_optimizer_.dynamicsSequence().dynamicsState(cnt).linearMomentum()(i);
            for (int i=0; i<3; i++)
                (*dofArray)[i+7] = dyn_optimizer_.dynamicsSequence().dynamicsState(cnt).angularMomentum()(i);
            
            return dofArray;
        }
        hpp::floatSeq* Problem::getResultantLimbDynamics(CORBA::UShort cnt, CORBA::UShort id) throw(hpp::Error){
            hpp::floatSeq* dofArray = new hpp::floatSeq();
            dofArray->length(9);

            for (int i=0; i<3; i++)
                (*dofArray)[i] = dyn_optimizer_.dynamicsSequence().dynamicsState(cnt).endeffectorForce(id)(i);
            for (int i=0; i<3; i++)
                (*dofArray)[i+3] = dyn_optimizer_.dynamicsSequence().dynamicsState(cnt).endeffectorTorque(id)(i);
            for (int i=0; i<3; i++)
                (*dofArray)[i+6] = dyn_optimizer_.dynamicsSequence().dynamicsState(cnt).endeffectorCoP(id)(i);                                

            return dofArray;
        }

    } // namespace impl
  } // namespace wholebodyStep
} // namespace hpp
