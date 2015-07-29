/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Luis G. Torres, Jonathan Gammell */

#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include <boost/make_shared.hpp>
#if OMPL_HAVE_EIGEN3
#include "ompl/base/samplers/informed/PathLengthDirectInfSampler.h"
#else
#include "ompl/base/samplers/informed/RejectionInfSampler.h"
#endif

//STa
#include "ompl/base/samplers/informed/RejectionInfSampler.h"

ompl::base::PathLengthOptimizationObjective::
PathLengthOptimizationObjective(const SpaceInformationPtr &si) :
    ompl::base::OptimizationObjective(si)
{
    description_ = "Path Length";

	//Test to use the objective in BIT*
	costToGoFn_ = boost::bind(&PathLengthOptimizationObjective::costToGo, this, _1, _2);
}

ompl::base::Cost ompl::base::PathLengthOptimizationObjective::stateCost(const State *s) const
{
    return identityCost();
}

ompl::base::Cost ompl::base::PathLengthOptimizationObjective::motionCost(const State *s1, const State *s2) const
{
    return Cost(si_->distance(s1, s2));
}

ompl::base::Cost ompl::base::PathLengthOptimizationObjective::motionCostHeuristic(const State *s1, const State *s2) const
{
    return motionCost(s1, s2);
}

ompl::base::InformedSamplerPtr ompl::base::PathLengthOptimizationObjective::allocInformedStateSampler(const ProblemDefinitionPtr probDefn, unsigned int maxNumberCalls) const
{
    return boost::make_shared<RejectionInfSampler>(probDefn, maxNumberCalls);
//    // Make the direct path-length informed sampler and return. If OMPL was compiled with Eigen, a direct version is available, if not a rejection-based technique can be used
//#if OMPL_HAVE_EIGEN3
//    return boost::make_shared<PathLengthDirectInfSampler>(probDefn, maxNumberCalls);
//#else
//    throw Exception("Direct sampling of the path-length objective requires Eigen, but this version of OMPL was compiled without Eigen support. If possible, please install Eigen and recompile OMPL. If this is not possible, you can manually create an instantiation of RejectionInfSampler to approximate the behaviour of direct informed sampling.");
//    // Return a null pointer to avoid compiler warnings
//    return ompl::base::InformedSamplerPtr();
//#endif
}

//Test to use the objective in BIT*
ompl::base::Cost ompl::base::PathLengthOptimizationObjective::costToGo(const State *state, const Goal *goal) const
{
	Cost bestGoalStateCost = infiniteCost();
    const GoalStates *gs = goal->hasType(GOAL_STATES) ? goal->as<GoalStates>() : NULL;
    if (gs)
    {
    	for (size_t i=0; i < gs->getStateCount(); ++i)
    	{
    		Cost currentGoalStateCost = stateCost(gs->getState(i));
    		if (isCostBetterThan(currentGoalStateCost, bestGoalStateCost))
    			bestGoalStateCost = currentGoalStateCost;
    	}
    	return combineCosts(stateCost(state), bestGoalStateCost);
    }
    else
    	return infiniteCost();
}
