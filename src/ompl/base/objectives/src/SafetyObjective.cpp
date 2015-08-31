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

/* Author: Luis G. Torres */

#include "ompl/base/objectives/SafetyObjective.h"
#include "ompl/tools/config/MagicConstants.h"
#include "ompl/geometric/PathGeometric.h"
#include <limits>

//STa test
#include <fstream>
#include <ompl/util/Time.h>



ompl::base::SafetyObjective::
SafetyObjective(const SpaceInformationPtr &si) :
SafetyObjective(si, NULL, false, 0.01, identityCost())
{
}

ompl::base::SafetyObjective::
SafetyObjective(const SpaceInformationPtr &si, ompl::base::SafeMotionValidator* smv) :
SafetyObjective(si, smv, false, 0.01, identityCost())
{
}

ompl::base::SafetyObjective::
SafetyObjective(const SpaceInformationPtr &si, ompl::base::SafeMotionValidator* smv, bool fast_dist, double travel_dist_limit) :
SafetyObjective(si, smv, fast_dist, travel_dist_limit, identityCost())
{
}

ompl::base::SafetyObjective::
SafetyObjective(const SpaceInformationPtr &si, ompl::base::SafeMotionValidator* smv, bool fast_dist, double travel_dist_limit, Cost cost_threshold) :
	OptimizationObjective(si),
	smv_(smv),
	fast_dist_(fast_dist),
	travel_dist_limit_(travel_dist_limit)
{
    isMinMaxObjective_ = true;
	this->setCostThreshold(cost_threshold);

	//TODO : return error if si_ has not a SafeStateValidityChecker
	ssvc_ = static_cast<ompl::base::SafeStateValidityChecker*>(si_->getStateValidityChecker().get());
}


bool ompl::base::SafetyObjective::isSymmetric() const
{
	return true;
}

ompl::base::Cost ompl::base::SafetyObjective::stateCost(const State *s) const
{
	double object_danger_factor;
	return Cost(ssvc_->computeRobotMinObstacleDistIndividualLinks(s, fast_dist_, object_danger_factor));
}

ompl::base::Cost ompl::base::SafetyObjective::stateCost(const State *s, double& object_danger_factor) const
{
	return Cost(ssvc_->computeRobotMinObstacleDistIndividualLinks(s, fast_dist_, object_danger_factor));
}

bool ompl::base::SafetyObjective::isCostBetterThan(Cost c1, Cost c2) const
{
    return c1.value() > c2.value() + magic::BETTER_PATH_COST_MARGIN;
}

bool ompl::base::SafetyObjective::isCostBetterThan(Cost c1, double factor1, Cost c2, double factor2) const
{
	return c1.value() * factor1 > c2.value() * factor2 + magic::BETTER_PATH_COST_MARGIN;
}

ompl::base::Cost ompl::base::SafetyObjective::motionCost(const State *s1, const State *s2) const
{
//	//STa test distance_motion
//	std::string homepath = getenv("HOME");
//	std::ofstream output_file((homepath + "/min_obs_dist_time.txt").c_str(), std::ios::out | std::ios::app);
//	std::ofstream output_file_2((homepath + "/min_obs_dist_result.txt").c_str(), std::ios::out | std::ios::app);
//	ompl::time::point init = ompl::time::now();
//	double min_obstacle_dist_dynamic_1 = smv_->minObstacleDistMotionIndividualObjects(s1,s2,0.1, false);
//	if (min_obstacle_dist_dynamic_1 < 0)
//		min_obstacle_dist_dynamic_1 =0;
//	ompl::time::duration dyn1 = ompl::time::now() - init;
//	double min_obstacle_dist_dynamic_2 = smv_->minObstacleDistMotionIndividualObjects(s1,s2,0.01, false);
//	if (min_obstacle_dist_dynamic_2 < 0)
//		min_obstacle_dist_dynamic_2 =0;
//	ompl::time::duration dyn2 = ompl::time::now() - init - dyn1;
//	double min_obstacle_dist_discrete_1 = smv_->minObstacleDistMotionDiscrete(s1,s2,1,false);
//	if (min_obstacle_dist_discrete_1 < 0)
//		min_obstacle_dist_discrete_1 =0;
//	ompl::time::duration dis1 = ompl::time::now() - init - dyn1 - dyn2;
//	double min_obstacle_dist_discrete_2 = smv_->minObstacleDistMotionDiscrete(s1,s2,5,false);
//	if (min_obstacle_dist_discrete_2 < 0)
//		min_obstacle_dist_discrete_2 =0;
//	ompl::time::duration dis2 = ompl::time::now() - init - dyn1 - dyn2 - dis1;
//	double min_obstacle_dist_discrete_3 = smv_->minObstacleDistMotionDiscrete(s1,s2,100,false);
//	if (min_obstacle_dist_discrete_3 < 0)
//		min_obstacle_dist_discrete_3 =0;
//	output_file << ompl::time::seconds(dyn1) << "  " << ompl::time::seconds(dyn2)<< "  " << ompl::time::seconds(dis1)<< "  " << ompl::time::seconds(dis2) << "\n";
//	output_file_2 << min_obstacle_dist_dynamic_1 << "  " << min_obstacle_dist_dynamic_2 << "  " << min_obstacle_dist_discrete_1 << "  " << min_obstacle_dist_discrete_2 << "  " << min_obstacle_dist_discrete_3  << "\n";
//	output_file.close();
//	output_file_2.close();
//	return motionCost(min_obstacle_dist_dynamic_1);

//	//STa test safety
//	std::string homepath = getenv("HOME");
//	std::ofstream output_file((homepath + "/safety_time.txt").c_str(), std::ios::out | std::ios::app);
//	std::ofstream output_file_2((homepath + "/safety_result.txt").c_str(), std::ios::out | std::ios::app);
//	ompl::time::point init = ompl::time::now();
//	double min_obstacle_dist = smv_->minObstacleDistMotionIndividualObjects(s1,s2,0.01, true);
//	double c1 = motionCost(min_obstacle_dist, 0.01, 5).value();
//	if (c1 < 0)
//		c1 =0;
//	ompl::time::duration dur = ompl::time::now() - init;
//	double c2 = motionCost(min_obstacle_dist, 0.05, 5).value();
//	if (c2 < 0)
//		c2=0;
//	double c3 = motionCost(min_obstacle_dist, 0.1, 5).value();
//	if (c3 < 0)
//		c3=0;
//	double c4 = motionCost(min_obstacle_dist, 0.5, 5).value();
//	if (c4 < 0)
//		c4 =0;
//	double c5 = motionCost(min_obstacle_dist, 1, 5).value();
//	if (c5 < 0)
//		c5 =0;
//	output_file << ompl::time::seconds(dur) << "\n";
//	output_file_2 << min_obstacle_dist<< "  " << c1 << "  " << c2 << "  " << c3 << "  " << c4 << "  " << c5  << "\n";
//	return Cost(c3);

	double factor;
	return Cost(smv_->minObstacleDistMotionIndividualObjects(s1,s2,travel_dist_limit_, fast_dist_, factor));

}

ompl::base::Cost ompl::base::SafetyObjective::motionCost(const State *s1, const State *s2, double& object_danger_factor) const
{
//	//STa test distance_motion
//	std::string homepath = getenv("HOME");
////	std::ofstream output_file((homepath + "/min_obs_dist_time.txt").c_str(), std::ios::out | std::ios::app);
//	std::ofstream output_file_2((homepath + "/min_obs_dist_result.txt").c_str(), std::ios::out | std::ios::app);
//	ompl::time::point init = ompl::time::now();
//	double min_obstacle_dist_dynamic_1 = smv_->minObstacleDistMotionIndividualObjects(s1,s2,travel_dist_limit_, fast_dist_, object_danger_factor);
//	if (min_obstacle_dist_dynamic_1 < 0)
//		min_obstacle_dist_dynamic_1 =0;
//	ompl::time::duration dyn1 = ompl::time::now() - init;
//
//	double min_obstacle_dist_discrete_1 = smv_->minObstacleDistMotionDiscrete(s1,s2,1,false);
//	if (min_obstacle_dist_discrete_1 < 0)
//		min_obstacle_dist_discrete_1 =0;
//	ompl::time::duration dis1 = ompl::time::now() - init - dyn1 ;
//
////	output_file << ompl::time::seconds(dyn1) << "  " << ompl::time::seconds(dyn2)<< "  " << ompl::time::seconds(dis1)<< "  " << ompl::time::seconds(dis2) << "\n";
//	if (min_obstacle_dist_dynamic_1 > min_obstacle_dist_discrete_1)
//		output_file_2 << min_obstacle_dist_dynamic_1 << "  " << min_obstacle_dist_discrete_1 <<  "\n";
////	output_file.close();
//	output_file_2.close();

	return Cost(smv_->minObstacleDistMotionIndividualObjects(s1,s2,travel_dist_limit_, fast_dist_, object_danger_factor));
}

ompl::base::Cost ompl::base::SafetyObjective::motionCostInterpolation(const State *s1, const State *s2) const
{
	Cost worstCost = this->identityCost();

	int nd = 10 * si_->getStateSpace()->validSegmentCount(s1, s2);

	if (nd > 1)
	{
		State *test = si_->allocState();
		for (int j = 1; j < nd; ++j)
		{
			si_->getStateSpace()->interpolate(s1, s2, (double) j / (double) nd, test);
			Cost testStateCost = Cost(ssvc_->computeRobotExactMinObstacleDist(test));

			if (this->isCostBetterThan(worstCost, testStateCost))
				worstCost = testStateCost;
		}
		si_->freeState(test);
	}

	// Lastly, check s2
	Cost lastCost = this->stateCost(s2);
	if (this->isCostBetterThan(worstCost, lastCost))
		worstCost = lastCost;

	return worstCost;

}

ompl::base::Cost ompl::base::SafetyObjective::combineCosts(Cost c1, Cost c2) const
{
    if (this->isCostBetterThan(c1, c2))
        return c2;
    else
        return c1;
}

ompl::base::Cost ompl::base::SafetyObjective::combineCosts(Cost c1, double factor1, Cost c2, double factor2, double& object_danger_factor) const
{
    if (this->isCostBetterThan(c1, factor1, c2, factor2))
    {
    	object_danger_factor = factor2;
        return c2;
    }
    else
    {
    	object_danger_factor = factor1;
        return c1;
    }
}

ompl::base::Cost ompl::base::SafetyObjective::identityCost() const
{
	return Cost(std::numeric_limits<double>::infinity());
}

ompl::base::Cost ompl::base::SafetyObjective::infiniteCost() const
{
    return Cost(0);
}
