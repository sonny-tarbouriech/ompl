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

#include "ompl/base/objectives/ManipulabilityObjective.h"
#include "ompl/tools/config/MagicConstants.h"
#include "ompl/geometric/PathGeometric.h"
#include <limits>

//STa test
#include <fstream>
#include <ompl/util/Time.h>

ompl::base::ManipulabilityObjective::
ManipulabilityObjective(const SpaceInformationPtr &si) :
	OptimizationObjective(si)
{
    this->setCostThreshold(identityCost());

    //TODO : return error if si_ has not a SafeStateValidityChecker
        ssvc_ = dynamic_cast<ompl::base::SafeStateValidityChecker*>(si_->getStateValidityChecker().get());
}


ompl::base::ManipulabilityObjective::
ManipulabilityObjective(const SpaceInformationPtr &si, double cost) :
	OptimizationObjective(si)
{
	this->setCostThreshold(Cost(cost));

	//TODO : return error if si_ has not a SafeStateValidityChecker
	    ssvc_ = dynamic_cast<ompl::base::SafeStateValidityChecker*>(si_->getStateValidityChecker().get());
}

bool ompl::base::ManipulabilityObjective::isSymmetric() const
{
	return true;
}

ompl::base::Cost ompl::base::ManipulabilityObjective::stateCost(const State *s) const
{
	return Cost(ssvc_->manipulability(s));
}

bool ompl::base::ManipulabilityObjective::isCostBetterThan(Cost c1, Cost c2) const
{
    return c1.value() > c2.value() + magic::BETTER_PATH_COST_MARGIN;
}

ompl::base::Cost ompl::base::ManipulabilityObjective::motionCost(const State *s1, const State *s2) const
{
//	//STa test singularity
//	std::string homepath = getenv("HOME");
//	std::ofstream output_file((homepath + "/singularity_time.txt").c_str(), std::ios::out | std::ios::app);
//	std::ofstream output_file_2((homepath + "/singularity_result.txt").c_str(), std::ios::out | std::ios::app);
//	ompl::time::point init = ompl::time::now();
//	float MAX_DANGER_DIST, ALPHA;
//	double min_dist = ssvc_->manipulability(s1, s2);
//	MAX_DANGER_DIST = 0.05, ALPHA = 1;
//	double c1 = 1 - (1 / (1 + exp((min_dist*(2/MAX_DANGER_DIST)-1)*ALPHA)));
//	ompl::time::duration dur = ompl::time::now() - init;
//	MAX_DANGER_DIST = 0.1, ALPHA = 1;
//	double c2 = 1 - (1 / (1 + exp((min_dist*(2/MAX_DANGER_DIST)-1)*ALPHA)));
//	MAX_DANGER_DIST = 0.2, ALPHA = 1;
//	double c3 = 1 - (1 / (1 + exp((min_dist*(2/MAX_DANGER_DIST)-1)*ALPHA)));
//	MAX_DANGER_DIST = 1, ALPHA = 1;
//	double c4 = 1 - (1 / (1 + exp((min_dist*(2/MAX_DANGER_DIST)-1)*ALPHA)));
//	MAX_DANGER_DIST = 2, ALPHA = 1;
//	double c5 = 1 - (1 / (1 + exp((min_dist*(2/MAX_DANGER_DIST)-1)*ALPHA)));
//	output_file << ompl::time::seconds(dur) << "\n";
//	output_file_2 << min_dist << "  " << c1 << "  " << c2 << "  " << c3 << "  " << c4 << "  " << c5  << "\n";
//	return Cost(c3);

//	double min_dist = ssvc_->manipulability(s1, s2);
//
//	return Cost(min_dist);

	Cost worstCost = this->identityCost();
	int nd = si_->getStateSpace()->validSegmentCount(s1, s2) ;
	if (nd > 1)
	{
		State *test = si_->allocState();
		for (int j = 1; j < nd; ++j)
		{
			si_->getStateSpace()->interpolate(s1, s2, (double) j / (double) nd, test);
			Cost testStateCost = this->stateCost(test);
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


ompl::base::Cost ompl::base::ManipulabilityObjective::combineCosts(Cost c1, Cost c2) const
{
    if (this->isCostBetterThan(c1, c2))
        return c2;
    else
        return c1;
}

ompl::base::Cost ompl::base::ManipulabilityObjective::identityCost() const
{
	return Cost(std::numeric_limits<double>::infinity());
}

ompl::base::Cost ompl::base::ManipulabilityObjective::infiniteCost() const
{
    return Cost(0);
}
