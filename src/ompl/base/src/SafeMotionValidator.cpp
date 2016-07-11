//STa
#include "ompl/base/SafeMotionValidator.h"
#include "ompl/util/Exception.h"
#include <queue>


#include "ompl/base/SpaceInformation.h"

void ompl::base::SafeMotionValidator::defaultSettings()
{
    stateSpace_ = si_->getStateSpace().get();
    if (!stateSpace_)
        throw Exception("No state space for motion validator");

    ssvc_ = dynamic_cast<ompl::base::SafeStateValidityChecker*>(si_->getStateValidityChecker().get());
    if (!ssvc_)
    	throw Exception("SafeMotionValidator needs a SafeStateValidityChecker to perform");
}

// Function copied from DiscreteMotionValidator to compare results and performances
bool ompl::base::SafeMotionValidator::checkMotion(const State *s1, const State *s2) const
{
   return checkMotion(s1,s2,1);
}

bool  ompl::base::SafeMotionValidator::checkMotion(const State *s1, const State *s2, double valid_segment_factor) const
{
	 /* assume motion starts in a valid configuration so s1 is valid */
	    if (!si_->isValid(s2))
	    {
	        invalid_++;
	        return false;
	    }

	    bool result = true;
	    int nd = stateSpace_->validSegmentCount(s1, s2) * valid_segment_factor;

	    /* initialize the queue of test positions */
	    std::queue< std::pair<int, int> > pos;
	    if (nd >= 2)
	    {
	        pos.push(std::make_pair(1, nd - 1));

	        /* temporary storage for the checked state */
	        State *test = si_->allocState();

	        /* repeatedly subdivide the path segment in the middle (and check the middle) */
	        while (!pos.empty())
	        {
	            std::pair<int, int> x = pos.front();

	            int mid = (x.first + x.second) / 2;
	            stateSpace_->interpolate(s1, s2, (double)mid / (double)nd, test);

	            if (!si_->isValid(test))
	            {
	                result = false;
	                break;
	            }

	            pos.pop();

	            if (x.first < mid)
	                pos.push(std::make_pair(x.first, mid - 1));
	            if (x.second > mid)
	                pos.push(std::make_pair(mid + 1, x.second));
	        }

	        si_->freeState(test);
	    }

	    if (result)
	        valid_++;
	    else
	        invalid_++;

	    return result;
}

bool ompl::base::SafeMotionValidator::checkMotion(const State *s1, const State *s2, std::pair<State*, double> &lastValid) const
{
	/* assume motion starts in a valid configuration so s1 is valid */

	bool result = true;
	int nd = stateSpace_->validSegmentCount(s1, s2);

	if (nd > 1)
	{
		/* temporary storage for the checked state */
		State *test = si_->allocState();

		for (int j = 1 ; j < nd ; ++j)
		{
			stateSpace_->interpolate(s1, s2, (double)j / (double)nd, test);
			if (!si_->isValid(test))
			{
				lastValid.second = (double)(j - 1) / (double)nd;
				if (lastValid.first)
					stateSpace_->interpolate(s1, s2, lastValid.second, lastValid.first);
				result = false;
				break;
			}
		}
		si_->freeState(test);
	}

	if (result)
		if (!si_->isValid(s2))
		{
			lastValid.second = (double)(nd - 1) / (double)nd;
			if (lastValid.first)
				stateSpace_->interpolate(s1, s2, lastValid.second, lastValid.first);
			result = false;
		}

	if (result)
		valid_++;
	else
		invalid_++;

	return result;
}

bool ompl::base::SafeMotionValidator::checkMotionIndividualLinksWithDist(const State *s1, const State *s2, double travel_dist_limit, double& min_obstacle_dist, bool fast_dist)
{
	std::priority_queue<SubSegment> ss_queue;
	min_obstacle_dist = std::numeric_limits<double>::infinity();

	std::vector<std::vector<double> > dist_s1_obs, dist_s2_obs;
	std::vector<double> dist_s1_self, dist_s2_self;
	std::vector<double> dist_travel;
	ssvc_->computeInitialDistDataObstacle(s1, s2, dist_s1_obs, dist_s2_obs, fast_dist);
	ssvc_->computeInitialDistDataSelf(s1, s2, dist_s1_self, dist_s2_self);
	ssvc_->computeInitialDistDataTravelModulation(s1, s2, dist_travel);

	for (size_t i = 0; i < ssvc_->getNbSafetyLinks(); ++i)
	{
		for (size_t j = 0; j < ssvc_->getNbObjects(); ++j)
		{

			if ((dist_s1_obs[i][j] <= 0) || (dist_s2_obs[i][j] <= 0) )
			{
				min_obstacle_dist = -1;
				return false;
			}
			SubSegment ss;
			ss.self_cc_ = false;
			ss.t_s1_ = 0;
			ss.t_s2_ = 1;
			ss.link_index_ = i;
			ss.object_index_ = j;
			ss.dist_s1_obs_ = dist_s1_obs[i][j];
			ss.dist_s2_obs_ = dist_s2_obs[i][j];
			ss.non_covered_length_ = dist_travel[i] - (dist_s1_obs[i][j] + dist_s2_obs[i][j]);
			ss_queue.push(ss);
		}
		if ((dist_s1_self[i] <= 0) || (dist_s2_self[i] <= 0))
		{
			min_obstacle_dist = -1;
			return false;
		}
		if (dist_travel[i] - (dist_s1_self[i] + dist_s2_self[i]) > 0)
		{
			SubSegment ss;
			ss.self_cc_ = true;
			ss.t_s1_ = 0;
			ss.t_s2_ = 1;
			ss.link_index_ = i;
			ss.dist_s1_obs_ = dist_s1_self[i];
			ss.dist_s2_obs_ = dist_s2_self[i];
			ss.non_covered_length_ = dist_travel[i] - (dist_s1_self[i] + dist_s2_self[i]);
			ss_queue.push(ss);
		}

	}
	bool flag = false;
	while (!flag && !ss_queue.empty())
	{
		SubSegment current_ss = ss_queue.top();
		ss_queue.pop();

		State *s_new = si_->allocState();

		double t = (2*current_ss.dist_s1_obs_ + current_ss.non_covered_length_) / (2*(current_ss.dist_s1_obs_ + current_ss.dist_s2_obs_ + current_ss.non_covered_length_));

		double t_abs = current_ss.t_s1_ + (current_ss.t_s2_ - current_ss.t_s1_) * t;


		stateSpace_->interpolate(s1, s2, t_abs, s_new);

		if (!current_ss.self_cc_)
		{
			double dist_snew_obs = ssvc_->computeLinkMinObstacleDist(s_new, current_ss.link_index_, current_ss.object_index_, fast_dist);
			if (dist_snew_obs <= 0)
			{
				min_obstacle_dist = -1;
				return false;
			}
			SubSegment ss1,ss2;

			ss1.self_cc_ = current_ss.self_cc_;
			ss1.t_s1_ = current_ss.t_s1_;
			ss1.t_s2_ = t_abs;
			ss1.link_index_ = current_ss.link_index_;
			ss1.object_index_ = current_ss.object_index_;
			ss1.dist_s1_obs_ = current_ss.dist_s1_obs_;
			ss1.dist_s2_obs_ = dist_snew_obs;
			ss1.non_covered_length_ = dist_travel[ss1.link_index_]*(ss1.t_s2_ - ss1.t_s1_) - (ss1.dist_s1_obs_ + ss1.dist_s2_obs_);
			ss_queue.push(ss1);

			ss2.self_cc_ = current_ss.self_cc_;
			ss2.t_s1_ = t_abs;
			ss2.t_s2_ = current_ss.t_s2_;
			ss2.link_index_ = current_ss.link_index_;
			ss2.object_index_ = current_ss.object_index_;
			ss2.dist_s1_obs_ = dist_snew_obs;
			ss2.dist_s2_obs_ = current_ss.dist_s2_obs_;
			ss2.non_covered_length_ = dist_travel[ss2.link_index_]*(ss2.t_s2_ - ss2.t_s1_) - (ss2.dist_s1_obs_ + ss2.dist_s2_obs_);
			ss_queue.push(ss2);
		}
		else
		{
			double dist_snew_self = ssvc_->computeLinkMinSelfDist(s_new, current_ss.link_index_);
			if (dist_snew_self <= 0)
			{
				min_obstacle_dist = -1;
				return false;
			}
			SubSegment ss1,ss2;

			ss1.self_cc_ = current_ss.self_cc_;
			ss1.t_s1_ = current_ss.t_s1_;
			ss1.t_s2_ = t_abs;
			ss1.link_index_ = current_ss.link_index_;
			ss1.object_index_ = current_ss.object_index_;
			ss1.dist_s1_obs_ = current_ss.dist_s1_obs_;
			ss1.dist_s2_obs_ = dist_snew_self;
			ss1.non_covered_length_ = dist_travel[ss1.link_index_]*(ss1.t_s2_ - ss1.t_s1_) - (ss1.dist_s1_obs_ + ss1.dist_s2_obs_);
			if (ss1.non_covered_length_ > 0)
				ss_queue.push(ss1);

			ss2.self_cc_ = current_ss.self_cc_;
			ss2.t_s1_ = t_abs;
			ss2.t_s2_ = current_ss.t_s2_;
			ss2.link_index_ = current_ss.link_index_;
			ss2.object_index_ = current_ss.object_index_;
			ss2.dist_s1_obs_ = dist_snew_self;
			ss2.dist_s2_obs_ = current_ss.dist_s2_obs_;
			ss2.non_covered_length_ = dist_travel[ss2.link_index_]*(ss2.t_s2_ - ss2.t_s1_) - (ss2.dist_s1_obs_ + ss2.dist_s2_obs_);
			if (ss2.non_covered_length_ > 0)
				ss_queue.push(ss2);
		}

		si_->freeState(s_new);

		if (!ss_queue.empty())
			flag = dist_travel[ss_queue.top().link_index_]*(ss_queue.top().t_s2_ - ss_queue.top().t_s1_) < travel_dist_limit;

	}

	if (!ss_queue.empty())
	{
		if (ss_queue.top().non_covered_length_ > 0)
		{
			min_obstacle_dist = -1;
			return false;
		}
		else
		{
			min_obstacle_dist = -ss_queue.top().non_covered_length_/2;
		}
	}
	return true;
}

bool ompl::base::SafeMotionValidator::checkMotionIndividualLinks(const State *s1, const State *s2, double travel_dist_limit, bool fast_dist)
{

	std::priority_queue<SubSegment> ss_queue;

	std::vector<std::vector<double> > dist_s1_obs, dist_s2_obs;
	std::vector<double> dist_s1_self, dist_s2_self;
	std::vector<double> dist_travel;
	ssvc_->computeInitialDistDataObstacle(s1, s2, dist_s1_obs, dist_s2_obs, fast_dist);
	ssvc_->computeInitialDistDataSelf(s1, s2, dist_s1_self, dist_s2_self);
	ssvc_->computeInitialDistDataTravelModulation(s1, s2, dist_travel);

	for (size_t i = 0; i < ssvc_->getNbSafetyLinks(); ++i)
	{
		for (size_t j = 0; j < ssvc_->getNbObjects(); ++j)
		{

			if ((dist_s1_obs[i][j] <= 0) || (dist_s2_obs[i][j] <= 0) )
			{
				return false;
			}
			if (dist_travel[i] - (dist_s1_obs[i][j] + dist_s2_obs[i][j]) > 0)
			{
				SubSegment ss;
				ss.self_cc_ = false;
				ss.t_s1_ = 0;
				ss.t_s2_ = 1;
				ss.link_index_ = i;
				ss.object_index_ = j;
				ss.dist_s1_obs_ = dist_s1_obs[i][j];
				ss.dist_s2_obs_ = dist_s2_obs[i][j];
				ss.non_covered_length_ = dist_travel[i] - (dist_s1_obs[i][j] + dist_s2_obs[i][j]);
				ss_queue.push(ss);
			}
		}
		if ((dist_s1_self[i] <= 0) || (dist_s2_self[i] <= 0))
		{
			return false;
		}
		if (dist_travel[i] - (dist_s1_self[i] + dist_s2_self[i]) > 0)
		{
			SubSegment ss;
			ss.self_cc_ = true;
			ss.t_s1_ = 0;
			ss.t_s2_ = 1;
			ss.link_index_ = i;
			ss.dist_s1_obs_ = dist_s1_self[i];
			ss.dist_s2_obs_ = dist_s2_self[i];
			ss.non_covered_length_ = dist_travel[i] - (dist_s1_self[i] + dist_s2_self[i]);
			ss_queue.push(ss);
		}

	}
	bool flag = false;
	while (!flag && !ss_queue.empty())
	{
		SubSegment current_ss = ss_queue.top();
		ss_queue.pop();

		State *s_new = si_->allocState();

		double t = (2*current_ss.dist_s1_obs_ + current_ss.non_covered_length_) / (2*(current_ss.dist_s1_obs_ + current_ss.dist_s2_obs_ + current_ss.non_covered_length_));
		double t_abs = current_ss.t_s1_ + (current_ss.t_s2_ - current_ss.t_s1_) * t;


		stateSpace_->interpolate(s1, s2, t_abs, s_new);

		if (!current_ss.self_cc_)
		{
			double dist_snew_obs = ssvc_->computeLinkMinObstacleDist(s_new, current_ss.link_index_, current_ss.object_index_, fast_dist);
			if (dist_snew_obs <= 0)
			{
				return false;
			}
			SubSegment ss1,ss2;

			ss1.self_cc_ = current_ss.self_cc_;
			ss1.t_s1_ = current_ss.t_s1_;
			ss1.t_s2_ = t_abs;
			ss1.link_index_ = current_ss.link_index_;
			ss1.object_index_ = current_ss.object_index_;
			ss1.dist_s1_obs_ = current_ss.dist_s1_obs_;
			ss1.dist_s2_obs_ = dist_snew_obs;
			ss1.non_covered_length_ = dist_travel[ss1.link_index_]*(ss1.t_s2_ - ss1.t_s1_) - (ss1.dist_s1_obs_ + ss1.dist_s2_obs_);
			if (ss1.non_covered_length_ > 0)
				ss_queue.push(ss1);

			ss2.self_cc_ = current_ss.self_cc_;
			ss2.t_s1_ = t_abs;
			ss2.t_s2_ = current_ss.t_s2_;
			ss2.link_index_ = current_ss.link_index_;
			ss2.object_index_ = current_ss.object_index_;
			ss2.dist_s1_obs_ = dist_snew_obs;
			ss2.dist_s2_obs_ = current_ss.dist_s2_obs_;
			ss2.non_covered_length_ = dist_travel[ss2.link_index_]*(ss2.t_s2_ - ss2.t_s1_) - (ss2.dist_s1_obs_ + ss2.dist_s2_obs_);
			if (ss2.non_covered_length_ > 0)
				ss_queue.push(ss2);
		}
		else
		{
			double dist_snew_self = ssvc_->computeLinkMinSelfDist(s_new, current_ss.link_index_);
			if (dist_snew_self <= 0)
			{
				return false;
			}
			SubSegment ss1,ss2;

			ss1.self_cc_ = current_ss.self_cc_;
			ss1.t_s1_ = current_ss.t_s1_;
			ss1.t_s2_ = t_abs;
			ss1.link_index_ = current_ss.link_index_;
			ss1.object_index_ = current_ss.object_index_;
			ss1.dist_s1_obs_ = current_ss.dist_s1_obs_;
			ss1.dist_s2_obs_ = dist_snew_self;
			ss1.non_covered_length_ = dist_travel[ss1.link_index_]*(ss1.t_s2_ - ss1.t_s1_) - (ss1.dist_s1_obs_ + ss1.dist_s2_obs_);
			if (ss1.non_covered_length_ > 0)
				ss_queue.push(ss1);

			ss2.self_cc_ = current_ss.self_cc_;
			ss2.t_s1_ = t_abs;
			ss2.t_s2_ = current_ss.t_s2_;
			ss2.link_index_ = current_ss.link_index_;
			ss2.object_index_ = current_ss.object_index_;
			ss2.dist_s1_obs_ = dist_snew_self;
			ss2.dist_s2_obs_ = current_ss.dist_s2_obs_;
			ss2.non_covered_length_ = dist_travel[ss2.link_index_]*(ss2.t_s2_ - ss2.t_s1_) - (ss2.dist_s1_obs_ + ss2.dist_s2_obs_);
			if (ss2.non_covered_length_ > 0)
				ss_queue.push(ss2);
		}

		si_->freeState(s_new);

		if (!ss_queue.empty())
			flag = dist_travel[ss_queue.top().link_index_]*(ss_queue.top().t_s2_ - ss_queue.top().t_s1_) < travel_dist_limit;

	}


	if (!ss_queue.empty() && ss_queue.top().non_covered_length_ > 0)
		return false;
	else
		return true;
}



double ompl::base::SafeMotionValidator::minObstacleDistMotionIndividualObjects(const State *s1, const State *s2, double travel_dist_limit, bool fast_dist, double& object_danger_factor) const
{

	size_t nb_segment_eval = 0;


	std::priority_queue<SubSegment> ss_queue;

	std::vector<std::vector<double> > dist_s1_obs, dist_s2_obs;
	std::vector<double> dist_travel;

	ssvc_->computeInitialDistDataObstacle(s1, s2, dist_s1_obs, dist_s2_obs, fast_dist);
	ssvc_->computeInitialDistDataTravelModulation(s1, s2, dist_travel);

	if (dist_travel[dist_travel.size() - 1] == 0)
	    throw Exception("minObstacleDistMotionIndividualObjects : The two states defining the motion are the same");

	for (size_t i = 0; i < ssvc_->getNbSafetyLinks(); ++i)
	{
		for (size_t j = 0; j < ssvc_->getNbObjects(); ++j)
		{
			if ((dist_s1_obs[i][j] <= 0) || (dist_s2_obs[i][j] <= 0) )
				return -1; //Collision

			SubSegment ss;
			ss.t_s1_ = 0;
			ss.t_s2_ = 1;
			ss.link_index_ = i;
			ss.object_index_ = j;
			ss.obj_danger_factor_ = ssvc_->getObjectDangerFactor(j);
			ss.dist_s1_obs_ = dist_s1_obs[i][j];
			ss.dist_s2_obs_ = dist_s2_obs[i][j];
			ss.non_covered_length_ = dist_travel[i] - (dist_s1_obs[i][j] + dist_s2_obs[i][j]);
			ss_queue.push(ss);
		}

	}
	bool flag = false;
	while (!flag && !ss_queue.empty())
	{
		SubSegment current_ss = ss_queue.top();
		ss_queue.pop();

		State *s_new = si_->allocState();

		//STa : Interpolation guidée par la distance. Attention a modifier t_s1_ et t_s2_ si utilisée.
		double t = (2*current_ss.dist_s1_obs_ + current_ss.non_covered_length_) / (2*(current_ss.dist_s1_obs_ + current_ss.dist_s2_obs_ + current_ss.non_covered_length_));
		double t_abs = current_ss.t_s1_ + (current_ss.t_s2_ - current_ss.t_s1_) * t;

		stateSpace_->interpolate(s1, s2, t_abs, s_new);


		double dist_snew_obs = ssvc_->computeLinkMinObstacleDist(s_new, current_ss.link_index_, current_ss.object_index_, fast_dist);

		if (dist_snew_obs <= 0)
			return -1; //Collision

		SubSegment ss1,ss2;

		ss1.t_s1_ = current_ss.t_s1_;
		ss1.t_s2_ = t_abs;
		ss1.link_index_ = current_ss.link_index_;
		ss1.object_index_ = current_ss.object_index_;
		ss1.obj_danger_factor_= current_ss.obj_danger_factor_;
		ss1.dist_s1_obs_ = current_ss.dist_s1_obs_;
		ss1.dist_s2_obs_ = dist_snew_obs;
		ss1.non_covered_length_ = dist_travel[ss1.link_index_]*(ss1.t_s2_ - ss1.t_s1_) - (ss1.dist_s1_obs_ + ss1.dist_s2_obs_);
		ss_queue.push(ss1);

		ss2.t_s1_ = t_abs;
		ss2.t_s2_ = current_ss.t_s2_;
		ss2.link_index_ = current_ss.link_index_;
		ss2.object_index_ = current_ss.object_index_;
		ss2.obj_danger_factor_= current_ss.obj_danger_factor_;
		ss2.dist_s1_obs_ = dist_snew_obs;
		ss2.dist_s2_obs_ = current_ss.dist_s2_obs_;
		ss2.non_covered_length_ = dist_travel[ss2.link_index_]*(ss2.t_s2_ - ss2.t_s1_) - (ss2.dist_s1_obs_ + ss2.dist_s2_obs_);
		ss_queue.push(ss2);

		si_->freeState(s_new);
		if (!ss_queue.empty())
			flag = dist_travel[ss_queue.top().link_index_]*(ss_queue.top().t_s2_ - ss_queue.top().t_s1_) < travel_dist_limit;

	}

	if (ss_queue.empty())
		return std::numeric_limits<double>::infinity();

	object_danger_factor = ss_queue.top().obj_danger_factor_;

	if (ss_queue.top().non_covered_length_ < 0)
	{
	    double min_dist = -ss_queue.top().non_covered_length_/2;
	    double min_obs = std::min(ss_queue.top().dist_s1_obs_, ss_queue.top().dist_s2_obs_);
	    return std::min(min_obs, min_dist);
	}
	else
	    return -1;
}


double ompl::base::SafeMotionValidator::minObstacleDistMotionDiscrete(const State *s1, const State *s2, double valid_segment_factor, bool fast_dist) const
{
	double min_dist_temp, min_dist = std::numeric_limits<double>::infinity();

	double factor;
	min_dist_temp = ssvc_->computeRobotMinObstacleDistIndividualLinks(s1, fast_dist, factor);
	if (min_dist_temp < min_dist)
	{
		min_dist = min_dist_temp;
	}
	min_dist_temp = ssvc_->computeRobotMinObstacleDistIndividualLinks(s2, fast_dist, factor);
	if (min_dist_temp < min_dist)
	{
		min_dist = min_dist_temp;
	}

    int nd = si_->getStateSpace()->validSegmentCount(s1, s2) * valid_segment_factor;

    if (nd > 1)
    {
        State *test = si_->allocState();
        for (int j = 1; j < nd; ++j)
        {
        	si_->getStateSpace()->interpolate(s1, s2, (double) j / (double) nd, test);
        	min_dist_temp = ssvc_->computeRobotMinObstacleDistIndividualLinks(test, fast_dist, factor);
        	if (min_dist_temp < min_dist)
        	{
        		min_dist = min_dist_temp;
        	}
        }
        si_->freeState(test);
    }
    return min_dist;
}

double ompl::base::SafeMotionValidator::minObstacleDistMotionDiscreteExact(const State *s1, const State *s2, double valid_segment_factor) const
{
	double min_dist_temp, min_dist = std::numeric_limits<double>::infinity();


	min_dist_temp = ssvc_->computeRobotExactMinObstacleDist(s1);
	if (min_dist_temp < min_dist)
	{
		min_dist = min_dist_temp;
	}
	min_dist_temp = ssvc_->computeRobotExactMinObstacleDist(s2);
	if (min_dist_temp < min_dist)
	{
		min_dist = min_dist_temp;
	}

	int nd = si_->getStateSpace()->validSegmentCount(s1, s2) * valid_segment_factor;

	if (nd > 1)
	{
		State *test = si_->allocState();
		for (int j = 1; j < nd; ++j)
		{
			si_->getStateSpace()->interpolate(s1, s2, (double) j / (double) nd, test);
			min_dist_temp = ssvc_->computeRobotExactMinObstacleDist(test);
			if (min_dist_temp < min_dist)
			{
				min_dist = min_dist_temp;
			}
		}
		si_->freeState(test);
	}
	return min_dist;
}

bool ompl::base::SafeMotionValidator::checkMotionSelfCCIndividualLinks(const State *s1, const State *s2, double travel_dist_limit) const
{
	std::priority_queue<SubSegment> ss_queue;

	std::vector<double> dist_s1_self, dist_s2_self;
	std::vector<double> dist_travel;

	ssvc_->computeInitialDistDataSelf(s1, s2, dist_s1_self, dist_s2_self);
	ssvc_->computeInitialDistDataTravel(s1, s2, dist_travel);


	for (size_t i = 0; i < ssvc_->getNbSafetyLinks(); ++i)
	{
		if ((dist_s1_self[i] <= 0) || (dist_s2_self[i] <= 0))
			return false;

		if (dist_travel[i] - (dist_s1_self[i] + dist_s2_self[i]) > 0)
		{
			SubSegment ss;
			ss.self_cc_ = true;
			ss.t_s1_ = 0;
			ss.t_s2_ = 1;
			ss.link_index_ = i;
			ss.dist_s1_obs_ = dist_s1_self[i];
			ss.dist_s2_obs_ = dist_s2_self[i];
			ss.non_covered_length_ = dist_travel[i] - (dist_s1_self[i] + dist_s2_self[i]);
			ss_queue.push(ss);
		}

	}
	bool flag = false;
	while (!flag && !ss_queue.empty())
	{
		SubSegment current_ss = ss_queue.top();
		ss_queue.pop();

		State *s_new = si_->allocState();

		double t = (2*current_ss.dist_s1_obs_ + current_ss.non_covered_length_) / (2*(current_ss.dist_s1_obs_ + current_ss.dist_s2_obs_ + current_ss.non_covered_length_));

		double t_abs = current_ss.t_s1_ + (current_ss.t_s2_ - current_ss.t_s1_) * t;


		stateSpace_->interpolate(s1, s2, t_abs, s_new);

		double dist_snew_self = ssvc_->computeLinkMinSelfDist(s_new, current_ss.link_index_);

		if (dist_snew_self <= 0)
			return false;

		SubSegment ss1,ss2;

		ss1.self_cc_ = current_ss.self_cc_;
		ss1.t_s1_ = current_ss.t_s1_;
		ss1.t_s2_ = t_abs;
		ss1.link_index_ = current_ss.link_index_;
		ss1.object_index_ = current_ss.object_index_;
		ss1.dist_s1_obs_ = current_ss.dist_s1_obs_;
		ss1.dist_s2_obs_ = dist_snew_self;
		ss1.non_covered_length_ = dist_travel[ss1.link_index_]*(ss1.t_s2_ - ss1.t_s1_) - (ss1.dist_s1_obs_ + ss1.dist_s2_obs_);
		if (ss1.non_covered_length_ > 0)
			ss_queue.push(ss1);

		ss2.self_cc_ = current_ss.self_cc_;
		ss2.t_s1_ = t_abs;
		ss2.t_s2_ = current_ss.t_s2_;
		ss2.link_index_ = current_ss.link_index_;
		ss2.object_index_ = current_ss.object_index_;
		ss2.dist_s1_obs_ = dist_snew_self;
		ss2.dist_s2_obs_ = current_ss.dist_s2_obs_;
		ss2.non_covered_length_ = dist_travel[ss2.link_index_]*(ss2.t_s2_ - ss2.t_s1_) - (ss2.dist_s1_obs_ + ss2.dist_s2_obs_);
		if (ss2.non_covered_length_ > 0)
			ss_queue.push(ss2);


		si_->freeState(s_new);

		if (!ss_queue.empty())
			flag = dist_travel[ss_queue.top().link_index_]*(ss_queue.top().t_s2_ - ss_queue.top().t_s1_) < travel_dist_limit;

	}
	if (!ss_queue.empty() && ss_queue.top().non_covered_length_ > 0)
		return false;
	else
		return true;
}

bool ompl::base::SafeMotionValidator::checkMotionSelfCCDiscrete(const State *s1, const State *s2, double valid_segment_factor) const
{
	/* assume motion starts in a valid configuration so s1 is valid */
	if (!ssvc_->isValidSelf(s2))
	{
		invalid_++;
		return false;
	}

	bool result = true;
	int nd = stateSpace_->validSegmentCount(s1, s2) * valid_segment_factor;

	/* initialize the queue of test positions */
	std::queue< std::pair<int, int> > pos;
	if (nd >= 2)
	{
		pos.push(std::make_pair(1, nd - 1));

		/* temporary storage for the checked state */
		State *test = si_->allocState();

		/* repeatedly subdivide the path segment in the middle (and check the middle) */
		while (!pos.empty())
		{
			std::pair<int, int> x = pos.front();

			int mid = (x.first + x.second) / 2;
			stateSpace_->interpolate(s1, s2, (double)mid / (double)nd, test);

			if (!ssvc_->isValidSelf(test))
			{
				result = false;
				break;
			}

			pos.pop();

			if (x.first < mid)
				pos.push(std::make_pair(x.first, mid - 1));
			if (x.second > mid)
				pos.push(std::make_pair(mid + 1, x.second));
		}

		si_->freeState(test);
	}

	if (result)
		valid_++;
	else
		invalid_++;

	return result;
}

bool ompl::base::SafeMotionValidator::checkMotionSelfCCDiscreteWS(const State *s1, const State *s2, double travel_dist_limit) const
{
	/* assume motion starts in a valid configuration so s1 is valid */
		if (!ssvc_->isValidSelf(s2))
		{
			invalid_++;
			return false;
		}

		std::vector<double> dist_travel;

		ssvc_->computeInitialDistDataTravel(s1, s2, dist_travel);

		double max_travel_dist = dist_travel.back();

		bool result = true;

		/* initialize the queue of test positions */
		std::queue< std::pair<double, double> > pos;
		if (max_travel_dist > travel_dist_limit)
		{
			pos.push(std::make_pair(0, 1));

			/* temporary storage for the checked state */
			State *test = si_->allocState();

			/* repeatedly subdivide the path segment in the middle (and check the middle) */
			while (!pos.empty())
			{
				std::pair<double, double> x = pos.front();

				double mid = (x.first + x.second) / 2;
				stateSpace_->interpolate(s1, s2, mid, test);

				if (!ssvc_->isValidSelf(test))
				{
					result = false;
					break;
				}

				pos.pop();

				if(max_travel_dist*(x.second-x.first) > travel_dist_limit)
				{
					pos.push(std::make_pair(x.first, mid));
					pos.push(std::make_pair(mid, x.second));
				}

			}

			si_->freeState(test);
		}

		if (result)
			valid_++;
		else
			invalid_++;

		return result;
}

bool ompl::base::SafeMotionValidator::checkMotionWorldIndividualLinks(const State *s1, const State *s2, double travel_dist_limit, bool fast_dist) const
{
	std::priority_queue<SubSegment> ss_queue;

	std::vector<std::vector<double> > dist_s1_obs, dist_s2_obs;
	std::vector<double> dist_travel;

	ssvc_->computeInitialDistDataObstacle(s1, s2, dist_s1_obs, dist_s2_obs, fast_dist);
	ssvc_->computeInitialDistDataTravelModulation(s1, s2, dist_travel);

	for (size_t i = 0; i < ssvc_->getNbSafetyLinks(); ++i)
		{
			for (size_t j = 0; j < ssvc_->getNbObjects(); ++j)
			{
				if ((dist_s1_obs[i][j] <= 0) || (dist_s2_obs[i][j] <= 0) )
					return false; //Collision

				if (dist_travel[i] - (dist_s1_obs[i][j] + dist_s2_obs[i][j]) > 0)
				{
					SubSegment ss;
					ss.t_s1_ = 0;
					ss.t_s2_ = 1;
					ss.link_index_ = i;
					ss.object_index_ = j;
					ss.dist_s1_obs_ = dist_s1_obs[i][j];
					ss.dist_s2_obs_ = dist_s2_obs[i][j];
					ss.non_covered_length_ = dist_travel[i] - (dist_s1_obs[i][j] + dist_s2_obs[i][j]);
					ss_queue.push(ss);
				}
			}

		}

	bool flag = false;
	while (!flag && !ss_queue.empty())
	{
		SubSegment current_ss = ss_queue.top();
		ss_queue.pop();

		State *s_new = si_->allocState();

		//STa : Interpolation guidée par la distance.
		double t = (2*current_ss.dist_s1_obs_ + current_ss.non_covered_length_) / (2*(current_ss.dist_s1_obs_ + current_ss.dist_s2_obs_ + current_ss.non_covered_length_));

		double t_abs = current_ss.t_s1_ + (current_ss.t_s2_ - current_ss.t_s1_) * t;

		stateSpace_->interpolate(s1, s2, t_abs, s_new);

		double dist_snew_obs = ssvc_->computeLinkMinObstacleDist(s_new, current_ss.link_index_, current_ss.object_index_, fast_dist);


		if (dist_snew_obs <= 0)
			return false;

		SubSegment ss1,ss2;

		ss1.self_cc_ = current_ss.self_cc_;
		ss1.t_s1_ = current_ss.t_s1_;
		ss1.t_s2_ = t_abs;
		ss1.link_index_ = current_ss.link_index_;
		ss1.object_index_ = current_ss.object_index_;
		ss1.dist_s1_obs_ = current_ss.dist_s1_obs_;
		ss1.dist_s2_obs_ = dist_snew_obs;
		ss1.non_covered_length_ = dist_travel[ss1.link_index_]*(ss1.t_s2_ - ss1.t_s1_) - (ss1.dist_s1_obs_ + ss1.dist_s2_obs_);
		if (ss1.non_covered_length_ > 0)
			ss_queue.push(ss1);

		ss2.self_cc_ = current_ss.self_cc_;
		ss2.t_s1_ = t_abs;
		ss2.t_s2_ = current_ss.t_s2_;
		ss2.link_index_ = current_ss.link_index_;
		ss2.object_index_ = current_ss.object_index_;
		ss2.dist_s1_obs_ = dist_snew_obs;
		ss2.dist_s2_obs_ = current_ss.dist_s2_obs_;
		ss2.non_covered_length_ = dist_travel[ss2.link_index_]*(ss2.t_s2_ - ss2.t_s1_) - (ss2.dist_s1_obs_ + ss2.dist_s2_obs_);
		if (ss2.non_covered_length_ > 0)
			ss_queue.push(ss2);


		si_->freeState(s_new);

		if (!ss_queue.empty())
			flag = dist_travel[ss_queue.top().link_index_]*(ss_queue.top().t_s2_ - ss_queue.top().t_s1_) < travel_dist_limit;

	}
	if (!ss_queue.empty() && ss_queue.top().non_covered_length_ > 0)
		return false;
	else
		return true;
}
