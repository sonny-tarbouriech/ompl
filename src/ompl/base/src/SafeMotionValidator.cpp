//STa

#include "ompl/base/SafeMotionValidator.h"
#include "ompl/util/Exception.h"
#include <queue>


#include "ompl/base/SpaceInformation.h"

//STa test
#include <fstream>
#include <ompl/util/Time.h>

void ompl::base::SafeMotionValidator::defaultSettings()
{
    stateSpace_ = si_->getStateSpace().get();
    if (!stateSpace_)
        throw Exception("No state space for motion validator");

    //TODO : return error if si_ has not a SafeStateValidityChecker
    ssvc_ = static_cast<ompl::base::SafeStateValidityChecker*>(si_->getStateValidityChecker().get());
}

// Function copied from DiscreteMotionValidator to compare results and performances
bool ompl::base::SafeMotionValidator::checkMotion(const State *s1, const State *s2) const
{
   return checkMotion(s1,s2,1);
}

bool  ompl::base::SafeMotionValidator::checkMotion(const State *s1, const State *s2, int valid_segment_factor) const
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

//STa test
bool ompl::base::SafeMotionValidator::checkMotionTEST(const State *s1, const State *s2)
{
	//STa test
	std::string homepath = getenv("HOME");
	std::ofstream output_file((homepath + "/checkmotion_time.txt").c_str(), std::ios::out | std::ios::app);
	std::ofstream output_file_2((homepath + "/checkmotion_result.txt").c_str(), std::ios::out | std::ios::app);
	ompl::time::point init = ompl::time::now();
	bool check_dynamic_1 = checkMotionIndividualLinks(s1,s2,0.1, false);
	ompl::time::duration dyn1 = ompl::time::now() - init;
	bool check_dynamic_2 = checkMotionIndividualLinks(s1,s2,0.01, false);
	ompl::time::duration dyn2 = ompl::time::now() - init - dyn1;
	bool check_discrete_1 = checkMotion(s1,s2);
	ompl::time::duration dis1 = ompl::time::now() - init - dyn1 - dyn2;
	bool check_discrete_2 = checkMotion(s1,s2,5);
	ompl::time::duration dis2 = ompl::time::now() - init - dyn1 - dyn2 - dis1;
	bool check_exact = checkMotion(s1,s2,100);
	output_file << ompl::time::seconds(dyn1) << "  " << ompl::time::seconds(dyn2)<< "  " << ompl::time::seconds(dis1)<< "  " << ompl::time::seconds(dis2) << "\n";
	output_file_2 << check_dynamic_1 << "  " << check_dynamic_2 << "  " << check_discrete_1 << "  " << check_discrete_2 << "  " << check_exact  << "\n";
	output_file.close();
	output_file_2.close();
	return check_exact;
}

//STa test
bool ompl::base::SafeMotionValidator::checkMotionSelfTEST(const State *s1, const State *s2)
{
	//STa test
	std::string homepath = getenv("HOME");
	std::ofstream output_file((homepath + "/checkmotionself_time.txt").c_str(), std::ios::out | std::ios::app);
	std::ofstream output_file_2((homepath + "/checkmotionself_result.txt").c_str(), std::ios::out | std::ios::app);
	ompl::time::point init = ompl::time::now();
	bool check_dynamic_1 = checkMotionSelfCCIndividualLinks(s1,s2,0.1);
	ompl::time::duration dyn1 = ompl::time::now() - init;
	bool check_dynamic_2 = checkMotionSelfCCIndividualLinks(s1,s2,0.01);
	ompl::time::duration dyn2 = ompl::time::now() - init - dyn1;
	bool check_discrete_1 = checkMotionSelfCCDiscrete(s1,s2,1);
	ompl::time::duration dis1 = ompl::time::now() - init - dyn1 - dyn2;
	bool check_discrete_2 = checkMotionSelfCCDiscrete(s1,s2,5);
	ompl::time::duration dis2 = ompl::time::now() - init - dyn1 - dyn2 - dis1;
	bool check_discrete_ws_1 = checkMotionSelfCCDiscreteWS(s1,s2,0.1);
	ompl::time::duration disws1 = ompl::time::now() - init - dyn1 - dyn2 - dis1 - dis2;
	bool check_discrete_ws_2 = checkMotionSelfCCDiscreteWS(s1,s2,0.01);
	ompl::time::duration disws2 = ompl::time::now() - init - dyn1 - dyn2 - dis1 - dis2 - disws1;
	bool check_exact = checkMotionSelfCCDiscrete(s1,s2,100);
	output_file << ompl::time::seconds(dyn1) << "  " << ompl::time::seconds(dyn2)<< "  " << ompl::time::seconds(dis1)<< "  " << ompl::time::seconds(dis2) << "  " << ompl::time::seconds(disws1)<< "  " << ompl::time::seconds(disws2) << "\n";
	output_file_2 << check_dynamic_1 << "  " << check_dynamic_2 << "  " << check_discrete_1 << "  " << check_discrete_2 << "  "  << check_discrete_ws_1 << "  " << check_discrete_ws_2 << "  " << check_exact  << "\n";
	output_file.close();
	output_file_2.close();
	return check_exact;
}

bool ompl::base::SafeMotionValidator::checkMotionIndividualLinksWithDist(const State *s1, const State *s2, double travel_dist_limit, double& min_obstacle_dist, bool fast_dist)
{
//	//STa temp
//	std::string homepath = getenv("HOME");
//	std::ofstream output_file_1((homepath + "/nb_segment_eval_IndividualObjectsWithSelfCC_dist.txt").c_str(), std::ios::out | std::ios::app);
//	size_t nb_segment_eval = 0;

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
//				//STa temp
//				output_file_1 << nb_segment_eval << "\n";
//				output_file_1.close();

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
//			//STa temp
//			output_file_1 << nb_segment_eval << "\n";
//			output_file_1.close();

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

		//STa : Interpolation guidée par la distance. Attention a modifier t_s1_ et t_s2_ si utilisée.
//		double t = current_ss.dist_s1_obs_ / (current_ss.dist_s1_obs_ + current_ss.dist_s2_obs_);
		double t = (2*current_ss.dist_s1_obs_ + current_ss.non_covered_length_) / (2*(current_ss.dist_s1_obs_ + current_ss.dist_s2_obs_ + current_ss.non_covered_length_));

		double t_abs = current_ss.t_s1_ + (current_ss.t_s2_ - current_ss.t_s1_) * t;


		stateSpace_->interpolate(s1, s2, t_abs, s_new);

		if (!current_ss.self_cc_)
		{
//			//STa temp
//			nb_segment_eval++;

			double dist_snew_obs = ssvc_->computeLinkMinObstacleDist(s_new, current_ss.link_index_, current_ss.object_index_, fast_dist);
			if (dist_snew_obs <= 0)
			{
//				//STa temp
//				output_file_1 << nb_segment_eval << "\n";
//				output_file_1.close();

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
//			//STa temp
//			nb_segment_eval++;

			double dist_snew_self = ssvc_->computeLinkMinSelfDist(s_new, current_ss.link_index_);
			if (dist_snew_self <= 0)
			{
//				//STa temp
//				output_file_1 << nb_segment_eval << "\n";
//				output_file_1.close();

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

	//STa temp
//	ompl::time::duration elapsed = ompl::time::now() - startTime;
//	std::string homepath = getenv("HOME");
//	std::ofstream output_file_1((homepath + "/log.txt").c_str(), std::ios::out | std::ios::app);
//	if (output_file_1)
//	{
//		output_file_1 << "checkMotionIndividualObjectsWithSelfCC with distance duration : " <<  ompl::time::seconds(elapsed) << " seconds \n ";
//		output_file_1.close();
//	}


//	//STa temp
//	output_file_1 << nb_segment_eval << "\n";
//	output_file_1.close();

//	std::cout << "non_covered_length_ = " << ss_queue.top().non_covered_length_ << "\n";

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
//	//STa temp
//	std::string homepath = getenv("HOME");
//	std::ofstream output_file_1((homepath + "/nb_segment_eval_safety_obj_dist.txt").c_str(), std::ios::out | std::ios::app);
//	output_file_1 << "min dist fcl = " << minObstacleDistMotionDiscrete(s1,s2, 1, fast_dist) << "\n";
//	output_file_1 << "min dist fcl factor 2 = " << minObstacleDistMotionDiscrete(s1,s2, 2) << "\n";
//	output_file_1 << "nb interpolation = " << si_->getStateSpace()->validSegmentCount(s1, s2) << "\n";
//	size_t nb_segment_eval = 0;

	std::priority_queue<SubSegment> ss_queue;

	std::vector<std::vector<double> > dist_s1_obs, dist_s2_obs;
	std::vector<double> dist_travel;

	ssvc_->computeInitialDistDataObstacle(s1, s2, dist_s1_obs, dist_s2_obs, fast_dist);
	ssvc_->computeInitialDistDataTravelModulation(s1, s2, dist_travel);

	std::vector<double> obj_danger_factor = ssvc_->getObjectDangerFactors();

	for (size_t i = 0; i < ssvc_->getNbSafetyLinks(); ++i)
	{
		for (size_t j = 0; j < ssvc_->getNbObjects(); ++j)
		{

			if ((dist_s1_obs[i][j] <= 0) || (dist_s2_obs[i][j] <= 0) )
			{
//				//STa temp
//				output_file_1 << nb_segment_eval << "\t";
//				output_file_1 << "min dist approx = " << -1 << "\n";
//				output_file_1 << nb_segment_eval << "\n \n \n \n";
//				output_file_1.close();

				return -1; //Collision
			}
			SubSegment ss;
			ss.t_s1_ = 0;
			ss.t_s2_ = 1;
			ss.link_index_ = i;
			ss.object_index_ = j;
			ss.obj_danger_factor_ = obj_danger_factor[j];
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

//		//STa temp
//		output_file_1 << "current_ss.dist_s1_obs_ = " << current_ss.dist_s1_obs_ << "\n"
//				<< "current_ss.dist_s2_obs_ = " << current_ss.dist_s2_obs_ << "\n"
//				<< "current_ss.dist_travel = " << dist_travel[current_ss.link_index_]*(current_ss.t_s2_ - current_ss.t_s1_) << "\n"
//				<< "current_ss.link_index_ = " << current_ss.link_index_ << "\n"
//				<< "current_ss.non_covered_length_ = " << current_ss.non_covered_length_ << "\n"
//				<< "current_ss.object_index_ = " << current_ss.object_index_ << "\n"
//				<< "current_ss.self_cc_ = " << current_ss.self_cc_ << "\n"
//				<< "current_ss.t_s1_ = " << current_ss.t_s1_ << "\n"
//				<< "current_ss.t_s2_ = " << current_ss.t_s2_ << "\n"
//				<< "t = " << t << "\n"
//				<< "t_abs = " << t_abs << "\n \n";

		stateSpace_->interpolate(s1, s2, t_abs, s_new);


		double dist_snew_obs = ssvc_->computeLinkMinObstacleDist(s_new, current_ss.link_index_, current_ss.object_index_, fast_dist);


//		//STa temp
//		nb_segment_eval++;

		if (dist_snew_obs <= 0)
		{
//			//STa temp
//			output_file_1 << nb_segment_eval << "\t";
//			output_file_1 << "dist_snew_obs <= 0 : min dist approx = -1 \n";
//			output_file_1 << nb_segment_eval << "\n \n \n \n";
//			output_file_1.close();

			return -1; //Collision
		}
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

//	//STa temp
//	output_file_1 << "ss_queue.dist_s1_obs_ = " << ss_queue.top().dist_s1_obs_ << "\n"
//			<< "ss_queue.dist_s2_obs_ = " << ss_queue.top().dist_s2_obs_ << "\n"
//			<< "ss_queue.dist_travel = " << ss_queue.top().non_covered_length_ + ss_queue.top().dist_s1_obs_+ss_queue.top().dist_s2_obs_ << "\n"
//			<< "ss_queue.link_index_ = " << ss_queue.top().link_index_ << "\n"
//			<< "ss_queue.non_covered_length_ = " << ss_queue.top().non_covered_length_ << "\n"
//			<< "ss_queue.object_index_ = " << ss_queue.top().object_index_ << "\n"
//			<< "ss_queue.self_cc_ = " << ss_queue.top().self_cc_ << "\n"
//			<< "ss_queue.t_s1_ = " << ss_queue.top().t_s1_ << "\n"
//			<< "ss_queue.t_s2_ = " << ss_queue.top().t_s2_ << "\n \n";

//	//STa temp
//	double d_temp = -ss_queue.top().non_covered_length_ <= 0 ? -1 : -ss_queue.top().non_covered_length_/2;
//	output_file_1 << nb_segment_eval << "\t";
//	output_file_1 << "min dist approx = " << d_temp << "\n \n";
//	output_file_1 << nb_segment_eval << "\n \n \n \n";
//	output_file_1.close();

	object_danger_factor = ss_queue.top().obj_danger_factor_;

	return (ss_queue.top().non_covered_length_ >= 0 ? -1 : -ss_queue.top().non_covered_length_/2);


}


double ompl::base::SafeMotionValidator::minObstacleDistMotionIndividualObjectsTEST(const State *s1, const State *s2, double travel_dist_limit, bool fast_dist) const
{
	//STa temp
	std::string homepath = getenv("HOME");
	std::ofstream output_file_1((homepath + "/nb_segment_eval_safety_obj_dist.txt").c_str(), std::ios::out | std::ios::app);
	size_t nb_segment_eval = 0;

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
			{
//				//STa temp
				output_file_1 << nb_segment_eval << "\n";
//				output_file_1 << "min dist approx modulation = " << -1 << "\n";
//				output_file_1 << nb_segment_eval << "\n \n \n \n";
				output_file_1.close();

				return -1; //Collision
			}
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

	bool flag = false;
	while (!flag && !ss_queue.empty())
	{
		SubSegment current_ss = ss_queue.top();
		ss_queue.pop();

//		//STa temp
//		output_file_1 << "current_ss.dist_s1_obs_ = " << current_ss.dist_s1_obs_ << "\n"
//				<< "current_ss.dist_s2_obs_ = " << current_ss.dist_s2_obs_ << "\n"
//				<< "current_ss.dist_travel = " << current_ss.non_covered_length_ + current_ss.dist_s1_obs_+current_ss.dist_s2_obs_ << "\n"
//				<< "current_ss.link_index_ = " << current_ss.link_index_ << "\n"
//				<< "current_ss.non_covered_length_ = " << current_ss.non_covered_length_ << "\n"
//				<< "current_ss.object_index_ = " << current_ss.object_index_ << "\n"
//				<< "current_ss.self_cc_ = " << current_ss.self_cc_ << "\n"
//				<< "current_ss.t_s1_ = " << current_ss.t_s1_ << "\n"
//				<< "current_ss.t_s2_ = " << current_ss.t_s2_ << "\n \n";

		State *s_new = si_->allocState();

		//STa : Interpolation guidée par la distance. Attention a modifier t_s1_ et t_s2_ si utilisée.
//		double t = current_ss.dist_s1_obs_ / (current_ss.dist_s1_obs_ + current_ss.dist_s2_obs_);
		double t = (2*current_ss.dist_s1_obs_ + current_ss.non_covered_length_) / (2*(current_ss.dist_s1_obs_ + current_ss.dist_s2_obs_ + current_ss.non_covered_length_));

		double t_abs = current_ss.t_s1_ + (current_ss.t_s2_ - current_ss.t_s1_) * t;

		stateSpace_->interpolate(s1, s2, t_abs, s_new);

		double dist_snew_obs = ssvc_->computeLinkMinObstacleDist(s_new, current_ss.link_index_, current_ss.object_index_, fast_dist);

		//STa temp
		nb_segment_eval++;

		if (dist_snew_obs <= 0)
		{
//			//STa temp
			output_file_1 << nb_segment_eval << "\n";
//			output_file_1 << "dist_snew_obs <= 0 : min dist approx modulation = -1 \n";
//			output_file_1 << nb_segment_eval << "\n \n \n \n";
			output_file_1.close();

			return -1; //Collision
		}
		SubSegment ss1,ss2;

		State *s_temp = si_->allocState();
		stateSpace_->interpolate(s1, s2, current_ss.t_s1_, s_temp);

		dist_travel[current_ss.link_index_] = ssvc_->computeDistTravelModulation(s_temp, s_new ,current_ss.link_index_);

		ss1.t_s1_ = current_ss.t_s1_;
		ss1.t_s2_ = t_abs;
		ss1.link_index_ = current_ss.link_index_;
		ss1.object_index_ = current_ss.object_index_;
		ss1.dist_s1_obs_ = current_ss.dist_s1_obs_;
		ss1.dist_s2_obs_ = dist_snew_obs;
		ss1.non_covered_length_ = dist_travel[ss1.link_index_] - (ss1.dist_s1_obs_ + ss1.dist_s2_obs_);
		ss_queue.push(ss1);

		stateSpace_->interpolate(s1, s2, current_ss.t_s2_, s_temp);
		dist_travel[current_ss.link_index_] = ssvc_->computeDistTravelModulation(s_new, s_temp ,current_ss.link_index_);

		ss2.t_s1_ = t_abs;
		ss2.t_s2_ = current_ss.t_s2_;
		ss2.link_index_ = current_ss.link_index_;
		ss2.object_index_ = current_ss.object_index_;
		ss2.dist_s1_obs_ = dist_snew_obs;
		ss2.dist_s2_obs_ = current_ss.dist_s2_obs_;
		ss2.non_covered_length_ = dist_travel[ss2.link_index_] - (ss2.dist_s1_obs_ + ss2.dist_s2_obs_);
		ss_queue.push(ss2);

		if (!ss_queue.empty())
		{
			stateSpace_->interpolate(s1, s2, ss_queue.top().t_s1_, s_temp);
			stateSpace_->interpolate(s1, s2, ss_queue.top().t_s2_, s_new);
			dist_travel[ss_queue.top().link_index_] = ssvc_->computeDistTravelModulation(s_temp, s_new ,ss_queue.top().link_index_);

			flag = dist_travel[ss_queue.top().link_index_] < travel_dist_limit;
		}

		si_->freeState(s_new);
		si_->freeState(s_temp);
	}


	if (ss_queue.empty())
		return std::numeric_limits<double>::infinity();

//	//STa temp
//	output_file_1 << "ss_queue.dist_s1_obs_ = " << ss_queue.top().dist_s1_obs_ << "\n"
//			<< "ss_queue.dist_s2_obs_ = " << ss_queue.top().dist_s2_obs_ << "\n"
//			<< "ss_queue.dist_travel = " << ss_queue.top().non_covered_length_ + ss_queue.top().dist_s1_obs_+ss_queue.top().dist_s2_obs_ << "\n"
//			<< "ss_queue.link_index_ = " << ss_queue.top().link_index_ << "\n"
//			<< "ss_queue.non_covered_length_ = " << ss_queue.top().non_covered_length_ << "\n"
//			<< "ss_queue.object_index_ = " << ss_queue.top().object_index_ << "\n"
//			<< "ss_queue.self_cc_ = " << ss_queue.top().self_cc_ << "\n"
//			<< "ss_queue.t_s1_ = " << ss_queue.top().t_s1_ << "\n"
//			<< "ss_queue.t_s2_ = " << ss_queue.top().t_s2_ << "\n \n";

//	//STa temp
//	double d_temp = -ss_queue.top().non_covered_length_ <= 0 ? -1 : -ss_queue.top().non_covered_length_/2;
	output_file_1 << nb_segment_eval << "\n";
//	output_file_1 << "min dist approx modulation = " << d_temp << "\n";
//	output_file_1 << nb_segment_eval << "\n \n \n \n";
	output_file_1.close();

	return (ss_queue.top().non_covered_length_ > 0 ? -1 : -ss_queue.top().non_covered_length_/2);


}



double ompl::base::SafeMotionValidator::minObstacleDistMotionDiscrete(const State *s1, const State *s2, int valid_segment_factor, bool fast_dist) const
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

double ompl::base::SafeMotionValidator::minObstacleDistMotionDiscreteExact(const State *s1, const State *s2, int valid_segment_factor) const
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
//	//STa temp
//	std::string homepath = getenv("HOME");
//	std::ofstream output_file_1((homepath + "/nb_segment_eval_SelfCCIndividualLinks.txt").c_str(), std::ios::out | std::ios::app);
//	size_t nb_segment_eval = 0;

	std::priority_queue<SubSegment> ss_queue;

	std::vector<double> dist_s1_self, dist_s2_self;
	std::vector<double> dist_travel;

	ssvc_->computeInitialDistDataSelf(s1, s2, dist_s1_self, dist_s2_self);
	ssvc_->computeInitialDistDataTravel(s1, s2, dist_travel);


	for (size_t i = 0; i < ssvc_->getNbSafetyLinks(); ++i)
	{
		if ((dist_s1_self[i] <= 0) || (dist_s2_self[i] <= 0))
		{
//			//STa temp
//			output_file_1 << nb_segment_eval << "\n";
//			output_file_1.close();

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

		//Interpolation guidée par la distance. Attention a modifier t_s1_ et t_s2_ si utilisée.
//		double t = current_ss.dist_s1_obs_ / (current_ss.dist_s1_obs_ + current_ss.dist_s2_obs_);
		double t = (2*current_ss.dist_s1_obs_ + current_ss.non_covered_length_) / (2*(current_ss.dist_s1_obs_ + current_ss.dist_s2_obs_ + current_ss.non_covered_length_));

		double t_abs = current_ss.t_s1_ + (current_ss.t_s2_ - current_ss.t_s1_) * t;


		stateSpace_->interpolate(s1, s2, t_abs, s_new);

//		//STa temp
//		nb_segment_eval ++;

		double dist_snew_self = ssvc_->computeLinkMinSelfDist(s_new, current_ss.link_index_);

		if (dist_snew_self <= 0)
		{
//			//STa temp
//			output_file_1 << nb_segment_eval << "\n";
//			output_file_1.close();

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


		si_->freeState(s_new);

		if (!ss_queue.empty())
			flag = dist_travel[ss_queue.top().link_index_]*(ss_queue.top().t_s2_ - ss_queue.top().t_s1_) < travel_dist_limit;

	}
//	//STa temp
//	output_file_1 << nb_segment_eval << "\n";
//	output_file_1.close();

	if (!ss_queue.empty() && ss_queue.top().non_covered_length_ > 0)
		return false;
	else
		return true;

}

bool ompl::base::SafeMotionValidator::checkMotionSelfCCDiscrete(const State *s1, const State *s2, int valid_segment_factor) const
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
				{
					return false; //Collision
				}
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
//		double t = current_ss.dist_s1_obs_ / (current_ss.dist_s1_obs_ + current_ss.dist_s2_obs_);
		double t = (2*current_ss.dist_s1_obs_ + current_ss.non_covered_length_) / (2*(current_ss.dist_s1_obs_ + current_ss.dist_s2_obs_ + current_ss.non_covered_length_));

		double t_abs = current_ss.t_s1_ + (current_ss.t_s2_ - current_ss.t_s1_) * t;

		stateSpace_->interpolate(s1, s2, t_abs, s_new);

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


		si_->freeState(s_new);

		if (!ss_queue.empty())
			flag = dist_travel[ss_queue.top().link_index_]*(ss_queue.top().t_s2_ - ss_queue.top().t_s1_) < travel_dist_limit;

	}
	if (!ss_queue.empty() && ss_queue.top().non_covered_length_ > 0)
		return false;
	else
		return true;

}
