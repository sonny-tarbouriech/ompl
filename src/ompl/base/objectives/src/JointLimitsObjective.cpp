#include "ompl/base/objectives/JointLimitsObjective.h"
#include "ompl/tools/config/MagicConstants.h"
#include "ompl/geometric/PathGeometric.h"
#include <limits>

//STa test
#include <fstream>
#include <ompl/util/Time.h>

ompl::base::JointLimitsObjective::
JointLimitsObjective(const SpaceInformationPtr &si) :
JointLimitsObjective(si, identityCost())
{
}


ompl::base::JointLimitsObjective::
JointLimitsObjective(const SpaceInformationPtr &si, Cost cost_threshold) :
OptimizationObjective(si)
{
    isMinMaxObjective_ = true;
	this->setCostThreshold(cost_threshold);

	//TODO : return error if si_ has not a SafeStateValidityChecker
	ssvc_ = dynamic_cast<ompl::base::SafeStateValidityChecker*>(si_->getStateValidityChecker().get());

	ssvc_->getJointLimits(q_min_,q_max_);

}

bool ompl::base::JointLimitsObjective::isSymmetric() const
{
	return true;
}

ompl::base::Cost ompl::base::JointLimitsObjective::stateCost(const State *s) const
{
	std::vector<double> q = ssvc_->getJointValues(s);
	double q_mid, q_delta, q_diff;
	double dist = 0;
	for(size_t i=0; i < q.size(); ++i)
	{
		q_mid = (q_max_[i]+q_min_[i])/2;
		q_delta = (q_max_[i]-q_min_[i])/2;

		q_diff = std::abs(q[i]-q_mid);

		dist += q_delta - q_diff;
	}
	dist /= q.size();

	return Cost(dist);
}


bool ompl::base::JointLimitsObjective::isCostBetterThan(Cost c1, Cost c2) const
{
    return c1.value() > c2.value() + magic::BETTER_PATH_COST_MARGIN;
}

ompl::base::Cost ompl::base::JointLimitsObjective::motionCost(const State *s1, const State *s2) const
{

//	//STa test joint_limit
//	std::string homepath = getenv("HOME");
//	std::ofstream output_file((homepath + "/joint_limit_time.txt").c_str(), std::ios::out | std::ios::app);
//	std::ofstream output_file_2((homepath + "/joint_limit_result.txt").c_str(), std::ios::out | std::ios::app);
//	ompl::time::point init = ompl::time::now();
//	double c1 = std::min(stateCost(s1, 0.5).value(), stateCost(s2, 0.5).value());
//	ompl::time::duration dur = ompl::time::now() - init;
//	double c2 = std::min(stateCost(s1, 0.7).value(), stateCost(s2, 0.7).value());
//	double c3 = std::min(stateCost(s1, 0.8).value(), stateCost(s2, 0.8).value());
//	double c4 = std::min(stateCost(s1, 0.9).value(), stateCost(s2, 0.9).value());
//	double c5 = std::min(stateCost(s1, 0.95).value(), stateCost(s2, 0.95).value());
//	output_file << ompl::time::seconds(dur) << "\n";
//	output_file_2 << c1 << "  " << c2 << "  " << c3 << "  " << c4 << "  " << c5  << "\n";
//	output_file.close();
//	output_file_2.close();
//	return Cost(c3);


	return combineCosts(stateCost(s1), stateCost(s2));
}


ompl::base::Cost ompl::base::JointLimitsObjective::combineCosts(Cost c1, Cost c2) const
{
    if (this->isCostBetterThan(c1, c2))
        return c2;
    else
        return c1;
}

ompl::base::Cost ompl::base::JointLimitsObjective::identityCost() const
{
	return Cost(std::numeric_limits<double>::infinity());
}

ompl::base::Cost ompl::base::JointLimitsObjective::infiniteCost() const
{
    return Cost(0);
}
