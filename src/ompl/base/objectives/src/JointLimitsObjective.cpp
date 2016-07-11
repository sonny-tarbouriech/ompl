#include "ompl/base/objectives/JointLimitsObjective.h"
#include "ompl/tools/config/MagicConstants.h"
#include "ompl/geometric/PathGeometric.h"
#include <limits>

ompl::base::JointLimitsObjective::
JointLimitsObjective(const SpaceInformationPtr &si) :
OptimizationObjective(si)
{
    isMinMaxObjective_ = true;
	this->setCostThreshold(identityCost());

	ssvc_ = dynamic_cast<ompl::base::SafeStateValidityChecker*>(si_->getStateValidityChecker().get());
	if (!ssvc_)
		throw Exception("JointLimitsObjective requires SafeStateValidityChecker to work");

	ssvc_->getJointLimits(q_min_,q_max_);
}


ompl::base::JointLimitsObjective::
JointLimitsObjective(const SpaceInformationPtr &si, Cost cost_threshold) :
OptimizationObjective(si)
{
    isMinMaxObjective_ = true;
	this->setCostThreshold(cost_threshold);

	ssvc_ = dynamic_cast<ompl::base::SafeStateValidityChecker*>(si_->getStateValidityChecker().get());
	if (!ssvc_)
		throw Exception("JointLimitsObjective requires SafeStateValidityChecker to work");

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
    return c1.value() > c2.value();
}

ompl::base::Cost ompl::base::JointLimitsObjective::motionCost(const State *s1, const State *s2) const
{
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
