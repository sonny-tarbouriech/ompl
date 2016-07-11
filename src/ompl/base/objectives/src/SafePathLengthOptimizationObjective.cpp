#include "ompl/base/objectives/SafePathLengthOptimizationObjective.h"

ompl::base::SafePathLengthOptimizationObjective::
SafePathLengthOptimizationObjective(const SpaceInformationPtr &si) :
ompl::base::OptimizationObjective(si),
goal_(NULL),
start_state_(NULL)
{
	isMinMaxObjective_ = false;
	description_ = "Safe Path Length";
}

ompl::base::SafePathLengthOptimizationObjective::
SafePathLengthOptimizationObjective(const SpaceInformationPtr &si, State** start, Goal** goal) :
ompl::base::OptimizationObjective(si),
goal_(goal),
start_state_(start)
{
    isMinMaxObjective_ = false;
	description_ = "Safe Path Length";
}

bool ompl::base::SafePathLengthOptimizationObjective::isSymmetric() const
{
	return true;
}

ompl::base::Cost ompl::base::SafePathLengthOptimizationObjective::identityCost() const
{
	return Cost(0);
}

ompl::base::Cost ompl::base::SafePathLengthOptimizationObjective::infiniteCost() const
{
	return Cost(std::numeric_limits<double>::infinity());
}

ompl::base::Cost ompl::base::SafePathLengthOptimizationObjective::combineCosts(Cost c1, Cost c2) const
{
	return Cost(c1.value() + c2.value());
}

ompl::base::Cost
ompl::base::SafePathLengthOptimizationObjective::combineCosts(double motion_length_c1, double motion_length_c2, double dist_start_to_motion_c1, double dist_end_to_goal_c2) const
{
	double d_cost = (dist_start_to_motion_c1 + motion_length_c1 + motion_length_c2 + dist_end_to_goal_c2);
	return Cost(d_cost);
}

ompl::base::Cost ompl::base::SafePathLengthOptimizationObjective::stateCost(const State *s) const
{
	return Cost(0);
}

ompl::base::Cost ompl::base::SafePathLengthOptimizationObjective::costToCome(const State *s) const
{
	if (*start_state_ != NULL)
		return Cost(si_->distance(*start_state_, s));
	else
		return Cost(0);
}

ompl::base::Cost ompl::base::SafePathLengthOptimizationObjective::costToGo(const State *s) const
{
	if (*goal_ != NULL)
		return base::goalRegionCostToGo(s, *goal_);
	else
		return Cost(0);
}


ompl::base::Cost
ompl::base::SafePathLengthOptimizationObjective::motionCost(const State *s1, const State *s2) const
{
	return Cost(si_->distance(s1, s2));
}

ompl::base::Cost
ompl::base::SafePathLengthOptimizationObjective::motionCost(const State *s1, const State *s2, double& motion_length, double& dist_start_to_motion, double& dist_end_to_goal) const
{
	motion_length = si_->distance(s1, s2);
	dist_start_to_motion = si_->distance(*start_state_, s1);
	dist_end_to_goal = base::goalRegionCostToGo(s2, *goal_).value();

	double d_cost = (dist_start_to_motion + motion_length + dist_end_to_goal);

	return Cost(d_cost);
}

ompl::base::Cost
ompl::base::SafePathLengthOptimizationObjective::motionCost(double motion_length, double dist_start_to_motion, double dist_end_to_goal) const
{
	double d_cost = dist_start_to_motion + motion_length + dist_end_to_goal;
	return Cost(d_cost);
}


ompl::base::Cost
ompl::base::SafePathLengthOptimizationObjective::motionCostHeuristic(const State *s1,
		const State *s2) const
{
	return motionCost(s1, s2);
}
