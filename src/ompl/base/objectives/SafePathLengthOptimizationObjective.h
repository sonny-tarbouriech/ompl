#ifndef OMPL_BASE_OBJECTIVES_SAFE_PATH_LENGTH_OPTIMIZATION_OBJECTIVE_
#define OMPL_BASE_OBJECTIVES_SAFE_PATH_LENGTH_OPTIMIZATION_OBJECTIVE_

#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/Goal.h"
#include "ompl/base/State.h"

namespace ompl
{
    namespace base
    {
        /** \brief An optimization objective which corresponds to optimizing path length. */
        class SafePathLengthOptimizationObjective : public OptimizationObjective
        {
        public:
        	SafePathLengthOptimizationObjective(const SpaceInformationPtr &si);
        	SafePathLengthOptimizationObjective(const SpaceInformationPtr &si, State** start, Goal** goal);

        	virtual bool isSymmetric() const;

        	virtual Cost identityCost() const;
        	virtual Cost infiniteCost() const;

        	virtual Cost stateCost(const State *s) const;
        	Cost costToGo(const State *s) const;
        	Cost costToCome(const State *s) const;

            virtual Cost motionCost(const State *s1, const State *s2) const;
            Cost motionCost(const State *s1, const State *s2, double& motion_length, double& dist_start_to_motion, double& dist_end_to_goal) const;
            Cost motionCost(double motion_length, double dist_start_to_motion, double dist_end_to_goal) const;

            virtual Cost combineCosts(Cost c1, Cost c2) const;
            Cost combineCosts(double motion_length_c1, double motion_length_c2, double dist_start_to_motion_c1, double dist_end_to_goal_c2) const;

            virtual Cost motionCostHeuristic(const State *s1, const State *s2) const;

        private:

            Goal** goal_;
            State** start_state_;

        };
    }
}

#endif
