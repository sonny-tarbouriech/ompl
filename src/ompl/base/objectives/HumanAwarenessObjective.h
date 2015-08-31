#ifndef OMPL_BASE_OBJECTIVES_HUMAN_AWARENESS_OBJECTIVE_
#define OMPL_BASE_OBJECTIVES_HUMAN_AWARENESS_OBJECTIVE_

#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/SafeStateValidityChecker.h"

namespace ompl
{
    namespace base
    {
        /** \brief Objective for attempting to maximize the safety along a path. */
        class HumanAwarenessObjective : public OptimizationObjective
        {
        public:

        	HumanAwarenessObjective(const SpaceInformationPtr &si);

        	HumanAwarenessObjective(const SpaceInformationPtr &si, Cost cost_threshold);

            virtual bool isSymmetric() const;

            /** \brief Defined as the safety of the state \e s, which is computed using the StateValidityChecker in this objective's SpaceInformation */
            virtual Cost stateCost(const State *s) const;

            virtual Cost motionCost(const State *s1, const State *s2) const;

            virtual Cost combineCosts(Cost c1, Cost c2) const;

            /** \brief Returns -infinity, since any cost combined with -infinity under this objective will always return the other cost. */
            virtual Cost identityCost() const;

            /** \brief Returns +infinity, since no path safety value can be considered worse than this. */
            virtual Cost infiniteCost() const;

        private:

            ompl::base::SafeStateValidityChecker* ssvc_;

        };
    }
}

#endif
