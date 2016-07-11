#ifndef OMPL_BASE_SAFE_MULTI_OPTIMIZATION_OBJECTIVE_
#define OMPL_BASE_SAFE_MULTI_OPTIMIZATION_OBJECTIVE_

#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/Goal.h"
#include "ompl/base/State.h"
#include "ompl/base/SafetyCost.h"

namespace ompl
{
	namespace base
	{
    	class SafeMultiOptimizationObjective : public MultiOptimizationObjective
    	{
    	public:
    		SafeMultiOptimizationObjective(const SpaceInformationPtr &si);
    		SafeMultiOptimizationObjective(const SpaceInformationPtr &si, bool minMaxObjectiveImprovement);

    		SafetyCost safeStateCost(const State *s) const;
    		SafetyCost safeCostToCome(const State *s, SafetyCost safe_state_cost) const;
    		SafetyCost safeCostToGo(const State *s, SafetyCost safe_state_cost) const;
    		SafetyCost safeIdentityCost() const;
    		SafetyCost safeInfiniteCost() const;

    		bool isCostDefined(SafetyCost c) const;
    		bool isSafetySatisfied(SafetyCost c) const;


            /** \brief Since we wish to maximize safety, and costs are equivalent to path safety, we return the greater of the two cost values. */
            virtual bool isCostBetterThan(Cost c1, Cost c2) const;
            bool isSafetyCostBetterThan(SafetyCost c1, SafetyCost c2) const;
            bool isMinMaxSafetyCostBetterThan(SafetyCost c1, SafetyCost c2) const;
            double safetyCostImprovement(SafetyCost c1, SafetyCost c2) const;

            virtual Cost motionCost(const State *s1, const State *s2) const;
            SafetyCost safeMotionCost(const State *s1, const State *s2) const;
            SafetyCost safeFastMotionCost(const State *s1, const State *s2, SafetyCost c1, SafetyCost c2) const;
            SafetyCost safeMotionCostSymmetric(const State *s1, const State *s2, SafetyCost symCost) const;


            virtual Cost combineCosts(Cost c1, Cost c2) const;
            SafetyCost safeCombineCosts(SafetyCost c1, SafetyCost c2) const;

            /** \brief Returns -infinity, since any cost combined with -infinity under this objective will always return the other cost. */
            virtual Cost identityCost() const;

            /** \brief Returns +infinity, since no path safety value can be considered worse than this. */
            virtual Cost infiniteCost() const;

            void addObjective(const OptimizationObjectivePtr& objective, double weight, std::string name);

            void NormalizeWeight();

            void addObjectiveFirst(const OptimizationObjectivePtr& objective, double weight);

            void setGoal(Goal* goal)
            {
            	goal_ = goal;
            }

            Goal*& getGoal()
            {
            	return goal_;
            }

            void setStart(State* start_state)
            {
            	start_state_ = start_state;
            }

            State*& getStart()
            {
            	return start_state_;
            }


            bool hasSafetyObjective()
            {
            	return (safety_obj_index_ >= 0);
            }

    	private:
             Goal* goal_;
             State* start_state_;

             double total_weight;
             int length_obj_index_;
             int safety_obj_index_;
             int joint_obj_index_;
             int manipulability_obj_index_;
             int awareness_obj_index_;

             bool minMaxObjectiveImprovement_;

        };
    }
}

#endif
