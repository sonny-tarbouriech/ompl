#include "ompl/base/objectives/SafeMultiOptimizationObjective.h"
#include "ompl/base/objectives/SafePathLengthOptimizationObjective.h"
#include "ompl/base/objectives/SafetyObjective.h"
#include "ompl/tools/config/MagicConstants.h"

ompl::base::SafeMultiOptimizationObjective::
SafeMultiOptimizationObjective(const SpaceInformationPtr &si) :
MultiOptimizationObjective(si),
goal_(NULL),
start_state_(NULL),
total_weight(0),
length_obj_index_(-1),
safety_obj_index_(-1),
joint_obj_index_(-1),
manipulability_obj_index_(-1),
awareness_obj_index_(-1),
minMaxObjectiveImprovement_(false)
{
	this->setCostThreshold(identityCost());
}

ompl::base::SafeMultiOptimizationObjective::
SafeMultiOptimizationObjective(const SpaceInformationPtr &si, bool minMaxObjectiveImprovement) :
MultiOptimizationObjective(si),
goal_(NULL),
start_state_(NULL),
total_weight(0),
length_obj_index_(-1),
safety_obj_index_(-1),
joint_obj_index_(-1),
manipulability_obj_index_(-1),
awareness_obj_index_(-1),
minMaxObjectiveImprovement_(minMaxObjectiveImprovement)
{
    this->setCostThreshold(identityCost());
}

ompl::base::SafetyCost ompl::base::SafeMultiOptimizationObjective::safeStateCost(const State *s) const
{
	SafetyCost c;
	for (size_t i=0; i< components_.size(); ++i)
	{
		Cost individual_cost;

		if (int(i) == safety_obj_index_)
		{
			double object_danger_factor;
			SafetyObjective* so = static_cast<SafetyObjective*>(components_[i].objective.get());
			individual_cost = so->stateCost(s, object_danger_factor);
			c.setObjectDangerFactor(object_danger_factor);

			if (individual_cost.value() <= 0)
			{
				c.setCollisionWorld(true);
				break;
			}
		}
		else individual_cost = components_[i].objective->stateCost(s);

		c.addCost(individual_cost);

	}
	return c;
}

ompl::base::SafetyCost ompl::base::SafeMultiOptimizationObjective::safeCostToCome(const State *s, SafetyCost safe_state_cost) const
{
    //Warning : The state s should not be in collision
	SafetyCost c;
	for (size_t i=0; i< components_.size(); ++i)
	{
		Cost individual_cost;
		if (int(i) == length_obj_index_)
		{
			SafePathLengthOptimizationObjective* spl = static_cast<SafePathLengthOptimizationObjective*>(components_[i].objective.get());
			individual_cost = spl->costToCome(s);
		}
		else
			individual_cost = safe_state_cost.getIndividualCost(i);

		c.addCost(individual_cost);
	}
	c.setObjectDangerFactor(safe_state_cost.getObjectDangerFactor());
	return c;
}

ompl::base::SafetyCost ompl::base::SafeMultiOptimizationObjective::safeCostToGo(const State *s, SafetyCost safe_state_cost) const
{
    //Warning : The state s should not be in collision
	SafetyCost c;
	for (size_t i=0; i< components_.size(); ++i)
	{
		Cost individual_cost;
		if (int(i) == length_obj_index_)
		{
			SafePathLengthOptimizationObjective* spl = static_cast<SafePathLengthOptimizationObjective*>(components_[i].objective.get());
			individual_cost = spl->costToGo(s);
		}
		else
			individual_cost = safe_state_cost.getIndividualCost(i);

		c.addCost(individual_cost);
	}
	c.setObjectDangerFactor(safe_state_cost.getObjectDangerFactor());
	return c;
}

ompl::base::SafetyCost ompl::base::SafeMultiOptimizationObjective::safeIdentityCost() const
{
	SafetyCost c;
	for (size_t i=0; i< components_.size(); ++i)
	{
		Cost individual_cost(components_[i].objective->identityCost());
		c.addCost(individual_cost);
	}
	return c;
}

ompl::base::SafetyCost ompl::base::SafeMultiOptimizationObjective::safeInfiniteCost() const
{
	SafetyCost c;
	for (size_t i=0; i< components_.size(); ++i)
	{
		Cost individual_cost(components_[i].objective->infiniteCost());
		c.addCost(individual_cost);
	}
	return c;
}

bool ompl::base::SafeMultiOptimizationObjective::isCostDefined(SafetyCost c) const
{
    if (c.getIndividualCostSize() != components_.size())
        return false;

    //STa : If any individual cost is not infinite, then the global cost is defined
    return (c.getIndividualCost(0).value() != components_[0].objective->infiniteCost().value());
}

bool ompl::base::SafeMultiOptimizationObjective::isSafetySatisfied(SafetyCost c) const
{
	if (c.getIndividualCostSize() != components_.size())
		return false;

	for (size_t i=0; i< components_.size(); ++i)
	{
		if(!components_[i].objective->isSatisfied(c.getIndividualCost(i)))
		{
			return false;
		}
	}
	return true;
}


bool ompl::base::SafeMultiOptimizationObjective::isCostBetterThan(Cost c1, Cost c2) const
{
	return c1.value() > c2.value();
}

bool ompl::base::SafeMultiOptimizationObjective::isSafetyCostBetterThan(SafetyCost c1, SafetyCost c2) const
{
	if (c1.getCollisionWorld())
		return false;
	else if (c2.getCollisionWorld())
		return true;

	double improv1 =0, improv2 =0, c1_value=0, c2_value=0, c1_factor=0, c2_factor=0;
	for (size_t i=0; i< components_.size(); ++i)
	{
	    double rate;
	    c1_value = c1.getIndividualCost(i).value();
	    c2_value = c2.getIndividualCost(i).value();
	    if (int(i) == safety_obj_index_)
	    {
	        c1_factor = c1.getObjectDangerFactor();
	        c2_factor = c2.getObjectDangerFactor();
	    }
	    if (minMaxObjectiveImprovement_ && c1.hasImprovedCosts() && c2.hasImprovedCosts())
	    {
	        bool isEquivalent;
	        if (int(i) == safety_obj_index_)
	            isEquivalent = static_cast<SafetyObjective*>(components_[i].objective.get())->isCostEquivalentTo(c1.getIndividualCost(i), c1_factor, c2.getIndividualCost(i), c2_factor);
	        else
	            isEquivalent = components_[i].objective->isCostEquivalentTo(c1.getIndividualCost(i), c2.getIndividualCost(i));
	        if (isEquivalent)
	        {
	        	c1_value = c1.getIndividualCostImproved(i).value();
	        	if (int(i) == safety_obj_index_)
	        		c1_factor = c1.getObjectDangerFactorImproved();

	        	c2_value = c2.getIndividualCostImproved(i).value();
	        	if (int(i) == safety_obj_index_)
	        		c2_factor = c2.getObjectDangerFactorImproved();
	        }
	    }

	    rate = c1_value / c2_value;
	    if (int(i) == safety_obj_index_)
	    {
	        rate *= c1_factor/c2_factor;
	    }

		if (components_[i].objective->infiniteCost().value() == 0)
		{
			improv1 += rate * components_[i].weight;
			improv2 += (1/rate) * components_[i].weight;
		}
		else
		{
			improv2 += rate * components_[i].weight;
			improv1 += (1/rate) * components_[i].weight;
		}
	}

	return improv1  > improv2;
}

bool ompl::base::SafeMultiOptimizationObjective::isMinMaxSafetyCostBetterThan(SafetyCost c1, SafetyCost c2) const
{
    if (c1.getIndividualCostSize() != components_.size() || c2.getIndividualCostSize() != components_.size())
    {
        OMPL_ERROR("SafeMultiOptimizationObjective isMinMaxSafetyCostBetterThan: Invalid individual cost size. c1 is of size %u, c2 is of size %u and the valid size is %u", c1.getIndividualCostSize(), c2.getIndividualCostSize(), components_.size());
    }

    double improv1 =0, improv2 =0;
    for (size_t i=0; i< components_.size(); ++i)
    {
        if (components_[i].objective->isMinMaxObjective())
        {
            double rate = c1.getIndividualCost(i).value() / c2.getIndividualCost(i).value();
            if (int(i) == safety_obj_index_)
            {
                rate *= double(c1.getObjectDangerFactor())/c2.getObjectDangerFactor();
            }
            if (components_[i].objective->infiniteCost().value() == 0)
            {
                improv1 += rate * components_[i].weight;
                improv2 += (1/rate) * components_[i].weight;
            }
            else
            {
                improv2 += rate * components_[i].weight;
                improv1 += (1/rate) * components_[i].weight;
            }
        }
    }
    return improv1  > improv2;
}

double ompl::base::SafeMultiOptimizationObjective::safetyCostImprovement(SafetyCost c1, SafetyCost c2) const
{
	double improv = 0;
	for (size_t i=0; i< components_.size(); ++i)
	{
		double rate = c1.getIndividualCost(i).value() / c2.getIndividualCost(i).value();
		if (components_[i].objective->infiniteCost().value() == 0)
		{
			improv += rate * components_[i].weight;
		}
		else
		{
			improv += (1/rate) * components_[i].weight;
		}
	}
	return improv;
}

ompl::base::Cost ompl::base::SafeMultiOptimizationObjective::motionCost(const State *s1, const State *s2) const
{
	Cost c(0);
	for (std::vector<Component>::const_iterator comp = components_.begin();
			comp != components_.end();
			++comp)
	{
		c = Cost(c.value() + comp->weight * (comp->objective->motionCost(s1,s2).value()));
	}
	return c;
}

ompl::base::SafetyCost ompl::base::SafeMultiOptimizationObjective::safeMotionCost(const State *s1, const State *s2) const
{
	SafetyCost c;
	for (size_t i=0; i< components_.size(); ++i)
	{
		Cost individual_cost;

		if (int(i) == safety_obj_index_)
		{
			double object_danger_factor;
			SafetyObjective* so = static_cast<SafetyObjective*>(components_[i].objective.get());
			individual_cost = so->motionCost(s1,s2, object_danger_factor);
			c.setObjectDangerFactor(object_danger_factor);
			if (individual_cost.value() <= 0)
			{
				c.setCollisionWorld(true);
				break;
			}
		}
		else
			individual_cost = components_[i].objective->motionCost(s1,s2);

		c.addCost(individual_cost);
	}
	return c;
}


ompl::base::SafetyCost ompl::base::SafeMultiOptimizationObjective::safeFastMotionCost(const State *s1, const State *s2, SafetyCost c1, SafetyCost c2) const
{
	SafetyCost c;
	for (size_t i=0; i< components_.size(); ++i)
	{
		Cost individual_cost;
		if (int(i) == length_obj_index_)
		{
			individual_cost = components_[i].objective->motionCost(s1,s2);

		}
		else if (int(i) == safety_obj_index_)
		{
			double object_danger_factor;
			SafetyObjective* so = static_cast<SafetyObjective*>(components_[i].objective.get());
			individual_cost = so->combineCosts(c1.getIndividualCost(i), c1.getObjectDangerFactor(), c2.getIndividualCost(i), c2.getObjectDangerFactor(), object_danger_factor);
			c.setObjectDangerFactor(object_danger_factor);
			if (individual_cost.value() <= 0)
			{
				c.setCollisionWorld(true);
				break;
			}
		}
		else
		{
			individual_cost = components_[i].objective->combineCosts(c1.getIndividualCost(i),c2.getIndividualCost(i));
		}
		c.addCost(individual_cost);
	}
	return c;
}

ompl::base::SafetyCost ompl::base::SafeMultiOptimizationObjective::safeMotionCostSymmetric(const State *s1, const State *s2, SafetyCost symCost) const
{
	SafetyCost c;

	if (symCost.getCollisionWorld())
	{
		c.setCollisionWorld(true);
		return c; //Collision world detected by safety objective
	}

	for (size_t i=0; i< components_.size(); ++i)
	{
		if (components_[i].objective->isSymmetric())
		{
			Cost individual_cost(symCost.getIndividualCost(i));
			c.addCost(individual_cost);
		}

		else //Not symmetric -> We need to compute
		{
			Cost individual_cost(components_[i].objective->motionCost(s1,s2));
			c.addCost(individual_cost);
		}
	}
	c.setObjectDangerFactor(symCost.getObjectDangerFactor());
	return c;
}


ompl::base::Cost ompl::base::SafeMultiOptimizationObjective::combineCosts(Cost c1, Cost c2) const
{
	if (this->isCostBetterThan(c1, c2))
		return c2;
	else
		return c1;
}

ompl::base::SafetyCost ompl::base::SafeMultiOptimizationObjective::safeCombineCosts(SafetyCost c1, SafetyCost c2) const
{
	SafetyCost c;
	SafetyObjective* so = NULL;

	if (c1.getCollisionWorld() || c2.getCollisionWorld())
	{
		c.setCollisionWorld(true);
		return c; //Collision world detected by safety objective
	}

	Cost individual_cost;
	for (size_t i=0; i< components_.size(); ++i)
	{
	    //STa : If current component corresponds to clearance objective, we need to cast it in order to get the specific function that uses object danger factors
        if (int(i) == safety_obj_index_)
        {
            so = static_cast<SafetyObjective*>(components_[i].objective.get());
            double object_danger_factor;
            individual_cost = so->combineCosts(c1.getIndividualCost(i), c1.getObjectDangerFactor(), c2.getIndividualCost(i), c2.getObjectDangerFactor(), object_danger_factor);
            c.setObjectDangerFactor(object_danger_factor);

            //STa : The clearance objective is also used as a collision checker
            if (individual_cost.value() <= 0)
            {
                c.setCollisionWorld(true);
                break;
            }
        }
        else
        {
            individual_cost = components_[i].objective->combineCosts(c1.getIndividualCost(i), c2.getIndividualCost(i));
        }
        c.addCost(individual_cost);

        //STa : If the following feature is enabled, a second cost is computed for min-max objectives. It represents the continuous cost improvement form the root of the tree.
        //If heuristic combination is being used, the second condition is false and we don't compute this cost
        if (minMaxObjectiveImprovement_ && (c1.isRoot() || c1.hasImprovedCosts()))
        {
            Cost c2_improved;
            double c2_object_danger_factor;

            if (so)
            {
                if (c1.isRoot())
                {
                    c.addCostImproved(individual_cost);
                    c.addIsImprovingCost(true);
                    c.setObjectDangerFactorImproved(c.getObjectDangerFactor());
                }
                else //c1 has necessarily improved costs
                {
                    if (c2.hasImprovedCosts())
                    {
                        c2_improved = c2.getIndividualCostImproved(i);
                        c2_object_danger_factor = c2.getObjectDangerFactorImproved();
                    }
                    else
                    {
                        c2_improved = c2.getIndividualCost(i);
                        c2_object_danger_factor = c2.getObjectDangerFactor();
                    }
                    if (c1.isImprovingCost(i) && !so->isCostBetterThan(c1.getIndividualCostImproved(i), c1.getObjectDangerFactorImproved(), c2_improved, c2_object_danger_factor))
                    {
                        c.addCostImproved(c2_improved);
                        c.addIsImprovingCost(true);
                        c.setObjectDangerFactorImproved(c2_object_danger_factor);

                    }
                    else
                    {
                        double object_danger_factor;
                        c.addCostImproved(so->combineCosts(c1.getIndividualCostImproved(i), c1.getObjectDangerFactorImproved(), c2_improved, c2_object_danger_factor, object_danger_factor));
                        c.addIsImprovingCost(false);
                        c.setObjectDangerFactorImproved(object_danger_factor);
                    }
                }
                so = NULL;
            }
            else
            {
                if (c1.isRoot() || !components_[i].objective->isMinMaxObjective())
                {
                    c.addCostImproved(individual_cost);
                    c.addIsImprovingCost(true);
                }
                else //c1 has necessarily improved costs
                {
                    if (c2.hasImprovedCosts())
                        c2_improved = c2.getIndividualCostImproved(i);
                    else
                        c2_improved = c2.getIndividualCost(i);
                    if (c1.isImprovingCost(i) && !components_[i].objective->isCostBetterThan(c1.getIndividualCostImproved(i), c2_improved))
                    {
                        c.addCostImproved(c2_improved);
                        c.addIsImprovingCost(true);
                    }
                    else
                    {
                        c.addCostImproved(components_[i].objective->combineCosts(c1.getIndividualCostImproved(i),c2_improved));
                        c.addIsImprovingCost(false);
                    }
                }
            }
        }
	}
	return c;
}

//STa: Unused
ompl::base::Cost ompl::base::SafeMultiOptimizationObjective::identityCost() const
{
	return Cost(1);
}

//STa: Unused
ompl::base::Cost ompl::base::SafeMultiOptimizationObjective::infiniteCost() const
{
	return Cost(0);
}

void ompl::base::SafeMultiOptimizationObjective::addObjective(const OptimizationObjectivePtr& objective,
		double weight, std::string name)
{
	if(strcmp(name.c_str(), "safety") == 0)
	{
		addObjectiveFirst(objective,weight);
		total_weight += weight;
		safety_obj_index_ = 0;
		if (length_obj_index_ >= 0)
			length_obj_index_++;
		if (joint_obj_index_ >= 0)
			joint_obj_index_++;
		if (manipulability_obj_index_ >= 0)
			manipulability_obj_index_++;
		if (awareness_obj_index_ >= 0)
			awareness_obj_index_++;
	}
	else
	{
		MultiOptimizationObjective::addObjective(objective,weight);
		total_weight += weight;
		if(strcmp(name.c_str(), "length") == 0)
			length_obj_index_ = components_.size()-1;
		else if (strcmp(name.c_str(), "joint") == 0)
			joint_obj_index_ = components_.size()-1;
		else if (strcmp(name.c_str(), "manipulability") == 0)
			manipulability_obj_index_ = components_.size()-1;
		else if (strcmp(name.c_str(), "awareness") == 0)
			awareness_obj_index_ = components_.size()-1;
	}
}

void ompl::base::SafeMultiOptimizationObjective::NormalizeWeight()
{
	for (size_t i=0; i< components_.size(); ++i)
	{
		components_[i].weight /= total_weight;
	}
	total_weight = 1;

}

void ompl::base::SafeMultiOptimizationObjective::addObjectiveFirst(const OptimizationObjectivePtr& objective,
		double weight)
{
	if (locked_)
	{
		throw Exception("This optimization objective is locked. No further objectives can be added.");
	}
	else
		components_.insert(components_.begin(),Component(objective, weight));
}

