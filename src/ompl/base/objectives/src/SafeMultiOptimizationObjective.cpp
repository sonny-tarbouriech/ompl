#include "ompl/base/objectives/SafeMultiOptimizationObjective.h"
#include "ompl/base/objectives/SafePathLengthOptimizationObjective.h"
#include "ompl/base/objectives/SafetyObjective.h"
#include "ompl/tools/config/MagicConstants.h"

//STa temp
#include <fstream>

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
//    //STa temp
//    std::cout << "enter safeCostToCome \n";
//    std::cout << "safe_state_cost.getCollisionWorld() = " << safe_state_cost.getCollisionWorld() << "\n" ;

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

//    //STa temp
//    std::cout << "exit safeCostToCome \n";

	return c;
}

ompl::base::SafetyCost ompl::base::SafeMultiOptimizationObjective::safeCostToGo(const State *s, SafetyCost safe_state_cost) const
{
//    //STa temp
//    std::cout << "enter safeCostToGo \n";

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

//    //STa temp
//    std::cout << "exit safeCostToGo \n";

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

	//	std::cout << "c.getIndividualCostSize() = " << c.getIndividualCostSize() << "\n";
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
	//std::cou << "enter isSafetyCostBetterThan \n";

//    if (c1.getIndividualCostSize() != components_.size() || c2.getIndividualCostSize() != components_.size())
//    {
//        OMPL_ERROR("SafeMultiOptimizationObjective isSafetyCostBetterThan: Invalid individual cost size. c1 is of size %u, c2 is of size %u and the valid size is %u", c1.getIndividualCostSize(), c2.getIndividualCostSize(), components_.size());
//    }

	if (c1.getCollisionWorld())
		return false;
	else if (c2.getCollisionWorld())
		return true;

	double improv1 =0, improv2 =0;
	for (size_t i=0; i< components_.size(); ++i)
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

	//std::cou << "exit isSafetyCostBetterThan \n";

	return improv1  > improv2;
}

bool ompl::base::SafeMultiOptimizationObjective::isMinMaxSafetyCostBetterThan(SafetyCost c1, SafetyCost c2) const
{
    //std::cou << "enter isMinMaxSafetyCostBetterThan \n";

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

    //std::cou << "exit isSafetyCostBetterThan \n";

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

ompl::base::Cost ompl::base::SafeMultiOptimizationObjective::motionCost(const State *s1,
		const State *s2) const
{
	Cost c(0);
	for (std::vector<Component>::const_iterator comp = components_.begin();
			comp != components_.end();
			++comp)
	{
		c = Cost(c.value() + comp->weight * (comp->objective->motionCost(s1,s2).value()));
	}

	//STa temp
	//    std::cout << "SafeMultiOptimizationObjective::motionCost : " <<  c << "\n";

	return c;
}

ompl::base::SafetyCost ompl::base::SafeMultiOptimizationObjective::safeMotionCost(const State *s1, const State *s2) const
{
//	std::cout << "enter safeMotionCost \n";

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
				//std::cou << "exit safeMotionCost \n";
				c.setCollisionWorld(true);
				break;
			}
		}
		else
			individual_cost = components_[i].objective->motionCost(s1,s2);

		c.addCost(individual_cost);
	}

//	std::cout << "exit safeMotionCost \n";

	return c;
}

//STa test

ompl::base::SafetyCost ompl::base::SafeMultiOptimizationObjective::safeMotionCostTEST(const State *s1, const State *s2) const
{
    SafetyCost c;
    for (size_t i=0; i< components_.size(); ++i)
    {
        if (int(i) == safety_obj_index_)
        {
            SafetyObjective* so = static_cast<SafetyObjective*>(components_[i].objective.get());
            Cost individual_cost(so->motionCostInterpolation(s1,s2));
            if (individual_cost.value() < 0)
                c.addCost(Cost(0)); //Collision with world
            else
            {
                c.addCost(individual_cost);
            }
        }

        else
        {
            Cost individual_cost(components_[i].objective->motionCost(s1,s2));
            c.addCost(individual_cost);
        }

    }

    return c;
}


ompl::base::SafetyCost ompl::base::SafeMultiOptimizationObjective::safeFastMotionCost(const State *s1, const State *s2, SafetyCost c1, SafetyCost c2) const
{
	//std::cou << "enter safeFastMotionCost \n";


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
				//std::cou << "exit safeFastMotionCost \n";
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
	//std::cou << "exit safeFastMotionCost \n";

	return c;
}

//STa : To use with TRRT algorithm
ompl::base::Cost ompl::base::SafeMultiOptimizationObjective::safeMotionMechanicalWork(const State *s1, const State *s2) const
{
	return safeMotionMechanicalWork(safeStateCost(s1), safeStateCost(s2));
}

ompl::base::Cost ompl::base::SafeMultiOptimizationObjective::safeMotionMechanicalWork(SafetyCost c1, SafetyCost c2) const
{
	if (c1.getIndividualCostSize() != c2.getIndividualCostSize())
	{
		throw Exception("Cannot compare costs of different sizes");
	}

	double dcost = 0;
	for (size_t i=0; i< c1.getIndividualCostSize(); ++i)
	{
		double rate;
		if (c1.getIndividualCost(i).value() > c2.getIndividualCost(i).value())
		{
			rate = c1.getIndividualCost(i).value() / c2.getIndividualCost(i).value();
			if (components_[i].objective->infiniteCost().value() == 0)
				rate = - rate;
		}
		else
		{
			rate = c2.getIndividualCost(i).value() / c1.getIndividualCost(i).value();
			if (!components_[i].objective->infiniteCost().value() == 0)
				rate = - rate;
		}
		dcost += rate * components_[i].weight;

	}
	return Cost(std::max(dcost, 0.0));
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

	//std::cou << "exit safeMotionCostSymmetric \n";


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

	if (c1.getCollisionWorld() || c2.getCollisionWorld())
	{
		c.setCollisionWorld(true);
		return c; //Collision world detected by safety objective
	}

	Cost individual_cost;
	for (size_t i=0; i< components_.size(); ++i)
	{
		if (int(i) == safety_obj_index_)
		{
		    //STa temp
		    if (c1.isImprovingSafety())
		    {
		        std::cout << "c1.getIndividualCost(i) = " << c1.getIndividualCost(i) << "\n";
                std::cout << "c2.getIndividualCost(i) = " << c2.getIndividualCost(i) << "\n \n";

		    }

			SafetyObjective* so = static_cast<SafetyObjective*>(components_[i].objective.get());
			if (minMaxObjectiveImprovement_ && !c1.isRoot() && c1.isImprovingSafety() && !so->isCostBetterThan(c1.getIndividualCost(i), c1.getObjectDangerFactor(), c2.getIndividualCost(i), c2.getObjectDangerFactor()))
			{
				//STa temp
				std::cout << "safety improved \n";

				individual_cost = c2.getIndividualCost(i);
				c.setObjectDangerFactor(c2.getObjectDangerFactor());
			}
			else
			{
				double object_danger_factor;
				individual_cost = so->combineCosts(c1.getIndividualCost(i), c1.getObjectDangerFactor(), c2.getIndividualCost(i), c2.getObjectDangerFactor(), object_danger_factor);
				c.setObjectDangerFactor(object_danger_factor);
				if (!c1.isRoot())
				    c.isImprovingSafety() = false;
			}

			if (individual_cost.value() <= 0)
			{
				c.setCollisionWorld(true);
				break;
			}
		}
		else if (int(i) == joint_obj_index_)
		{
			if (minMaxObjectiveImprovement_ && !c1.isRoot() && c1.isImprovingJoint() && !components_[i].objective->isCostBetterThan(c1.getIndividualCost(i), c2.getIndividualCost(i)))
			{
				//STa temp
				std::cout << "joint limits improved \n";


				individual_cost = c2.getIndividualCost(i);
			}
			else
			{
				individual_cost = Cost(components_[i].objective->combineCosts(c1.getIndividualCost(i), c2.getIndividualCost(i)));
				if (!c1.isRoot())
				    c.isImprovingJoint() = false;
			}
		}
		else if (int(i) == manipulability_obj_index_)
		{
			if (minMaxObjectiveImprovement_ && !c1.isRoot() &&  c1.isImprovingManipulability() && !components_[i].objective->isCostBetterThan(c1.getIndividualCost(i), c2.getIndividualCost(i)))
			{
				//STa temp
				std::cout << "manipulability improved \n";


				individual_cost = c2.getIndividualCost(i);
			}
			else
			{
				individual_cost = Cost(components_[i].objective->combineCosts(c1.getIndividualCost(i), c2.getIndividualCost(i)));
				if (!c1.isRoot())
				    c.isImprovingManipulability() = false;
			}
		}
		else if (int(i) == awareness_obj_index_)
		{
			if (minMaxObjectiveImprovement_ && !c1.isRoot() && c1.isImprovingAwareness() && !components_[i].objective->isCostBetterThan(c1.getIndividualCost(i), c2.getIndividualCost(i)))
			{
//				//STa temp
//				std::cout << "awareness improved \n";


				individual_cost = c2.getIndividualCost(i);
			}
			else
			{
				individual_cost = Cost(components_[i].objective->combineCosts(c1.getIndividualCost(i), c2.getIndividualCost(i)));
				if (!c1.isRoot())
				    c.isImprovingAwareness() = false;
			}
		}

		else
			individual_cost = Cost(components_[i].objective->combineCosts(c1.getIndividualCost(i), c2.getIndividualCost(i)));

		c.addCost(individual_cost);
	}
	//std::cou << "exit safeCombineCosts \n";


	return c;
}


ompl::base::Cost ompl::base::SafeMultiOptimizationObjective::identityCost() const
{
	return Cost(1);
}

ompl::base::Cost ompl::base::SafeMultiOptimizationObjective::infiniteCost() const
{
	return Cost(0);
}

ompl::base::SafetyCost ompl::base::SafeMultiOptimizationObjective::safeMotionCostHeuristic(const State *s1, const State *s2) const
{
	SafetyCost c;
	for (size_t i=0; i< components_.size(); ++i)
	{
		Cost individual_cost;
		if (int(i) == length_obj_index_)
		{
			individual_cost = components_[i].objective->motionCost(s1,s2);
		}
		else
		{
			Cost c1, c2;
			c1 = components_[i].objective->stateCost(s1);
			c2 = components_[i].objective->stateCost(s2);
			individual_cost = components_[i].objective->combineCosts(c1,c2);
			if (int(i) == safety_obj_index_ && individual_cost.value() <= 0)
			{
				c.setCollisionWorld(true);
				break;
			}
		}
		c.addCost(individual_cost);
	}
	return c;
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

