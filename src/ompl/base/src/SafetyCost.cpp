//STa

#include "ompl/base/SafetyCost.h"

ompl::base::SafetyCost::SafetyCost() :  is_root_(false), collision_world_(false), object_danger_factor_(1.0)
{
}

std::ostream& ompl::base::operator<<(std::ostream& stream, SafetyCost c)
{
	if (c.getCollisionWorld())
	{
		stream << "COLLISION_WORLD";
		return stream;
	}

	for (size_t i=0;i < c.getIndividualCostSize(); ++i)
	{

		stream << c.getIndividualCost(i) << "  ";
	}
	//STa temp
//	stream << "(object_danger_factor_ = " << c.getObjectDangerFactor() << ")";


	return stream;
}

ompl::base::Cost ompl::base::SafetyCost::getIndividualCost(size_t index) const
{
	return individual_cost_[index];
}

ompl::base::Cost ompl::base::SafetyCost::getIndividualCostImproved(size_t index) const
{
    return individual_cost_improved_[index];
}
bool ompl::base::SafetyCost::isImprovingCost(size_t index) const
{
    return is_improving_cost_[index];
}

bool ompl::base::SafetyCost::hasImprovedCosts() const
{
    return (individual_cost_improved_.size() > 0);
}


size_t ompl::base::SafetyCost::getIndividualCostSize() const
{
	return individual_cost_.size();
}

void ompl::base::SafetyCost::addCost(Cost cost)
{
	individual_cost_.push_back(cost);
}

void ompl::base::SafetyCost::addCostImproved(Cost cost)
{
    individual_cost_improved_.push_back(cost);
}

void ompl::base::SafetyCost::addIsImprovingCost(bool value)
{
    is_improving_cost_.push_back(value);
}

void ompl::base::SafetyCost::setCollisionWorld(bool value)
{
	collision_world_ = value;
}

bool ompl::base::SafetyCost::getCollisionWorld()
{
	return collision_world_;
}

void ompl::base::SafetyCost::setObjectDangerFactor(double value)
{
	object_danger_factor_ = value;
}

double ompl::base::SafetyCost::getObjectDangerFactor()
{
	return object_danger_factor_;
}

void ompl::base::SafetyCost::setObjectDangerFactorImproved(double value)
{
    object_danger_factor_improved_ = value;
}

double ompl::base::SafetyCost::getObjectDangerFactorImproved()
{
    return object_danger_factor_improved_;
}



//bool& ompl::base::SafetyCost::isImprovingSafety()
//{
//	return is_improving_safety;
//}
//bool& ompl::base::SafetyCost::isImprovingJoint()
//{
//	return is_improving_joint;
//}
//
//bool& ompl::base::SafetyCost::isImprovingManipulability()
//{
//	return is_improving_manipulability;
//}
//
//bool& ompl::base::SafetyCost::isImprovingAwareness()
//{
//	return is_improving_awareness;
//}

bool& ompl::base::SafetyCost::isRoot()
{
    return is_root_;
}

