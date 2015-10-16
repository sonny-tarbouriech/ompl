//STa
#ifndef OMPL_BASE_SAFETY_COST_
#define OMPL_BASE_SAFETY_COST_

#include "ompl/base/Cost.h"
#include <vector>


namespace ompl
{
namespace base
{
/** \brief Definition of a cost value. Can represent the cost of a motion or the cost of a state. */
class SafetyCost
{
public:
	/** \brief Construct cost with a specified value */
	SafetyCost();

	Cost getIndividualCost(size_t index) const;
	size_t getIndividualCostSize() const;
	void addCost(Cost cost);


	void setCollisionWorld(bool value);
	bool getCollisionWorld();

	void setObjectDangerFactor(double value);
	double getObjectDangerFactor();

	bool& isImprovingSafety();
	bool& isImprovingJoint();
	bool& isImprovingManipulability();
	bool& isImprovingAwareness();
	bool& isRoot();


	SafetyCost& operator=(SafetyCost other)
	{
		this->individual_cost_.resize(other.getIndividualCostSize());
		for (size_t i=0;i < other.getIndividualCostSize(); ++i)
		{
			this->individual_cost_[i] = other.getIndividualCost(i);
		}
		this->is_improving_safety = other.is_improving_safety;
		this->is_improving_joint = other.is_improving_joint;
		this->is_improving_manipulability = other.is_improving_manipulability;
		this->is_improving_awareness = other.is_improving_awareness;
		this->collision_world_ = other.collision_world_;
		this->object_danger_factor_ = other.object_danger_factor_;

		return *this;
	}


private:

	std::vector<Cost> individual_cost_;

	bool is_improving_safety;
	bool is_improving_joint;
	bool is_improving_manipulability;
	bool is_improving_awareness;
	bool is_root_;
	bool collision_world_;


	double object_danger_factor_;
};

std::ostream& operator<<(std::ostream& stream, SafetyCost c);






}
}
#endif
