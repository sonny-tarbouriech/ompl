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
	Cost getIndividualCostImproved(size_t index) const;
	bool isImprovingCost(size_t index) const;

	bool hasImprovedCosts() const;

	size_t getIndividualCostSize() const;

	void addCost(Cost cost);
	void addCostImproved(Cost cost);
	void addIsImprovingCost(bool value);


	void setCollisionWorld(bool value);
	bool getCollisionWorld();

	void setObjectDangerFactor(double value);
	double getObjectDangerFactor();

    void setObjectDangerFactorImproved(double value);
    double getObjectDangerFactorImproved();

	bool& isRoot();

private:

	std::vector<Cost> individual_cost_;
	std::vector<bool> is_improving_cost_;
	std::vector<Cost> individual_cost_improved_;

	bool is_root_;
	bool collision_world_;


	double object_danger_factor_;
	double object_danger_factor_improved_;
};

std::ostream& operator<<(std::ostream& stream, SafetyCost c);


}
}
#endif
