// STa

#ifndef OMPL_BASE_SAFE_STATE_VALIDITY_CHECKER_
#define OMPL_BASE_SAFE_STATE_VALIDITY_CHECKER_

#include "ompl/base/StateValidityChecker.h"
#include <vector>

//STa
#include <iostream>

namespace ompl
{

    namespace base
    {
        class SafeStateValidityChecker : public StateValidityChecker
        {
        public:

            /** \brief Constructor */
        	SafeStateValidityChecker(SpaceInformation *si) : StateValidityChecker(si)
        	{
        	}

        	/** \brief Constructor */
        	SafeStateValidityChecker(const SpaceInformationPtr &si) : StateValidityChecker(si)
        	{
        	}

            virtual ~SafeStateValidityChecker()
            {
            }

            virtual bool isValidSelf(const ompl::base::State *state) const = 0;
            virtual void getCollidingLinksFCL(std::vector<std::string> &links, std::vector<double>& colliding_joint_values, const State *state) const = 0;
            virtual void computeInitialDistDataObstacle(const State *s1, const State *s2,  std::vector<std::vector<double> >& dist_s1_obs, std::vector<std::vector<double> >& dist_s2_obs, bool fast_dist) = 0;
            virtual void computeInitialDistDataSelf(const State *s1, const State *s2,  std::vector<double>& dist_s1_self, std::vector<double>& dist_s2_self) = 0;
            virtual void computeInitialDistDataTravel(const State *s1, const State *s2,  std::vector<double>& dist_travel) = 0;
            virtual void computeInitialDistDataTravelModulation(const State *s1, const State *s2,  std::vector<double>& dist_travel) = 0;
            virtual double computeDistTravelModulation(const State *s1, const State *s2, size_t link_index) = 0;
            virtual double computeLinkMinObstacleDist(const State *state, int link_index, int object_index,  bool fast_dist) = 0;
            virtual double computeLinkMinSelfDist(const State *state, int link_index) = 0;
            virtual double computeRobotMinObstacleDistIndividualLinks(const State *state,  bool fast_dist, double& object_danger_factor) const = 0;
            virtual double computeRobotExactMinObstacleDist(const State *state) const = 0;


            virtual double manipulability(const State* /*state*/) const = 0;

            virtual double humanAwareness(const State* /*state*/) const = 0;

            virtual std::vector<double> getJointValues(const ompl::base::State *state) const  = 0;
            virtual void getJointLimits(std::vector<double>& q_min, std::vector<double>& q_max) const  = 0;

            virtual void printStatePositions(const ompl::base::State *state, std::ostream &out = std::cout) const = 0;


            virtual bool humanPresence() const = 0;
            virtual size_t getNbSafetyLinks() const = 0;
            virtual size_t getNbObjects() const = 0;
            virtual double getObjectDangerFactor(size_t index) const = 0;

        };
    }
}

#endif
