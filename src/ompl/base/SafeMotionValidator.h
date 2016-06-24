//STa

#ifndef OMPL_BASE_SAFE_MOTION_VALIDATOR_
#define OMPL_BASE_SAFE_MOTION_VALIDATOR_

#include "ompl/base/MotionValidator.h"
//#include "ompl/base/SpaceInformation.h"
#include "ompl/base/SafeStateValidityChecker.h"

namespace ompl
{

    namespace base
    {

    //STa
    class SpaceInformation;
    class StateSpace;

        /** \brief A motion validator that only uses the state validity checker. Motions are checked for validity at a specified resolution. */
        class SafeMotionValidator : public MotionValidator
        {
        public:

            /** \brief Constructor */
            SafeMotionValidator(SpaceInformation *si) : MotionValidator(si)
            {
                defaultSettings();
            }

            /** \brief Constructor */
            SafeMotionValidator(const SpaceInformationPtr &si) : MotionValidator(si)
            {
                defaultSettings();
            }

            virtual ~SafeMotionValidator()
            {
            }

            virtual bool checkMotion(const State *s1, const State *s2) const;
            virtual bool checkMotion(const State *s1, const State *s2, std::pair<State*, double> &lastValid) const;
            bool checkMotion(const State *s1, const State *s2, double valid_segment_factor) const;

            //STa test
            bool checkMotionTEST(const State *s1, const State *s2);
            bool checkMotionSelfTEST(const State *s1, const State *s2);


            bool checkMotionIndividualLinksWithDist(const State *s1, const State *s2, double travel_dist_limit, double& min_obstacle_dist, bool fast_dist);
            bool checkMotionIndividualLinks(const State *s1, const State *s2, double travel_dist_limit, bool fast_dist);
            bool checkMotionSelfCCIndividualLinks(const State *s1, const State *s2, double travel_dist_limit) const;
            bool checkMotionSelfCCDiscrete(const State *s1, const State *s2, double valid_segment_factor) const;
            bool checkMotionSelfCCDiscreteWS(const State *s1, const State *s2, double travel_dist_limit) const;

            bool checkMotionWorldIndividualLinks(const State *s1, const State *s2, double travel_dist_limit, bool fast_dist) const;


            double minObstacleDistMotionIndividualObjects(const State *s1, const State *s2, double travel_dist_limit, bool fast_dist, double& object_danger_factor) const;
            double minObstacleDistMotionIndividualObjectsTEST(const State *s1, const State *s2, double travel_dist_limit, bool fast_dist) const;

            double minObstacleDistMotionDiscrete(const State *s1, const State *s2, double valid_segment_factor, bool fast_dist) const;
            double minObstacleDistMotionDiscreteExact(const State *s1, const State *s2, double valid_segment_factor) const;


        private:

            StateSpace *stateSpace_;
            ompl::base::SafeStateValidityChecker* ssvc_;

            void defaultSettings();

            struct SubSegment
            {
                double t_s1_;
                double t_s2_;
                double non_covered_length_;
                double dist_s1_obs_;
                double dist_s2_obs_;
                int link_index_;
                int object_index_;
                bool self_cc_;
                double obj_danger_factor_;

                bool operator<(const SubSegment& ss) const
                {
                	if (non_covered_length_ < 0 && ss.non_covered_length_ < 0)
                		return non_covered_length_ * obj_danger_factor_ < ss.non_covered_length_ * ss.obj_danger_factor_;
                	else
                		return (non_covered_length_ < ss.non_covered_length_);
                }

                bool operator>(const SubSegment& ss) const
                {
                    return (ss.non_covered_length_ < non_covered_length_);
                }
            };


        };

    }
}

#endif
