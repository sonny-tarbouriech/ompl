#ifndef OMPL_GEOMETRIC_PLANNERS_CFOREST_SAFECFORESTSTATESPACEWRAPPER_
#define OMPL_GEOMETRIC_PLANNERS_CFOREST_SAFECFORESTSTATESPACEWRAPPER_

#include "ompl/geometric/planners/cforest/CForestStateSampler.h"
#include "ompl/base/StateSpace.h"
#include "ompl/base/Planner.h"

namespace ompl
{
    namespace geometric
    {
        class SafeCForest;
    }

    namespace base
    {
        /** \brief State space wrapper to use together with CForest. It adds some functionalities
           to the regular state spaces necessary to CForest. */
        class SafeCForestStateSpaceWrapper : public StateSpace
        {
        public:
        	SafeCForestStateSpaceWrapper(geometric::SafeCForest *safecforest, base::StateSpace *space)
                : safecforest_(safecforest), space_(space), planner_(NULL)
            {
                setName(space->getName() + "SafeCForestWrapper");
            }

            ~SafeCForestStateSpaceWrapper()
            {
            }

            void setPlanner(base::Planner *planner)
            {
                planner_ = planner;
            }

            const base::Planner* getPlanner() const
            {
                return planner_;
            }

            geometric::SafeCForest* getSafeCForestInstance() const
            {
                return safecforest_;
            }

            virtual StateSamplerPtr allocDefaultStateSampler() const;

            virtual StateSamplerPtr allocStateSampler() const;

            virtual void setup();

            virtual bool isCompound() const
            {
                return space_->isCompound();
            }
            virtual bool isDiscrete() const
            {
                return space_->isDiscrete();
            }
            virtual bool isHybrid() const
            {
                return space_->isHybrid();
            }
            virtual bool isMetricSpace() const
            {
                return space_->isMetricSpace();
            }
            virtual bool hasSymmetricDistance() const
            {
                return space_->hasSymmetricDistance();
            }
            virtual bool hasSymmetricInterpolate() const
            {
                return space_->hasSymmetricInterpolate();
            }
            virtual double getLongestValidSegmentFraction() const
            {
                return space_->getLongestValidSegmentFraction();
            }
            virtual void setLongestValidSegmentFraction(double segmentFraction)
            {
                space_->setLongestValidSegmentFraction(segmentFraction);
            }
            virtual unsigned int validSegmentCount(const State *state1, const State *state2) const
            {
                return space_->validSegmentCount(state1, state2);
            }
            virtual unsigned int getDimension() const
            {
                return space_->getDimension();
            }
            virtual double getMaximumExtent() const
            {
                return space_->getMaximumExtent();
            }
            virtual double getMeasure() const
            {
                return space_->getMeasure();
            }
            virtual void enforceBounds(State *state) const
            {
                space_->enforceBounds(state);
            }
            virtual bool satisfiesBounds(const State *state) const
            {
                return space_->satisfiesBounds(state);
            }
            virtual void copyState(State *destination, const State *source) const
            {
                space_->copyState(destination, source);
            }
            virtual double distance(const State *state1, const State *state2) const
            {
                return space_->distance(state1, state2);
            }
            virtual unsigned int getSerializationLength() const
            {
                return space_->getSerializationLength();
            }
            virtual void serialize(void *serialization, const State *state) const
            {
                space_->serialize(serialization, state);
            }
            virtual void deserialize(State *state, const void *serialization) const
            {
                space_->deserialize(state, serialization);
            }
            virtual bool equalStates(const State *state1, const State *state2) const
            {
                return space_->equalStates(state1, state2);
            }
            virtual void interpolate(const State *from, const State *to, const double t, State *state) const
            {
                space_->interpolate(from, to, t, state);
            }
            virtual State* allocState() const
            {
                return space_->allocState();
            }
            virtual void freeState(State *state) const
            {
                space_->freeState(state);
            }
            virtual double* getValueAddressAtIndex(State *state, const unsigned int index) const
            {
                return space_->getValueAddressAtIndex(state, index);
            }
            virtual void registerProjections()
            {
                space_->registerProjections();
            }
            virtual void printState(const State *state, std::ostream &out) const
            {
                space_->printState(state, out);
            }
            virtual void printSettings(std::ostream &out) const
            {
                space_->printSettings(out);
            }
            virtual void printProjections(std::ostream &out) const
            {
                space_->printProjections(out);
            }
            virtual void sanityChecks(double zero, double eps, unsigned int flags) const
            {
                space_->sanityChecks(zero, eps, flags);
            }
            virtual void sanityChecks() const
            {
                space_->sanityChecks();
            }
            virtual StateSamplerPtr allocSubspaceStateSampler(const StateSpace *subspace) const
            {
                return space_->allocSubspaceStateSampler(subspace);
            }
            virtual void computeLocations()
            {
                space_->computeLocations();
            }

        protected:
            geometric::SafeCForest    *safecforest_;
            StateSpace            *space_;
            Planner               *planner_;
        };
    }
}

#endif
