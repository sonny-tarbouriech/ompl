#include "ompl/geometric/planners/cforest/SafeCForestStateSpaceWrapper.h"
#include "ompl/geometric/planners/cforest/SafeCForest.h"

ompl::base::StateSamplerPtr ompl::base::SafeCForestStateSpaceWrapper::allocDefaultStateSampler() const
{
    StateSamplerPtr sampler = StateSamplerPtr(new CForestStateSampler(this, space_->allocDefaultStateSampler()));
    safecforest_->addSampler(sampler);
    return sampler;
}

ompl::base::StateSamplerPtr ompl::base::SafeCForestStateSpaceWrapper::allocStateSampler() const
{
    StateSamplerPtr sampler = StateSamplerPtr(new CForestStateSampler(this, space_->allocStateSampler()));
    safecforest_->addSampler(sampler);
    return sampler;
}

void ompl::base::SafeCForestStateSpaceWrapper::setup()
{
    space_->setup();
    maxExtent_ = space_->getMaximumExtent();
    longestValidSegmentFraction_ = space_->getLongestValidSegmentFraction();
    longestValidSegmentCountFactor_ = space_->getValidSegmentCountFactor();
    longestValidSegment_ = space_->getLongestValidSegmentLength();
    projections_ = space_->getRegisteredProjections();
    params_ = space_->params();

    valueLocationsInOrder_ = space_->getValueLocations();
    valueLocationsByName_ = space_->getValueLocationsByName();
    substateLocationsByName_ = space_->getSubstateLocationsByName();
}
