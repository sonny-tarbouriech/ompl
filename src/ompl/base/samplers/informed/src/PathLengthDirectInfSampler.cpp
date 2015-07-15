/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, University of Toronto
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the University of Toronto nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Authors: Jonathan Gammell */

#include "ompl/base/samplers/informed/PathLengthDirectInfSampler.h"
#include "ompl/util/Exception.h"
#include "ompl/base/OptimizationObjective.h"
//For ompl::base::GoalSampleableRegion, which both GoalState and GoalStates derive from:
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"

// For boost::make_shared
#include <boost/make_shared.hpp>
// For std::vector
#include <vector>

namespace ompl
{
    namespace base
    {
        /////////////////////////////////////////////////////////////////////////////////////////////
        //Public functions:

        // The direct ellipsoid sampling class for path-length:
        PathLengthDirectInfSampler::PathLengthDirectInfSampler(const ProblemDefinitionPtr probDefn, unsigned int maxNumberCalls)
          : InformedSampler(probDefn, maxNumberCalls),
            informedIdx_(0u),
            uninformedIdx_(0u)
        {
            // Variables
            // The number of start states
            unsigned int numStarts;
            // The number of goal states
            unsigned numGoals;
            // The foci of the PHSs as a std::vector of states. Goals must be nonconst, as we need to allocate them (unfortunately):
            std::vector<const State*> startStates;
            std::vector<State*> goalStates;

            if (probDefn_->getGoal()->hasType(ompl::base::GOAL_SAMPLEABLE_REGION) == false)
            {
                throw Exception("PathLengthDirectInfSampler: The direct path-length informed sampler currently only supports goals that can be cast to a sampleable goal region (i.e., are countable sets).");
            }

            /// \todo We don't check for the cost-to-go heuristic in the optimization objective, as this direct sampling is only for Euclidean distance.

            // Store the number of starts and goals
            numStarts = probDefn_->getStartStateCount();
            numGoals = probDefn_->getGoal()->as<ompl::base::GoalSampleableRegion>()->maxSampleCount();

            // Sanity check that there is atleast one of each
            if (numStarts < 1u || numGoals < 1u)
            {
                throw Exception("PathLengthDirectInfSampler: There must be at least 1 start and and 1 goal state when the informed sampler is created.");
            }

            // Check that the provided statespace is compatible and extract the necessary indices.
            // The statespace must either be R^n or SE(2) or SE(3)
            if (InformedSampler::space_->isCompound() == false)
            {
                if (InformedSampler::space_->getType() == STATE_SPACE_REAL_VECTOR)
                {
                    informedIdx_ = 0u;
                    uninformedIdx_ = 0u;
                }
                else
                {
                    throw Exception("PathLengthDirectInfSampler only supports RealVector, SE2 and SE3 StateSpaces.");
                }
            }
            else if (InformedSampler::space_->isCompound() == true)
            {
                // Check that it is SE2 or SE3
                if (InformedSampler::space_->getType() == STATE_SPACE_SE2 || InformedSampler::space_->getType() == STATE_SPACE_SE3)
                {
                    // Variable:
                    // An ease of use upcasted pointer to the space as a compound space
                    const CompoundStateSpace* compoundSpace = InformedSampler::space_->as<CompoundStateSpace>();

                    // Sanity check
                    if (compoundSpace->getSubspaceCount() != 2u)
                    {
                        // Pout
                        throw Exception("The provided compound StateSpace is SE(2) or SE(3) but does not have exactly 2 subspaces.");
                    }

                    // Iterate over the state spaces, finding the real vector and SO components.
                    for (unsigned int idx = 0u; idx < InformedSampler::space_->as<CompoundStateSpace>()->getSubspaceCount(); ++idx)
                    {
                        // Check if the space is real-vectored, SO2 or SO3
                        if (compoundSpace->getSubspace(idx)->getType() == STATE_SPACE_REAL_VECTOR)
                        {
                            informedIdx_ = idx;
                        }
                        else if (compoundSpace->getSubspace(idx)->getType() == STATE_SPACE_SO2)
                        {
                            uninformedIdx_ = idx;
                        }
                        else if (compoundSpace->getSubspace(idx)->getType() == STATE_SPACE_SO3)
                        {
                            uninformedIdx_ = idx;
                        }
                        else
                        {
                            // Pout
                            throw Exception("The provided compound StateSpace is SE(2) or SE(3) but contains a subspace that is not R^2, R^3, SO(2), or SO(3).");
                        }
                    }
                }
                else
                {
                    throw Exception("PathLengthDirectInfSampler only supports RealVector, SE2 and SE3 statespaces.");
                }
            }

            // Create a sampler for the whole space that we can use if we have no information
            baseSampler_ = InformedSampler::space_->allocDefaultStateSampler();

            // Check if the space is compound
            if (InformedSampler::space_->isCompound() == false)
            {
                // It is not.

                // The informed subspace is the full space
                informedSubSpace_ = InformedSampler::space_;

                // And the uniformed subspace and its associated sampler are null
                uninformedSubSpace_ = StateSpacePtr();
                uninformedSubSampler_ = StateSamplerPtr();
            }
            else
            {
                // It is

                // Get a pointer to the informed subspace...
                informedSubSpace_ = InformedSampler::space_->as<CompoundStateSpace>()->getSubspace(informedIdx_);

                // And the uninformed subspace is the remainder.
                uninformedSubSpace_ = InformedSampler::space_->as<CompoundStateSpace>()->getSubspace(uninformedIdx_);

                // Create a sampler for the uniformed subset:
                uninformedSubSampler_ = uninformedSubSpace_->allocDefaultStateSampler();
            }

            // Store the foci, first the starts:
            for (unsigned int i = 0u; i < numStarts; ++i)
            {
                startStates.push_back(probDefn_->getStartState(i));
            }


            // Extract the state of each goal one and place into the goal vector!
            for (unsigned int i = 0u; i < numGoals; ++i)
            {
                // Allocate a state onto the back of the vector:
                goalStates.push_back(InformedSampler::space_->allocState());

                // Now sample a goal into that state:
                probDefn_->getGoal()->as<ompl::base::GoalSampleableRegion>()->sampleGoal(goalStates.back());
            }

            // Now, iterate create a PHS for each start-goal pair
            // Each start
            for (unsigned int i = 0u; i < numStarts; ++i)
            {
                // Variable
                // The start as a vector
                std::vector<double> startFocusVector = getInformedSubstate(startStates.at(i));

                // Each goal
                for (unsigned int j = 0u; j < numGoals; ++ j)
                {
                    // Variable
                    // The goal as a vector
                    std::vector<double> goalFocusVector = getInformedSubstate(goalStates.at(j));

                    // Create the definition of the PHS
                    listPhsPtrs_.push_back(boost::make_shared<ProlateHyperspheroid>(informedSubSpace_->getDimension(), &startFocusVector[0], &goalFocusVector[0]));
                }
            }

            // Finally deallocate the states in the goal state vector:
            for (unsigned int i = 0u; i < numGoals; ++i)
            {
                // Free the state in the vector:
                InformedSampler::space_->freeState(goalStates.at(i));
            }

            if (listPhsPtrs_.size() > 100u)
            {
                OMPL_WARN("PathLengthDirectInfSampler: Rejection sampling is used in order to maintain uniform density in the presence of overlapping informed subsets. At some number of independent subsets, this will become prohibitively expensive.");
            }
        }



        PathLengthDirectInfSampler::~PathLengthDirectInfSampler()
        {
        }



        bool PathLengthDirectInfSampler::sampleUniform(State *statePtr, const Cost &maxCost)
        {
            // Variable
            // The persistent iteration counter:
            unsigned int iter = 0u;

            //Call the sampleUniform helper function with my iteration counter:
            return sampleUniform(statePtr, maxCost, &iter);
        }



        bool PathLengthDirectInfSampler::sampleUniform(State *statePtr, const Cost &minCost, const Cost &maxCost)
        {
            // Sample from the larger PHS until the sample does not lie within the smaller PHS.
            // Since volume in a sphere/spheroid is proportionately concentrated near the surface, this isn't horribly inefficient, though a direct method would be better

            // Variable
            // Whether we were successful in creating an informed sample. Initially not:
            bool foundSample = false;

            // Spend numIters_ iterations trying to find an informed sample:
            for (unsigned int i = 0u; i < InformedSampler::numIters_ && foundSample == false; ++i)
            {
                // Call the helper function for the larger PHS. It will move our iteration counter:
                foundSample = sampleUniform(statePtr, maxCost, &i);

                // Did we find a sample?
                if (foundSample == true)
                {
                    // We did, but that was only inside the bigger PHS, we need to assure it's outside the smaller one which occurs if the minCost is *better* than that of the sample:
                    foundSample = InformedSampler::opt_->isCostBetterThan(minCost, heuristicSolnCost(statePtr));
                }
                // No else, no sample was found.
            }

            // All done, one way or the other.
            return foundSample;
        }



        bool PathLengthDirectInfSampler::hasInformedMeasure() const
        {
            return true;
        }



        double PathLengthDirectInfSampler::getInformedMeasure(const Cost &currentCost) const
        {
            // Variable
            // The measure of the informed set
            double informedMeasure = 0.0;

            // The informed measure is then the sum of the measure of the individual PHSs for the given cost:
            for (std::list<ompl::ProlateHyperspheroidPtr>::const_iterator phsIter = listPhsPtrs_.begin(); phsIter != listPhsPtrs_.end(); ++phsIter)
            {
                //It is nonsensical for a PHS to have a transverse diameter less than the distance between its foci, so skip those that do
                if (currentCost.value() > (*phsIter)->getMinTransverseDiameter())
                {
                    informedMeasure = informedMeasure + (*phsIter)->getPhsMeasure(currentCost.value());
                }
                //No else, this value is better than this ellipse. It will get removed later.
            }

            // And if the space is compound, further multiplied by the measure of the uniformed subspace
            if (InformedSampler::space_->isCompound() == true)
            {
                informedMeasure = informedMeasure * uninformedSubSpace_->getMeasure();
            }

            // Return the smaller of the two measures
            return std::min(InformedSampler::space_->getMeasure(), informedMeasure);
        }



        Cost PathLengthDirectInfSampler::heuristicSolnCost(const State *statePtr) const
        {
            // Variable
            // The raw data in the state
            std::vector<double> rawData = getInformedSubstate(statePtr);
            // The Cost, infinity to start
            Cost minCost = InformedSampler::opt_->infiniteCost();

            // Iterate over the separate subsets and return the minimum
            for (std::list<ompl::ProlateHyperspheroidPtr>::const_iterator phsIter = listPhsPtrs_.begin(); phsIter != listPhsPtrs_.end(); ++phsIter)
            {
                /** \todo Use a heuristic function for the full solution cost defined in OptimizationObjective or some new Heuristic class once said function is defined. */
                minCost = InformedSampler::opt_->betterCost(minCost, Cost((*phsIter)->getPathLength(&rawData[0])));
            }

            return minCost;
        }
        /////////////////////////////////////////////////////////////////////////////////////////////



        /////////////////////////////////////////////////////////////////////////////////////////////
        //Private functions:
        bool PathLengthDirectInfSampler::sampleUniform(State *statePtr, const Cost &maxCost, unsigned int *iters)
        {
            // Variable
            // Whether we were successful in creating an informed sample. Initially not:
            bool foundSample = false;

            //Whether we successfully returnes
            // Check if a solution path has been found
            if (InformedSampler::opt_->isFinite(maxCost) == false)
            {
                // We don't have a solution yet, we sample from our basic sampler instead...
                baseSampler_->sampleUniform(statePtr);

                //Up our counter by one:
                ++(*iters);

                // Mark that we sampled:
                foundSample = true;
            }
            else // We have a solution
            {
                // Update the definitions of the PHSs
                updatePhsDefinitions(maxCost);

                // Sample from the PHSs.

                // When the summed measure is suitably large, it makes more sense to just sample from the entire planning space and keep the sample if it lies in any PHS
                // Only check this if we have more than 1 goal
                if (listPhsPtrs_.size() > 1u)
                {
                    // Check if the average measure is greater than half the domain's measure. Half is an arbitrary number.
                    if (summedMeasure_/static_cast<double>(listPhsPtrs_.size()) > 0.5*informedSubSpace_->getMeasure())
                    {
                        // The measure is large, sample from the entire world and keep if it's in a PHS
                        while (foundSample == false && *iters < InformedSampler::numIters_)
                        {
                            // Generate a sample by sampling the boundary and rejecting if it is not in *any* PHS
                            foundSample = sampleBoundsRejectFunc(statePtr, boost::bind(&PathLengthDirectInfSampler::isInAnyPhs, this, _1), maxCost, iters);
                        }
                    }
                    else
                    {
                        // The measure is sufficiently small that we will perform direct PHS sampling weighted by relative measure
                        foundSample = sampleRandomPhs(statePtr, maxCost, iters);
                    }
                }
                else
                {
                    // Only one PHS, so sample it.
                    foundSample = sampleRandomPhs(statePtr, maxCost, iters);
                }
            }

            // Return:
            return foundSample;
        }



        bool PathLengthDirectInfSampler::sampleUniformIgnoreBounds(State *statePtr, const Cost &maxCost, unsigned int *iters)
        {
            // Variable
            // The informed subset of the sample as a vector
            std::vector<double> informedVector(informedSubSpace_->getDimension());
            // Whether we've found a sample
            bool foundSample = false;

            // Update the PHSs
            updatePhsDefinitions(maxCost);

            // Due to the possibility of overlap between multiple PHSs, we keep a sample with a probability of 1/K, where K is the number of PHSs the sample is in.
            while (foundSample == false && *iters < InformedSampler::numIters_)
            {
                // Variable
                // The random PHS in use for this sample.
                ProlateHyperspheroidCPtr phsCPtr = randomPhsPtr();

                // Sample the PHS directly
                // Use the PHS to get a sample in the informed subspace irrespective of boundary
                rng_.uniformProlateHyperspheroid(phsCPtr, &informedVector[0]);

                // Now, if we were successful with either method, we weigh the probability of keeping the sample by the number of overlaps
                foundSample = keepSample(informedVector);
            }

            // If we found sample, inflate into the full state space (if there was an uniformed component)
            createFullState(statePtr, informedVector);

            // Return possible successful
            return foundSample;
        }



        bool PathLengthDirectInfSampler::sampleUniformIgnoreBounds(State *statePtr, const Cost &minCost, const Cost &maxCost, unsigned int *iters)
        {
            // Sample from the larger PHS until the sample does not lie within the smaller PHS.
            // Since volume in a sphere/spheroid is proportionately concentrated near the surface, this isn't horribly inefficient, though a direct method would be better

            // Variable
            // Whether we were successful in creating an informed sample. Initially not:
            bool foundSample = false;

            // Spend numIters_ iterations trying to find an sample that is within the bounds:
            for (unsigned int i = 0u; i < InformedSampler::numIters_ && foundSample == false; ++i)
            {
                // Get a sample inside the large PHS:
                sampleUniformIgnoreBounds(statePtr, maxCost, iters);

                // Check if it is also outside the smaller PHS, which occurs if the minCost is *better* than that of the sample:
                foundSample = InformedSampler::opt_->isCostBetterThan(minCost, heuristicSolnCost(statePtr));
            }

            return foundSample;
        }



        bool PathLengthDirectInfSampler::sampleRandomPhs(State *statePtr, const Cost &maxCost, unsigned int *iters)
        {
            // Variable
            // Whether we were successful in creating an informed sample. Initially not:
            bool foundSample = false;

            // Due to the possibility of overlap between multiple PHSs, we keep a sample with a probability of 1/K, where K is the number of PHSs the sample is in.
            while (foundSample == false && *iters < InformedSampler::numIters_)
            {
                // Variable
                // The random PHS in use for this sample.
                ProlateHyperspheroidCPtr phsCPtr = randomPhsPtr();

                // Check if this PHS is too large to sample directly
                if (phsCPtr->getPhsMeasure() > informedSubSpace_->getMeasure())
                {
                    // Just sample the bounds and reject if not in this PHS
                    foundSample = sampleBoundsRejectFunc(statePtr, boost::bind(&PathLengthDirectInfSampler::isInPhs, this, phsCPtr, _1), maxCost, iters);
                }
                else
                {
                    // Sample the PHS directly and reject if not in the bounds
                    foundSample = samplePhsRejectBounds(statePtr, phsCPtr, maxCost, iters);
                }

                // Now, if we were successful with either method, we weigh the probability of keeping the sample by the number of overlaps
                if (foundSample == true)
                {
                    // Keep with probability 1/K
                    foundSample = keepSample(getInformedSubstate(statePtr));
                }
                // No else, continue loop
            }

            return foundSample;
        }



        bool PathLengthDirectInfSampler::sampleBoundsRejectFunc(State* statePtr, KeepFunc keepFunc, const Cost &maxCost, unsigned int *iters)
        {
            // Variable
            // Whether we've found a sample:
            bool foundSample  = false;

            // Spend numIters_ iterations trying to find an informed sample:
            while (*iters < InformedSampler::numIters_ && foundSample == false)
            {
                // Generate a random sample
                baseSampler_->sampleUniform(statePtr);

                // The informed substate
                std::vector<double> informedVector = getInformedSubstate(statePtr);

                // Check if the informed state is in our PHSs.
                foundSample = keepFunc(informedVector);

                // Increment the provided counter
                ++(*iters);
            }

            // successful?
            return foundSample;
        }



        bool PathLengthDirectInfSampler::samplePhsRejectBounds(State *statePtr, ProlateHyperspheroidCPtr phsCPtr, const Cost &maxCost, unsigned int *iters)
        {
            // Variable
            // Whether we've found a sample:
            bool foundSample  = false;
            // The informed subset of the sample as a vector
            std::vector<double> informedVector(informedSubSpace_->getDimension());

            // Spend numIters_ iterations trying to find an sample that is within the bounds:
            while (*iters < InformedSampler::numIters_ && foundSample == false)
            {
                // Use the PHS to get a sample in the informed subspace irrespective of boundary
                rng_.uniformProlateHyperspheroid(phsCPtr, &informedVector[0]);

                // Turn into a state of our full space
                createFullState(statePtr, informedVector);

                // Check if the resulting state is in the problem:
                foundSample = InformedSampler::space_->satisfiesBounds(statePtr);

                // Increment our counter
                ++(*iters);
            }

            // successful?
            return foundSample;
        }



        std::vector<double> PathLengthDirectInfSampler::getInformedSubstate(const State *statePtr) const
        {
            // Variable
            // The raw data in the state
            std::vector<double> rawData(informedSubSpace_->getDimension());

            // Get the raw data
            if (InformedSampler::space_->isCompound() == false)
            {
                informedSubSpace_->copyToReals(rawData, statePtr);
            }
            else
            {
                informedSubSpace_->copyToReals(rawData, statePtr->as<CompoundState>()->components[informedIdx_]);
            }

            return rawData;
        }



        void PathLengthDirectInfSampler::createFullState(State * statePtr, const std::vector<double> &informedVector)
        {

            // If there is an extra "uninformed" subspace, we need to add that to the state before converting the raw vector representation into a state....
            if (InformedSampler::space_->isCompound() == false)
            {
                // No, space_ == informedSubSpace_
                // Copy into the state pointer
                informedSubSpace_->copyFromReals(statePtr, informedVector);
            }
            else
            {
                // Yes, we need to also sample the uninformed subspace
                // Variables
                // A state for the uninformed subspace
                State *uninformedState = uninformedSubSpace_->allocState();

                // Copy the informed subspace into the state pointer
                informedSubSpace_->copyFromReals(statePtr->as<CompoundState>()->components[informedIdx_], informedVector);

                // Sample the uniformed subspace
                uninformedSubSampler_->sampleUniform(uninformedState);

                // Copy the informed subspace into the state pointer
                uninformedSubSpace_->copyState(statePtr->as<CompoundState>()->components[uninformedIdx_], uninformedState);

                // Free the state
                uninformedSubSpace_->freeState(uninformedState);
            }
        }



        void PathLengthDirectInfSampler::updatePhsDefinitions(const Cost &maxCost)
        {
            // Variable
            // The iterator for the list:
            std::list<ompl::ProlateHyperspheroidPtr>::iterator phsIter = listPhsPtrs_.begin();

            // Iterate over the list of PHSs, updating the summed measure
            // Reset the sum
            summedMeasure_ = 0.0;
            while (phsIter != listPhsPtrs_.end())
            {
                // Check if the specific PHS can ever be better than the given maxCost, i.e., if the distance between the foci is less than the current max cost
                if ((*phsIter)->getMinTransverseDiameter() < maxCost.value())
                {
                    // It can, update it

                    // Update the transverse diameter
                    (*phsIter)->setTransverseDiameter(maxCost.value());

                    // Increment the summed measure of the ellipses.
                    summedMeasure_ = summedMeasure_ + (*phsIter)->getPhsMeasure();

                    // Increment the iterator
                    ++phsIter;
                }
                else if (listPhsPtrs_.size() > 1u)
                {
                    // It can't, if it is not the last PHS, remove it

                    // Remove the iterator to delete from the list, this returns the next:
                    /// \todo Make sure this doesn't cause problems for JIT sampling?
                    phsIter = listPhsPtrs_.erase(phsIter);
                }
                //No else, don't remove the last one.
            }
        }



        ompl::ProlateHyperspheroidPtr PathLengthDirectInfSampler::randomPhsPtr()
        {
            // Variable
            // The return value
            ompl::ProlateHyperspheroidPtr rval;

            // If we only have one PHS, this can be simplified:
            if (listPhsPtrs_.size() == 1u)
            {
                // One PHS, keep this simple.

                // Return it
                rval = listPhsPtrs_.front();
            }
            else
            {
                // We have more than one PHS to consider

                // Variables
                // A randomly generated number in the interval [0,1]
                double randDbl = rng_.uniform01();
                // The running measure
                double runningRelativeMeasure = 0.0;

                // The probability of using each PHS is weighted by it's measure. Therefore, if we iterate up the list of PHSs, the first one who's relative measure is greater than the PHS randomly selected
                for (std::list<ompl::ProlateHyperspheroidPtr>::const_iterator phsIter = listPhsPtrs_.begin(); phsIter != listPhsPtrs_.end() && static_cast<bool>(rval) == false; ++phsIter)
                {
                    // Update the running measure
                    runningRelativeMeasure = runningRelativeMeasure + (*phsIter)->getPhsMeasure()/summedMeasure_;

                    // Check if it's now greater than the proportion of the summed measure
                    if (runningRelativeMeasure > randDbl)
                    {
                        // It is, return this PHS:
                        rval = *phsIter;
                    }
                    // No else, continue
                }
            }

            // Return
            return rval;
        }



        bool PathLengthDirectInfSampler::keepSample(const std::vector<double>& informedVector)
        {
            // Variable
            // The return value, do we keep this sample? Start true.
            bool keep = true;

            // Is there more than 1 goal?
            if (listPhsPtrs_.size() > 1u)
            {
                // There is, do work

                // Variable
                // The number of PHSs the sample is in
                unsigned int numIn = numberOfPhsInclusions(informedVector);
                // The random number between [0,1]
                double randDbl = rng_.uniform01();

                // Keep the sample if the random number is less than 1/K
                keep = (randDbl <= 1.0/static_cast<double>(numIn));
            }
            // No else, keep is true by default.

            return keep;
        }



        bool PathLengthDirectInfSampler::isInAnyPhs(const std::vector<double>& informedVector) const
        {
            // Variable
            // The return value, whether the given state is in any PHS
            bool inPhs = false;

            // Iterate over the list, stopping as soon as we get our first true
            for (std::list<ompl::ProlateHyperspheroidPtr>::const_iterator phsIter = listPhsPtrs_.begin(); phsIter != listPhsPtrs_.end() && inPhs == false; ++ phsIter)
            {
                inPhs = isInPhs(*phsIter, informedVector);
            }

            return inPhs;
        }



        bool PathLengthDirectInfSampler::isInPhs(const ProlateHyperspheroidCPtr &phsCPtr, const std::vector<double> &informedVector) const
        {
            return phsCPtr->isInPhs(&informedVector[0]);
        }



        unsigned int PathLengthDirectInfSampler::numberOfPhsInclusions(const std::vector<double>& informedVector) const
        {
            // Variable
            // The return value, the number of PHSs the vector is in
            unsigned int numInclusions = 0u;

            // Iterate over the list counting
            for (std::list<ompl::ProlateHyperspheroidPtr>::const_iterator phsIter = listPhsPtrs_.begin(); phsIter != listPhsPtrs_.end(); ++ phsIter)
            {
                // Conditionally increment
                if ((*phsIter)->isInPhs(&informedVector[0]) == true)
                {
                    ++numInclusions;
                }
                // No else
            }

            return numInclusions;
        }
        /////////////////////////////////////////////////////////////////////////////////////////////
    }; // base
};  // ompl
