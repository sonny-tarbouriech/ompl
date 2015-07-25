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

//My definition:
#include "ompl/geometric/planners/bitstar/BITstar.h"

//For, you know, math
#include <cmath>
//For stringstreams
#include <sstream>
//For stream manipulations
#include <iomanip>
//For boost make_shared
#include <boost/make_shared.hpp>
//For boost::bind
#include <boost/bind.hpp>
//For boost math constants
#include <boost/math/constants/constants.hpp>

//For OMPL_INFORM et al.
#include "ompl/util/Console.h"
//For exceptions:
#include "ompl/util/Exception.h"
//For geometric equations like unitNBallMeasure
#include "ompl/util/GeometricEquations.h"
//For ompl::base::GoalSampleableRegion, which both GoalState and GoalStates derive from:
#include "ompl/base/goals/GoalSampleableRegion.h"
//For getDefaultNearestNeighbors
#include "ompl/tools/config/SelfConfig.h"
//For ompl::geometric::path
#include "ompl/geometric/PathGeometric.h"
//For the default optimization objective:
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"



namespace ompl
{
    namespace geometric
    {
        /////////////////////////////////////////////////////////////////////////////////////////////
        //Public functions:
        BITstar::BITstar(const ompl::base::SpaceInformationPtr& si, const std::string& name /*= "BITstar"*/)
            : ompl::base::Planner(si, name),
            sampler_(),
            opt_(),
            startVertices_(),
            goalVertices_(),
            curGoalVertex_(),
            stateNN_(),
            intQueue_(),
            sampleDensity_(0.0),
            r_(0.0), //Purposeful Gibberish
            k_rgg_(0.0), //Purposeful Gibberish
            k_(0u), //Purposeful Gibberish
            bestCost_( std::numeric_limits<double>::infinity() ), //Gets set in setup to the proper calls from OptimizationObjective
            bestLength_(0u),
            prunedCost_( std::numeric_limits<double>::infinity() ), //Gets set in setup to the proper calls from OptimizationObjective
            prunedMeasure_(Planner::si_->getSpaceMeasure()),
            minCost_( std::numeric_limits<double>::infinity() ), //Gets set in setup to the proper calls from OptimizationObjective
            costSampled_( std::numeric_limits<double>::infinity() ), //Gets set in setup to the proper calls from OptimizationObjective
            hasSolution_(false),
            stopLoop_(false),
            approximateSoln_(false),
            approximateDiff_(-1.0),
            numIterations_(0u),
            numBatches_(0u),
            numPrunings_(0u),
            numSamples_(0u),
            numVertices_(0u),
            numCurrentFreeStates_(0u),
            numCurrentConnectedStates_(0u),
            numFreeStatesPruned_(0u),
            numVerticesDisconnected_(0u),
            numRewirings_(0u),
            numStateCollisionChecks_(0u),
            numEdgeCollisionChecks_(0u),
            numNearestNeighbours_(0u),
            useStrictQueueOrdering_(true),
            rewireFactor_(1.1),
            samplesPerBatch_(100u),
            useFailureTracking_(false),
            useKNearest_(false),
            usePruning_(true),
            pruneFraction_(0.02),
            delayRewiring_(false),
            stopOnSolnChange_(false)
        {
            //Specify my planner specs:
            Planner::specs_.recognizedGoal = ompl::base::GOAL_SAMPLEABLE_REGION;
            Planner::specs_.multithreaded = false;
            Planner::specs_.approximateSolutions = false; //For now!
            Planner::specs_.optimizingPaths = true;
            Planner::specs_.directed = true;
            Planner::specs_.provingSolutionNonExistence = false;

            OMPL_INFORM("%s: TODO: Implement approximate solution support.", Planner::getName().c_str());

            //Register my setting callbacks
            Planner::declareParam<double>("rewire_factor", this, &BITstar::setRewireFactor, &BITstar::getRewireFactor, "1.0:0.01:3.0");
            Planner::declareParam<unsigned int>("samples_per_batch", this, &BITstar::setSamplesPerBatch, &BITstar::getSamplesPerBatch, "1:1:1000000");

            //More advanced setting callbacks that aren't necessary to be exposed to Python. Uncomment if desired.
            //Planner::declareParam<bool>("use_strict_queue_ordering", this, &BITstar::setStrictQueueOrdering, &BITstar::getStrictQueueOrdering, "0,1");
            //Planner::declareParam<bool>("use_edge_failure_tracking", this, &BITstar::setUseFailureTracking, &BITstar::getUseFailureTracking, "0,1");
            //Planner::declareParam<bool>("use_k_nearest", this, &BITstar::setKNearest, &BITstar::getKNearest, "0,1");
            //Planner::declareParam<bool>("use_graph_pruning", this, &BITstar::setPruning, &BITstar::getPruning, "0,1");
            //Planner::declareParam<double>("prune_threshold_as_fractional_cost_change", this, &BITstar::setPruneThresholdFraction, &BITstar::getPruneThresholdFraction, "0.0:0.01:1.0");
            //Planner::declareParam<bool>("delay_rewiring_to_first_solution", this, &BITstar::setDelayRewiringUntilInitialSolution, &BITstar::getDelayRewiringUntilInitialSolution, "0,1");
            //Planner::declareParam<bool>("stop_on_each_solution_improvement", this, &BITstar::setStopOnSolnImprovement, &BITstar::getStopOnSolnImprovement, "0,1");

            //Register my progress info:
            addPlannerProgressProperty("best cost DOUBLE", boost::bind(&BITstar::bestCostProgressProperty, this));
            addPlannerProgressProperty("number of segments in solution path INTEGER", boost::bind(&BITstar::bestLengthProgressProperty, this));
            addPlannerProgressProperty("current free states INTEGER", boost::bind(&BITstar::currentFreeProgressProperty, this));
            addPlannerProgressProperty("current graph vertices INTEGER", boost::bind(&BITstar::currentVertexProgressProperty, this));
            addPlannerProgressProperty("state collision checks INTEGER", boost::bind(&BITstar::stateCollisionCheckProgressProperty, this));
            addPlannerProgressProperty("edge collision checks INTEGER", boost::bind(&BITstar::edgeCollisionCheckProgressProperty, this));
            addPlannerProgressProperty("nearest neighbour calls INTEGER", boost::bind(&BITstar::nearestNeighbourProgressProperty, this));

            //Extra progress info that aren't necessary for every day use. Uncomment if desired.
            //addPlannerProgressProperty("vertex queue size INTEGER", boost::bind(&BITstar::vertexQueueSizeProgressProperty, this));
            //addPlannerProgressProperty("edge queue size INTEGER", boost::bind(&BITstar::edgeQueueSizeProgressProperty, this));
            //addPlannerProgressProperty("iterations INTEGER", boost::bind(&BITstar::iterationProgressProperty, this));
            //addPlannerProgressProperty("batches INTEGER", boost::bind(&BITstar::batchesProgressProperty, this));
            //addPlannerProgressProperty("graph prunings INTEGER", boost::bind(&BITstar::pruningProgressProperty, this));
            //addPlannerProgressProperty("total states generated INTEGER", boost::bind(&BITstar::totalStatesCreatedProgressProperty, this));
            //addPlannerProgressProperty("vertices constructed INTEGER", boost::bind(&BITstar::verticesConstructedProgressProperty, this));
            //addPlannerProgressProperty("states pruned INTEGER", boost::bind(&BITstar::statesPrunedProgressProperty, this));
            //addPlannerProgressProperty("graph vertices disconnected INTEGER", boost::bind(&BITstar::verticesDisconnectedProgressProperty, this));
            //addPlannerProgressProperty("rewiring edges INTEGER", boost::bind(&BITstar::rewiringProgressProperty, this));
        }



        BITstar::~BITstar()
        {
        }



        void BITstar::setup()
        {
            //Call the base class setup:
            Planner::setup();

            //Do some sanity checks
            //Make sure we have a problem definition
            if(Planner::pdef_ == false)
            {
                OMPL_ERROR("%s::setup() was called without a problem definition.", Planner::getName().c_str());
                Planner::setup_ = false;
                return;
            }

            //Make sure we have an optimization objective
            if (Planner::pdef_->hasOptimizationObjective() == false)
            {
                OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length.", Planner::getName().c_str());
                Planner::pdef_->setOptimizationObjective( boost::make_shared<base::PathLengthOptimizationObjective> (Planner::si_) );
            }

            //If the problem definition *has* a goal, make sure it is of appropriate type
            if (static_cast<bool>(Planner::pdef_->getGoal()) == true)
            {
                if (Planner::pdef_->getGoal()->hasType(ompl::base::GOAL_SAMPLEABLE_REGION) == false)
                {
                    OMPL_ERROR("%s::setup() BIT* currently only supports goals that can be cast to a sampleable goal region (i.e., are countable sets).", Planner::getName().c_str());
                    Planner::setup_ = false;
                    return;
                }
                //No else, of correct type.
            }
            //No else, called without a goal. Is this MoveIt?

            //Store the optimization objective for future ease of use
            opt_ = Planner::pdef_->getOptimizationObjective();

            //Configure the nearest-neighbour construct.
            //Only allocate if it is empty (as it can be set to a specific version by a call to setNearestNeighbors)
            if (static_cast<bool>(stateNN_) == false)
            {
                stateNN_.reset( ompl::tools::SelfConfig::getDefaultNearestNeighbors<VertexPtr>(this) );
            }
            //No else, already allocated (by a call to setNearestNeighbors())

            //Configure:
            stateNN_->setDistanceFunction(boost::bind(&BITstar::nnDistance, this, _1, _2));

            //Configure the queue
            //boost::make_shared can only take 9 arguments, so be careful:
            intQueue_ = boost::make_shared<IntegratedQueue> (opt_, boost::bind(&BITstar::nearestStates, this, _1, _2), boost::bind(&BITstar::lowerBoundHeuristicVertex, this, _1), boost::bind(&BITstar::currentHeuristicVertex, this, _1), boost::bind(&BITstar::lowerBoundHeuristicEdge, this, _1), boost::bind(&BITstar::currentHeuristicEdge, this, _1), boost::bind(&BITstar::currentHeuristicEdgeTarget, this, _1));
            intQueue_->setUseFailureTracking(useFailureTracking_);
            intQueue_->setDelayedRewiring(delayRewiring_);

            //If the problem definition has at least one start and goal, allocate a sampler
            if (Planner::pdef_->getStartStateCount() > 0u && Planner::pis_.haveMoreGoalStates() == true)
            {
                //There is a start and goal, allocate
                sampler_ = opt_->allocInformedStateSampler(Planner::pdef_, std::numeric_limits<unsigned int>::max());
            }
            //No else, this will get allocated when we get the updated start/goal.

            //Set the best-cost, pruned-cost, sampled-cost and min-cost to the proper opt_-based values:
            bestCost_ = opt_->infiniteCost();
            prunedCost_ = opt_->infiniteCost();
            minCost_ = opt_->infiniteCost();
            costSampled_ = opt_->infiniteCost();

            //Add any start and goals vertices that exist to the queue:
            this->addAllStartsAndGoalStates();

            //Finally initialize the nearestNeighbour terms:
            this->initializeNearestTerms();

            //Debug: Output an estimate of the state measure:
//            this->estimateMeasures();
        }



        void BITstar::clear()
        {
            //Clear all the variables.
            //Keep this in the order of the constructors list:

            //The various convenience pointers:
            sampler_.reset();
            opt_.reset();
            startVertices_.clear();
            goalVertices_.clear();
            curGoalVertex_.reset();

            //The list of states
            if (static_cast<bool>(stateNN_) == true)
            {
                stateNN_->clear();
                stateNN_.reset();
            }
            //No else, not allocated

            //The queue:
            if (static_cast<bool>(intQueue_) == true)
            {
                intQueue_->clear();
                intQueue_.reset();
            }

            //DO NOT reset the parameters:
            //useStrictQueueOrdering_
            //rewireFactor_
            //samplesPerBatch_
            //useFailureTracking_
            //useKNearest_
            //usePruning_
            //pruneFraction_
            //delayRewiring_
            //stopOnSolnChange_

            //Reset the various calculations? TODO: Should I recalculate them?
            sampleDensity_ = 0.0;
            r_ = 0.0;
            k_rgg_ = 0.0; //This is a double for better rounding later
            k_ = 0u;
            bestCost_ = ompl::base::Cost(std::numeric_limits<double>::infinity());
            bestLength_ = 0u;
            prunedCost_ = ompl::base::Cost(std::numeric_limits<double>::infinity());
            prunedMeasure_ = Planner::si_->getSpaceMeasure();
            minCost_ = ompl::base::Cost(0.0);
            costSampled_ = ompl::base::Cost(0.0);
            hasSolution_ = false;
            stopLoop_ = false;
            approximateSoln_ = false;
            approximateDiff_ = -1.0;
            numIterations_ = 0u;
            numSamples_ = 0u;
            numVertices_ = 0u;
            numFreeStatesPruned_ = 0u;
            numVerticesDisconnected_ = 0u;
            numStateCollisionChecks_ = 0u;
            numEdgeCollisionChecks_ = 0u;
            numNearestNeighbours_ = 0u;
            numRewirings_ = 0u;
            numBatches_ = 0u;
            numPrunings_ = 0u;

            //Mark as not setup:
            setup_ = false;

            //Call my base clear:
            Planner::clear();
        }



        ompl::base::PlannerStatus BITstar::solve(const ompl::base::PlannerTerminationCondition& ptc)
        {
            Planner::checkValidity();
            OMPL_INFORM("%s: Searching for a solution to the given planning problem.", Planner::getName().c_str());

            //Reset the manual stop to the iteration loop:
            stopLoop_ = false;

            //Run the outerloop until we're stopped, a suitable cost is found, or until we find the minimum possible cost within tolerance:
            while (opt_->isSatisfied(bestCost_) == false && ptc == false && (this->isCostBetterThan(minCost_, bestCost_) == true || Planner::pis_.haveMoreStartStates() == true || Planner::pis_.haveMoreGoalStates() == true) && stopLoop_ == false)
            {
                this->iterate();
            }

            if (hasSolution_ == true)
            {
                this->endSuccessMessage();

                this->publishSolution();
            }
            else
            {
                this->endFailureMessage();
            }

            //PlannerStatus(addedSolution, approximate)
            return ompl::base::PlannerStatus(hasSolution_, approximateSoln_);
        }



        void BITstar::getPlannerData(ompl::base::PlannerData& data) const
        {
            //Get the base planner class data:
            Planner::getPlannerData(data);

            //Add states
            if (stateNN_)
            {
                //Variables:
                //The list of states:
                std::vector<VertexPtr> states;

                //Get the list of states
                stateNN_->list(states);

                //Iterate through it turning each into a vertex
                for (std::vector<VertexPtr>::const_iterator sIter = states.begin(); sIter != states.end(); ++sIter)
                {

                    //Is the vertex the start?
                    if ((*sIter)->isRoot() == true)
                    {
                        //Yes, add as a start vertex:
                        data.addStartVertex(ompl::base::PlannerDataVertex((*sIter)->stateConst()));
                    }
                    //Does it have a parent?
                    else if ((*sIter)->hasParent() == true)
                    {
                        //Yes, add as a regular vertex:
                        data.addVertex(ompl::base::PlannerDataVertex((*sIter)->stateConst()));

                        //And as an incoming edge
                        data.addEdge(ompl::base::PlannerDataVertex((*sIter)->getParentConst()->stateConst()), ompl::base::PlannerDataVertex((*sIter)->stateConst()));
                    }
                    else
                    {
                        //Add as a regular vertex without any connection:
                        data.addVertex(ompl::base::PlannerDataVertex((*sIter)->stateConst()));
                    }
                }
            }
            //No else.

            //Did we find a solution?
            if (hasSolution_ == true)
            {
                data.markGoalState(curGoalVertex_->stateConst());
            }
        }



        std::pair<ompl::base::State const*, ompl::base::State const*> BITstar::getNextEdgeInQueue()
        {
            //Variable:
            //The next edge as a basic pair of states
            std::pair<ompl::base::State const*, ompl::base::State const*> nextEdge;

            //If we're using strict queue ordering, make sure the queue is up to date
            if(useStrictQueueOrdering_ == true)
            {
                //Resort the queues as necessary (if the graph has been rewired).
                this->resort();
            }

            if (intQueue_->isEmpty() == false)
            {
                //The next edge in the queue:
                nextEdge = std::make_pair(intQueue_->frontEdge().first->state(), intQueue_->frontEdge().second->state());
            }
            else
            {
                //An empty edge:
                nextEdge = std::make_pair<ompl::base::State*, ompl::base::State*>(NULL, NULL);
            }

            return nextEdge;
        }



        ompl::base::Cost BITstar::getNextEdgeValueInQueue()
        {
            //Variable
            //The cost of the next edge
            ompl::base::Cost nextCost;

            //If we're using strict queue ordering, make sure the queue is up to date
            if(useStrictQueueOrdering_ == true)
            {
                //Resort the queues as necessary (if the graph has been rewired).
                this->resort();
            }

            if (intQueue_->isEmpty() == false)
            {
                //The next cost in the queue:
                nextCost = intQueue_->frontEdgeValue().first;
            }
            else
            {
                //An infinite cost:
                nextCost =  opt_->infiniteCost();
            }

            return nextCost;
        }



        void BITstar::getEdgeQueue(std::vector<std::pair<VertexConstPtr, VertexConstPtr> >* edgesInQueue)
        {
            intQueue_->listEdges(edgesInQueue);
        }



        void BITstar::getVertexQueue(std::vector<VertexConstPtr>* verticesInQueue)
        {
            intQueue_->listVertices(verticesInQueue);
        }



        template<template<typename T> class NN>
        void BITstar::setNearestNeighbors()
        {
            //Check if the problem is already setup, if so, the NN structs have data in them and you can't really change them:
            if (Planner::setup_ == true)
            {
                throw ompl::Exception("The type of nearest neighbour datastructure cannot be changed once a planner is setup. ");
            }
            else
            {
                //The problem isn't setup yet, create NN struct of the specified type:
                stateNN_ = boost::make_shared< NN<VertexPtr> >();
            }
        }
        /////////////////////////////////////////////////////////////////////////////////////////////



        /////////////////////////////////////////////////////////////////////////////////////////////
        //Protected functions:
        void BITstar::estimateMeasures()
        {
            OMPL_INFORM("%s: Estimating the measure of the planning domain. This is a debugging function that does not have any effect on the planner.", Planner::getName().c_str());
            //Variables:
            //The total number of samples:
            unsigned int numTotalSamples;
            //The resulting samples in free:
            unsigned int numFreeSamples;
            //The resulting samples in obs:
            unsigned int numObsSamples;
            //The sample fraction of free:
            double fractionFree;
            //The sample fraction of obs:
            double fractionObs;
            //The total measure of the space:
            double totalMeasure;
            //The resulting estimate of the free measure
            double freeMeasure;
            //The resulting estimate of the obs measure
            double obsMeasure;

            //Set the total number of samples
            numTotalSamples = 100000u;
            numFreeSamples = 0u;
            numObsSamples = 0u;

            //Draw samples, classifying each one
            for (unsigned int i = 0u; i < numTotalSamples; ++i)
            {
                //Allocate a state
                ompl::base::State* aState = Planner::si_->allocState();

                //Sample:
                sampler_->sampleUniform(aState, bestCost_);

                //Check if collision free
                if (Planner::si_->isValid(aState) == true)
                {
                    ++numFreeSamples;
                }
                else
                {
                    ++numObsSamples;
                }
            }

            //Calculate the fractions:
            fractionFree = static_cast<double>(numFreeSamples)/static_cast<double>(numTotalSamples);

            fractionObs = static_cast<double>(numObsSamples)/static_cast<double>(numTotalSamples);

            //Get the total measure of the space
            totalMeasure = Planner::si_->getSpaceMeasure();

            //Calculate the measure of the free space
            freeMeasure = fractionFree*totalMeasure;

            //Calculate the measure of the obs space
            obsMeasure = fractionObs*totalMeasure;

            //Announce
            OMPL_INFORM("%s: %u samples (%u free, %u in collision) from a space with measure %.4f estimates %.2f%% free and %.2f%% in collision (measures of %.4f and %.4f, respectively).", Planner::getName().c_str(), numTotalSamples, numFreeSamples, numObsSamples, totalMeasure, 100.0*fractionFree, 100.0*fractionObs, freeMeasure, obsMeasure);
        }


        void BITstar::iterate()
        {
            //Info:
            ++numIterations_;

            //If we're using strict queue ordering, make sure the queues are up to date
            if(useStrictQueueOrdering_ == true)
            {
                //The queues will be resorted if the graph has been rewired.
                this->resort();
            }

            //If the edge queue is empty, that must mean we're either starting from scratch, or just finished a batch. Either way, make a batch of samples and fill the queue for the first time:
            if (intQueue_->isEmpty() == true)
            {
                this->newBatch();
            }
            else
            {
                //If the edge queue is not empty, then there is work to do!

                //Variables:
                //The current edge:
                VertexPtrPair bestEdge;

                //Pop the minimum edge
                intQueue_->popFrontEdge(&bestEdge);

                //In the best case, can this edge improve our solution given the current graph?
                //g_t(v) + c_hat(v,x) + h_hat(x) < g_t(x_g)
                if (this->isCostBetterThan( this->combineCosts(bestEdge.first->getCost(), this->edgeCostHeuristic(bestEdge), this->costToGoHeuristic(bestEdge.second)), bestCost_ ) == true)
                {
                    //Variables:
                    //The true cost of the edge:
                    ompl::base::Cost trueEdgeCost;

                    //Get the true cost of the edge
                    trueEdgeCost = this->trueEdgeCost(bestEdge);

                    //Can this actual edge ever improve our solution?
                    //g_hat(v) + c(v,x) + h_hat(x) < g_t(x_g)
                    if (this->isCostBetterThan( this->combineCosts(this->costToComeHeuristic(bestEdge.first), trueEdgeCost, this->costToGoHeuristic(bestEdge.second)),  bestCost_ ) == true)
                    {
                        //Does this edge have a collision?
                        if (this->checkEdge(bestEdge) == true)
                        {
                            //Does the current edge improve our graph?
                            //g_t(v) + c(v,x) < g_t(x)
                            if (this->isCostBetterThan( opt_->combineCosts(bestEdge.first->getCost(), trueEdgeCost), bestEdge.second->getCost() ) == true)
                            {
                                //YAAAAH. Add the edge! Propagate cost updates to descendants.
                                //addEdge will update the queue and handle the extra work that occurs if this edge improves the solution.
                                this->addEdge(bestEdge, trueEdgeCost, true);

                                //Prune the edge queue of any unnecessary incoming edges
                                intQueue_->pruneEdgesTo(bestEdge.second);

                                //We will only prune the whole graph/samples on a new batch.
                            }
                            //No else, this edge may be useful at some later date.
                        }
                        else if (useFailureTracking_ == true)
                        {
                            //If the edge failed, and we're tracking failures, record.
                            //This edge has a collision and can never be helpful. Poor edge. Add the target to the list of failed children for the source:
                            bestEdge.first->markAsFailedChild(bestEdge.second);
                        }
                        //No else, we failed and we're not tracking those
                    }
                    else if (useFailureTracking_ == true)
                    {
                        //If the edge failed, and we're tracking failures, record.
                        //This edge either has a very high edge cost and can never be helpful. Poor edge. Add the target to the list of failed children for the source
                        bestEdge.first->markAsFailedChild(bestEdge.second);
                    }
                    //No else, we failed and we're not tracking those
                }
                else if (intQueue_->isSorted() == false)
                {
                    //The edge cannot improve our solution, but the queue is imperfectly sorted, so we must resort before we give up.
                    this->resort();
                }
                else
                {
                    //Else, I cannot improve the current solution, and as the queue is perfectly sorted and I am the best edge, no one can improve the current solution . Give up on the batch:
                    intQueue_->finish();
                }
            } //Integrated queue not empty.
        }



        void BITstar::newBatch()
        {
            //Variable:
            //The states as a vector:
            std::vector<VertexPtr> vertices;

            //Info:
            ++numBatches_;

            //Reset the queue:
            intQueue_->reset();

            //Prune the graph (if enabled)
            this->prune();

            //Add any new starts/goals:
            if (Planner::pis_.haveMoreStartStates() == true || Planner::pis_.haveMoreGoalStates() == true)
            {
                //Inform
                OMPL_INFORM("%s: Added new starts and/or goals and rebuilding the queue.", Planner::getName().c_str());

                //Add the starts and goals
                this->addAllStartsAndGoalStates();

                //Flag the queue as unsorted downstream from every starts:
                for (std::list<VertexPtr>::const_iterator sIter = startVertices_.begin(); sIter != startVertices_.end(); ++sIter)
                {
                    intQueue_->markVertexUnsorted(*sIter);
                }

                //Resort the queue
                intQueue_->resort();

                //Reallocate the sampler:
                sampler_ = opt_->allocInformedStateSampler(Planner::pdef_, std::numeric_limits<unsigned int>::max());
            }
            //No else

            //Set the cost sampled to the minimum
            costSampled_ = minCost_;

            //Calculate the sampling density (currently unused but for eventual JIT sampling)
            sampleDensity_ = static_cast<double>(samplesPerBatch_)/prunedMeasure_;
        }



        void BITstar::updateSamples(const VertexPtr& vertex)
        {
            //Check if we need to sample (This is in preparation for JIT sampling:)
            if (this->isCostBetterThan(costSampled_, bestCost_))
            {
                //Update the sampler counter:
                numSamples_ = numSamples_ + samplesPerBatch_;

                //Generate samples
                for (unsigned int i = 0u; i < samplesPerBatch_; ++i)
                {
                    //Variable
                    //The new state:
                    VertexPtr newState = boost::make_shared<Vertex>(Planner::si_, opt_);

                    //Sample:
                    sampler_->sampleUniform(newState->state(), bestCost_);

                    //If the state is collision free, add it to the list of free states
                    //We're counting density in the total state space, not free space
                    ++numStateCollisionChecks_;
                    if (Planner::si_->isValid(newState->stateConst()) == true)
                    {
                        //Add the new state as a sample
                        this->addSample(newState);
                    }
                }

                //Mark that we've sampled all cost spaces (This is in preparation for JIT sampling)
                costSampled_ = opt_->infiniteCost();

                //Finally, update the nearest-neighbour terms
                this->updateNearestTerms();
            }
            //No else, the samples are up to date
        }



        bool BITstar::prune()
        {
            //Variable:
            //Whether or not we pruned, start as unpruned
            bool vertexPruned = false;

            //Test if we should we do a little tidying up:
            //Is pruning enabled? Do we have a solution? Has the solution changed enough?
            if ( (usePruning_ == true) && (hasSolution_ == true) && (std::abs(this->fractionalChange(bestCost_, prunedCost_)) > pruneFraction_) )
            {
                //Variables:
                //The current measure of the problem space:
                double informedMeasure;

                informedMeasure = sampler_->getInformedMeasure(bestCost_);

                //Is there good reason to prune? I.e., is the informed subset measurably less than the total problem domain? If an informed measure is not available, we'll assume yes:
                if ( (sampler_->hasInformedMeasure() == true && informedMeasure < si_->getSpaceMeasure()) || (sampler_->hasInformedMeasure() == false) )
                {
                    //Variable:
                    //The number of vertices and samples pruned
                    std::pair<unsigned int, unsigned int> numPruned;

                    OMPL_INFORM("%s: Pruning the planning problem from a solution of %.4f to %.4f, resulting in a change of problem size from %.4f to %.4f.", Planner::getName().c_str(), prunedCost_.value(), bestCost_.value(), prunedMeasure_, informedMeasure);

                    //Increment the pruning counter:
                    ++numPrunings_;

                    //First, prune the starts/goals:
                    this->pruneStartsGoals();

                    //Prune the graph. This can be done extra efficiently by using some info in the integrated queue.
                    //This requires access to the nearest neighbour structure so vertices can be removed (if necessary).
                    numPruned = intQueue_->prune(curGoalVertex_, stateNN_);

                    //The number of vertices and samples pruned are incrementally updated.
                    numVerticesDisconnected_ = numVerticesDisconnected_ + numPruned.first;
                    numFreeStatesPruned_ = numFreeStatesPruned_ + numPruned.second;

                    //As are the counts for the number of connected and unconnected states in the NN struct
                    numCurrentConnectedStates_ = numCurrentConnectedStates_ - numPruned.first;
                    numCurrentFreeStates_ = numCurrentFreeStates_ - numPruned.second;

                    //Store the cost at which we pruned:
                    prunedCost_ = bestCost_;

                    //And the measure:
                    prunedMeasure_ = informedMeasure;

                    //Check if any states have actually been pruned
                    vertexPruned = (numPruned.second > 0u);
                }
                //No else, it's not worth the work to prune...
            }
            //No else, why was I called?

            return vertexPruned;
        }



        void BITstar::resort()
        {
            //Variable:
            //The number of vertices disconencted
            unsigned int numDisconnected;

            //Resorting requires access to the nearest neighbour structure as resorting will not reinsert vertices that should be pruned.
            //The number of vertices pruned is also incrementally updated.
            numDisconnected = intQueue_->resort();

            //Incrementally update the number of vertices pruned
            numVerticesDisconnected_ = numVerticesDisconnected_ + numDisconnected;

            //And shift those pruned vertices from connected to free:
            numCurrentConnectedStates_ = numCurrentConnectedStates_ - numDisconnected;
            numCurrentFreeStates_ = numCurrentFreeStates_ + numDisconnected;
        }



        void BITstar::publishSolution()
        {
            //Variable
            //A vector of vertices from goal->start:
            std::vector<VertexConstPtr> reversePath;
            //The path geometric
            boost::shared_ptr<ompl::geometric::PathGeometric> pathGeoPtr;

            //Allocate the pathGeoPtr
            pathGeoPtr = boost::make_shared<ompl::geometric::PathGeometric>(Planner::si_);

            //Iterate up the chain from the goal, creating a backwards vector:
            reversePath.push_back(curGoalVertex_);
            //Push back until we get to the root
            while (reversePath.back()->isRoot() == false)
            {
                //Check the case where the chain ends incorrectly. This is unnecessary but sure helpful in debugging:
                if (reversePath.back()->hasParent() == false)
                {
                    OMPL_ERROR("%s: The path to the goal does not originate at a start state.", Planner::getName().c_str());
                }

                //Push back the parent onto the vector:
                reversePath.push_back( reversePath.back()->getParentConst() );
            }

            //Now iterate that vector in reverse, putting the states into the path geometric
            for (std::vector<VertexConstPtr>::const_reverse_iterator vIter = reversePath.rbegin(); vIter != reversePath.rend(); ++vIter)
            {
                pathGeoPtr->append( (*vIter)->stateConst() );
            }

            //Now create the solution
            ompl::base::PlannerSolution soln(pathGeoPtr);

            //Mark the name:
            soln.setPlannerName(Planner::getName());

            //Mark as exact or approximate:
            if (approximateSoln_ == true)
            {
                soln.setApproximate(approximateDiff_);
            }

            //Mark whether the solution met the optimization objective:
            soln.optimized_ = opt_->isSatisfied(bestCost_);

            //Add the solution to the Problem Definition:
            Planner::pdef_->addSolutionPath(soln);
        }



        void BITstar::pruneStartsGoals()
        {
            //Are there superfluous starts to prune?
            if (startVertices_.size() > 1u)
            {
                //Yes, Iterate through the list

                //Variable
                //The iterator to the start:
                std::list<VertexPtr>::iterator startIter = startVertices_.begin();

                //Run until at the end:
                while (startIter != startVertices_.end())
                {
                    //Check if this start has met the criteria to be pruned
                    if (intQueue_->vertexPruneCondition(*startIter)  == true)
                    {
                        //It does, update counters
                        ++numVerticesDisconnected_;
                        --numCurrentConnectedStates_;
                        ++numCurrentFreeStates_;

                        //Remove the start vertex, it will remain as a free state until the next prune
                        intQueue_->eraseVertex(*startIter);

                        //Remove from the list, this returns the next iterator
                        startIter = startVertices_.erase(startIter);
                    }
                    else
                    {
                        //Still valid, move to the next one:
                        ++startIter;
                    }
                }
            }
            //No else, can't prune 1 start.

            //Are there superfluous goals to prune?
            if (goalVertices_.size() > 1u)
            {
                //Yes, Iterate through the list

                //Variable
                //The iterator to the start:
                std::list<VertexPtr>::iterator goalIter = goalVertices_.begin();

                //Run until at the end:
                while (goalIter != goalVertices_.end())
                {
                    //Check if this vertex is in the tree
                    if ((*goalIter)->isInTree() == true)
                    {
                        //Check if it meets the vertex-prune criteria
                        if (intQueue_->vertexPruneCondition(*goalIter)  == true)
                        {
                            //It does, update counters
                            ++numVerticesDisconnected_;
                            --numCurrentConnectedStates_;
                            ++numCurrentFreeStates_;

                            //Remove the goal vertex, it will remain as a free state until the next prune
                            intQueue_->eraseVertex(*goalIter);

                            //Remove from the list, this returns the next iterator
                            goalIter = goalVertices_.erase(goalIter);
                        }
                        else
                        {
                            //The goal is still valid, get the next
                            ++goalIter;
                        }
                    }
                    else
                    {
                        //Check if it meets the sample-prune criteria
                        if (intQueue_->samplePruneCondition(*goalIter)  == true)
                        {
                            //It does, that means that on the next prune, the vertex will be removed.

                            //For now. just remove it from list, this returns the next iterator
                            goalIter = goalVertices_.erase(goalIter);
                        }
                        else
                        {
                            //The goal is still valid, get the next
                            ++goalIter;
                        }
                    }
                }
            }
            //No else, can't prune 1 goal.
        }



        bool BITstar::checkEdge(const VertexPtrPair& edge)
        {
            ++numEdgeCollisionChecks_;
            return Planner::si_->checkMotion(edge.first->state(), edge.second->state());
        }



        void BITstar::addEdge(const VertexPtrPair& newEdge, const ompl::base::Cost& edgeCost, const bool& updateDescendants)
        {
            if (newEdge.first->isInTree() == false)
            {
                throw ompl::Exception("Adding an edge from a vertex not connected to the graph");
            }

            //This should be a debug-level-only assert some day:
            /*
            if (this->isCostEquivalentTo(this->trueEdgeCost(newEdge), edgeCost) == false)
            {
                throw ompl::Exception("You have passed the wrong edge cost to addEdge.");
            }
            */

            //If the vertex is currently in the tree, we need to rewire
            if (newEdge.second->hasParent() == true)
            {
                //Replace the edge
                this->replaceParent(newEdge, edgeCost, updateDescendants);
            }
            else
            {
                //If not, we just add the vertex, first connect:

                //Add a child to the parent, not updating costs:
                newEdge.first->addChild(newEdge.second, false);

                //Add a parent to the child, updating descendant costs if requested:
                newEdge.second->addParent(newEdge.first, edgeCost, updateDescendants);

                //Then add to the queues as necessary
                this->addVertex(newEdge.second);
            }

            //If the path to the goal has changed, we may need to update the cached info about the solution cost or solution length:
            this->updateGoalVertex();
        }



        void BITstar::replaceParent(const VertexPtrPair& newEdge, const ompl::base::Cost& edgeCost, const bool& updateDescendants)
        {
            if (newEdge.second->getParent() == newEdge.first)
            {
                throw ompl::Exception("The new and old parents of the given rewiring are the same.");
            }

            //This would be a good debug-level-only assert
            if (this->isCostBetterThan(newEdge.second->getCost(), opt_->combineCosts(newEdge.first->getCost(), edgeCost)) == true)
            {
                throw ompl::Exception("The new edge will increase the cost-to-come of the vertex!");
            }

            //Increment our counter:
            ++numRewirings_;

            //Remove the child from the parent, not updating costs
            newEdge.second->getParent()->removeChild(newEdge.second, false);

            //Remove the parent from the child, not updating costs
            newEdge.second->removeParent(false);

            //Add the child to the parent, not updating costs
            newEdge.first->addChild(newEdge.second, false);

            //Add the parent to the child. This updates the cost of the child as well as all it's descendents (if requested).
            newEdge.second->addParent(newEdge.first, edgeCost, updateDescendants);

            //Mark the queues as unsorted below this child
            intQueue_->markVertexUnsorted(newEdge.second);
        }



        void BITstar::updateGoalVertex()
        {
            //Variable
            //Whether we've updated the goal, be pessimistic.
            bool goalUpdated = false;
            //The the new goal, start with the current goal
            VertexPtr newBestGoal = curGoalVertex_;
            //The new cost, start as the current bestCost_
            ompl::base::Cost newCost = bestCost_;

            //Iterate through the list of goals, and see if the solution has changed
            for (std::list<VertexPtr>::const_iterator gIter = goalVertices_.begin(); gIter != goalVertices_.end(); ++gIter)
            {
                //First, is this goal even in the tree?
                if ((*gIter)->isInTree() == true)
                {
                    //Next, is there currently a solution?
                    if (static_cast<bool>(newBestGoal) == true)
                    {
                        //There is already a solution, is it to to this goal?
                        if (*gIter == newBestGoal)
                        {
                            //Ah-ha, We meet again! Are we doing any better? We check the length as sometimes the path length changes with minimal change in cost.
                            if (this->isCostEquivalentTo((*gIter)->getCost(), newCost) == false || ((*gIter)->getDepth() + 1u) != bestLength_)
                            {
                                //The path to the current best goal has changed, so we need to update it.
                                goalUpdated = true;
                                newBestGoal = *gIter;
                                newCost = newBestGoal->getCost();
                            }
                            //No else, no change
                        }
                        else
                        {
                            //It is not to this goal, we have a second solution! What an easy problem... but is it better?
                            if (this->isCostBetterThan((*gIter)->getCost(), newCost) == true)
                            {
                                //It is! Save this as a better goal:
                                goalUpdated = true;
                                newBestGoal = *gIter;
                                newCost = newBestGoal->getCost();
                            }
                            //No else, not a better solution
                        }
                    }
                    else
                    {
                        //There isn't a preexisting solution, that means that any goal is an update:
                        goalUpdated = true;
                        newBestGoal = *gIter;
                        newCost = newBestGoal->getCost();
                    }
                }
                //No else, can't be a better solution if it's not in the spanning tree, can it?
            }

            //Did we update the goal?
            if (goalUpdated == true)
            {
                //We have a better solution!
                if (hasSolution_ == false)
                {
                    approximateSoln_ = false;
                    approximateDiff_ = -1.0;
                }

                //Mark that we have a solution
                hasSolution_ = true;
                intQueue_->hasSolution();

                //Store the current goal
                curGoalVertex_ = newBestGoal;

                //Update the best cost:
                bestCost_ = newCost;

                //and best length
                bestLength_ = curGoalVertex_->getDepth() + 1u;

                //Update the queue threshold:
                intQueue_->setThreshold(bestCost_);

                //Stop the solution loop if enabled:
                stopLoop_ = stopOnSolnChange_ ;

                //Brag:
                this->goalMessage();
            }
            //No else, the goal didn't change
        }



        void BITstar::addAllStartsAndGoalStates()
        {
            //Add the new starts and goals to the lists of said vertices.
            //Do goals first, as they are only added as samples
            while (Planner::pis_.haveMoreGoalStates() == true)
            {
                //Allocate the vertex pointer
                goalVertices_.push_back(boost::make_shared<Vertex>(Planner::si_, opt_));

                //Copy the value into the state
                Planner::si_->copyState(goalVertices_.back()->state(), Planner::pis_.nextGoal());

                //And add this goal to the set of samples:
                this->addSample(goalVertices_.back());
            }

            //And then do the for starts. We do this last as the starts are added to the queue, which uses a cost-to-go heuristic in it's ordering, and for that we want all the goals updated.
            while (Planner::pis_.haveMoreStartStates() == true)
            {
                //Allocate the vertex pointer:
                startVertices_.push_back(boost::make_shared<Vertex>(Planner::si_, opt_, true));

                //Copy the value into the state:
                Planner::si_->copyState(startVertices_.back()->state(), Planner::pis_.nextStart());

                //Add this start to the queue:
                this->addVertex(startVertices_.back());
            }

            //Now, we need to update the minimum cost
            for (std::list<VertexPtr>::const_iterator sIter = startVertices_.begin(); sIter != startVertices_.end(); ++sIter)
            {
                //Take the better of the min cost so far and the cost-to-go from this start
                minCost_ = this->betterCost(minCost_, this->costToGoHeuristic(*sIter));
            }
        }



        void BITstar::addSample(const VertexPtr& newSample)
        {
            //Mark as new
            newSample->markNew();

            //Add to the NN structure:
            stateNN_->add(newSample);

            //Increment the number of unconnected states in the NN struct
            ++numCurrentFreeStates_;
        }



        void BITstar::addVertex(const VertexPtr& newVertex)
        {
            //Make sure it's connected first, so that the queue gets updated properly. This is a day of debugging I'll never get back
            if (newVertex->isInTree() == false)
            {
                throw ompl::Exception("Vertices must be connected to the graph before adding");
            }

            //Add to the queue:
            intQueue_->insertVertex(newVertex);

            //Increment the number of vertices added:
            ++numVertices_;

            //And the number of vertices in the NN struct
            ++numCurrentConnectedStates_;
        }



        void BITstar::nearestStates(const VertexPtr& vertex, std::vector<VertexPtr>* neighbourVertices)
        {
            //Make sure sampling has happened first:
            this->updateSamples(vertex);

            //Increment our counter:
            ++numNearestNeighbours_;

            if (useKNearest_ == true)
            {
                stateNN_->nearestK(vertex, k_, *neighbourVertices);
            }
            else
            {
                stateNN_->nearestR(vertex, r_, *neighbourVertices);
            }
        }



        double BITstar::nnDistance(const VertexPtr& a, const VertexPtr& b) const
        {
            //Using RRTstar as an example, this order gives us the distance FROM the queried state TO the other neighbours in the structure.
            //The distance function between two states
            if (!a->state())
            {
                throw ompl::Exception("a->state is unallocated");
            }
            if (!b->state())
            {
                throw ompl::Exception("b->state is unallocated");
            }
            return Planner::si_->distance(b->state(), a->state());
        }



        ompl::base::Cost BITstar::lowerBoundHeuristicVertex(const VertexPtr& vertex) const
        {
            return opt_->combineCosts( this->costToComeHeuristic(vertex), this->costToGoHeuristic(vertex) );
        }



        ompl::base::Cost BITstar::currentHeuristicVertex(const VertexPtr& vertex) const
        {
            return opt_->combineCosts( vertex->getCost(), this->costToGoHeuristic(vertex) );
        }


        ompl::base::Cost BITstar::lowerBoundHeuristicEdge(const VertexPtrPair& edgePair) const
        {
            return this->combineCosts(this->costToComeHeuristic(edgePair.first), this->edgeCostHeuristic(edgePair), this->costToGoHeuristic(edgePair.second));
        }



        ompl::base::Cost BITstar::currentHeuristicEdge(const VertexPtrPair& edgePair) const
        {
            return opt_->combineCosts(this->currentHeuristicEdgeTarget(edgePair), this->costToGoHeuristic(edgePair.second));
        }



        ompl::base::Cost BITstar::currentHeuristicEdgeTarget(const VertexPtrPair& edgePair) const
        {
            return opt_->combineCosts(edgePair.first->getCost(), this->edgeCostHeuristic(edgePair));
        }



        ompl::base::Cost BITstar::costToComeHeuristic(const VertexPtr& vertex) const
        {
            //Variable
            //The current best cost to the state, initialize to infinity
            ompl::base::Cost curBest = opt_->infiniteCost();

            //Iterate over the list of starts, finding the minimum estimated cost-to-come to the state
            for (std::list<VertexPtr>::const_iterator startIter = startVertices_.begin(); startIter != startVertices_.end(); ++startIter)
            {
                //Update the cost-to-come as the better of the best so far and the new one
                curBest = this->betterCost(curBest, opt_->motionCostHeuristic((*startIter)->state(), vertex->state()));
            }

            //Return
            return curBest;
        }



        ompl::base::Cost BITstar::edgeCostHeuristic(const VertexPtrPair& edgePair) const
        {
            return opt_->motionCostHeuristic(edgePair.first->state(), edgePair.second->state());
        }



        ompl::base::Cost BITstar::costToGoHeuristic(const VertexPtr& vertex) const
        {
            //Variable
            //The current best cost to a goal from the state, initialize to infinity
            ompl::base::Cost curBest = opt_->infiniteCost();

            //Iterate over the list of goals, finding the minimum estimated cost-to-go from the state
            for (std::list<VertexPtr>::const_iterator goalIter = goalVertices_.begin(); goalIter != goalVertices_.end(); ++goalIter)
            {
                //Update the cost-to-go as the better of the best so far and the new one
                curBest = this->betterCost(curBest, opt_->motionCostHeuristic(vertex->state(), (*goalIter)->state()));
            }

            //Return
            return curBest;
        }


        ompl::base::Cost BITstar::trueEdgeCost(const VertexPtrPair& edgePair) const
        {
            return opt_->motionCost(edgePair.first->state(), edgePair.second->state());
        }



        ompl::base::Cost BITstar::neighbourhoodCost() const
        {
            OMPL_INFORM("%s: TODO: Write neighbourhoodCost() more generally.", Planner::getName().c_str());
            return ompl::base::Cost( 2.0*r_ );
        }



        bool BITstar::isCostBetterThan(const ompl::base::Cost& a, const ompl::base::Cost& b) const
        {
            return a.value() < b.value();
        }



        bool BITstar::isCostWorseThan(const ompl::base::Cost& a, const ompl::base::Cost& b) const
        {
            //If b is better than a, then a is worse than b
            return this->isCostBetterThan(b, a);
        }



        bool BITstar::isCostEquivalentTo(const ompl::base::Cost& a, const ompl::base::Cost& b) const
        {
            //If a is not better than b, and b is not better than a, then they are equal
            return !this->isCostBetterThan(a,b) && !this->isCostBetterThan(b,a);
        }



        bool BITstar::isCostNotEquivalentTo(const ompl::base::Cost& a, const ompl::base::Cost& b) const
        {
            //If a is better than b, or b is better than a, then they are not equal
            return this->isCostBetterThan(a,b) || this->isCostBetterThan(b,a);
        }



        bool BITstar::isCostBetterThanOrEquivalentTo(const ompl::base::Cost& a, const ompl::base::Cost& b) const
        {
            //If b is not better than a, then a is better than, or equal to, b
            return !this->isCostBetterThan(b, a);
        }



        bool BITstar::isCostWorseThanOrEquivalentTo(const ompl::base::Cost& a, const ompl::base::Cost& b) const
        {
            //If a is not better than b, than a is worse than, or equal to, b
            return !this->isCostBetterThan(a,b);
        }



        bool BITstar::isFinite(const ompl::base::Cost& cost) const
        {
            return this->isCostBetterThan(cost, opt_->infiniteCost());
        }



        ompl::base::Cost BITstar::betterCost(const ompl::base::Cost& a, const ompl::base::Cost& b) const
        {
            if (this->isCostBetterThan(b,a))
            {
                return b;
            }
            else
            {
                return a;
            }
        }



        ompl::base::Cost BITstar::combineCosts(const ompl::base::Cost& a, const ompl::base::Cost& b, const ompl::base::Cost& c) const
        {
            return opt_->combineCosts(a, opt_->combineCosts(b, c));
        }



        ompl::base::Cost BITstar::combineCosts(const ompl::base::Cost& a, const ompl::base::Cost& b, const ompl::base::Cost& c, const ompl::base::Cost& d) const
        {
            return opt_->combineCosts(a, this->combineCosts(b, c, d));
        }



        double BITstar::fractionalChange(const ompl::base::Cost& newCost, const ompl::base::Cost& oldCost) const
        {
            return this->fractionalChange(newCost, oldCost, oldCost);
        }



        double BITstar::fractionalChange(const ompl::base::Cost& newCost, const ompl::base::Cost& oldCost, const ompl::base::Cost& refCost) const
        {
            //If the old cost is not finite, than we call that infinite percent improvement
            if (this->isFinite(oldCost) == false)
            {
                //Return infinity (but not beyond)
                return std::numeric_limits<double>::infinity();
            }
            else
            {
                //Calculate and return
                return ( newCost.value() - oldCost.value() )/refCost.value();
            }
        }



        void BITstar::initializeNearestTerms()
        {
            //Calculate the k-nearest constant
            k_rgg_ = this->minimumRggK();

            this->updateNearestTerms();
        }



        void BITstar::updateNearestTerms()
        {
            //Variables:
            //The number of samples:
            unsigned int N;

            //Calculate the number of N:
            N = stateNN_->size();

            if (useKNearest_ == true)
            {
                k_ = this->calculateK(N);
            }
            else
            {
                r_ = this->calculateR(N);
            }
        }



        double BITstar::calculateR(unsigned int N) const
        {
            //Variables
            //The dimension cast as a double for readibility;
            double dimDbl = static_cast<double>(Planner::si_->getStateDimension());
            //The size of the graph
            double cardDbl = static_cast<double>(N);

            //Calculate the term and return
            return this->minimumRggR()*std::pow( std::log(cardDbl)/cardDbl, 1/dimDbl );
        }



        unsigned int BITstar::calculateK(unsigned int N) const
        {
            //Calculate the term and return
            return std::ceil( k_rgg_ * std::log(static_cast<double>(N)) );
        }



        double BITstar::minimumRggR() const
        {
            //Variables
            //The dimension cast as a double for readibility;
            double dimDbl = static_cast<double>(Planner::si_->getStateDimension());

            //Calculate the term and return
            return rewireFactor_*2.0*std::pow( (1.0 + 1.0/dimDbl)*( prunedMeasure_/unitNBallMeasure(Planner::si_->getStateDimension()) ), 1.0/dimDbl ); //RRG radius (biggest for unit-volume problem)
            //return rewireFactor_*std::pow( 2.0*(1.0 + 1.0/dimDbl)*( prunedMeasure_/unitNBallMeasure(Planner::si_->getStateDimension()) ), 1.0/dimDbl ); //RRT* radius (smaller for unit-volume problem)
            //return rewireFactor_*2.0*std::pow( (1.0/dimDbl)*( prunedMeasure_/unitNBallMeasure(Planner::si_->getStateDimension()) ), 1.0/dimDbl ); //FMT* radius (smallest for R2, equiv to RRT* for R3 and then middle for higher d. All unit-volume)
        }



        double BITstar::minimumRggK() const
        {
            //Variables
            //The dimension cast as a double for readibility;
            double dimDbl = static_cast<double>(Planner::si_->getStateDimension());

            //Calculate the term and return
            return rewireFactor_*(boost::math::constants::e<double>() + (boost::math::constants::e<double>() / dimDbl)); //RRG k-nearest
        }



        void BITstar::goalMessage() const
        {
            OMPL_INFORM("%s: Found a solution consisting of %u vertices with a total cost of %.4f in %u iterations (%u vertices, %u rewirings). Graph currently has %u vertices.", Planner::getName().c_str(), bestLength_, bestCost_.value(), numIterations_, numVertices_, numRewirings_, numCurrentConnectedStates_);
        }



        void BITstar::endSuccessMessage() const
        {
            OMPL_INFORM("%s: Found a final solution of cost %.4f from %u samples by using %u vertices and %u rewirings. Final graph has %u vertices.", Planner::getName().c_str(), bestCost_.value(), numSamples_, numVertices_, numRewirings_, numCurrentConnectedStates_);
        }



        void BITstar::endFailureMessage() const
        {
            OMPL_INFORM("%s: Did not find a solution from %u samples after %u iterations, %u vertices and %u rewirings.", Planner::getName().c_str(), numSamples_, numIterations_, numVertices_, numRewirings_);
        }



        void BITstar::statusMessage(const ompl::msg::LogLevel& msgLevel, const std::string& status) const
        {
            //Check if we need to create the message
            if (msgLevel >= ompl::msg::getLogLevel())
            {
                //Variable
                //The message as a stream:
                std::stringstream outputStream;

                //Create the stream:
                //The name of the planner
                outputStream << Planner::getName();
                outputStream << " (";
                //The current path cost:
                outputStream << "l: " << std::setw(6) << std::setfill(' ') << std::setprecision(5) << bestCost_.value();
                //The number of batches:
                outputStream << ", b: " << std::setw(5) << std::setfill(' ') << numBatches_;
                //The number of iterations
                outputStream << ", i: " << std::setw(5) << std::setfill(' ') << numIterations_;
                //The number of states current in the graph
                outputStream << ", g: " << std::setw(5) << std::setfill(' ') << numCurrentConnectedStates_;
                //The number of free states
                outputStream << ", f: " << std::setw(5) << std::setfill(' ') << numCurrentFreeStates_;
                //The number edges in the queue:
                outputStream << ", q: " << std::setw(5) << std::setfill(' ') << intQueue_->numEdges();
                //The number of samples generated
                outputStream << ", s: " << std::setw(5) << std::setfill(' ') << numSamples_;
                //The number of vertices ever added to the graph:
                outputStream << ", v: " << std::setw(5) << std::setfill(' ') << numVertices_;
                //The number of prunings:
                outputStream << ", p: " << std::setw(5) << std::setfill(' ') << numPrunings_;
                //The number of rewirings:
                outputStream << ", r: " << std::setw(5) << std::setfill(' ') << numRewirings_;
                //The number of nearest-neighbour calls
                outputStream << ", n: " << std::setw(5) << std::setfill(' ') << numNearestNeighbours_;
                //The number of state collision checks:
                outputStream << ", c(s): " << std::setw(5) << std::setfill(' ') << numStateCollisionChecks_;
                //The number of edge collision checks:
                outputStream << ", c(e): " << std::setw(5) << std::setfill(' ') << numEdgeCollisionChecks_;
                outputStream << "):    ";
                //The message:
                outputStream << status;


                if (msgLevel == ompl::msg::LOG_DEBUG)
                {
                    OMPL_DEBUG("%s", outputStream.str().c_str());
                }
                else if (msgLevel == ompl::msg::LOG_INFO)
                {
                    OMPL_INFORM("%s", outputStream.str().c_str());
                }
                else if (msgLevel == ompl::msg::LOG_WARN)
                {
                    OMPL_WARN("%s", outputStream.str().c_str());
                }
                else if (msgLevel == ompl::msg::LOG_ERROR)
                {
                    OMPL_ERROR("%s", outputStream.str().c_str());
                }
                else
                {
                    throw ompl::Exception("Log level not recognized");
                }
            }
            //No else, this message is below the log level
        }
        /////////////////////////////////////////////////////////////////////////////////////////////



        /////////////////////////////////////////////////////////////////////////////////////////////
        //Boring sets/gets (Public) and progress properties (Protected):
        void BITstar::setRewireFactor(double rewireFactor)
        {
            rewireFactor_ = rewireFactor;

            //Check if there's things to update
            if (this->isSetup() == true)
            {
                //Reinitialize the terms:
                this->initializeNearestTerms();
            }
        }



        double BITstar::getRewireFactor() const
        {
            return rewireFactor_;
        }



        void BITstar::setSamplesPerBatch(unsigned int n)
        {
            samplesPerBatch_ = n;
        }



        unsigned int BITstar::getSamplesPerBatch() const
        {
            return samplesPerBatch_;
        }



        void BITstar::setKNearest(bool useKNearest)
        {
            //Check if the flag has changed
            if (useKNearest != useKNearest_)
            {
                //Set the k-nearest flag
                useKNearest_ = useKNearest;

                if (useKNearest_ == true)
                {
                    //Warn that this isn't exactly implemented
                    OMPL_WARN("%s: The implementation of the k-Nearest version of BIT* is not 100%% correct.", Planner::getName().c_str()); //This is because we have a separate nearestNeighbours structure for samples and vertices and you don't know what fraction of K to ask for from each...
                }

                //Check if there's things to update
                if (this->isSetup() == true)
                {
                    //Reinitialize the terms:
                    this->initializeNearestTerms();
                }
            }
            //No else, it didn't change.
        }



        bool BITstar::getKNearest() const
        {
            return useKNearest_;
        }



        void BITstar::setUseFailureTracking(bool trackFailures)
        {
            //Store
            useFailureTracking_ = trackFailures;

            //Configure queue if constructed:
            if (intQueue_)
            {
                intQueue_->setUseFailureTracking(useFailureTracking_);
            }
        }



        bool BITstar::getUseFailureTracking() const
        {
            return useFailureTracking_;
        }


        void BITstar::setStrictQueueOrdering(bool beStrict)
        {
            useStrictQueueOrdering_ = beStrict;
        }



        bool BITstar::getStrictQueueOrdering() const
        {
            return useStrictQueueOrdering_;
        }



        void BITstar::setPruning(bool prune)
        {
            if (prune == false)
            {
                OMPL_WARN("%s: Turning pruning off does not turn a fake pruning on, as it should.", Planner::getName().c_str());
            }

            usePruning_ = prune;
        }



        bool BITstar::getPruning() const
        {
            return usePruning_;
        }



        void BITstar::setPruneThresholdFraction(double fractionalChange)
        {
            if (fractionalChange < 0.0 || fractionalChange > 1.0)
            {
                throw ompl::Exception("Prune threshold must be specified as a fraction between [0, 1].");
            }

            pruneFraction_ = fractionalChange;
        }



        double BITstar::getPruneThresholdFraction() const
        {
            return pruneFraction_;
        }



        void BITstar::setDelayRewiringUntilInitialSolution(bool delayRewiring)
        {
            delayRewiring_ = delayRewiring;

            //Configure queue if constructed:
            if (intQueue_)
            {
                intQueue_->setDelayedRewiring(delayRewiring_);
            }
        }



        bool BITstar::getDelayRewiringUntilInitialSolution() const
        {
            return delayRewiring_;
        }



        void BITstar::setStopOnSolnImprovement(bool stopOnChange)
        {
            stopOnSolnChange_ = stopOnChange;
        }



        bool BITstar::getStopOnSolnImprovement() const
        {
            return stopOnSolnChange_;
        }



        ompl::base::Cost BITstar::bestCost() const
        {
            return bestCost_;
        }



        std::string BITstar::bestCostProgressProperty() const
        {
            return boost::lexical_cast<std::string>(this->bestCost().value());
        }



        std::string BITstar::bestLengthProgressProperty() const
        {
            return boost::lexical_cast<std::string>(bestLength_);
        }



        std::string BITstar::currentFreeProgressProperty() const
        {
            return boost::lexical_cast<std::string>(numCurrentFreeStates_);
        }



        std::string BITstar::currentVertexProgressProperty() const
        {
            return boost::lexical_cast<std::string>(numCurrentConnectedStates_);
        }



        std::string BITstar::vertexQueueSizeProgressProperty() const
        {
            return boost::lexical_cast<std::string>(intQueue_->numVertices());
        }



        std::string BITstar::edgeQueueSizeProgressProperty() const
        {
            return boost::lexical_cast<std::string>(intQueue_->numEdges());
        }



        unsigned int BITstar::numIterations() const
        {
            return numIterations_;
        }



        std::string BITstar::iterationProgressProperty() const
        {
            return boost::lexical_cast<std::string>(this->numIterations());
        }



        unsigned int BITstar::numBatches() const
        {
            return numBatches_;
        }



        std::string BITstar::batchesProgressProperty() const
        {
            return boost::lexical_cast<std::string>(this->numBatches());
        }



        std::string BITstar::pruningProgressProperty() const
        {
            return boost::lexical_cast<std::string>(numPrunings_);
        }



        std::string BITstar::totalStatesCreatedProgressProperty() const
        {
            return boost::lexical_cast<std::string>(numSamples_);
        }



        std::string BITstar::verticesConstructedProgressProperty() const
        {
            return boost::lexical_cast<std::string>(numVertices_);
        }



        std::string BITstar::statesPrunedProgressProperty() const
        {
            return boost::lexical_cast<std::string>(numFreeStatesPruned_);
        }



        std::string BITstar::verticesDisconnectedProgressProperty() const
        {
            return boost::lexical_cast<std::string>(numVerticesDisconnected_);
        }



        std::string BITstar::rewiringProgressProperty() const
        {
            return boost::lexical_cast<std::string>(numRewirings_);
        }



        std::string BITstar::stateCollisionCheckProgressProperty() const
        {
            return boost::lexical_cast<std::string>(numStateCollisionChecks_);
        }



        std::string BITstar::edgeCollisionCheckProgressProperty() const
        {
            return boost::lexical_cast<std::string>(numEdgeCollisionChecks_);
        }



        std::string BITstar::nearestNeighbourProgressProperty() const
        {
            return boost::lexical_cast<std::string>(numNearestNeighbours_);
        }
        /////////////////////////////////////////////////////////////////////////////////////////////
    }//geometric
}//ompl
