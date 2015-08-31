/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Rice University
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
 *   * Neither the name of the Rice University nor the names of its
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

/* Authors: Alejandro Perez, Sertac Karaman, Ryan Luna, Luis G. Torres, Ioan Sucan, Javier V Gomez */

#include "ompl/geometric/planners/rrt/SafeBiRRTstar.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include <algorithm>
#include <limits>
#include <map>
#include <queue>
#include <boost/math/constants/constants.hpp>

//STa
#include "ompl/base/objectives/SafetyObjective.h"
#include "ompl/base/objectives/ManipulabilityObjective.h"
#include "ompl/base/objectives/JointLimitsObjective.h"
#include "ompl/base/objectives/SafePathLengthOptimizationObjective.h"
#include "ompl/base/objectives/HumanAwarenessObjective.h"
#include "ompl/base/goals/GoalLazySamples.h"
#include <boost/thread.hpp>

//STa temp
#include <fstream>

ompl::geometric::SafeBiRRTstar::SafeBiRRTstar(const base::SpaceInformationPtr &si) :
base::Planner(si, "SafeBiRRTstar"),
maxDistance_(0.0),
lastGoalMotion_(NULL),
prune_(true),
localBias_(true),
localBiasAttempt_(1),
pruneStatesThreshold_(1),
iterations_(0),
bestCost_(base::SafetyCost()),
improveSolutionBias_(0.05),
treeConnectionIndex_(0)
{
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;
    specs_.canReportIntermediateSolutions = true;

    //STa
    fast_dist_ = true;
    travel_dist_limit_=0.01;


    Planner::declareParam<double>("range", this, &SafeBiRRTstar::setRange, &SafeBiRRTstar::getRange, "0.:1.:10000.");


    addPlannerProgressProperty("iterations INTEGER",
            boost::bind(&SafeBiRRTstar::getIterationCount, this));
    addPlannerProgressProperty("best cost REAL",
            boost::bind(&SafeBiRRTstar::getBestCost, this));
}

ompl::geometric::SafeBiRRTstar::~SafeBiRRTstar()
{
    freeMemory();
}


void ompl::geometric::SafeBiRRTstar::treeConnection::updateWholeMotionCost(const base::SafeMultiOptimizationObjective* safe_multi_opt)
{
    wholeMotionCost = safe_multi_opt->safeCombineCosts(startTreeMotion->cost, goalTreeMotion->cost);
}

std::vector<const ompl::base::State*> ompl::geometric::SafeBiRRTstar::treeConnection::getPath()
{
    std::vector<const base::State*> solutionStates;
    Motion* solution = startTreeMotion;
    while(solution != NULL)
    {
        solutionStates.insert(solutionStates.begin(), solution->state);
        solution = solution->parent;
    }
    // We need to check one of their parents to avoid a duplicate state in the solution path
    solution = goalTreeMotion->parent;
    while(solution != NULL)
    {
        solutionStates.push_back(solution->state);
        solution = solution->parent;
    }
    return solutionStates;
}

std::vector<const ompl::base::State*> ompl::geometric::SafeBiRRTstar::treeConnection::getStatesToShare()
{
    std::vector<const base::State*> statesToShare;
    Motion* currentMotion = startTreeMotion;
    //We don't want to share the start state as it is already present in other trees
    while(currentMotion->parent != NULL)
    {
        if (!currentMotion->isStateShared)
        {
            statesToShare.insert(statesToShare.begin(), currentMotion->state);
            currentMotion->isStateShared = true;
        }
        currentMotion = currentMotion->parent;
    }
    //Because startTreeMotion->state == goalTreeMotion->state, the state is already shared
    goalTreeMotion->isStateShared = true;
    // We need to check one of their parents to avoid a duplicate state in the solution path
    currentMotion = goalTreeMotion->parent;
    //Same for goal state
    while(currentMotion->parent != NULL)
    {
        if (!currentMotion->isStateShared)
        {
            statesToShare.push_back(currentMotion->state);
            currentMotion->isStateShared = true;
        }
        currentMotion = currentMotion->parent;
    }
    return statesToShare;
}

void ompl::geometric::SafeBiRRTstar::treeConnection::getWorstMotion(const base::SafeMultiOptimizationObjective* safe_multi_opt, Motion*& worstMotion, bool& isStartTree)
{
    worstMotion = startTreeMotion;
    Motion* currentMotion = startTreeMotion->parent;
    while(currentMotion->parent != NULL)
    {
        //It is irrelevant to compare incCost for the path length objective, compare only minmax objective
        if (safe_multi_opt->isMinMaxSafetyCostBetterThan(worstMotion->incCost, currentMotion->incCost))
        {
            worstMotion = currentMotion;
            isStartTree = true;
        }

        currentMotion = currentMotion->parent;
    }

    currentMotion = goalTreeMotion;
    while(currentMotion->parent != NULL)
    {
        if (safe_multi_opt->isMinMaxSafetyCostBetterThan(worstMotion->incCost, currentMotion->incCost))
        {
            worstMotion = currentMotion;
            isStartTree = false;
        }

        currentMotion = currentMotion->parent;
    }
}

ompl::base::SafetyCost ompl::geometric::SafeBiRRTstar::Motion::heuristicCost(const base::SafeMultiOptimizationObjective* safe_multi_opt,  const bool shortest) const
{
    base::SafetyCost costToCome, costToGo;
    if (shortest)
    {
        costToCome = safe_multi_opt->safeCostToCome(state, stateCost);
        costToGo = safe_multi_opt->safeCostToGo(state, stateCost);
    }
    else
    {
        costToCome = cost;
        costToGo = safe_multi_opt->safeCostToGo(state, stateCost);
    }
    return safe_multi_opt->safeCombineCosts(costToCome, costToGo);
}

void ompl::geometric::SafeBiRRTstar::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    //	//STa temp
    //	maxDistance_ /= 2;

    if (!si_->getStateSpace()->hasSymmetricDistance() || !si_->getStateSpace()->hasSymmetricInterpolate())
    {
        OMPL_WARN("%s requires a state space with symmetric distance and symmetric interpolation.", getName().c_str());
    }

    // Configuring nearest neighbors structures for the planning trees
    if (!tStart_)
        tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
    if (!tGoal_)
        tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
    tStart_->setDistanceFunction(boost::bind(&SafeBiRRTstar::distanceFunction, this, _1, _2));
    tGoal_->setDistanceFunction(boost::bind(&SafeBiRRTstar::distanceFunction, this, _1, _2));

    if (pdef_)
    {
        //STa
        safe_motion_validator_ = new ompl::base::SafeMotionValidator(si_.get());
        ssvc_ = static_cast<ompl::base::SafeStateValidityChecker*>(si_->getStateValidityChecker().get());
        getOptimalSafetyObjective();
        pdef_->setOptimizationObjective(opt_);

    }
    else
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    }

    connectionRange_ = 100.0 * si_->getStateSpace()->getLongestValidSegmentLength();

}

void ompl::geometric::SafeBiRRTstar::getOptimalSafetyObjective()
{

    safe_multi_opt_ = new ompl::base::SafeMultiOptimizationObjective(si_);
    ompl::base::OptimizationObjectivePtr lengthObj(new ompl::base::SafePathLengthOptimizationObjective(si_, &safe_multi_opt_->getStart() ,&safe_multi_opt_->getGoal()));
    ompl::base::OptimizationObjectivePtr safetyObj(new ompl::base::SafetyObjective(si_, safe_motion_validator_, fast_dist_, travel_dist_limit_));
    ompl::base::OptimizationObjectivePtr jointLimitsObj(new ompl::base::JointLimitsObjective(si_));
    ompl::base::OptimizationObjectivePtr manipulabilityObj(new ompl::base::ManipulabilityObjective(si_));
    ompl::base::OptimizationObjectivePtr humanAwarenessObj(new ompl::base::HumanAwarenessObjective(si_));
    safe_multi_opt_->addObjective(lengthObj, 1, "length");
    safe_multi_opt_->addObjective(safetyObj, 10, "safety");
    //	safe_multi_opt_->addObjective(jointLimitsObj, 1, "joint");
    //	safe_multi_opt_->addObjective(manipulabilityObj, 1, "manipulability");
    if (ssvc_->humanPresence())
        safe_multi_opt_->addObjective(humanAwarenessObj, 1, "awareness");
    safe_multi_opt_->NormalizeWeight();

    //Safe RRT*
    opt_ = ompl::base::OptimizationObjectivePtr(safe_multi_opt_);
}



void ompl::geometric::SafeBiRRTstar::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (tStart_)
        tStart_->clear();
    if (tGoal_)
        tGoal_->clear();

    lastGoalMotion_ = NULL;

    iterations_ = 0;
    bestCost_ = base::SafetyCost();

}


ompl::base::PlannerStatus ompl::geometric::SafeBiRRTstar::solve(const base::PlannerTerminationCondition &ptc)
{
    //STa test
//    std::string homepath = getenv("HOME");
//    std::ofstream output_file((homepath + "/safebirrtstar.txt").c_str(), std::ios::out | std::ios::app);
    //	ompl::time::point init = ompl::time::now();
    //	ompl::time::duration dur;

    //STa test
    bool localstate = false;
    size_t localStateImprovementCount = 0;
    size_t randomStateImprovementCount = 0;

    checkValidity();

    safe_multi_opt_->setGoal(pdef_->getGoal().get());

    while (const base::State *startState = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, startState);
        motion->root = motion->state;
        motion->stateCost = safe_multi_opt_->safeStateCost(motion->state);
        motion->cost = motion->stateCost;
        motion->isGoalTree = false;
        if (!motion->stateCost.getCollisionWorld())
        {
            tStart_->add(motion);
            startMotion_ = motion;
        }
    }

    if (tStart_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    safe_multi_opt_->setStart(startMotion_->state);


    // Do the same for the goal but only until one valid state is found
    do
    {
        const base::State *goalState = pis_.nextGoal(ptc);
        if (goalState)
        {
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, goalState);
            motion->root = motion->state;
            motion->stateCost = safe_multi_opt_->safeStateCost(motion->state);
            motion->cost = motion->stateCost;
            motion->isGoalTree = true;
            if (!motion->stateCost.getCollisionWorld())
            {
                tGoal_->add(motion);
                goalMotion_.push_back(motion);
            }
        }
    } while (ptc == false && tGoal_->size() == 0);

    if (tGoal_->size() == 0)
    {
        OMPL_ERROR("%s: Goal tree has no valid states!", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), tStart_->size());


    bool feasibleSolution       = false;


    bestCost_ = safe_multi_opt_->safeInfiniteCost();

    bool sufficientlyShort = false;

    Motion   *rmotion   = new Motion(si_);
    base::State *rstate = rmotion->state;

    Motion   *xmotion   = new Motion(si_);
    base::State *xstate = xmotion->state;


    // e+e/d.  K-nearest RRT*
    k_rrg_           = boost::math::constants::e<double>() +
            (boost::math::constants::e<double>() / (double)si_->getStateSpace()->getDimension());

    std::vector<Motion*>       nbh;

    std::vector<base::SafetyCost>    costs;
    std::vector<base::SafetyCost>    incCosts;
    std::vector<std::size_t>   sortedCostIndices;
    std::vector<bool>   			exactCost;

    std::vector<int>           valid;
    rewireTest_ = 0;
    statesGenerated_ = 0;
    statesPruned_ = 0;

    TreeData tree = tStart_;
    TreeData otherTree = tGoal_;

    //STa : If the planner is used with SafeCforest, bestSharedCost_ is updated with the best cost among planning threads. Else, it is the same as BestCost_
    const base::ReportSafeIntermediateSolutionFn safeIntermediateSolutionCallback = pdef_->getSafeIntermediateSolutionCallback();
    if (!safeIntermediateSolutionCallback)
        bestSharedCost_ = &bestCost_;

    base::SafetyCost currentBestSharedCost;

    while (ptc == false)
    {
        iterations_++;
        checkForSolution_ = false;

        currentBestSharedCost = *bestSharedCost_;
        //STa temp
        std::cout << "currentBestSharedCost = " << currentBestSharedCost << "\n";

        // Add available goal states
        while (pis_.haveMoreGoalStates())
        {
            const base::State *goalState = pis_.nextGoal();

            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, goalState);
            motion->root = motion->state;
            motion->stateCost = safe_multi_opt_->safeStateCost(motion->state);
            motion->cost =  motion->stateCost;
            motion->isGoalTree = true;
            if (!motion->stateCost.getCollisionWorld())
            {
                tGoal_->add(motion);
                goalMotion_.push_back(motion);
            }
        }
        // Local optimization
       if (localBias_ && feasibleSolution && rng_.uniform01() < improveSolutionBias_)
       {
           //TODO : Store the worst motion in the treeConnection element and update when necessary
           Motion* worst_motion;
           bool isStartTree, canImprove;
           connection_[bestIndex_]->getWorstMotion(safe_multi_opt_, worst_motion, isStartTree);
           double radius = si_->getStateSpace()->distance(worst_motion->state, worst_motion->parent->state)/2;
           base::State* worst_state = si_->allocState();
           si_->getStateSpace()->interpolate(worst_motion->state, worst_motion->parent->state,0.5,worst_state);
           base::SafetyCost rstateCost;
           size_t attempt = 0;
           do
           {
               sampler_->sampleUniformNear(rstate, worst_state, radius);
               rstateCost = safe_multi_opt_->safeStateCost(rstate);
               attempt++;
               canImprove = (!rstateCost.getCollisionWorld() && safe_multi_opt_->isMinMaxSafetyCostBetterThan(rstateCost, worst_motion->incCost));
           } while(!canImprove && (attempt < localBiasAttempt_));

           if (!canImprove)
               continue;


           if (isStartTree)
           {
               tree = tStart_;
               otherTree = tGoal_;
           }
           else
           {
               tree = tGoal_;
               otherTree = tStart_;
           }
           //STa test
//           std::cout << "worst motion incCost = " << worst_motion->incCost << "\n";
//           std::cout << "rstateCost = " << rstateCost << "\n";
           localstate = true;
       }
       else
       {
           // sample random state
           sampler_->sampleUniform(rstate);

           //STa test
           localstate = false;
       }


        statesGenerated_++;

        Motion* result; // the motion that gets added in extendTree
        if (extendTree(rmotion, tree, result) != FAILED) // we added something new to the tree
        {
//            //STa temp
//            std::cout << "extendTree returned true \n";

            // Try to connect the other tree to the node we just added
            if (connectTrees(result, otherTree, xmotion))
            {
//                //STa temp
//                std::cout << "connectTrees returned true \n";

                feasibleSolution = true;
            }
//            else
//                //STa temp
//                std::cout << "connectTrees returned false \n";
        }
//        else
//            //STa temp
//            std::cout << "extendTree returned false \n";


        // Checking for solution or iterative improvement
        if (checkForSolution_)
        {
            //STa temp
            if (localstate)
                localStateImprovementCount++;
            else
                randomStateImprovementCount++;
//            std::cout << "bestCost_ = " << bestCost_ << "\n";
            //			//STa test
            //			dur = ompl::time::now() - init;
            //			output_file <<  bestCost_ << ompl::time::now() << "\n";


            sufficientlyShort = safe_multi_opt_->isSafetySatisfied(bestCost_);

            if (sufficientlyShort)
            {
                break;
            }

            //Share the solution states with the others planning threads
            if (safeIntermediateSolutionCallback)
            {
                std::vector<const base::State *> statesToShare = connection_[bestIndex_]->getStatesToShare();
                safeIntermediateSolutionCallback(this, statesToShare, bestCost_);
            }
        }

        //STa temp
        std::cout << "bestSharedCost_ = " << *bestSharedCost_ << "\n";

        //Generic way to generate the pruning process
        if (prune_ && safe_multi_opt_->isSafetyCostBetterThan(*bestSharedCost_, currentBestSharedCost))
        {
            statesPruned_ += pruneTree(tStart_, true, *bestSharedCost_);
            statesPruned_ += pruneTree(tGoal_, false, *bestSharedCost_);

            //If CForest is running, all the connection for this planner could be removed by the pruning process. If the best solution is removed, reset the best cost.
            if (connection_.find(bestIndex_) == connection_.end())
            {
                feasibleSolution = false;
                bestCost_ = safe_multi_opt_->infiniteCost();
            }
        }

        std::swap(tree, otherTree);
    }

    if (feasibleSolution)
    {
        //		//STa temp
        //		std::cout << "bestIndex_ = " << bestIndex_ << "\n";
        //		std::cout << "connection_.size()  = " << connection_.size() << "\n";

        ptc.terminate();
        // construct the solution path
        std::vector<const base::State*> mpath = connection_[bestIndex_]->getPath();


        // set the solution path
        PathGeometric *geoPath = new PathGeometric(si_);
        for (size_t i = 0; i < mpath.size() ; ++i)
        {
            geoPath->append(mpath[i]);
        }
        base::PathPtr path(geoPath);
        // Add the solution path.
        base::PlannerSolution psol(path);
        psol.setPlannerName(getName());

        // Does the solution satisfy the optimization objective?
        psol.setOptimized(opt_, bestCost_, sufficientlyShort);
        pdef_->addSolutionPath(psol);


        //STa temp
        //		base::SafetyCost incCost, prevCost, curCost;
        //		prevCost = safe_multi_opt_->safeStateCost(mpath[mpath.size() - 1]);
        //		for (int i = mpath.size() - 1 ; i > 0 ; --i)
        //		{
        //			incCost = safe_multi_opt_->safeMotionCostTEST(mpath[i], mpath[i-1]);
        //			curCost = safe_multi_opt_->safeCombineCosts(prevCost, incCost);
        //			prevCost = curCost;
        //		}
        //		if(base::goalRegionCostToGo(startMotion_->state, pdef_->getGoal().get()).value() > 0.01)
        //		{
        //			std::string homepath = getenv("HOME");
        //			std::ofstream output_file((homepath + "/safe_rrt_star.txt").c_str(), std::ios::out | std::ios::app);
        //
        //			output_file << std::setw(10) << 1 << "\t";
        //			for (size_t i=0; i < curCost.getIndividualCostSize(); ++i)
        //			{
        //				output_file << std::setw(10) << curCost.getIndividualCost(i).value() << "\t";
        //			}
        ////			output_file << std::setw(10) << ompl::time::seconds(dur_first_sol) << "\t";
        //			output_file << std::setw(10) << statesGenerated_ << "\t";
        //			output_file << std::setw(10) << rewireTest_ << "\n";
        //			output_file.close();
        //		}
    }

    si_->freeState(rstate);
    si_->freeState(xstate);
    delete rmotion;
    delete xmotion;

    if (prune_)
        OMPL_INFORM("%s: Created %u new states. Pruned %u states. Checked %u rewire options. %u goal states in tree.", getName().c_str(), statesGenerated_, statesPruned_, rewireTest_, goalMotion_.size());
    else
        OMPL_INFORM("%s: Created %u new states. Checked %u rewire options. %u goal states in tree.", getName().c_str(), statesGenerated_, rewireTest_, goalMotion_.size());

//    //STa test
//    output_file << improveSolutionBias_ << " "
//                << localBiasAttempt_ << " "
//                << localStateImprovementCount << " "
//                << --randomStateImprovementCount << " "
//                << statesGenerated_ << " "
//                << rewireTest_ << " "
//                << bestCost_ << "\n";
//    output_file.close();

    return feasibleSolution ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}


ompl::geometric::SafeBiRRTstar::GrowResult ompl::geometric::SafeBiRRTstar::extendTree(Motion* toMotion, TreeData& tree, Motion*& result)
{
    // Nearest neighbor
    Motion *nearest = tree->nearest(toMotion);
    return extendTree(toMotion, tree, result, nearest);
}

ompl::geometric::SafeBiRRTstar::GrowResult ompl::geometric::SafeBiRRTstar::extendTree(Motion* toMotion, TreeData& tree, Motion*& result, Motion* nearest)
{
//    //STa test
//    std::string homepath = getenv("HOME");
//    std::ofstream output_file((homepath + "/extendTree.txt").c_str(), std::ios::out | std::ios::app);

//    //STa temp
//    std::cout << "Enter extendTree \n";

    bool reach = true;
    std::vector<Motion*>       nbh;

    std::vector<base::SafetyCost>    costs;
    std::vector<base::SafetyCost>    incCosts;
    std::vector<std::size_t>   sortedCostIndices;
    std::vector<bool>   			exactCost;

    std::vector<int>           valid;

    result = new Motion(si_);
    si_->copyState(result->state, toMotion->state);

    // our functor for sorting nearest neighbors
    CostIndexCompare compareFn(costs, *opt_);

    // Compute the state to extend toward
    double d = si_->distance(nearest->state, result->state);

//    output_file << "d = " << d << "\n";

    // Truncate the random state to be no more than maxDistance_ from nearest neighbor
    if (d > maxDistance_)
    {
        si_->getStateSpace()->interpolate(nearest->state, result->state, maxDistance_ / d, result->state);
        d = maxDistance_;
        reach = false;

//        output_file << "d = " << d << "\n";
    }


    result->stateCost = safe_multi_opt_->safeStateCost(result->state);

    //If the sampled state is in collision, we continue
    if (result->stateCost.getCollisionWorld() || !ssvc_->isValidSelf(result->state))
    {
//        output_file << "return FAILED \n \n";
//        output_file.close();
        return FAILED;
    }

    base::SafetyCost heuristicCost = result->heuristicCost(safe_multi_opt_);

    //If the sampled state can't improve the solution, we continue
    if (safe_multi_opt_->isSafetyCostBetterThan(*bestSharedCost_, heuristicCost))
    {
//        output_file << "return FAILED \n \n";
//        output_file.close();
        return FAILED;
    }


    // Find nearby neighbors of the new motion - k-nearest RRT*
    unsigned int k = std::ceil(k_rrg_ * log((double)(tree->size() + 1)));
    tree->nearestK(result, k, nbh);

    rewireTest_ += nbh.size();


    // cache for distance computations
    //
    // Our cost caches only increase in size, so they're only
    // resized if they can't fit the current neighborhood
    if (costs.size() < nbh.size())
    {
        costs.resize(nbh.size());
        incCosts.resize(nbh.size());
        sortedCostIndices.resize(nbh.size());
        exactCost.resize(nbh.size());
    }

    // cache for motion validity (only useful in a symmetric space)
    //
    // Our validity caches only increase in size, so they're
    // only resized if they can't fit the current neighborhood
    if (valid.size() < nbh.size())
        valid.resize(nbh.size());
    std::fill(valid.begin(), valid.begin() + nbh.size(), 0);

    // Finding the nearest neighbor to connect to
    // By default, neighborhood states are sorted by cost, and collision checking
    // is performed in increasing order of cost

    // calculate all costs and distances
    for (std::size_t i = 0 ; i < nbh.size(); ++i)
    {
        //TODO: To be checked
        incCosts[i] = safe_multi_opt_->safeFastMotionCost(nbh[i]->state, result->state ,nbh[i]->stateCost, result->stateCost);
        costs[i] = safe_multi_opt_->safeCombineCosts(nbh[i]->cost, incCosts[i]);
        exactCost[i] = false;

//        output_file << "fast incCosts[" << i <<"] = " << incCosts[i] << "\n";
//        output_file << "fast costs[" << i <<"] = " << costs[i] << "\n";
    }

    // sort the nodes
    //
    // we're using index-value pairs so that we can get at
    // original, unsorted indices
    for (std::size_t i = 0; i < nbh.size(); ++i)
        sortedCostIndices[i] = i;
    std::sort(sortedCostIndices.begin(), sortedCostIndices.begin() + nbh.size(),
            compareFn);

    // collision check until a valid motion is found
    //
    // ASYMMETRIC CASE: it's possible that none of these
    // neighbors are valid. This is fine, because motion
    // already has a connection to the tree through
    // nmotion (with populated cost fields!).
    size_t cpt = 0;

    //STa TODO : Add a factor to favor nearer states?
    for (std::vector<std::size_t>::const_iterator i = sortedCostIndices.begin();
            i != sortedCostIndices.begin() + nbh.size();
            ++i)
    {
        incCosts[*i] = safe_multi_opt_->safeMotionCost(nbh[*i]->state, result->state);
        costs[*i] = safe_multi_opt_->safeCombineCosts(nbh[*i]->cost, incCosts[*i]);
        exactCost[*i]=true;
        bool check_motion;
        if (safe_multi_opt_->hasSafetyObjective())
            check_motion = (!incCosts[*i].getCollisionWorld()) && safe_motion_validator_->checkMotionSelfCCDiscrete(nbh[*i]->state, result->state, 1);
        else
            check_motion = safe_motion_validator_->checkMotionWorldIndividualLinks(nbh[*i]->state, result->state, travel_dist_limit_, fast_dist_) && safe_motion_validator_->checkMotionSelfCCDiscrete(nbh[*i]->state, result->state, 1);

//        output_file << "exact incCosts[" << *i <<"] = " << incCosts[*i] << "\n";
//        output_file << "exact costs[" << *i <<"] = " << costs[*i] << "\n";

        if (check_motion)
        {
//            output_file << "Parent = " << *i << "\n";

            valid[*i] = 1;
            //Is the exact cost improved?
            if(result->parent == NULL || safe_multi_opt_->isSafetyCostBetterThan(costs[*i], result->cost))
            {
                result->incCost = incCosts[*i];
                result->cost = costs[*i];
                result->parent = nbh[*i];
                result->root = nbh[*i]->root;
                result->isGoalTree = nbh[*i]->isGoalTree;
            }

            //If this is not the last neighbor but we are sure this is the best one, we can break
            if ((cpt >= nbh.size() - 1) || safe_multi_opt_->isSafetyCostBetterThan(result->cost, costs[*(i+1)]))
                break;
        }
        else valid[*i] = -1;
        cpt ++;
    }

    if(result->parent == NULL)
    {
//        output_file << "return FAILED \n \n";
//        output_file.close();
        return FAILED;
    }

    // add motion to the tree
    tree->add(result);
    result->parent->children.push_back(result);

    for (std::size_t i = 0; i < nbh.size(); ++i)
    {
        base::SafetyCost nbhIncCost;
        base::SafetyCost nbhNewCost;

        //STa : Check if the motion is invalid or if the neighbor belongs to the same branch
        if ((valid[i] == -1) || result->isParent(nbh[i]))
            continue;


        //STa : All objectives are symmetric with the exception of Path length obj
        // Fast cost
        nbhIncCost = safe_multi_opt_->safeMotionCostSymmetric(result->state, nbh[i]->state, incCosts[i]);
        nbhNewCost = safe_multi_opt_->safeCombineCosts(result->cost, nbhIncCost);

//        output_file << "fast nbhIncCost[" << i <<"] = " << nbhIncCost << "\n";
//        output_file << "fast nbhNewCost[" << i <<"] = " << nbhNewCost << "\n";

        if (!safe_multi_opt_->isSafetyCostBetterThan(nbhNewCost, nbh[i]->cost))
            continue;

        if(!exactCost[i])
        {
            //Exact cost
            nbhIncCost = safe_multi_opt_->safeMotionCost(result->state, nbh[i]->state);
            nbhNewCost = safe_multi_opt_->safeCombineCosts(result->cost, nbhIncCost);

            if (nbhIncCost.getCollisionWorld() || !safe_multi_opt_->isSafetyCostBetterThan(nbhNewCost, nbh[i]->cost))
                continue;

//            output_file << "exact nbhIncCost[" << i <<"] = " << nbhIncCost << "\n";
//            output_file << "exact nbhNewCost[" << i <<"] = " << nbhNewCost << "\n";
        }

        bool motionValid = safe_motion_validator_->checkMotionSelfCCDiscrete(result->state, nbh[i]->state, 1);
        if (!safe_multi_opt_->hasSafetyObjective() && motionValid)
            motionValid = safe_motion_validator_->checkMotionWorldIndividualLinks(result->state, nbh[i]->state, travel_dist_limit_, fast_dist_);

        if (motionValid)
        {
//            output_file << "Child = " << i << "\n";

            // Remove this node from its parent list
            removeFromParent (nbh[i]);

            // Add this node to the new parent
            nbh[i]->parent = result;
            nbh[i]->incCost = nbhIncCost;
            nbh[i]->cost = nbhNewCost;
            nbh[i]->parent->children.push_back(nbh[i]);

            // Update the costs of the node's children
            updateChildCosts(nbh[i]);
        }

    }

//    if (reach)
//        output_file << "return SUCCESS \n \n";
//    else
//        output_file << "return ADVANCED \n \n";
//    output_file.close();

    return reach ? SUCCESS : ADVANCED;

}

bool ompl::geometric::SafeBiRRTstar::connectTrees(Motion* nmotion, TreeData& tree, Motion* xmotion)
{
    // Get the nearest state to nmotion in tree (nmotion is NOT in tree)
    Motion *nearest = tree->nearest(nmotion);
    double dist = si_->distance(nearest->state, nmotion->state);

    // Do not attempt a connection if the trees are far apart
    if (dist > connectionRange_)
        return false;

    // Copy the resulting state into our scratch space
    si_->copyState(xmotion->state, nmotion->state);

    // Do not try to connect states directly.  Must chop up the
    // extension into segments, just in case one piece fails
    // the transition test
    GrowResult result;
    Motion* next = NULL;
    do
    {
        // Extend tree from nearest toward xmotion
        // Store the result into next
        // This function MAY trash xmotion
        result = extendTree(xmotion, tree, next, nearest);

        if (result == ADVANCED)
        {
            nearest = next;

//            std::cout << "result == ADVANCED \n";

            // xmotion may get trashed during extension, so we reload it here
            //    		si_->copyState(xmotion->state, nmotion->state);  // xmotion may get trashed during extension, so we reload it here
        }
    } while (result == ADVANCED);

    // Successful connection
    if (result == SUCCESS)
    {
//        std::cout << "result == SUCCESS \n";

        bool treeIsStart = tree == tStart_;
        Motion* startMotion = treeIsStart ? next : nmotion;
        Motion* goalMotion  = treeIsStart ? nmotion : next;

        // Make sure start-goal pair is valid
        if (pdef_->getGoal()->isStartGoalPairValid(startMotion->root, goalMotion->root))
        {

            treeConnection* tc = new treeConnection;

            tc->startTreeMotion = startMotion;
            tc->goalTreeMotion = goalMotion;

            tc->updateWholeMotionCost(safe_multi_opt_);
            tc->index = treeConnectionIndex_ ++;

            connection_[tc->index] = tc;

            //Each motion needs to know its connections with the other tree
            startMotion->connectionIndex.insert(tc->index);
            goalMotion->connectionIndex.insert(tc->index);

            //        	//STa temp
            //        	std::cout << "bestCost_ = " << bestCost_ << "\n";
            //        	std::cout << "tc->wholeMotionCost = " << tc->wholeMotionCost << "\n";

            if (safe_multi_opt_->isSafetyCostBetterThan(tc->wholeMotionCost, bestCost_))
            {
                bestCost_ = tc->wholeMotionCost;
                bestIndex_ = tc->index;
                checkForSolution_ = true;
            }

            return true;
        }
    }
//        else if (result == FAILED)
//        	std::cout << "result == FAILED \n";

    return false;
}


void ompl::geometric::SafeBiRRTstar::removeFromParent(Motion *m)
{
    if (m->parent)
    {
        for (std::vector<Motion*>::iterator it = m->parent->children.begin ();
                it != m->parent->children.end (); ++it)
        {
            if (*it == m)
            {
                m->parent->children.erase(it);
                break;
            }
        }
    }
//    //STa temp
//    else if (m->state == startMotion_->state)
//        std::cout << "\n \n \n ERROR : Start state deleted!! \n \n \n";
}

void ompl::geometric::SafeBiRRTstar::updateChildCosts(Motion *m)
{
    for (boost::unordered_set<size_t>::iterator it = m->connectionIndex.begin(); it != m->connectionIndex.end(); ++it)
    {
        treeConnection* tc = connection_.find(*it)->second;
        tc->updateWholeMotionCost(safe_multi_opt_);
        if (safe_multi_opt_->isSafetyCostBetterThan(tc->wholeMotionCost, bestCost_))
        {
            bestCost_ = tc->wholeMotionCost;
            bestIndex_ = tc->index;
            checkForSolution_ = true;
        }
    }

    for (std::size_t i = 0; i < m->children.size(); ++i)
    {
        m->children[i]->cost = safe_multi_opt_->safeCombineCosts(m->cost, m->children[i]->incCost);
        updateChildCosts(m->children[i]);
    }
}


void ompl::geometric::SafeBiRRTstar::freeMemory()
{
    if (tStart_)
    {
        std::vector<Motion*> motions;
        tStart_->list(motions);
        for (std::size_t i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }
    if (tGoal_)
    {
        std::vector<Motion*> motions;
        tGoal_->list(motions);
        for (std::size_t i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }
}

void ompl::geometric::SafeBiRRTstar::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (tStart_)
        tStart_->list(motions);

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (std::size_t i = 0 ; i < motions.size() ; ++i)
    {
        if (motions[i]->parent == NULL)
            data.addStartVertex(base::PlannerDataVertex(motions[i]->state));
        else
            data.addEdge(base::PlannerDataVertex(motions[i]->parent->state),
                    base::PlannerDataVertex(motions[i]->state));
    }
}


ompl::base::SafetyCost ompl::geometric::SafeBiRRTstar::heuristicCost(const base::State* state, const base::SafetyCost state_cost , const bool shortest) const
{
    base::SafetyCost costToCome, costToGo;
    if (shortest)
    {
        costToCome = safe_multi_opt_->safeCostToCome(state, state_cost);
        costToGo = safe_multi_opt_->safeCostToGo(state, state_cost);
    }
    else
    {
        costToCome = state_cost;
        costToGo = safe_multi_opt_->safeCostToGo(state, state_cost);
    }
    return safe_multi_opt_->safeCombineCosts(costToCome, costToGo);
}

//ompl::base::SafetyCost ompl::geometric::SafeBiRRTstar::heuristicCost(const Motion *m1, const Motion *m2 , const base::SafetyCost motionCost, const bool shortest) const
//{
//    base::SafetyCost costToCome, costToGo;
//    if (shortest)
//    {
//        if (m2->isGoalTree)
//        {
//            costToCome = safe_multi_opt_->safeCostToCome(m1->state, m1->heuristicCost);
//            costToGo = safe_multi_opt_->safeCostToGo(m2->state, m2->heuristicCost);
//        }
//        else
//        {
//            costToCome = safe_multi_opt_->safeCostToCome(m2->state, m2->heuristicCost);
//            costToGo = safe_multi_opt_->safeCostToGo(m1->state, m1->heuristicCost);
//        }
//    }
//    else
//    {
//        costToCome = m1->cost;
//        costToGo = m2->cost;
//    }
//    return safe_multi_opt_->safeCombineCosts(safe_multi_opt_->safeCombineCosts(costToCome, motionCost), costToGo);
//}

//STa
int ompl::geometric::SafeBiRRTstar::pruneTree(TreeData tree, bool isStartTree, const base::SafetyCost pruneTreeCost)
{
//    //STa temp
//    std::cout << "\n Enter SafeRRTstar::pruneTree \n";
//    std::cout << "tree_size = " << tree->size() << "\n";
//    std::cout << "pruneTreeCost : " << pruneTreeCost << "\n";

    const int tree_size = tree->size();
    pruneScratchSpace_.newTree.reserve(tree_size);
    pruneScratchSpace_.newTree.clear();
    pruneScratchSpace_.toBePruned.reserve(tree_size);
    pruneScratchSpace_.toBePruned.clear();
    pruneScratchSpace_.candidates.clear();

    //STa
    std::vector<size_t> goalToBePrunedIndex;

    //STa
    if (isStartTree)
        pruneScratchSpace_.candidates.push_back(startMotion_);
    else
        for (size_t i=0; i < goalMotion_.size(); ++i)
            pruneScratchSpace_.candidates.push_back(goalMotion_[i]);


    std::size_t j = 0;
    while (j != pruneScratchSpace_.candidates.size())
    {
//        //STa temp
//        std::cout << "pruneScratchSpace_.candidates.size() : " << pruneScratchSpace_.candidates.size() << "\n";

//        //STa temp
//        std::cout << "pruneTree1 is calling heuristicCost \n";

        Motion *candidate = pruneScratchSpace_.candidates[j];
        if (safe_multi_opt_->isSafetyCostBetterThan(pruneTreeCost, candidate->heuristicCost(safe_multi_opt_)))
        {
            pruneScratchSpace_.toBePruned.push_back(candidate);

            //Backup goal states that have to be deleted form goalMotion_ vector
            if (!isStartTree && j < goalMotion_.size())
            {
                goalToBePrunedIndex.push_back(j);
            }

//            //STa temp
//            std::cout << "pruneTree2 is calling heuristicCost \n";
//          std::cout << "toBePruned : " << candidate->heuristicCost(safe_multi_opt_) << "\n";

        }
        else
        {
            pruneScratchSpace_.newTree.push_back(candidate);
            pruneScratchSpace_.candidates.insert(pruneScratchSpace_.candidates.end(),
                    candidate->children.begin(), candidate->children.end());
        }
        j++;
    }

    // To create the new nn takes one order of magnitude in time more than just checking how many
    // states would be pruned. Therefore, only prune if it removes a significant amount of states.
    if ((double)pruneScratchSpace_.newTree.size() / tree_size < pruneStatesThreshold_)
    {
        for (std::size_t i = 0; i < pruneScratchSpace_.toBePruned.size(); ++i)
            deleteBranch(pruneScratchSpace_.toBePruned[i]);

        tree->clear();
        tree->add(pruneScratchSpace_.newTree);

        if (!isStartTree)
        {
            for (size_t i =0; i < goalToBePrunedIndex.size(); ++i)
            {
                //Delete goal states from goalMotion_ vector
                goalMotion_.erase(goalMotion_.begin() + goalToBePrunedIndex[i] - i);
            }
        }

//        //STa temp
//        std::cout << "new tree size = " << tree->size() << "\n";
//        std::cout << "Exit SafeRRTstar::pruneTree \n";
        return (tree_size - pruneScratchSpace_.newTree.size());
    }
//    //STa temp
//    std::cout << "Exit SafeRRTstar::pruneTree \n \n";
    return 0;
}

void ompl::geometric::SafeBiRRTstar::deleteBranch(Motion *motion)
{
    removeFromParent(motion);

    std::vector<Motion *>& toDelete = pruneScratchSpace_.candidates;
    toDelete.clear();
    toDelete.push_back(motion);

    while (!toDelete.empty())
    {
        Motion *mto_delete = toDelete.back();
        toDelete.pop_back();

        for(std::size_t i = 0; i < mto_delete->children.size(); ++i)
            toDelete.push_back(mto_delete->children[i]);

        //Remove the tree connections for the pruned motion. We also need to remove the connection index from the other tree.
        for (boost::unordered_set<size_t>::iterator it = mto_delete->connectionIndex.begin(); it != mto_delete->connectionIndex.end(); ++it)
        {
            if (mto_delete->isGoalTree)
                connection_.find(*it)->second->startTreeMotion->connectionIndex.erase(*it);
            else
                connection_.find(*it)->second->goalTreeMotion->connectionIndex.erase(*it);

            connection_.erase(*it);
        }


        si_->freeState(mto_delete->state);
        delete mto_delete;
    }
}

