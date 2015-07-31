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
iterations_(0),
bestCost_(base::SafetyCost()),
improveSolutionBias_(0.05)
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


void ompl::geometric::SafeBiRRTstar::treeConnexion::updateWholeMotionCost(const base::SafeMultiOptimizationObjective* safe_multi_opt)
{
    wholeMotionCost = safe_multi_opt->safeCombineCosts(startTreeMotion->cost, goalTreeMotion->cost);
}

std::vector<ompl::base::State*> ompl::geometric::SafeBiRRTstar::treeConnexion::getPath()
{
    std::vector<base::State*> solutionStates;
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
    //	//STa test
    //	std::string homepath = getenv("HOME");
    //	std::ofstream output_file((homepath + "/safebirrtstar.txt").c_str(), std::ios::out | std::ios::app);
    //	ompl::time::point init = ompl::time::now();
    //	ompl::time::duration dur;

    checkValidity();
    size_t  goalMotionCount = 0;

    safe_multi_opt_->setGoal(pdef_->getGoal().get());

    while (const base::State *startState = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, startState);
        motion->root = motion->state;
        motion->cost = safe_multi_opt_->safeStateCost(motion->state);
        motion->heuristicCost = costToGo(motion->state, motion->cost);
        motion->isGoalTree = false;
        tStart_->add(motion);
        startMotion_ = motion;
    }

    safe_multi_opt_->setStart(startMotion_->state);

    if (tStart_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    // Do the same for the goal but only once
    const base::State *goalState = pis_.nextGoal(ptc);
    if (goalState)
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, goalState);
        motion->root = motion->state;
        motion->cost = safe_multi_opt_->safeStateCost(motion->state);
        motion->heuristicCost = costToGo(motion->state, motion->cost);
        motion->isGoalTree = true;
        tGoal_->add(motion);
        goalMotionCount++;
    }
    else
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

    TreeData tree = tStart_;
    TreeData otherTree = tGoal_;

    while (ptc == false)
    {
        iterations_++;
        checkForSolution_ = false;

        // Add available goal states
        while (pis_.haveMoreGoalStates())
        {
            const base::State *goalState = pis_.nextGoal();

            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, goalState);
            motion->root = motion->state;
            motion->cost = safe_multi_opt_->safeStateCost(motion->state);
            motion->heuristicCost = costToGo(motion->state, motion->cost);
            motion->isGoalTree = true;
            tGoal_->add(motion);
            goalMotionCount++;

        }

        // sample random state
        sampler_->sampleUniform(rstate);


        statesGenerated_++;

        Motion* result; // the motion that gets added in extendTree
        if (extendTree(rmotion, tree, result) != FAILED) // we added something new to the tree
        {
            //			//STa temp
            //			std::cout << "extendTree returned true \n";

            // Try to connect the other tree to the node we just added
            if (connectTrees(result, otherTree, xmotion))
            {
                //				//STa temp
                //				std::cout << "connectTrees returned true \n";

                feasibleSolution = true;
            }
            //			else
            //				//STa temp
            //				std::cout << "connectTrees returned false \n";
        }
        //		else
        //			//STa temp
        //			std::cout << "extendTree returned false \n";


        // Checking for solution or iterative improvement
        if (checkForSolution_)
        {
            //STa temp
            std::cout << "bestCost_ = " << bestCost_ << "\n";
            //			//STa test
            //			dur = ompl::time::now() - init;
            //			output_file <<  bestCost_ << ompl::time::now() << "\n";


            sufficientlyShort = safe_multi_opt_->isSafetySatisfied(bestCost_);

            if (sufficientlyShort)
            {
                break;
            }
        }
        std::swap(tree, otherTree);
    }

    if (feasibleSolution)
    {
        //		//STa temp
        //		std::cout << "bestIndex_ = " << bestIndex_ << "\n";
        //		std::cout << "connexion_.size()  = " << connexion_.size() << "\n";

        ptc.terminate();
        // construct the solution path
        std::vector<base::State*> mpath = connexion_[bestIndex_]->getPath();


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

    OMPL_INFORM("%s: Created %u new states. Checked %u rewire options. %u goal states in tree.", getName().c_str(), statesGenerated_, rewireTest_, goalMotionCount);

    //STa test
    //	output_file << "\n";
    //	output_file.close();

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
    //	std::string homepath = getenv("HOME");
    //	std::ofstream output_file((homepath + "/extendTree.txt").c_str(), std::ios::out | std::ios::app);

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


    base::SafetyCost state_cost = safe_multi_opt_->safeStateCost(result->state);
    result->heuristicCost = costToGo(result->state, state_cost);

    //    output_file << "state_cost = " << state_cost << "\n";
    //    output_file << "heuristicCost = " << result->heuristicCost << "\n";

    //If the sampled state can't improve the solution, we continue
    if (result->heuristicCost.getCollisionWorld() || !ssvc_->isValidSelf(result->state) || safe_multi_opt_->isSafetyCostBetterThan(bestCost_, result->heuristicCost))
    {
        //    	output_file << "return FAILED \n \n";
        //    	output_file.close();
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
        incCosts[i] = safe_multi_opt_->safeFastMotionCost(nbh[i]->state, result->state ,nbh[i]->heuristicCost, result->heuristicCost);
        costs[i] = safe_multi_opt_->safeCombineCosts(nbh[i]->cost, incCosts[i]);
        exactCost[i] = false;

        //    	output_file << "fast incCosts[" << i <<"] = " << incCosts[i] << "\n";
        //    	output_file << "fast costs[" << i <<"] = " << costs[i] << "\n";
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

    //STa TODO : Add a factor to favor nearer states
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

        //    	output_file << "exact incCosts[" << *i <<"] = " << incCosts[*i] << "\n";
        //    	output_file << "exact costs[" << *i <<"] = " << costs[*i] << "\n";

        if (check_motion)
        {
            //	    	output_file << "Parent = " << *i << "\n";

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

            if ((cpt >= nbh.size() - 1) || safe_multi_opt_->isSafetyCostBetterThan(result->cost, costs[*(i+1)]))
                break;
        }
        else valid[*i] = -1;
        cpt ++;
    }

    if(result->parent == NULL)
    {
        //    	output_file << "return FAILED \n \n";
        //    	output_file.close();
        return FAILED;
    }

    // add motion to the tree
    tree->add(result);
    result->parent->children.push_back(result);

    for (std::size_t i = 0; i < nbh.size(); ++i)
    {
        base::SafetyCost nbhIncCost;
        base::SafetyCost nbhNewCost;

        bool motionValid;

        //STa : Check if the neighbor belongs to the same branch
        if (!result->isParent(nbh[i]))
        {
            //STa : All objectives are symmetric with the exception of Path length obj
            // Fast cost
            nbhIncCost = safe_multi_opt_->safeMotionCostSymmetric(result->state, nbh[i]->state, incCosts[i]);
            nbhNewCost = safe_multi_opt_->safeCombineCosts(result->cost, nbhIncCost);

            //	    	output_file << "fast nbhIncCost[" << i <<"] = " << nbhIncCost << "\n";
            //	    	output_file << "fast nbhNewCost[" << i <<"] = " << nbhNewCost << "\n";

            if (safe_multi_opt_->isSafetyCostBetterThan(nbhNewCost, nbh[i]->cost))
            {
                if (valid[i] == 0)
                {
                    if(!exactCost[i])
                    {
                        //Exact cost
                        nbhIncCost = safe_multi_opt_->safeMotionCost(result->state, nbh[i]->state);
                        nbhNewCost = safe_multi_opt_->safeCombineCosts(result->cost, nbhIncCost);

                        //				    	output_file << "exact nbhIncCost[" << i <<"] = " << nbhIncCost << "\n";
                        //				    	output_file << "exact nbhNewCost[" << i <<"] = " << nbhNewCost << "\n";
                    }

                    if (exactCost[i] || safe_multi_opt_->isSafetyCostBetterThan(nbhNewCost, nbh[i]->cost))
                    {
                        if (safe_multi_opt_->hasSafetyObjective())
                            motionValid = (!nbhIncCost.getCollisionWorld()) && safe_motion_validator_->checkMotionSelfCCDiscrete(result->state, nbh[i]->state, 1);
                        else
                            motionValid = safe_motion_validator_->checkMotionWorldIndividualLinks(result->state, nbh[i]->state, travel_dist_limit_, fast_dist_) && safe_motion_validator_->checkMotionSelfCCDiscrete(result->state, nbh[i]->state, 1);

                        if (motionValid)
                        {
                            //							output_file << "Child = " << i << "\n";

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
                }
            }
        }
    }

    //	if (reach)
    //		output_file << "return SUCCESS \n \n";
    //	else
    //		output_file << "return ADVANCED \n \n";
    //	output_file.close();

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

            //    		std::cout << "result == ADVANCED \n";

            // xmotion may get trashed during extension, so we reload it here
            //    		si_->copyState(xmotion->state, nmotion->state);  // xmotion may get trashed during extension, so we reload it here
        }
    } while (result == ADVANCED);

    // Successful connection
    if (result == SUCCESS)
    {
        //    	std::cout << "result == SUCCESS \n";

        bool treeIsStart = tree == tStart_;
        Motion* startMotion = treeIsStart ? next : nmotion;
        Motion* goalMotion  = treeIsStart ? nmotion : next;

        // Make sure start-goal pair is valid
        if (pdef_->getGoal()->isStartGoalPairValid(startMotion->root, goalMotion->root))
        {

            treeConnexion* tc = new treeConnexion;

            tc->startTreeMotion = startMotion;
            tc->goalTreeMotion = goalMotion;

            tc->updateWholeMotionCost(safe_multi_opt_);
            tc->index = connexion_.size();

            connexion_.push_back(tc);

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
    //    else if (result == FAILED)
    //    	std::cout << "result == FAILED \n";

    return false;
}


void ompl::geometric::SafeBiRRTstar::removeFromParent(Motion *m)
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

void ompl::geometric::SafeBiRRTstar::updateChildCosts(Motion *m)
{
    for (std::size_t i = 0; i < m->connexion.size(); ++i)
    {
        m->connexion[i]->updateWholeMotionCost(safe_multi_opt_);
        if (safe_multi_opt_->isSafetyCostBetterThan(m->connexion[i]->wholeMotionCost, bestCost_))
        {
            bestCost_ = m->connexion[i]->wholeMotionCost;
            bestIndex_ = m->connexion[i]->index;
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


ompl::base::SafetyCost ompl::geometric::SafeBiRRTstar::costToGo(const base::State* state, const base::SafetyCost state_cost , const bool shortest) const
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

ompl::base::SafetyCost ompl::geometric::SafeBiRRTstar::costToGo(const Motion *m1, const Motion *m2 , const base::SafetyCost motionCost, const bool shortest) const
{
    base::SafetyCost costToCome, costToGo;
    if (shortest)
    {
        if (m2->isGoalTree)
        {
            costToCome = safe_multi_opt_->safeCostToCome(m1->state, m1->heuristicCost);
            costToGo = safe_multi_opt_->safeCostToGo(m2->state, m2->heuristicCost);
        }
        else
        {
            costToCome = safe_multi_opt_->safeCostToCome(m2->state, m2->heuristicCost);
            costToGo = safe_multi_opt_->safeCostToGo(m1->state, m1->heuristicCost);
        }
    }
    else
    {
        costToCome = m1->cost;
        costToGo = m2->cost;
    }
    return safe_multi_opt_->safeCombineCosts(safe_multi_opt_->safeCombineCosts(costToCome, motionCost), costToGo);
}

