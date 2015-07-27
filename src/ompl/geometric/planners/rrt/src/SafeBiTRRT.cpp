/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
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

/* Author: Ryan Luna */

#include <limits>

#include "ompl/geometric/planners/rrt/SafeBiTRRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/MechanicalWorkOptimizationObjective.h"
#include "ompl/tools/config/MagicConstants.h"
#include "ompl/tools/config/SelfConfig.h"

//STa
#include "ompl/base/objectives/SafetyObjective.h"
#include "ompl/base/objectives/ManipulabilityObjective.h"
#include "ompl/base/objectives/JointLimitsObjective.h"
#include "ompl/base/objectives/SafePathLengthOptimizationObjective.h"
#include <boost/thread.hpp>


//STa test
#include <fstream>

ompl::geometric::SafeBiTRRT::SafeBiTRRT(const base::SpaceInformationPtr &si) : base::Planner(si, "SafeBiTRRT")
{
    specs_.approximateSolutions = false;
    specs_.directed = true;

    maxDistance_ = 0.0; // set in setup()
    connectionPoint_ = std::make_pair<Motion*, Motion*>(NULL, NULL);

    Planner::declareParam<double>("range", this, &SafeBiTRRT::setRange, &SafeBiTRRT::getRange, "0.:1.:10000.");

    // SafeBiTRRT Specific Variables
    frontierThreshold_ = 0.0; // set in setup()
    setTempChangeFactor(0.1); // how much to increase the temp each time
    initTemperature_ = 100; // where the temperature starts out
    frontierNodeRatio_ = 0.1; // 1/10, or 1 non-frontier for every 10 frontier
    path_length_weight_ = 1 / si_->getMaximumExtent();

    //STa
	fast_dist_ = true;
	travel_dist_limit_=0.01;

    Planner::declareParam<double>("temp_change_factor", this, &SafeBiTRRT::setTempChangeFactor, &SafeBiTRRT::getTempChangeFactor,"0.:.1:1.");
    Planner::declareParam<double>("init_temperature", this, &SafeBiTRRT::setInitTemperature, &SafeBiTRRT::getInitTemperature);
    Planner::declareParam<double>("frontier_threshold", this, &SafeBiTRRT::setFrontierThreshold, &SafeBiTRRT::getFrontierThreshold);
    Planner::declareParam<double>("frontier_node_ratio", this, &SafeBiTRRT::setFrontierNodeRatio, &SafeBiTRRT::getFrontierNodeRatio);
//    Planner::declareParam<base::SafetyCost>("cost_threshold", this, &SafeBiTRRT::setCostThreshold, &SafeBiTRRT::getCostThreshold);
}

ompl::geometric::SafeBiTRRT::~SafeBiTRRT()
{
    freeMemory();
}

void ompl::geometric::SafeBiTRRT::freeMemory()
{
    std::vector<Motion*> motions;

    if (tStart_)
    {
        tStart_->list(motions);
        for (unsigned int i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }

    if (tGoal_)
    {
        tGoal_->list(motions);
        for (unsigned int i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }
}

void ompl::geometric::SafeBiTRRT::clear()
{
    Planner::clear();
    freeMemory();
    if (tStart_)
        tStart_->clear();
    if (tGoal_)
        tGoal_->clear();
    connectionPoint_ = std::make_pair<Motion*, Motion*>(NULL, NULL);

    // TRRT specific variables
    temp_ = initTemperature_;
    nonfrontierCount_ = 1;
    frontierCount_ = 1; // init to 1 to prevent division by zero error
    if (opt_)
        bestCost_ = worstCost_ = safe_multi_opt_->safeIdentityCost();
}

void ompl::geometric::SafeBiTRRT::setup()
{
	if (pdef_)
	{
		Planner::setup();
		tools::SelfConfig sc(si_, getName());

		// Configuring the range of the planner
		if (maxDistance_ < std::numeric_limits<double>::epsilon())
		{
			sc.configurePlannerRange(maxDistance_);
			maxDistance_ *= magic::COST_MAX_MOTION_LENGTH_AS_SPACE_EXTENT_FRACTION;
		}

		// Configuring nearest neighbors structures for the planning trees
		if (!tStart_)
			tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
		if (!tGoal_)
			tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
		tStart_->setDistanceFunction(boost::bind(&SafeBiTRRT::distanceFunction, this, _1, _2));
		tGoal_->setDistanceFunction(boost::bind(&SafeBiTRRT::distanceFunction, this, _1, _2));

		//STa
		safe_motion_validator_ = new ompl::base::SafeMotionValidator(si_.get());
		getOptimalSafetyObjective();
		ssvc_ = static_cast<ompl::base::SafeStateValidityChecker*>(si_->getStateValidityChecker().get());
		costThreshold_ = safe_multi_opt_->safeInfiniteCost();

		// Set the threshold that decides if a new node is a frontier node or non-frontier node
		if (frontierThreshold_ < std::numeric_limits<double>::epsilon())
		{
			frontierThreshold_ = si_->getMaximumExtent() * 0.01;
			OMPL_DEBUG("%s: Frontier threshold detected to be %lf", getName().c_str(), frontierThreshold_);
		}

		// initialize TRRT specific variables
		temp_ = initTemperature_;
		nonfrontierCount_ = 1;
		frontierCount_ = 1; // init to 1 to prevent division by zero error
		worstCost_ = safe_multi_opt_->safeIdentityCost();
		bestCost_ = safe_multi_opt_->safeInfiniteCost();

		connectionRange_ = 10.0 * si_->getStateSpace()->getLongestValidSegmentLength();

	}
	else
	{
		OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
		setup_ = false;
	}
}

void ompl::geometric::SafeBiTRRT::getOptimalSafetyObjective()
{

	safe_multi_opt_ = new ompl::base::SafeMultiOptimizationObjective(si_);
	ompl::base::OptimizationObjectivePtr lengthObj(new ompl::base::SafePathLengthOptimizationObjective(si_, &safe_multi_opt_->getStart() ,&safe_multi_opt_->getGoal()));
	ompl::base::OptimizationObjectivePtr safetyObj(new ompl::base::SafetyObjective(si_, safe_motion_validator_, fast_dist_, travel_dist_limit_));
	ompl::base::OptimizationObjectivePtr jointLimitsObj(new ompl::base::JointLimitsObjective(si_));
	ompl::base::OptimizationObjectivePtr manipulabilityObj(new ompl::base::ManipulabilityObjective(si_));
	safe_multi_opt_->addObjective(lengthObj, 0.4, "length");
	safe_multi_opt_->addObjective(safetyObj, 0.4, "safety");
	safe_multi_opt_->addObjective(jointLimitsObj, 0.1, "joint");
//	safe_multi_opt_->addObjective(manipulabilityObj, 0.1, "manipulability");
	safe_multi_opt_->NormalizeWeight();

	//Safe RRT*
	opt_ = ompl::base::OptimizationObjectivePtr(safe_multi_opt_);

}

ompl::geometric::SafeBiTRRT::Motion* ompl::geometric::SafeBiTRRT::addMotion(const base::State* state, TreeData& tree, Motion* parent)
{
    Motion *motion = new Motion(si_);
    si_->copyState(motion->state, state);
    motion->cost = costToGo(motion);
    motion->parent = parent;
    motion->root = parent ? parent->root : NULL;

    if (safe_multi_opt_->isSafetyCostBetterThan(motion->cost, bestCost_)) // motion->cost is better than the existing best
        bestCost_ = motion->cost;
    if (safe_multi_opt_->isSafetyCostBetterThan(worstCost_, motion->cost)) // motion->cost is worse than the existing worst
        worstCost_ = motion->cost;

    // Add start motion to the tree
    tree->add(motion);
    return motion;
}

bool ompl::geometric::SafeBiTRRT::transitionTest(Motion* nearest, Motion* toMotion)
{
//	//STa temp
//	std::cout << "Enter transitionTest \n";
//	std::cout << "toMotion->cost = " << toMotion->cost << "\n";
//	std::cout << "costThreshold_ = " << costThreshold_ << "\n \n";

    // Disallow any cost that is not better than the cost threshold
    if (!safe_multi_opt_->isSafetyCostBetterThan(toMotion->cost, costThreshold_))
        return false;

    double dmech_work = safe_multi_opt_->safeMotionMechanicalWork(nearest->cost, toMotion->cost).value();
    dmech_work += path_length_weight_ * si_->distance(nearest->state, toMotion->state);

//    //STa temp
//    std::cout << "mech_work = " << dmech_work << "\n";

    base::Cost mech_work = base::Cost(dmech_work);
    // Always accept if the cost is near or below zero
    if (mech_work.value() < 1e-4)
        return true;

    double dCost = mech_work.value();
    double transitionProbability = exp(-dCost / temp_);

//    //STa temp
//    std::cout << "transitionProbability = " << transitionProbability << "\n";
//    std::cout << "bestCost_ = " << bestCost_ << "\n";
//    std::cout << "worstCost_ = " << worstCost_ << "\n";

    if (transitionProbability > 0.5)
    {
        double costRange = safe_multi_opt_->safeMotionMechanicalWork(worstCost_, bestCost_).value();

//        //STa temp
//        std::cout << "costRange = " << costRange << "\n";

        if (fabs(costRange) > 1e-4) // Do not divide by zero
            // Successful transition test.  Decrease the temperature slightly
            temp_ /= exp(dCost / (0.1 * costRange));

        return true;
    }


    // The transition failed.  Increase the temperature (slightly)
    temp_ *= tempChangeFactor_;
//    //STa temp
//    std::cout << "temp_ = " << temp_ << "\n \n";
    return false;
}

bool ompl::geometric::SafeBiTRRT::minExpansionControl(double dist)
{
    if (dist > frontierThreshold_)  // Exploration
    {
        ++frontierCount_;
        return true;
    }
    else  // Refinement
    {
        // Check the current ratio first before accepting it
        if ((double)nonfrontierCount_ / (double)frontierCount_ > frontierNodeRatio_)
            return false;

        ++nonfrontierCount_;
        return true;
    }
}

ompl::geometric::SafeBiTRRT::GrowResult ompl::geometric::SafeBiTRRT::extendTree(Motion* nearest, TreeData& tree, Motion* toMotion, Motion*& result)
{
//	//STa temp
//	std::cout << "Enter extendTree \n";

    bool reach = true;

    // Compute the state to extend toward
    double d = si_->distance(nearest->state, toMotion->state);
    // Truncate the random state to be no more than maxDistance_ from nearest neighbor
    if (d > maxDistance_)
    {
        si_->getStateSpace()->interpolate(nearest->state, toMotion->state, maxDistance_ / d, toMotion->state);
        d = maxDistance_;
        reach = false;
    }

    toMotion->cost = safe_multi_opt_->safeStateCost(toMotion->state);

    // Validating the motion
    // If we are in the goal tree, we validate the motion in reverse
    // si_->checkMotion assumes that the first argument is valid, so we must check this explicitly
    // If the motion is valid, check the probabilistic transition test and the
    // expansion control to ensure high quality nodes are added.
    bool validMotion = (tree == tStart_ ? si_->checkMotion(nearest->state, toMotion->state) :
                        si_->isValid(toMotion->state) && si_->checkMotion(toMotion->state, nearest->state)) &&
                        transitionTest(nearest, toMotion) &&
                       minExpansionControl(d);

    if (validMotion)
    {
        result = addMotion(toMotion->state, tree, nearest);

//    	//STa temp
//    	std::cout << "Exit extendTree \n";

        return reach ? SUCCESS : ADVANCED;
    }

//	//STa temp
//	std::cout << "Exit extendTree \n";

    return FAILED;
}

ompl::geometric::SafeBiTRRT::GrowResult ompl::geometric::SafeBiTRRT::extendTree(Motion* toMotion, TreeData& tree, Motion*& result)
{
    // Nearest neighbor
    Motion *nearest = tree->nearest(toMotion);
    return extendTree(nearest, tree, toMotion, result);
}

bool ompl::geometric::SafeBiTRRT::connectTrees(Motion* nmotion, TreeData& tree, Motion* xmotion)
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
        result = extendTree(nearest, tree, xmotion, next);

        if (result == ADVANCED)
        {
            nearest = next;

            // xmotion may get trashed during extension, so we reload it here
            si_->copyState(xmotion->state, nmotion->state);  // xmotion may get trashed during extension, so we reload it here
        }
    } while (result == ADVANCED);

    // Successful connection
    if (result == SUCCESS)
    {
        bool treeIsStart = tree == tStart_;
        Motion* startMotion = treeIsStart ? next : nmotion;
        Motion* goalMotion  = treeIsStart ? nmotion : next;

        // Make sure start-goal pair is valid
        if (pdef_->getGoal()->isStartGoalPairValid(startMotion->root, goalMotion->root))
        {
            // Since we have connected, nmotion->state and next->state have the same value
            // We need to check one of their parents to avoid a duplicate state in the solution path
            // One of these must be true, since we do not ever attempt to connect start and goal directly.
            if (startMotion->parent)
                startMotion = startMotion->parent;
            else
                goalMotion = goalMotion->parent;

            connectionPoint_ = std::make_pair(startMotion, goalMotion);
            return true;
        }
    }

    return false;
}

ompl::base::PlannerStatus ompl::geometric::SafeBiTRRT::solve(const base::PlannerTerminationCondition &ptc)
{
	//STa test
	std::string homepath = getenv("HOME");
	std::ofstream output_file((homepath + "/safebitrrt.txt").c_str(), std::ios::out | std::ios::app);

    // Basic error checking
    checkValidity();

    // Goal information
    base::Goal                 *goal = pdef_->getGoal().get();
    base::GoalSampleableRegion *gsr  = dynamic_cast<base::GoalSampleableRegion*>(goal);

    if (!gsr)
    {
        OMPL_ERROR("%s: Goal object does not derive from GoalSampleableRegion", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

	while (!dynamic_cast<base::GoalSampleableRegion*>(goal)->canSample() && ptc == false)
	{
//		boost::this_thread::sleep(ompl::time::seconds(.0001));
	}

	//STa
	safe_multi_opt_->setGoal(goal);

    // Loop through the (valid) input states and add them to the start tree
    while (const base::State *state = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, state);
        motion->cost = costToGo(motion);
        motion->root = motion->state; // this state is the root of a tree

        if (tStart_->size() == 0)  // do not overwrite best/worst from a prior call to solve
            worstCost_ = bestCost_ = motion->cost;

        // Add start motion to the tree
        tStart_->add(motion);

		//STa TODO: consider the case of multiple start states
		safe_multi_opt_->setStart(motion->state);
    }

    if (tStart_->size() == 0)
    {
        OMPL_ERROR("%s: Start tree has no valid states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    // Do the same for the goal tree, if it is empty, but only once
    if (tGoal_->size() == 0)
    {
    	const base::State *state = pis_.nextGoal(ptc);
    	if (state)
    	{
    		Motion* motion = addMotion(state, tGoal_);
    		motion->root = motion->state; // this state is the root of a tree
    	}
    }

    if (tGoal_->size() == 0)
    {
        OMPL_ERROR("%s: Goal tree has no valid states!", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }


    OMPL_INFORM("%s: Planning started with %d states already in datastructure", getName().c_str(), (int)(tStart_->size() + tGoal_->size()));

    base::StateSamplerPtr sampler = si_->allocStateSampler();

    Motion   *rmotion   = new Motion(si_);
    base::State *rstate = rmotion->state;

    Motion   *xmotion   = new Motion(si_);
    base::State *xstate = xmotion->state;

    TreeData tree = tStart_;
    TreeData otherTree = tGoal_;

    bool solved = false;

    // Planning loop
    while (ptc == false)
    {
        // Check if there are more goal states
        if (pis_.getSampledGoalsCount() < tGoal_->size() / 2)
        {
            if (const base::State *state = pis_.nextGoal())
            {
                Motion* motion = addMotion(state, tGoal_);
                motion->root = motion->state; // this state is the root of a tree
            }
        }

        // Sample a state uniformly at random
        sampler->sampleUniform(rstate);

        Motion* result; // the motion that gets added in extendTree

        if (extendTree(rmotion, tree, result) != FAILED) // we added something new to the tree
        {
            // Try to connect the other tree to the node we just added
            if (connectTrees(result, otherTree, xmotion))
            {
                // The trees have been connected.  Construct the solution path
                Motion *solution = connectionPoint_.first;
                std::vector<Motion*> mpath1;
                while (solution != NULL)
                {
                    mpath1.push_back(solution);
                    solution = solution->parent;
                }
                solution = connectionPoint_.second;
                std::vector<Motion*> mpath2;
                while (solution != NULL)
                {
                    mpath2.push_back(solution);
                    solution = solution->parent;
                }
                PathGeometric *path = new PathGeometric(si_);
                path->getStates().reserve(mpath1.size() + mpath2.size());
                for (int i = mpath1.size() - 1 ; i >= 0 ; --i)
                    path->append(mpath1[i]->state);
                for (unsigned int i = 0 ; i < mpath2.size() ; ++i)
                    path->append(mpath2[i]->state);

                //STa temp
//                path->printAsMatrix(std::cout);
                output_file << getPathSafetyCost(path) << "\n";

                pdef_->addSolutionPath(base::PathPtr(path), false, 0.0, getName());
                solved = true;
                break;
            }
        }

        std::swap(tree, otherTree);
    }

    si_->freeState(rstate);
    si_->freeState(xstate);
    delete rmotion;
    delete xmotion;

    //STa test
    output_file.close();

    OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(), tStart_->size(), tGoal_->size());
    return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

//STa
ompl::base::SafetyCost ompl::geometric::SafeBiTRRT::getPathSafetyCost(PathGeometric* pg)
{
	if (pg->getStates().empty()) return safe_multi_opt_->safeIdentityCost();
	// Compute path cost by accumulating the cost along the path
	base::SafetyCost cost(safe_multi_opt_->safeStateCost(pg->getStates().front()));

	for (std::size_t i = 1; i < pg->getStateCount(); ++i)
	{
		base::SafetyCost motion_cost = safe_multi_opt_->safeMotionCost(pg->getState(i - 1), pg->getState(i));
		//STa temp
		std::cout << "Motion cost = " << motion_cost << "\n";
		cost = safe_multi_opt_->safeCombineCosts(cost, motion_cost);
	}
	//STa temp
	std::cout << "cost = " << cost << "\n \n";

	return cost;
}

void ompl::geometric::SafeBiTRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (tStart_)
        tStart_->list(motions);
    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
        if (motions[i]->parent == NULL)
            data.addStartVertex(base::PlannerDataVertex(motions[i]->state, 1));
        else
        {
            data.addEdge(base::PlannerDataVertex(motions[i]->parent->state, 1),
                         base::PlannerDataVertex(motions[i]->state, 1));
        }
    }

    motions.clear();
    if (tGoal_)
        tGoal_->list(motions);
    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
        if (motions[i]->parent == NULL)
            data.addGoalVertex(base::PlannerDataVertex(motions[i]->state, 2));
        else
        {
            // The edges in the goal tree are reversed to be consistent with start tree
            data.addEdge(base::PlannerDataVertex(motions[i]->state, 2),
                         base::PlannerDataVertex(motions[i]->parent->state, 2));
        }
    }

    // Add the edge connecting the two trees
    data.addEdge(data.vertexIndex(connectionPoint_.first->state), data.vertexIndex(connectionPoint_.second->state));
}

ompl::base::SafetyCost ompl::geometric::SafeBiTRRT::costToGo(const Motion *motion) const
{
	base::SafetyCost costToCome, costToGo;
	costToCome = safe_multi_opt_->safeStateCost(motion->state);
	costToGo = safe_multi_opt_->safeCostToGo(motion->state, costToCome);
	return safe_multi_opt_->safeCombineCosts(costToCome, costToGo);
}


