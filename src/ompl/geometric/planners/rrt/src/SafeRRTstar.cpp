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

/* Authors: Sonny Tarbouriech */

#include "ompl/geometric/planners/rrt/SafeRRTstar.h"
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


ompl::geometric::SafeRRTstar::SafeRRTstar(const base::SpaceInformationPtr &si) :
base::Planner(si, "SafeRRTstar"),
goalBias_(0.05),
maxDistance_(0.0),
delayCC_(true),
lastGoalMotion_(NULL),
prune_(false),
pruneStatesThreshold_(0.95),
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

	Planner::declareParam<double>("range", this, &SafeRRTstar::setRange, &SafeRRTstar::getRange, "0.:1.:10000.");
	Planner::declareParam<double>("goal_bias", this, &SafeRRTstar::setGoalBias, &SafeRRTstar::getGoalBias, "0.:.05:1.");
	Planner::declareParam<bool>("delay_collision_checking", this, &SafeRRTstar::setDelayCC, &SafeRRTstar::getDelayCC, "0,1");
	Planner::declareParam<bool>("prune", this, &SafeRRTstar::setPrune, &SafeRRTstar::getPrune, "0,1");
	Planner::declareParam<double>("prune_states_threshold", this, &SafeRRTstar::setPruneStatesImprovementThreshold, &SafeRRTstar::getPruneStatesImprovementThreshold, "0.:.01:1.");

	addPlannerProgressProperty("iterations INTEGER",
			boost::bind(&SafeRRTstar::getIterationCount, this));
	addPlannerProgressProperty("best cost REAL",
			boost::bind(&SafeRRTstar::getBestCost, this));
}

ompl::geometric::SafeRRTstar::~SafeRRTstar()
{
	freeMemory();
}


void ompl::geometric::SafeRRTstar::setup()
{
	Planner::setup();
	tools::SelfConfig sc(si_, getName());
	sc.configurePlannerRange(maxDistance_);
	if (!si_->getStateSpace()->hasSymmetricDistance() || !si_->getStateSpace()->hasSymmetricInterpolate())
	{
		OMPL_WARN("%s requires a state space with symmetric distance and symmetric interpolation.", getName().c_str());
	}

	if (!nn_)
		nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
	nn_->setDistanceFunction(boost::bind(&SafeRRTstar::distanceFunction, this, _1, _2));

	if (pdef_)
	{
		//STa
		safe_motion_validator_ = new ompl::base::SafeMotionValidator(si_.get());
		getOptimalSafetyObjective();
		ssvc_ = static_cast<ompl::base::SafeStateValidityChecker*>(si_->getStateValidityChecker().get());

	}
	else
	{
		OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
		setup_ = false;
	}

}

void ompl::geometric::SafeRRTstar::getOptimalSafetyObjective()
{
    safe_multi_opt_ = new ompl::base::SafeMultiOptimizationObjective(si_);
    ompl::base::OptimizationObjectivePtr lengthObj(new ompl::base::SafePathLengthOptimizationObjective(si_, &safe_multi_opt_->getStart() ,&safe_multi_opt_->getGoal()));
    ompl::base::OptimizationObjectivePtr safetyObj(new ompl::base::SafetyObjective(si_, safe_motion_validator_, fast_dist_, travel_dist_limit_));
    ompl::base::OptimizationObjectivePtr jointLimitsObj(new ompl::base::JointLimitsObjective(si_));
    ompl::base::OptimizationObjectivePtr manipulabilityObj(new ompl::base::ManipulabilityObjective(si_));
    ompl::base::OptimizationObjectivePtr humanAwarenessObj(new ompl::base::HumanAwarenessObjective(si_));
    safe_multi_opt_->addObjective(lengthObj, 1, "length");
    safe_multi_opt_->addObjective(safetyObj, 1, "safety");
    safe_multi_opt_->addObjective(jointLimitsObj, 1, "joint");
    safe_multi_opt_->addObjective(manipulabilityObj, 1, "manipulability");
    if (ssvc_->humanPresence())
        safe_multi_opt_->addObjective(humanAwarenessObj, 1, "awareness");
    safe_multi_opt_->NormalizeWeight();

	//Safe RRT*
	opt_ = ompl::base::OptimizationObjectivePtr(safe_multi_opt_);
}



void ompl::geometric::SafeRRTstar::clear()
{
	Planner::clear();
	sampler_.reset();
	freeMemory();
	if (nn_)
		nn_->clear();

	lastGoalMotion_ = NULL;
	goalMotions_.clear();

	iterations_ = 0;
	bestCost_ = base::SafetyCost();

}


ompl::base::PlannerStatus ompl::geometric::SafeRRTstar::solve(const base::PlannerTerminationCondition &ptc)
{
	checkValidity();
	base::Goal                  *goal   = pdef_->getGoal().get();
	base::GoalSampleableRegion  *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

	while (!goal_s->canSample() && ptc == false)
	{
		boost::this_thread::sleep(ompl::time::seconds(.0001));
	}

	safe_multi_opt_->setGoal(pdef_->getGoal().get());

	while (const base::State *st = pis_.nextStart())
	{
		Motion *motion = new Motion(si_);
		si_->copyState(motion->state, st);
		motion->cost = safe_multi_opt_->safeStateCost(motion->state);
		motion->stateCost = motion->cost;
		nn_->add(motion);
		startMotion_ = motion;
	}

	safe_multi_opt_->setStart(startMotion_->state);

	if (nn_->size() == 0)
	{
		OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
		return base::PlannerStatus::INVALID_START;
	}

	if (!sampler_)
		sampler_ = si_->allocStateSampler();

	OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

	if (prune_ && !si_->getStateSpace()->isMetricSpace())
		OMPL_WARN("%s: tree pruning was activated but since the state space %s is not a metric space, "
				"the optimization objective might not satisfy the triangle inequality. You may need to disable pruning."
				, getName().c_str(), si_->getStateSpace()->getName().c_str());

	const base::ReportSafeIntermediateSolutionFn safeIntermediateSolutionCallback = pdef_->getSafeIntermediateSolutionCallback();

	Motion *solution       = lastGoalMotion_;

	bestCost_ = safe_multi_opt_->safeInfiniteCost();

	Motion *approximation  = NULL;
	double approximatedist = std::numeric_limits<double>::infinity();
	bool sufficientlyShort = false;

	Motion *rmotion        = new Motion(si_);
	base::State *rstate    = rmotion->state;
	base::State *xstate    = si_->allocState();

	// e+e/d.  K-nearest RRT*
	double k_rrg           = boost::math::constants::e<double>() +
			(boost::math::constants::e<double>() / (double)si_->getStateSpace()->getDimension());

	std::vector<Motion*>       nbh;

	std::vector<base::SafetyCost>    costs;
	std::vector<base::SafetyCost>    incCosts;
	std::vector<std::size_t>   sortedCostIndices;
	std::vector<bool>   			exactCost;

	std::vector<int>           valid;
	unsigned int               rewireTest = 0;
	unsigned int               statesGenerated = 0;


	if (solution)
		OMPL_INFORM("%s: Starting planning with existing solution", getName().c_str());
	OMPL_INFORM("%s: Initial k-nearest value of %u", getName().c_str(), (unsigned int)std::ceil(k_rrg * log((double)(nn_->size() + 1))));

	// our functor for sorting nearest neighbors
	CostIndexCompare compareFn(costs, *opt_);

	while (ptc == false)
	{
		iterations_++;

		// sample random state (with goal biasing)
		// Goal samples are only sampled until maxSampleCount() goals are in the tree, to prohibit duplicate goal states.
		if (goal_s && goalMotions_.size() < goal_s->maxSampleCount() && rng_.uniform01() < goalBias_ && goal_s->canSample())
			goal_s->sampleGoal(rstate);
		else
			sampler_->sampleUniform(rstate);

		// find closest state in the tree
		Motion *nmotion = nn_->nearest(rmotion);

		if (safeIntermediateSolutionCallback && si_->equalStates(nmotion->state, rstate))
			continue;

		base::State *dstate = rstate;

		// find state to add to the tree
		double d = si_->distance(nmotion->state, rstate);
		if (d > maxDistance_)
		{
			si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
			dstate = xstate;
		}

		statesGenerated++;

		// create a motion
		Motion *motion = new Motion(si_);
		si_->copyState(motion->state, dstate);
		motion->stateCost = safe_multi_opt_->safeStateCost(dstate);

		//If the sampled state can't improve the solution, we continue
		if (motion->stateCost.getCollisionWorld() || !ssvc_->isValidSelf(dstate) || safe_multi_opt_->isSafetyCostBetterThan(bestCost_, costToGo(motion)))
			continue;

		if (goal->isSatisfied(motion->state))
			motion->has_goal_state = true;
		else
			motion->has_goal_state = false;

		// Find nearby neighbors of the new motion - k-nearest RRT*
		unsigned int k = std::ceil(k_rrg * log((double)(nn_->size() + 1)));
		nn_->nearestK(motion, k, nbh);

		rewireTest += nbh.size();

		//STa : Erase goal nbh if the current sample is also a goal state.
		if (motion->has_goal_state)
		{
			for (std::vector<Motion*>::iterator it = nbh.begin() ; it != nbh.end();)
			{
				if ((*it)->has_goal_state)
				{
					it = nbh.erase(it);
				}
				else
					++ it;
			}
		}

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
			incCosts[i] = safe_multi_opt_->safeFastMotionCost(nbh[i]->state, motion->state ,nbh[i]->stateCost, motion->stateCost);
			costs[i] = safe_multi_opt_->safeCombineCosts(nbh[i]->cost, incCosts[i]);
			exactCost[i] = false;
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

		for (std::vector<std::size_t>::const_iterator i = sortedCostIndices.begin();
				i != sortedCostIndices.begin() + nbh.size();
				++i)
		{
			if (!nbh[*i]->has_goal_state)
			{
				incCosts[*i] = safe_multi_opt_->safeMotionCost(nbh[*i]->state, motion->state);
				costs[*i] = safe_multi_opt_->safeCombineCosts(nbh[*i]->cost, incCosts[*i]);
				exactCost[*i]=true;
				bool check_motion;
				if (safe_multi_opt_->hasSafetyObjective())
					check_motion = (!incCosts[*i].getCollisionWorld()) && safe_motion_validator_->checkMotionSelfCCDiscrete(nbh[*i]->state, motion->state, 1);
				else
					check_motion = safe_motion_validator_->checkMotionWorldIndividualLinks(nbh[*i]->state, motion->state, travel_dist_limit_, fast_dist_) && safe_motion_validator_->checkMotionSelfCCDiscrete(nbh[*i]->state, motion->state, 1);

				if (check_motion)
				{
					motion->incCost = incCosts[*i];
					motion->cost = costs[*i];
					motion->parent = nbh[*i];
					valid[*i] = 1;
					if ((cpt >= nbh.size() - 1) || safe_multi_opt_->isSafetyCostBetterThan(costs[*i], costs[*(i+1)]))
						break;
				}
				else valid[*i] = -1;
			}
			cpt ++;
		}

		//STa
		if(motion->parent == NULL)
			continue;

		if (prune_)
		{
			if (safe_multi_opt_->isSafetyCostBetterThan(costToGo(motion, false), bestCost_))
			{
				nn_->add(motion);
				motion->parent->children.push_back(motion);
			}
			else // If the new motion does not improve the best cost it is ignored.
			{
				--statesGenerated;
				si_->freeState(motion->state);
				delete motion;
				continue;
			}
		}
		else
		{
			// add motion to the tree
			nn_->add(motion);
			motion->parent->children.push_back(motion);
		}

		bool checkForSolution = false;
		if (!motion->has_goal_state)
		{
			for (std::size_t i = 0; i < nbh.size(); ++i)
			{
				if ((nbh[i] != motion->parent) && (nbh[i]->parent != NULL))
				{
					// Fast cost
					base::SafetyCost nbhIncCost = safe_multi_opt_->safeMotionCostSymmetric(motion->state, nbh[i]->state, incCosts[i]);
					base::SafetyCost nbhNewCost = safe_multi_opt_->safeCombineCosts(motion->cost, nbhIncCost);

					if (safe_multi_opt_->isSafetyCostBetterThan(nbhNewCost, nbh[i]->cost))
					{
						if (valid[i] == 0)
						{
							bool motionValid;

							if(!exactCost[i])
							{
								//Exact cost
								nbhIncCost = safe_multi_opt_->safeMotionCost(motion->state, nbh[i]->state);
								nbhNewCost = safe_multi_opt_->safeCombineCosts(motion->cost, nbhIncCost);
							}

							if (exactCost[i] || safe_multi_opt_->isSafetyCostBetterThan(nbhNewCost, nbh[i]->cost))
							{
								if (safe_multi_opt_->hasSafetyObjective())
									motionValid = (!nbhIncCost.getCollisionWorld()) && safe_motion_validator_->checkMotionSelfCCDiscrete(motion->state, nbh[i]->state, 1);
								else
									motionValid = safe_motion_validator_->checkMotionWorldIndividualLinks(motion->state, nbh[i]->state, travel_dist_limit_, fast_dist_) && safe_motion_validator_->checkMotionSelfCCDiscrete(motion->state, nbh[i]->state, 1);


								if (motionValid)
								{

									// Remove this node from its parent list
									removeFromParent (nbh[i]);

									// Add this node to the new parent
									nbh[i]->parent = motion;
									nbh[i]->incCost = nbhIncCost;
									nbh[i]->cost = nbhNewCost;
									nbh[i]->parent->children.push_back(nbh[i]);

									// Update the costs of the node's children
									updateChildCosts(nbh[i]);

									checkForSolution = true;
								}
							}
						}
					}
				}
			}
		}

		// Add the new motion to the goalMotion_ list, if it satisfies the goal
		double distanceFromGoal;
		if (goal->isSatisfied(motion->state, &distanceFromGoal))
		{
			goalMotions_.push_back(motion);
			checkForSolution = true;
		}

		// Checking for solution or iterative improvement
		if (checkForSolution)
		{
			bool updatedSolution = false;
			for (size_t i = 0; i < goalMotions_.size(); ++i)
			{
				if (safe_multi_opt_->isSafetyCostBetterThan(goalMotions_[i]->cost, bestCost_))
				{
					bestCost_ = goalMotions_[i]->cost;
					updatedSolution = true;
				}

				sufficientlyShort = safe_multi_opt_->isSafetySatisfied(goalMotions_[i]->cost);

				if (sufficientlyShort)
				{
					solution = goalMotions_[i];
					break;
				}
				else if (!solution ||
						safe_multi_opt_->isSafetyCostBetterThan(goalMotions_[i]->cost,solution->cost))
				{
					solution = goalMotions_[i];
					updatedSolution = true;
				}

			}

			if (updatedSolution)
			{
				if (prune_)
				{
					int n = pruneTree(bestCost_);
					statesGenerated -= n;
				}

				if (safeIntermediateSolutionCallback)
				{
					std::vector<const base::State *> spath;
					Motion *intermediate_solution = solution->parent; // Do not include goal state to simplify code.

					while (intermediate_solution->parent != 0) // Do not include the start state.
					{
						spath.push_back(intermediate_solution->state);
						intermediate_solution = intermediate_solution->parent;
					}
					safeIntermediateSolutionCallback(this, spath, bestCost_);
				}
			}
		}
		// terminate if a sufficient solution is found
		if (solution && sufficientlyShort)
			break;
	}

	bool approximate = (solution == NULL);
	bool addedSolution = false;
	if (approximate)
		solution = approximation;
	else
		lastGoalMotion_ = solution;

	if (solution != NULL)
	{
		ptc.terminate();
		// construct the solution path
		std::vector<Motion*> mpath;
		while (solution != NULL)
		{
			mpath.push_back(solution);
			solution = solution->parent;
		}

		// set the solution path
		PathGeometric *geoPath = new PathGeometric(si_);
		for (int i = mpath.size() - 1 ; i >= 0 ; --i)
			geoPath->append(mpath[i]->state);

		base::PathPtr path(geoPath);
		// Add the solution path.
		base::PlannerSolution psol(path);
		psol.setPlannerName(getName());
		if (approximate)
			psol.setApproximate(approximatedist);
		// Does the solution satisfy the optimization objective?
		psol.setOptimized(opt_, bestCost_, sufficientlyShort);
		pdef_->addSolutionPath(psol);

		addedSolution = true;
	}
	si_->freeState(xstate);
	if (rmotion->state)
		si_->freeState(rmotion->state);
	delete rmotion;

	OMPL_INFORM("%s: Created %u new states. Checked %u rewire options. %u goal states in tree.", getName().c_str(), statesGenerated, rewireTest, goalMotions_.size());

	return base::PlannerStatus(addedSolution, approximate);
}


void ompl::geometric::SafeRRTstar::removeFromParent(Motion *m)
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

void ompl::geometric::SafeRRTstar::updateChildCosts(Motion *m)
{
	for (std::size_t i = 0; i < m->children.size(); ++i)
	{
		m->children[i]->cost = safe_multi_opt_->safeCombineCosts(m->cost, m->children[i]->incCost);
		updateChildCosts(m->children[i]);
	}
}

ompl::geometric::SafeRRTstar::Motion* ompl::geometric::SafeRRTstar::getWorstMotion(Motion *solution)
{
	std::vector<Motion*> mpath;
	Motion *motion_temp = solution;
	while (motion_temp != NULL)
	{
		mpath.push_back(motion_temp);
		motion_temp = motion_temp->parent;
	}

	double cost_improvement = 0, cost_temp;
	Motion* worst_motion = NULL;
	for (int i = mpath.size() - 1 ; i > 0 ; --i)
	{
		cost_temp = safe_multi_opt_->safetyCostImprovement(mpath[i]->cost, mpath[i-1]->cost);
		if (cost_temp > cost_improvement)
		{
			cost_improvement = cost_temp;
			worst_motion = mpath[i-1];
		}
	}

	return worst_motion;
}

void ompl::geometric::SafeRRTstar::freeMemory()
{
	if (nn_)
	{
		std::vector<Motion*> motions;
		nn_->list(motions);
		for (std::size_t i = 0 ; i < motions.size() ; ++i)
		{
			if (motions[i]->state)
				si_->freeState(motions[i]->state);
			delete motions[i];
		}
	}
}

void ompl::geometric::SafeRRTstar::getPlannerData(base::PlannerData &data) const
{
	Planner::getPlannerData(data);

	std::vector<Motion*> motions;
	if (nn_)
		nn_->list(motions);

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

int ompl::geometric::SafeRRTstar::pruneTree(const base::SafetyCost pruneTreeCost)
{
	const int tree_size = nn_->size();
	pruneScratchSpace_.newTree.reserve(tree_size);
	pruneScratchSpace_.newTree.clear();
	pruneScratchSpace_.toBePruned.reserve(tree_size);
	pruneScratchSpace_.toBePruned.clear();
	pruneScratchSpace_.candidates.clear();
	pruneScratchSpace_.candidates.push_back(startMotion_);
	std::size_t j = 0;
	while (j != pruneScratchSpace_.candidates.size())
	{
		Motion *candidate = pruneScratchSpace_.candidates[j++];
		if (safe_multi_opt_->isSafetyCostBetterThan(pruneTreeCost, costToGo(candidate)))
		{
			pruneScratchSpace_.toBePruned.push_back(candidate);
		}
		else
		{
			pruneScratchSpace_.newTree.push_back(candidate);
			pruneScratchSpace_.candidates.insert(pruneScratchSpace_.candidates.end(),
					candidate->children.begin(), candidate->children.end());
		}
	}

	// To create the new nn takes one order of magnitude in time more than just checking how many
	// states would be pruned. Therefore, only prune if it removes a significant amount of states.
	if ((double)pruneScratchSpace_.newTree.size() / tree_size < pruneStatesThreshold_)
	{
		for (std::size_t i = 0; i < pruneScratchSpace_.toBePruned.size(); ++i)
			deleteBranch(pruneScratchSpace_.toBePruned[i]);

		nn_->clear();
		nn_->add(pruneScratchSpace_.newTree);

		return (tree_size - pruneScratchSpace_.newTree.size());
	}
	return 0;
}

void ompl::geometric::SafeRRTstar::deleteBranch(Motion *motion)
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

		//STa
		if (mto_delete->has_goal_state)
			for (size_t i = 0; i < goalMotions_.size(); ++i)
				if (mto_delete == goalMotions_[i])
					goalMotions_.erase(goalMotions_.begin() + i);

		si_->freeState(mto_delete->state);
		delete mto_delete;
	}
}

ompl::base::SafetyCost ompl::geometric::SafeRRTstar::costToGo(const Motion *motion, const bool shortest) const
{
	base::SafetyCost costToCome, costToGo;
	if (shortest)
	{
		costToCome = motion->stateCost;
		costToGo = safe_multi_opt_->safeCostToGo(motion->state, costToCome);
		return safe_multi_opt_->safeCombineCosts(costToCome, costToGo);
	}
	else
	{
		costToCome = motion->cost;
		costToGo = safe_multi_opt_->safeCostToGo(motion->state, costToCome);
		return safe_multi_opt_->safeCombineCosts(costToCome, costToGo);
	}
}
