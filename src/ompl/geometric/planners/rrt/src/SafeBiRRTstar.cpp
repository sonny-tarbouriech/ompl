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

#include "ompl/geometric/planners/cforest/CForestStateSampler.h"


//STa temp
#include <fstream>

ompl::geometric::SafeBiRRTstar::SafeBiRRTstar(const base::SpaceInformationPtr &si) :
base::Planner(si, "SafeBiRRTstar"),
num_thread_(0),
cforestEnabled_(false),
maxDistance_(0.0),
lastGoalMotion_(NULL),
pruneEnabled_(true),
pruneStatesThreshold_(0.95),
localBiasEnabled_(true),
localBiasRate_(0.2),
minMaxObjectiveImprovementEnabled_(true),
fastCostEnabled_(true),
heuristicRejectionEnabled_(true),
anytimeEnabled_(true),
iterations_(0),
bestCost_(base::SafetyCost()),
treeConnectionIndex_(0)
{
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;
    specs_.canReportIntermediateSolutions = true;

    //STa
    fast_dist_ = true;
    travel_dist_limit_=0.1;
    valid_segment_factor_ = 0.1;

    //STa test
    rewireTest_ = 0;
    statesGenerated_ = 0;
    statesPruned_ = 0;


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

std::vector<const ompl::base::State*> ompl::geometric::SafeBiRRTstar::treeConnection::getStatesToShare(boost::unordered_map<size_t, treeConnection*>& connection)
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
            if (currentMotion->connectionIndex !=0)
            {
                if (currentMotion->isGoalTree)
                    connection.find(currentMotion->connectionIndex)->second->startTreeMotion->isStateShared = true;
                else
                    connection.find(currentMotion->connectionIndex)->second->goalTreeMotion->isStateShared = true;
            }
        }
        currentMotion = currentMotion->parent;
    }
    //Because startTreeMotion->state == goalTreeMotion->state, the state is already shared --
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
            if (currentMotion->connectionIndex !=0)
            {
                if (currentMotion->isGoalTree)
                    connection.find(currentMotion->connectionIndex)->second->startTreeMotion->isStateShared = true;
                else
                    connection.find(currentMotion->connectionIndex)->second->goalTreeMotion->isStateShared = true;
            }
        }
        currentMotion = currentMotion->parent;
    }
    return statesToShare;
}


std::vector<ompl::base::SafetyCost> ompl::geometric::SafeBiRRTstar::treeConnection::getIncCosts()
{
    std::vector<base::SafetyCost> incCosts;
    Motion* solution = startTreeMotion;
    while(solution->parent != NULL)
    {
        incCosts.insert(incCosts.begin(), solution->incCost);
        solution = solution->parent;
    }
    solution = goalTreeMotion;
    while(solution->parent != NULL)
    {
        incCosts.push_back(solution->incCost);
        solution = solution->parent;
    }
    return incCosts;
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

    safe_multi_opt_ = new ompl::base::SafeMultiOptimizationObjective(si_, minMaxObjectiveImprovementEnabled_);
    ompl::base::OptimizationObjectivePtr lengthObj(new ompl::base::SafePathLengthOptimizationObjective(si_, &safe_multi_opt_->getStart() ,&safe_multi_opt_->getGoal()));
    ompl::base::OptimizationObjectivePtr safetyObj(new ompl::base::SafetyObjective(si_, safe_motion_validator_, fast_dist_, travel_dist_limit_));
    ompl::base::OptimizationObjectivePtr jointLimitsObj(new ompl::base::JointLimitsObjective(si_));
    ompl::base::OptimizationObjectivePtr manipulabilityObj(new ompl::base::ManipulabilityObjective(si_));
    ompl::base::OptimizationObjectivePtr humanAwarenessObj(new ompl::base::HumanAwarenessObjective(si_));
    if(ssvc_->getNbObjects() > 0)
    	safe_multi_opt_->addObjective(safetyObj, 0.1, "safety");
    safe_multi_opt_->addObjective(lengthObj, 0.1, "length");
    safe_multi_opt_->addObjective(jointLimitsObj, 0.1, "joint");
    safe_multi_opt_->addObjective(manipulabilityObj, 0.1, "manipulability");
    if (ssvc_->humanPresence())
        safe_multi_opt_->addObjective(humanAwarenessObj, 0.1, "awareness");
    safe_multi_opt_->NormalizeWeight();

    //STa temp
    std::cout << "human_presence = " << ssvc_->humanPresence() << std::endl ;

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

    //STa
    connection_.clear();
    treeConnectionIndex_ = 0;

}


ompl::base::PlannerStatus ompl::geometric::SafeBiRRTstar::solve(const base::PlannerTerminationCondition &ptc)
{
//    //STa test
//    std::string homepath = getenv("HOME");
//    std::ofstream output_file;
//    std::stringstream file_name;
//    file_name << homepath << "/SafeBiRRTstar" << num_thread_ << ".txt";
//    output_file.open(file_name.str().c_str(), std::ios::out | std::ios::app);
//    ompl::time::point init = ompl::time::now();
//    ompl::time::duration dur;

    checkValidity();

    safe_multi_opt_->setGoal(pdef_->getGoal().get());

    while (const base::State *startState = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, startState);
        motion->root = motion->state;
        motion->stateCost = safe_multi_opt_->safeStateCost(motion->state);
        motion->cost = motion->stateCost;
        motion->cost.isRoot() = true;
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
            motion->cost.isRoot() = true;
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
//    base::State *rstate = rmotion->state;

    // e+e/d.  K-nearest RRT*
    k_rrg_           = boost::math::constants::e<double>() +
            (boost::math::constants::e<double>() / (double)si_->getStateSpace()->getDimension());

    std::vector<Motion*>       nbh;

    std::vector<base::SafetyCost>    costs;
    std::vector<base::SafetyCost>    incCosts;
    std::vector<std::size_t>   sortedCostIndices;
    std::vector<bool>   			exactCost;

    std::vector<int>           valid;
    localstate_ = false;

    TreeData tree = tStart_;
    TreeData otherTree = tGoal_;

    //STa : If the planner is used with SafeCforest, bestSharedCost_ is updated with the best cost among planning threads. Else, it is the same as BestCost_
    const base::ReportSafeIntermediateSolutionFn safeIntermediateSolutionCallback = pdef_->getSafeIntermediateSolutionCallback();

    if (!cforestEnabled_)
        bestSharedCost_ = &bestCost_;

    base::SafetyCost currentBestSharedCost;

    //STa : If the planner is used with SafeCforest,
    bool cForestStateSampler = false;
    if (dynamic_cast<ompl::base::CForestStateSampler*>(sampler_.get()))
        cForestStateSampler = true;

    //STa
    base::State* worstState = si_->allocState();
    double worstStateRadius = 0;
    bool worstStateIsStartTree;
    bool worstStateUpToDate = false;

    while (ptc == false)
    {
        iterations_++;
        checkForSolution_ = false;

        currentBestSharedCost = *bestSharedCost_;
//        //STa temp
//        std::cout << "currentBestSharedCost = " << currentBestSharedCost << "\n";

        // Add available goal states
        while (pis_.haveMoreGoalStates())
        {
            const base::State *goalState = pis_.nextGoal();
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, goalState);

            //Ensure that the tree doesn't already have a near state
            Motion *nearest = tGoal_->nearest(motion);
            double d = si_->distance(nearest->state, goalState);
            if (d < 0.001)
            	continue;

            motion->root = motion->state;
            motion->stateCost = safe_multi_opt_->safeStateCost(motion->state);
            motion->cost =  motion->stateCost;
            motion->cost.isRoot() = true;
            motion->isGoalTree = true;
            if (!motion->stateCost.getCollisionWorld())
            {
                tGoal_->add(motion);
                goalMotion_.push_back(motion);
            }
        }
        // Local optimization
       if (localBiasEnabled_ && feasibleSolution && rng_.uniform01() < localBiasRate_)
       {
           if (!worstStateUpToDate)
           {
               Motion* worstMotion;
               connection_[bestIndex_]->getWorstMotion(safe_multi_opt_, worstMotion, worstStateIsStartTree);
               worstStateRadius = si_->getStateSpace()->distance(worstMotion->state, worstMotion->parent->state)/2;
               si_->getStateSpace()->interpolate(worstMotion->state, worstMotion->parent->state,0.5,worstState);
               worstStateUpToDate = true;
           }

           sampler_->sampleUniformNear(rmotion->state, worstState, worstStateRadius);

           if (worstStateIsStartTree)
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
           localstate_ = true;
           rmotion->isStateShared = false;
       }
       else
       {
           // sample random state
           if (cForestStateSampler)
           {
               static_cast<ompl::base::CForestStateSampler*>(sampler_.get())->sampleUniform(rmotion->state, rmotion->isStateShared);
           }
           else
           {
               sampler_->sampleUniform(rmotion->state);
               rmotion->isStateShared = false;
           }

           //STa test
           localstate_ = false;
       }

        statesGenerated_++;

        Motion* result; // the motion that gets added in extendTree
        if (extendTree(rmotion, tree, result) != FAILED) // we added something new to the tree
        {
//            //STa temp
//            std::cout << "extendTree returned true \n";

            // Try to connect the other tree to the node we just added
            if (connectTrees(result, otherTree))
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
            //We need to find the worst state again
            worstStateUpToDate = false;


//            //STa test
//            dur = ompl::time::now() - init;
//            output_file << (dur.total_microseconds() * 1e-6) << " "
//                    << statesGenerated_ << " "
//                    << rewireTest_ << " "
//                    << localstate_ << " "
//                    << bestCost_ << "\n";



            sufficientlyShort = safe_multi_opt_->isSafetySatisfied(bestCost_);

            if (sufficientlyShort)
            {
                break;
            }

            //Share the solution states with the others planning threads
            if (safeIntermediateSolutionCallback && safe_multi_opt_->isSafetyCostBetterThan(bestCost_,currentBestSharedCost))
            {
                std::vector<const base::State *> statesToShare;
                if(cforestEnabled_)
                	statesToShare = connection_[bestIndex_]->getStatesToShare(connection_);
                safeIntermediateSolutionCallback(this, statesToShare, bestCost_);
            }
        }

//        //STa temp
//        if (!safeIntermediateSolutionCallback)
//            std::cout << "bestCost_ = " << bestCost_ << "\n";

        //Generic way to generate the pruning process
        if (pruneEnabled_ && safe_multi_opt_->isSafetyCostBetterThan(*bestSharedCost_, currentBestSharedCost))
        {
            statesPruned_ += pruneTree(tStart_, true, *bestSharedCost_);
            statesPruned_ += pruneTree(tGoal_, false, *bestSharedCost_);

            //If CForest is running, all the connection for this planner could be removed by the pruning process. If the best solution is removed, reset the best cost.
            if (connection_.find(bestIndex_) == connection_.end())
            {
                feasibleSolution = false;
                bestCost_ = safe_multi_opt_->safeInfiniteCost();
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

//        //STa test sc2
//        std::vector<base::SafetyCost> iCost = connection_[bestIndex_]->getIncCosts();
//        double totDist = 0, curDist = 0;;
//        for(size_t i=0; i < iCost.size(); ++i)
//            totDist += iCost[i].getIndividualCost(1).value(); //Obj #1 is path length
//        for(size_t i=0; i < iCost.size(); ++i)
//        {
//            curDist += iCost[i].getIndividualCost(1).value();
//            output_file << iCost[i] << " " << curDist/totDist << "\n";
//        }
//        output_file << "\n";


//        //STa temp
//        std::string homepath = getenv("HOME");
//        std::ofstream output_file((homepath + "/safe_bi_rrt_star.txt").c_str(), std::ios::out | std::ios::app);
//        base::SafetyCost incCost, prevCost, curCost1, curCost2;
//        prevCost = safe_multi_opt_->safeStateCost(mpath[mpath.size() - 1]);
//        for (int i = mpath.size() - 1 ; i > 0 ; --i)
//        {
//            output_file << "state 1 : \n";
//            si_->printState(mpath[i], output_file);
//            output_file << "state 2 : \n";
//            si_->printState(mpath[i-1], output_file);
//            incCost = safe_multi_opt_->safeMotionCostTEST(mpath[i], mpath[i-1]);
//            curCost1 = safe_multi_opt_->safeCombineCosts(prevCost, incCost);
//            prevCost = curCost1;
//            output_file << std::setw(10) << incCost << "\n";
//
//        }
//        output_file << "\n";
//        prevCost = safe_multi_opt_->safeStateCost(mpath[mpath.size() - 1]);
//        for (int i = mpath.size() - 1 ; i > 0 ; --i)
//        {
//            output_file << "state 1 : \n";
//            si_->printState(mpath[i], output_file);
//            output_file << "state 2 : \n";
//            si_->printState(mpath[i-1], output_file);
//            incCost = safe_multi_opt_->safeMotionCost(mpath[i], mpath[i-1]);
//            curCost2 = safe_multi_opt_->safeCombineCosts(prevCost, incCost);
//            prevCost = curCost2;
//            output_file << std::setw(10) << incCost << "\n";
//
//        }
//        output_file << "\n" << curCost1 << "\n";
//        output_file << curCost2 << "\n";
//        output_file << "\n \n";
//        output_file.close();

//        if(base::goalRegionCostToGo(startMotion_->state, pdef_->getGoal().get()).value() > 0.01)
//        {
//
//            //            output_file << std::setw(10) << connection_[bestIndex_]->startTreeMotion->cost << "\t";
//            //            output_file << std::setw(10) << connection_[bestIndex_]->startTreeMotion->cost.getCollisionWorld() << "\t";
//            //            output_file << std::setw(10) << connection_[bestIndex_]->goalTreeMotion->cost << "\t";
//            //            output_file << std::setw(10) << connection_[bestIndex_]->goalTreeMotion->cost.getCollisionWorld() << "\t";
//
//
//            //            output_file << std::setw(10) << curCost1 << "\t";
//            //            output_file << std::setw(10) << curCost2 << "\t";
//
//
//            //			output_file << std::setw(10) << ompl::time::seconds(dur_first_sol) << "\t";
//            //            output_file << std::setw(10) << statesGenerated_ << "\t";
//            //            output_file << std::setw(10) << rewireTest_ << "\n";
//        }
    }

    si_->freeState(worstState);
    si_->freeState(rmotion->state);
    delete rmotion;

    if (pruneEnabled_)
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

//        //STa test
//        output_file << "\n";
//        output_file.close();


    return feasibleSolution ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}


ompl::geometric::SafeBiRRTstar::GrowResult ompl::geometric::SafeBiRRTstar::extendTree(Motion* toMotion, TreeData& tree, Motion*& result)
{
//    //STa temp
//    std::cout << "tree size = " << tree->size() << "\n";

    // Nearest neighbor
    if (tree->size() > 0)
    {
        Motion *nearest = tree->nearest(toMotion);
        return extendTree(toMotion, tree, result, nearest);
    }
    else
        return FAILED;
}

ompl::geometric::SafeBiRRTstar::GrowResult ompl::geometric::SafeBiRRTstar::extendTree(Motion* toMotion, TreeData& tree, Motion*& result, Motion* nearest)
{
//    //STa test
//    std::string homepath = getenv("HOME");
//    std::ofstream output_file;
//    output_file.open((homepath + "/extendTree.txt").c_str(), std::ios::out | std::ios::app);


//    //STa temp
//    std::cout << "Enter extendTree \n";

    bool reach = true;
    std::vector<Motion*>       nbh;

    std::vector<base::SafetyCost>    costs;
    std::vector<base::SafetyCost>    incCosts;
    std::vector<std::size_t>   sortedCostIndices;
    std::vector<bool>   			exactCost;

    std::vector<int>           valid;

    // our functor for sorting nearest neighbors
    CostIndexCompare compareFn(costs, *opt_);

    // Compute the state to extend toward
    double d = si_->distance(nearest->state, toMotion->state);

    if (d < std::numeric_limits<float>::epsilon())
        return FAILED;

    result = new Motion(si_);

    // Truncate the random state to be no more than maxDistance_ from nearest neighbor
    if (d > maxDistance_)
    {
        si_->getStateSpace()->interpolate(nearest->state, toMotion->state, maxDistance_ / d, result->state);
        d = maxDistance_;
        reach = false;
    }
    else
    {
        si_->copyState(result->state, toMotion->state);
        //STa : When used with CForest, the sampled state could come from an other planner
        result->isStateShared = toMotion->isStateShared;
    }


    result->stateCost = safe_multi_opt_->safeStateCost(result->state);

    //If the sampled state is in collision, we continue
    if (result->stateCost.getCollisionWorld() || !ssvc_->isValidSelf(result->state))
    {
        //STa test
//        output_file << "return FAILED \n \n";
//        output_file.close();
        si_->freeState(result->state);
        delete result;
        return FAILED;
    }

    if(heuristicRejectionEnabled_ && !safe_multi_opt_->isCostDefined(*bestSharedCost_))
    {
        base::SafetyCost heuristicCost = result->heuristicCost(safe_multi_opt_);

        //If the sampled state can't improve the solution, we continue
        if (safe_multi_opt_->isSafetyCostBetterThan(*bestSharedCost_, heuristicCost))
        {
            //STa test
            //        output_file << "return FAILED \n \n";
            //        output_file.close();
            si_->freeState(result->state);
            delete result;
            return FAILED;
        }
    }


    if (anytimeEnabled_ && !safe_multi_opt_->isCostDefined(*bestSharedCost_))
    {
        //No solution has been found yet, the planner behaves like RRT (only the nearest neighbor is considered)
        nbh.push_back(nearest);
    }
    else
    {
        // Find nearby neighbors of the new motion - k-nearest RRT*
        unsigned int k = std::ceil(k_rrg_ * log((double)(tree->size() + 1)));
        tree->nearestK(result, k, nbh);
    }

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
    valid.resize(nbh.size());
    std::fill(valid.begin(), valid.begin() + nbh.size(), 0);

    // Finding the nearest neighbor to connect to
    // By default, neighborhood states are sorted by cost, and collision checking
    // is performed in increasing order of cost

    // calculate all costs and distances
    for (std::size_t i = 0 ; i < nbh.size(); ++i)
    {
        if (fastCostEnabled_)
        {
            incCosts[i] = safe_multi_opt_->safeFastMotionCost(nbh[i]->state, result->state ,nbh[i]->stateCost, result->stateCost);
            costs[i] = safe_multi_opt_->safeCombineCosts(nbh[i]->cost, incCosts[i]);
            exactCost[i] = false;
        }
        else
        {
            incCosts[i] = safe_multi_opt_->safeMotionCost(nbh[i]->state, result->state);
            costs[i] = safe_multi_opt_->safeCombineCosts(nbh[i]->cost, incCosts[i]);
            exactCost[i] = true;
        }
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
    size_t nbhCount = 0;

    //STa TODO : Add a factor to favor nearer states?
    for (std::vector<std::size_t>::const_iterator i = sortedCostIndices.begin();
            i != sortedCostIndices.begin() + nbh.size();
            ++i)
    {
        if (fastCostEnabled_)
        {
            incCosts[*i] = safe_multi_opt_->safeMotionCost(nbh[*i]->state, result->state);
            costs[*i] = safe_multi_opt_->safeCombineCosts(nbh[*i]->cost, incCosts[*i]);
            exactCost[*i]=true;
        }
        bool check_motion;
        if (safe_multi_opt_->hasSafetyObjective())
            check_motion = (!incCosts[*i].getCollisionWorld()) && safe_motion_validator_->checkMotionSelfCCDiscrete(nbh[*i]->state, result->state, valid_segment_factor_);
        else
            check_motion = safe_motion_validator_->checkMotionWorldIndividualLinks(nbh[*i]->state, result->state, travel_dist_limit_, fast_dist_) && safe_motion_validator_->checkMotionSelfCCDiscrete(nbh[*i]->state, result->state, valid_segment_factor_);

        //STa test
//        output_file << "exact incCosts[" << *i <<"] = " << incCosts[*i] << "\n";
//        output_file << "exact costs[" << *i <<"] = " << costs[*i] << "\n";

        if (check_motion)
        {
            //STa test
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
            if ((nbhCount >= nbh.size() - 1) || safe_multi_opt_->isSafetyCostBetterThan(result->cost, costs[*(i+1)]))
                break;
        }
        else
            valid[*i] = -1;
        nbhCount++;
    }

    if(result->parent == NULL)
    {
        //STa test
//        output_file << "return FAILED \n \n";
//        output_file.close();
        si_->freeState(result->state);
        delete result;
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
        if ((valid[i] == -1) || !nbh[i]->parent || result->isParent(nbh[i]))
            continue;

        //STa : All objectives are symmetric
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

            //STa test
//            output_file << "exact nbhIncCost[" << i <<"] = " << nbhIncCost << "\n";
//            output_file << "exact nbhNewCost[" << i <<"] = " << nbhNewCost << "\n";
        }

        bool motionValid = safe_motion_validator_->checkMotionSelfCCDiscrete(result->state, nbh[i]->state, valid_segment_factor_);
        if (!safe_multi_opt_->hasSafetyObjective() && motionValid)
            motionValid = safe_motion_validator_->checkMotionWorldIndividualLinks(result->state, nbh[i]->state, travel_dist_limit_, fast_dist_);

        if (motionValid)
        {
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

    //STa test
//    if (reach)
//        output_file << "return SUCCESS \n \n";
//    else
//        output_file << "return ADVANCED \n \n";
//    output_file.close();

    return reach ? SUCCESS : ADVANCED;

}

bool ompl::geometric::SafeBiRRTstar::connectTrees(Motion* nmotion, TreeData& tree)
{
    // Get the nearest state to nmotion in tree (nmotion is NOT in tree)
    Motion *nearest;
    if (tree->size() > 0)
        nearest = tree->nearest(nmotion);
    else
        return false;

    //If a connection already exists between the trees through a common parent state, we don't try to make an other connection
    if(nmotion->hasCommonParentState(nearest))
        return false;

    double dist = si_->distance(nearest->state, nmotion->state);

    // Do not attempt a connection if the trees are far apart
    if (dist > connectionRange_)
        return false;

    // Do not try to connect states directly.  Must chop up the
    // extension into segments, just in case one piece fails
    // the transition test
    GrowResult result;
    do
    {
        Motion* next = NULL;
        result = extendTree(nmotion, tree, next, nearest);

        if (result != FAILED)
            nearest = next;

    } while (result == ADVANCED);

    // Successful connection
    if (result == SUCCESS)
    {
        bool treeIsStart = tree == tStart_;
        Motion* startMotion = treeIsStart ? nearest : nmotion;
        Motion* goalMotion  = treeIsStart ? nmotion : nearest;

        // Make sure start-goal pair is valid
        if (pdef_->getGoal()->isStartGoalPairValid(startMotion->root, goalMotion->root))
        {
            treeConnection* tc = new treeConnection;

            tc->startTreeMotion = startMotion;
            tc->goalTreeMotion = goalMotion;

            tc->updateWholeMotionCost(safe_multi_opt_);
            tc->index = ++treeConnectionIndex_;

            connection_[tc->index] = tc;

            //Each motion needs to know its connections with the other tree
            startMotion->connectionIndex = treeConnectionIndex_;
            goalMotion->connectionIndex = treeConnectionIndex_;

            if (safe_multi_opt_->isSafetyCostBetterThan(tc->wholeMotionCost, bestCost_))
            {
                bestCost_ = tc->wholeMotionCost;
                bestIndex_ = tc->index;
                checkForSolution_ = true;
            }

            return true;
        }
    }
    return false;
}


void ompl::geometric::SafeBiRRTstar::removeFromParent(Motion *m)
{
    if (m->parent)
    {
        for (std::vector<Motion*>::iterator it = m->parent->children.begin();
                it != m->parent->children.end (); ++it)
        {
            if (*it == m)
            {
                m->parent->children.erase(it);
                break;
            }
        }
    }
}

void ompl::geometric::SafeBiRRTstar::updateChildCosts(Motion *m)
{
    if (m->connectionIndex != 0)
    {
        treeConnection* tc = connection_.find(m->connectionIndex)->second;
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

    //STa
    for (boost::unordered_map<size_t, treeConnection*>::iterator it = connection_.begin(); it != connection_.end(); ++it)
    {
        delete it->second;
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

//STa
int ompl::geometric::SafeBiRRTstar::pruneTree(TreeData tree, bool isStartTree, const base::SafetyCost pruneTreeCost)
{
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
        Motion *candidate = pruneScratchSpace_.candidates[j];
        if (safe_multi_opt_->isSafetyCostBetterThan(pruneTreeCost, candidate->heuristicCost(safe_multi_opt_)))
        {
            pruneScratchSpace_.toBePruned.push_back(candidate);

            //Backup goal states that have to be deleted form goalMotion_ vector
            if (!isStartTree && j < goalMotion_.size())
            {
                goalToBePrunedIndex.push_back(j);
            }
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

        if (tree->size() == 0)
        {
            if (isStartTree)
                throw Exception("pruneTree: Start tree is empty!");
            else
                throw Exception("pruneTree: Goal tree is empty!");
        }
        if (!isStartTree)
        {
            for (size_t i =0; i < goalToBePrunedIndex.size(); ++i)
            {
                //Delete goal states from goalMotion_ vector
                goalMotion_.erase(goalMotion_.begin() + goalToBePrunedIndex[i] - i);
            }
        }
        return (tree_size - pruneScratchSpace_.newTree.size());
    }
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
        if (mto_delete->connectionIndex != 0)
        {
            if (mto_delete->isGoalTree)
                connection_.find(mto_delete->connectionIndex)->second->startTreeMotion->connectionIndex = 0;
            else
                connection_.find(mto_delete->connectionIndex)->second->goalTreeMotion->connectionIndex = 0;

            connection_.erase(mto_delete->connectionIndex);
        }

        si_->freeState(mto_delete->state);
        delete mto_delete;
    }
}

