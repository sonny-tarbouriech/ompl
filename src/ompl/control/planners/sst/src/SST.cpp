/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick
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
*   * Neither the name of Rutgers University nor the names of its
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

/* Authors: Zakary Littlefield */

#include "ompl/control/planners/sst/SST.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/MinimaxObjective.h"
#include "ompl/base/objectives/MaximizeMinClearanceObjective.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/objectives/MechanicalWorkOptimizationObjective.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>

ompl::control::SST::SST(const SpaceInformationPtr &si) : base::Planner(si, "SST")
{
    specs_.approximateSolutions = true;
    siC_ = si.get();
    prevSolution_.clear();
    prevSolutionControls_.clear();
    prevSolutionSteps_.clear();

    goalBias_ = 0.05;
    selectionRadius_ = 0.2;
    pruningRadius_ = 0.1;

    Planner::declareParam<double>("goal_bias", this, &SST::setGoalBias, &SST::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("selection_radius", this, &SST::setSelectionRadius, &SST::getSelectionRadius, "0.:.1:100");
    Planner::declareParam<double>("pruning_radius", this, &SST::setPruningRadius, &SST::getPruningRadius, "0.:.1:100");
}

ompl::control::SST::~SST()
{
    freeMemory();
}

void ompl::control::SST::setup()
{
    base::Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
    nn_->setDistanceFunction(boost::bind(&SST::distanceFunction, this, _1, _2));
    if (!witnesses_)
        witnesses_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
    witnesses_->setDistanceFunction(boost::bind(&SST::distanceFunction, this, _1, _2));

    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
        {
            opt_ = pdef_->getOptimizationObjective();
            if (dynamic_cast<base::MaximizeMinClearanceObjective*>(opt_.get()) || dynamic_cast<base::MinimaxObjective*>(opt_.get()))
                OMPL_WARN("%s: Asymptotic near-optimality has only been proven with Lipschitz continuous cost functions w.r.t. state and control. This optimization objective will result in undefined behavior", getName().c_str());
        }
        else
        {
            OMPL_WARN("%s: No optimization object set. Using path length", getName().c_str());
            opt_.reset(new base::PathLengthOptimizationObjective(si_));
            pdef_->setOptimizationObjective(opt_);
        }
    }

    prevSolutionCost_ = opt_->infiniteCost();

}

void ompl::control::SST::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    if (witnesses_)
        witnesses_->clear();
    prevSolutionCost_ = opt_->infiniteCost();
}

void ompl::control::SST::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion*> motions;
        nn_->list(motions);
        for (unsigned int i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state_)
                si_->freeState(motions[i]->state_);
            if (motions[i]->control_)
                siC_->freeControl(motions[i]->control_);
            delete motions[i];
        }
    }
    if (witnesses_)
    {
        std::vector<Motion*> witnesses;
        witnesses_->list(witnesses);
        for (unsigned int i = 0 ; i < witnesses.size() ; ++i)
        {
            delete witnesses[i];
        }
    }
    for (unsigned int i = 0 ; i < prevSolution_.size() ; ++i)
    {
        if (prevSolution_[i])
            si_->freeState(prevSolution_[i]);
    }
    prevSolution_.clear();
    for (unsigned int i = 0 ; i < prevSolutionControls_.size() ; ++i)
    {
        if (prevSolutionControls_[i])
            siC_->freeControl(prevSolutionControls_[i]);
    }
    prevSolutionControls_.clear();
    prevSolutionSteps_.clear();
}

ompl::control::SST::Motion* ompl::control::SST::selectNode(ompl::control::SST::Motion *sample)
{
    std::vector<Motion*> ret;
    Motion *selected = NULL;
    base::Cost bestCost = opt_->infiniteCost();
    nn_->nearestR(sample, selectionRadius_, ret);
    for (unsigned int i = 0; i < ret.size(); i++)
    {
        if (!ret[i]->inactive_ && opt_->isCostBetterThan(ret[i]->accCost_, bestCost))
        {
            bestCost = ret[i]->accCost_;
            selected = ret[i];
        }
    }
    if (selected == NULL)
    {
        int k = 1;
        while (selected == NULL)
        {
            nn_->nearestK(sample, k, ret);
            for (unsigned int i=0; i < ret.size() && selected == NULL; i++)
                if (!ret[i]->inactive_)
                    selected = ret[i];
            k += 5;
        }
    }
    return selected;
}

ompl::control::SST::Witness* ompl::control::SST::findClosestWitness(ompl::control::SST::Motion *node)
{
    if(witnesses_->size() > 0)
    {
        Witness* closest = static_cast<Witness*>(witnesses_->nearest(node));
        if (distanceFunction(closest,node) > pruningRadius_)
        {
            closest = new Witness(siC_);
            closest->linkRep(node);
            si_->copyState(closest->state_, node->state_);
            witnesses_->add(closest);
        }
        return closest;
    }
    else
    {
        Witness* closest = new Witness(siC_);
        closest->linkRep(node);
        si_->copyState(closest->state_, node->state_);
        witnesses_->add(closest);
        return closest;
    }
}

ompl::base::PlannerStatus ompl::control::SST::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal                   *goal = pdef_->getGoal().get();
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(siC_);
        si_->copyState(motion->state_, st);
        siC_->nullControl(motion->control_);
        nn_->add(motion);
        motion->accCost_ = opt_->identityCost();
        findClosestWitness(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocControlSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure\n", getName().c_str(), nn_->size());

    Motion *solution  = NULL;
    Motion *approxsol = NULL;
    double  approxdif = std::numeric_limits<double>::infinity();
    bool sufficientlyShort = false;

    Motion      *rmotion = new Motion(siC_);
    base::State  *rstate = rmotion->state_;
    Control       *rctrl = rmotion->control_;
    base::State  *xstate = si_->allocState();

    unsigned iterations = 0;

    while (ptc == false)
    {

        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        /* find closest state in the tree */
        Motion *nmotion = selectNode(rmotion);



        /* sample a random control that attempts to go towards the random state, and also sample a control duration */
        controlSampler_->sample(rctrl);
        unsigned int cd = rng_.uniformInt(siC_->getMinControlDuration(),siC_->getMaxControlDuration());
        unsigned int propCd = siC_->propagateWhileValid(nmotion->state_,rctrl,cd,rstate);


        if (propCd == cd)
        {
            base::Cost incCost = opt_->motionCost(nmotion->state_, rstate);
            base::Cost cost = opt_->combineCosts(nmotion->accCost_, incCost);
            Witness* closestWitness = findClosestWitness(rmotion);


            if (closestWitness->rep_ == rmotion || opt_->isCostBetterThan(cost,closestWitness->rep_->accCost_))
            {
                Motion* oldRep = closestWitness->rep_;
                /* create a motion */
                Motion *motion = new Motion(siC_);
                motion->accCost_ = cost;
                si_->copyState(motion->state_, rmotion->state_);
                siC_->copyControl(motion->control_, rctrl);
                motion->steps_ = cd;
                motion->parent_ = nmotion;
                nmotion->numChildren_++;
                closestWitness->linkRep(motion);

                nn_->add(motion);
                double dist = 0.0;
                bool solv = goal->isSatisfied(motion->state_, &dist);
                if (solv && opt_->isCostBetterThan(motion->accCost_,prevSolutionCost_))
                {
                    approxdif = dist;
                    solution = motion;

                    for (unsigned int i = 0 ; i < prevSolution_.size() ; ++i)
                        if (prevSolution_[i])
                            si_->freeState(prevSolution_[i]);
                    prevSolution_.clear();
                    for (unsigned int i = 0 ; i < prevSolutionControls_.size() ; ++i)
                        if (prevSolutionControls_[i])
                            siC_->freeControl(prevSolutionControls_[i]);
                    prevSolutionControls_.clear();
                    prevSolutionSteps_.clear();


                    Motion* solTrav = solution;
                    while(solTrav->parent_!=NULL)
                    {
                        prevSolution_.push_back(si_->cloneState(solTrav->state_) );
                        prevSolutionControls_.push_back(siC_->cloneControl(solTrav->control_) );
                        prevSolutionSteps_.push_back(solTrav->steps_ );
                        solTrav = solTrav->parent_;
                    }
                    prevSolution_.push_back(si_->cloneState(solTrav->state_) );
                    prevSolutionCost_ = solution->accCost_;




                    OMPL_INFORM("Found solution with cost %.2f",solution->accCost_.value());
                    sufficientlyShort = opt_->isSatisfied(solution->accCost_);
                    if (sufficientlyShort)
                        break;
                }
                if (solution==NULL && dist < approxdif)
                {
                    approxdif = dist;
                    approxsol = motion;



                    for (unsigned int i = 0 ; i < prevSolution_.size() ; ++i)
                        if (prevSolution_[i])
                            si_->freeState(prevSolution_[i]);
                    prevSolution_.clear();
                    for (unsigned int i = 0 ; i < prevSolutionControls_.size() ; ++i)
                        if (prevSolutionControls_[i])
                            siC_->freeControl(prevSolutionControls_[i]);
                    prevSolutionControls_.clear();
                    prevSolutionSteps_.clear();

                    Motion *solTrav = approxsol;
                    while (solTrav->parent_!=NULL)
                    {
                        prevSolution_.push_back(si_->cloneState(solTrav->state_) );
                        prevSolutionControls_.push_back(siC_->cloneControl(solTrav->control_) );
                        prevSolutionSteps_.push_back(solTrav->steps_ );
                        solTrav = solTrav->parent_;
                    }
                    prevSolution_.push_back(si_->cloneState(solTrav->state_) );
                }

                if (oldRep != rmotion)
                {
                    oldRep->inactive_ = true;
                    nn_->remove(oldRep);
                    while (oldRep->inactive_ && oldRep->numChildren_==0)
                    {
                        if (oldRep->state_)
                            si_->freeState(oldRep->state_);
                        if (oldRep->control_)
                            siC_->freeControl(oldRep->control_);

                        oldRep->state_=NULL;
                        oldRep->control_=NULL;
                        oldRep->parent_->numChildren_--;
                        Motion* oldRepParent = oldRep->parent_;
                        delete oldRep;
                        oldRep = oldRepParent;
                    }
                }

            }
        }
        iterations++;
    }

    bool solved = false;
    bool approximate = false;
    if (solution == NULL)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != NULL)
    {
        /* set the solution path */
        PathControl *path = new PathControl(si_);
        for (int i = prevSolution_.size() - 1 ; i >= 1 ; --i)
            path->append(prevSolution_[i], prevSolutionControls_[i-1], prevSolutionSteps_[i-1] * siC_->getPropagationStepSize());
        path->append(prevSolution_[0]);
        solved = true;
        pdef_->addSolutionPath(base::PathPtr(path), approximate, approxdif, getName());
    }

    si_->freeState(xstate);
    if (rmotion->state_)
        si_->freeState(rmotion->state_);
    if (rmotion->control_)
        siC_->freeControl(rmotion->control_);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states in %u iterations", getName().c_str(), nn_->size(),iterations);

    return base::PlannerStatus(solved, approximate);
}

void ompl::control::SST::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    std::vector<Motion*> allMotions;
    if (nn_)
        nn_->list(motions);

    for(unsigned i=0;i<motions.size();i++)
    {
        if(motions[i]->numChildren_==0)
        {
            allMotions.push_back(motions[i]);
        }
    }
    for(unsigned i=0;i<allMotions.size();i++)
    {
        if(allMotions[i]->parent_!=NULL)
        {
            allMotions.push_back(allMotions[i]->parent_);
        }
    }

    double delta = siC_->getPropagationStepSize();

    if (prevSolution_.size()!=0)
        data.addGoalVertex(base::PlannerDataVertex(prevSolution_[0]));

    for (unsigned int i = 0 ; i < allMotions.size() ; ++i)
    {
        const Motion *m = allMotions[i];
        if (m->parent_)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->parent_->state_),
                             base::PlannerDataVertex(m->state_),
                             control::PlannerDataEdgeControl(m->control_, m->steps_ * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->parent_->state_),
                             base::PlannerDataVertex(m->state_));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(m->state_));
    }
}
