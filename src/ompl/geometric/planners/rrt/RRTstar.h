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

/* Authors: Alejandro Perez, Sertac Karaman, Ryan Luna, Luis G. Torres, Ioan Sucan, Javier V Gomez, Jonathan Gammell */

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_RRTSTAR_
#define OMPL_GEOMETRIC_PLANNERS_RRT_RRTSTAR_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/datastructures/NearestNeighbors.h"

#include <limits>
#include <vector>
#include <utility>


namespace ompl
{

    namespace geometric
    {

        /**
           @anchor gRRTstar
           @par Short description
           \ref gRRTstar "RRT*" (optimal RRT) is an asymptotically-optimal incremental
           sampling-based motion planning algorithm. \ref gRRTstar "RRT*" algorithm is
           guaranteed to converge to an optimal solution, while its
           running time is guaranteed to be a constant factor of the
           running time of the \ref gRRT "RRT". The notion of optimality is with
           respect to the distance function defined on the state space
           we are operating on. See ompl::base::Goal::setMaximumPathLength() for
           how to set the maximally allowed path length to reach the goal.
           If a solution path that is shorter than ompl::base::Goal::getMaximumPathLength() is
           found, the algorithm terminates before the elapsed time.
           @par External documentation
           S. Karaman and E. Frazzoli, Sampling-based
           Algorithms for Optimal Motion Planning, International Journal of Robotics
           Research, Vol 30, No 7, 2011.
           http://arxiv.org/abs/1105.1186
        */

        /** \brief Optimal Rapidly-exploring Random Trees */
        class RRTstar : public base::Planner
        {
        public:

            RRTstar(const base::SpaceInformationPtr &si);

            virtual ~RRTstar();

            virtual void getPlannerData(base::PlannerData &data) const;

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear();

            /** \brief Set the goal bias

                In the process of randomly selecting states in
                the state space to attempt to go towards, the
                algorithm may in fact choose the actual goal state, if
                it knows it, with some probability. This probability
                is a real number between 0.0 and 1.0; its value should
                usually be around 0.05 and should not be too large. It
                is probably a good idea to use the default value. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
            }

            /** \brief Set the range the planner is supposed to use.

                This parameter greatly influences the runtime of the
                algorithm. It represents the maximum length of a
                motion to be added in the tree of motions. */
            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            /** \brief Get the range the planner is using */
            double getRange() const
            {
                return maxDistance_;
            }

            /** \brief Set the rewiring scale factor, s, such that r_rrg = s \times r_rrg* (or k_rrg = s \times k_rrg*) */
            void setRewireFactor(double rewireFactor)
            {
                rewireFactor_ = rewireFactor;
            }

            /** \brief Set the rewiring scale factor, s, such that r_rrg = s \times r_rrg* > r_rrg* (or k_rrg = s \times k_rrg* > k_rrg*) */
            double getRewireFactor() const
            {
                return rewireFactor_;
            }

            /** \brief Set a different nearest neighbors datastructure */
            template<template<typename T> class NN>
            void setNearestNeighbors()
            {
                nn_.reset(new NN<Motion*>());
            }

            /** \brief Option that delays collision checking procedures.
                When it is enabled, all neighbors are sorted by cost. The
                planner then goes through this list, starting with the lowest
                cost, checking for collisions in order to find a parent. The planner
                stops iterating through the list when a collision free parent is found.
                This prevents the planner from collision checking each neighbor, reducing
                computation time in scenarios where collision checking procedures are expensive.*/
            void setDelayCC(bool delayCC)
            {
                delayCC_ = delayCC;
            }

            /** \brief Get the state of the delayed collision checking option */
            bool getDelayCC() const
            {
                return delayCC_;
            }

            /** \brief Controls whether the tree is pruned during the search. This pruning removes
                a vertex \e and \e its \e descendants if the lower-bounding estimate of a solution
                constrained to pass the the \e vertex  is greater than the current solution.
                As the ancestory of the descendent vertices can change at anytime, this means that the removed
                descendents may actually be capable of later providing a better solution once
                their incoming path passes through a different vertex (e.g., a change in homotopy class) */
            void setPrune(const bool prune)
            {
                prune_ = prune;
            }

            /** \brief Get the state of the pruning option. */
            bool getPrune() const
            {
                return prune_;
            }

            /** \brief Set the percentage threshold (between 0 and 1) for pruning the tree. If the new tree has removed
                at least this percentage of states, the tree will be finally pruned. */
            void setPruneStatesImprovementThreshold(const double pp)
            {
                pruneStatesThreshold_ = pp;
            }

            /** \brief Get the current prune states percentage threshold parameter. */
            double getPruneStatesImprovementThreshold () const
            {
                return pruneStatesThreshold_;
            }

            /** \brief Use a k-nearest search for rewiring instead of a r-disc search. */
            void setKNearest(bool useKNearest)
            {
                useKNearest_ = useKNearest;
            }

            /** \brief Get the state of using a k-nearest search for rewiring. */
            bool getKNearest() const
            {
                return useKNearest_;
            }

            /** \brief Use \e Informed \e RRT*.
            This means that once a problem is found, the search is focused only to the subproblem that could contain a better solution.
            Currently only implemented for problems with a single goal that are seeking to minimize path length in
            R^n (i.e., RealVectorStateSpace), SE(2) (i.e., SE2StateSpace), or SE(3) (i.e., SE3StateSpace).
            @par J D. Gammell, S. S. Srinivasa, T. D. Barfoot, "Informed RRT*: Optimal Sampling-based
            Path Planning Focused via Direct Sampling of an Admissible Ellipsoidal Heuristic."
            IROS 2014. DOI: <a href="http://dx.doi.org/10.1109/IROS.2014.6942976">10.1109/IROS.2014.6942976</a>.
            <a href="http://www.youtube.com/watch?v=d7dX5MvDYTc">Illustration video</a>.
            <a href="http://www.youtube.com/watch?v=nsl-5MZfwu4">Short description video</a>. */
            void setInformedSampling(bool informedSampling);

            /** \brief Get the state of informed sampling */
            bool getInformedSampling() const
            {
                return useInformedSampling_;
            }

            /** \brief Set the number of attempts to make while performing informed sampling */
            void setInformedSamplingAttempts(unsigned int numAttempts)
            {
                numInfAttempts_ = numAttempts;
            }

            /** \brief Get the number of attempts to make while performing informed sampling */
            unsigned int getInformedSamplingAttempts() const
            {
                return numInfAttempts_ ;
            }

            virtual void setup();

            unsigned int numIterations() const
            {
                return iterations_;
            }

            ompl::base::Cost bestCost() const
            {
                return bestCost_;
            }

        protected:

            /** \brief Representation of a motion */
            class Motion
            {
            public:
                /** \brief Constructor that allocates memory for the state. This constructor automatically allocates memory for \e state, \e cost, and \e incCost */
                Motion(const base::SpaceInformationPtr &si) :
                    state(si->allocState()),
                    parent(NULL)
                {
                }

                ~Motion()
                {
                }

                /** \brief The state contained by the motion */
                base::State       *state;

                /** \brief The parent motion in the exploration tree */
                Motion            *parent;

                /** \brief The cost up to this motion */
                base::Cost        cost;

                /** \brief The incremental cost of this motion's parent to this motion (this is stored to save distance computations in the updateChildCosts() method) */
                base::Cost        incCost;

                /** \brief The set of motions descending from the current motion */
                std::vector<Motion*> children;
            };

            /** \brief Create the samplers */
            void allocSampler();

            /** \brief Generate a sample */
            void sampleUniform(base::State *statePtr);

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            // For sorting a list of costs and getting only their sorted indices
            struct CostIndexCompare
            {
                CostIndexCompare(const std::vector<base::Cost>& costs,
                                 const base::OptimizationObjective &opt) :
                    costs_(costs), opt_(opt)
                {}
                bool operator()(unsigned i, unsigned j)
                {
                    return opt_.isCostBetterThan(costs_[i],costs_[j]);
                }
                const std::vector<base::Cost>& costs_;
                const base::OptimizationObjective &opt_;
            };

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** \brief Gets the neighbours of a given motion, using either k-nearest of radius as appropriate. */
            void getNeighbors(Motion *motion, std::vector<Motion*> &nbh);

            /** \brief Removes the given motion from the parent's child list */
            void removeFromParent(Motion *m);

            /** \brief Updates the cost of the children of this node if the cost up to this node has changed */
            void updateChildCosts(Motion *m);

            /** \brief Prunes all those states which estimated total cost is higher than pruneTreeCost.
                Returns the number of motions pruned. Depends on the parameter set by setPruneStatesImprovementThreshold() */
            int pruneTree(const base::Cost pruneTreeCost);

            /** \brief Deletes (frees memory) the motion and its children motions. */
            void deleteBranch(Motion *motion);

            /** \brief Computes the solution cost heuristically as the cost to come from start to the motion plus
                 the cost to go from the motion to the goal. If \e estimate is true, a heuristic estimate of the
                 cost to come is used; otherwise, the current cost to come is used. */
            base::Cost solutionHeuristic(const Motion *motion, const bool estimate = true) const;

            /** \brief Computes the cost-to-go heuristically for goal states and goal regions using the motionCost given by the optimization objective.*/
            base::Cost defaultCostToGoHeuristic(const base::State *state, const base::Goal *goal) const;

            /** \brief Count the number of vertices that lay outside the informed subset and store the number in numVerticesWorseThanSoln_.
            This is a sort of virtual pruning that avoids the problem of removing promising descendents simply
            because of the quality of their ancestors.
            \todo Replace with an actually pruning method. */
            void countNumberOfVerticesWorseThanSoln();

            /** \brief State sampler */
            base::StateSamplerPtr                          sampler_;

            /** \brief An informed sampler */
            base::InformedSamplerPtr                       infSampler_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            boost::shared_ptr< NearestNeighbors<Motion*> > nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
            double                                         goalBias_;

            /** \brief The maximum length of a motion to be added to a tree */
            double                                         maxDistance_;

            /** \brief The random number generator */
            RNG                                            rng_;

            /** \brief Option to use k-nearest search for rewiring */
            bool                                           useKNearest_;

            /** \brief The rewiring factor, s, so that r_rrg = s \times r_rrg* > r_rrg* (or k_rrg = s \times k_rrg* > k_rrg*) */
            double                                         rewireFactor_;

            /** \brief A constant for k-nearest rewiring calculations */
            double                                         k_rrg_;
            /** \brief A constant for r-disc rewiring calculations */
            double                                         r_rrg_;

            /** \brief Option to delay and reduce collision checking within iterations */
            bool                                           delayCC_;

            /** \brief Objective we're optimizing */
            base::OptimizationObjectivePtr                 opt_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion                                         *lastGoalMotion_;

            /** \brief A list of states in the tree that satisfy the goal condition */
            std::vector<Motion*>                           goalMotions_;

            /** \brief If this value is set to true, tree pruning will be enabled. */
            bool                                           prune_;

            /** \brief The tree is only pruned is the percentage of states to prune is above this threshold (between 0 and 1). */
            double                                         pruneStatesThreshold_;

            /** \brief Option to use informed sampling */
            bool                                           useInformedSampling_;

            /** \brief The number of attempts to make at informed sampling */
            unsigned int                                   numInfAttempts_;

            /** \brief The number of vertices in the graph that cannot improve the solution. This is used by Informed RRT* to calculate the number of samples in the sub planning problem.*/
            unsigned int                                   numVerticesWorseThanSoln_;

            struct PruneScratchSpace { std::vector<Motion*> newTree, toBePruned, candidates; } pruneScratchSpace_;

            /** \brief Stores the Motion containing the last added initial start state. */
            Motion *                                       startMotion_;

            ///////////////////////////////////////
            // Planner progress property functions
            std::string getIterationCount() const
            {
                return boost::lexical_cast<std::string>(iterations_);
            }
            std::string getCollisionCheckCount() const
            {
                return boost::lexical_cast<std::string>(collisionChecks_);
            }
            std::string getBestCost() const
            {
                return boost::lexical_cast<std::string>(bestCost_);
            }

            //////////////////////////////
            // Planner progress properties
            /** \brief Number of iterations the algorithm performed */
            unsigned int                                   iterations_;
            /** \brief Number of collisions checks performed by the algorithm */
            unsigned int                                   collisionChecks_;
            /** \brief Best cost found so far by algorithm */
            base::Cost                                     bestCost_;
        };
    }
}

#endif
