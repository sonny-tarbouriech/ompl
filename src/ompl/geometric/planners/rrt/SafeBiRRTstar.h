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

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_SAFEBIRRTSTAR_
#define OMPL_GEOMETRIC_PLANNERS_RRT_SAFEBIRRTSTAR_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/datastructures/NearestNeighbors.h"

//STa
#include "ompl/base/objectives/SafeMultiOptimizationObjective.h"
#include "ompl/base/SafetyCost.h"
#include "ompl/base/SafeMotionValidator.h"

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
class SafeBiRRTstar : public base::Planner
{
public:

	SafeBiRRTstar(const base::SpaceInformationPtr &si);

	virtual ~SafeBiRRTstar();

	virtual void getPlannerData(base::PlannerData &data) const;

	virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

	virtual void clear();


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



	virtual void setup();

	//STa
	void getOptimalSafetyObjective();
	base::SafeMultiOptimizationObjective* getSafeMultiOptimizationObjective() const
	{
		return safe_multi_opt_;
	}

	///////////////////////////////////////
	// Planner progress property functions
	std::string getIterationCount() const
	{
		return boost::lexical_cast<std::string>(iterations_);
	}
	std::string getBestCost() const
	{
		return boost::lexical_cast<std::string>(bestCost_);
	}

protected:

	class treeConnexion;

	/** \brief Representation of a motion */
	class Motion
	{
	public:
		/** \brief Constructor that allocates memory for the state. This constructor automatically allocates memory for \e state, \e cost, and \e incCost */
		Motion(const base::SpaceInformationPtr &si) :
			state(si->allocState()),
			parent(NULL),
			root(NULL),
			isGoalTree(false),
			otherTreeConnexionFailure(0)


	{
	}


		~Motion()
		{
		}


		//STa
		bool isParent(Motion* other)
		{
			if (parent == NULL)
				return false;
			else if (parent == other)
				return true;
			else
				return parent->isParent(other);
		}



		/** \brief The state contained by the motion */
		base::State       *state;

		/** \brief The parent motion in the exploration tree */
		Motion            *parent;

		/// \brief Pointer to the root of the tree this motion is
				/// contained in.
		const base::State *root;

		/** \brief The cost up to this motion */
		base::SafetyCost        cost;

		/** \brief The incremental cost of this motion's parent to this motion (this is stored to save distance computations in the updateChildCosts() method) */
		base::SafetyCost        incCost;

		/** \brief The set of motions descending from the current motion */
		std::vector<Motion*> children;

		//STa
		bool 				isGoalTree;
		base::SafetyCost 	heuristicCost;

		std::vector<treeConnexion*> connexion;

		size_t 			otherTreeConnexionFailure;
	};

	class treeConnexion
	{
	public:
		treeConnexion(){};
		~treeConnexion();

		void updateWholeMotionCost(const base::SafeMultiOptimizationObjective* safe_multi_opt);
		std::vector<base::State*> getPath();


		Motion* startTreeMotion;
		Motion* goalTreeMotion;
		base::SafetyCost wholeMotionCost;

		size_t index;
	};

	 typedef boost::shared_ptr< NearestNeighbors<Motion*> > TreeData;


	/** \brief Free the memory allocated by this planner */
	void freeMemory();

	// For sorting a list of costs and getting only their sorted indices
	struct CostIndexCompare
	{
		CostIndexCompare(const std::vector<base::SafetyCost>& costs,
				const base::OptimizationObjective &opt) :
					costs_(costs), opt_(opt)
		{}
		bool operator()(unsigned i, unsigned j)
		{
			return static_cast<const base::SafeMultiOptimizationObjective*>(&opt_)->isSafetyCostBetterThan(costs_[i],costs_[j]);
		}
		const std::vector<base::SafetyCost>& costs_;
		const base::OptimizationObjective &opt_;
	};

	/** \brief Compute distance between motions (actually distance between contained states) */
	double distanceFunction(const Motion *a, const Motion *b) const
	{
		//            	if (a->has_goal_state && b->has_goal_state)
		//            	{
		//            		return si_->getStateSpace()->getMaximumExtent();
		//            	}
		return si_->distance(a->state, b->state);
	}

	/** \brief Removes the given motion from the parent's child list */
	void removeFromParent(Motion *m);

	/** \brief Updates the cost of the children of this node if the cost up to this node has changed */
	void updateChildCosts(Motion *m);



	/** \brief Computes the Cost To Go heuristically as the cost to come from start to motion plus
                 the cost to go from motion to goal. If \e shortest is true, the estimated cost to come
                 start-motion is given. Otherwise, this cost to come is the current motion cost. */
	base::SafetyCost costToGo(const base::State* state, const base::SafetyCost state_cost , const bool shortest = true) const;
	base::SafetyCost costToGo(const Motion *m1, const Motion *m2 , const base::SafetyCost motionCost, const bool shortest = true) const;

	enum GrowResult
	{
		/// No extension was possible
		FAILED,
		/// Progress was made toward extension
		ADVANCED,
		/// The desired state was reached during extension
		SUCCESS
	};

	/// \brief Extend \e tree toward the state in \e rmotion.
	/// Store the result of the extension, if any, in result
	GrowResult extendTree(Motion* toMotion, TreeData& tree, Motion*& result);

    /// \brief Extend \e tree from \e nearest toward \e toMotion.
    /// Store the result of the extension, if any, in result
    GrowResult extendTree(Motion* toMotion, TreeData& tree, Motion*& result, Motion* nearest);

    /// \brief Attempt to connect \e tree to \e nmotion, which is in
    /// the other tree.  \e xmotion is scratch space and will be overwritten
    bool connectTrees(Motion* nmotion, TreeData& tree, Motion* xmotion);



	/** \brief State sampler */
	base::StateSamplerPtr                          sampler_;


     /// \brief The start tree
     TreeData 										tStart_;

     /// \brief The goal tree
     TreeData 										tGoal_;

	/** \brief A nearest-neighbors datastructure containing the tree of motions */
//	boost::shared_ptr< NearestNeighbors<Motion*> > nn_;


	/** \brief The maximum length of a motion to be added to a tree */
	double                                         maxDistance_;

	/** \brief The random number generator */
	RNG                                            rng_;


	/** \brief Objective we're optimizing */
	base::OptimizationObjectivePtr                 opt_;

	/** \brief The most recent goal motion.  Used for PlannerData computation */
	Motion                                         *lastGoalMotion_;



	/** \brief Stores the Motion containing the last added initial start state. */
	Motion *                                       startMotion_;

	//////////////////////////////
	// Planner progress properties
	/** \brief Number of iterations the algorithm performed */
	unsigned int                                   iterations_;
	/** \brief Best cost found so far by algorithm */
	base::SafetyCost                                   bestCost_;
	size_t 												bestIndex_;

	//STa
	base::SafeMotionValidator* 			    safe_motion_validator_;
	base::SafeMultiOptimizationObjective* 	    safe_multi_opt_;
	base::SafeStateValidityChecker*		    ssvc_;
	bool 									fast_dist_;
	double 									travel_dist_limit_;


	double                                         improveSolutionBias_;

	std::vector<treeConnexion*> 			connexion_;

	bool							checkForSolution_;

	// e+e/d.  K-nearest RRT*
	double k_rrg_;
	unsigned int               rewireTest_ ;
	unsigned int               statesGenerated_;

    /// \brief The range at which the algorithm will attempt to connect
    /// the two trees.
    double  connectionRange_;




};
}
}

#endif
