#ifndef OMPL_GEOMETRIC_PLANNERS_CFOREST_SAFECFOREST_
#define OMPL_GEOMETRIC_PLANNERS_CFOREST_SAFECFOREST_


#include "ompl/geometric/planners/cforest/SafeCForestStateSpaceWrapper.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/tools/config/SelfConfig.h"

#include <boost/thread.hpp>

#include <vector>

//STa
#include "ompl/base/SafetyCost.h"
#include "ompl/base/objectives/SafeMultiOptimizationObjective.h"



namespace ompl
{

    namespace geometric
    {

        /**
           @anchor gCForest
           @par Short description
           CForest (Coupled Forest of Random Engrafting Search Trees) is a
           parallelization framework that is designed for single-query shortest
           path planning algorithms. It is not a planning algorithm <em>per se</em>.

           CForest is designed to be used with any random tree algorithm that operates
           in any configuration space such that: 1) the search tree has almost sure
           convergence to the optimal solution and 2) the configuration space obeys
           the triangle inequality. It relies in the OptimizationObjective set for
           the underlying planners.

           See also the extensive documentation [here](CForest.html).

           @par External documentation
           M. Otte, N. Correll, C-FOREST: Parallel Shortest Path Planning With
           Superlinear Speedup, IEEE Transactions on Robotics, Vol 20, No 3, 2013.
           DOI: [10.1109/TRO.2013.2240176](http://dx.doi.org/10.1109/TRO.2013.2240176)<br>
           [[PDF]](http://ieeexplore.ieee.org/ielx5/8860/6522877/06425493.pdf?tp=&amp;arnumber=6425493&amp;isnumber=6522877)
        */

        /** \brief Coupled Forest of Random Engrafting Search Trees */
        class SafeCForest : public base::Planner
        {
        public:

        	SafeCForest(const base::SpaceInformationPtr &si);

            virtual ~SafeCForest();

            virtual void getPlannerData(base::PlannerData &data) const;

            virtual void clear();

            /** \brief Add an specific planner instance. */
            template <class T>
            void addPlannerInstance()
            {
                base::SafeCForestStateSpaceWrapper *scfspace = new base::SafeCForestStateSpaceWrapper(this, si_->getStateSpace().get());
                base::StateSpacePtr space(scfspace);
                base::SpaceInformationPtr si(new base::SpaceInformation(space));
                si->setStateValidityChecker(si_->getStateValidityChecker());
                si->setMotionValidator(si_->getMotionValidator());
                base::PlannerPtr planner(new T(si));
                scfspace->setPlanner(planner.get());
                addPlannerInstanceInternal(planner);
            }

            /** \brief Add an specific planner instance. */
            template <class T>
            void addPlannerInstances(std::size_t num = 2)
            {
                planners_.reserve(planners_.size() + num);
                for (std::size_t i = 0 ; i < num; ++i)
                {
                    addPlannerInstance<T>();
                }
            }

            /** \brief Remove all planner instances */
            void clearPlannerInstances()
            {
                planners_.clear();
            }
            /** \brief Return an specific planner instance. */
            base::PlannerPtr& getPlannerInstance(const std::size_t idx)
            {
                return planners_[idx];
            }

            virtual void setup();

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            void addSampler(base::StateSamplerPtr sampler)
            {
                addSamplerMutex_.lock();
                samplers_.push_back(sampler);
                addSamplerMutex_.unlock();
            }

            /** \brief Option to control whether the tree is pruned during the search. */
            void setPrune(const bool prune)
            {
                prune_ = prune;
            }

            /** \brief Get the state of the pruning option. */
            bool getPrune() const
            {
                return prune_;
            }

            /** \brief Set default number of threads to use when no planner instances are specified by the user. */
            void setNumThreads(unsigned int numThreads = 0);

            /** \brief Get default number of threads used by CForest when no planner instances are specified by the user. */
            unsigned int getNumThreads()
            {
                return numThreads_;
            }

            /** \brief Get best cost among all the planners. */
            std::string getBestCost() const;

            /** \brief Get number of paths shared by the algorithm. */
            std::string getNumPathsShared() const;

            /** \brief Get number of states actually shared by the algorithm. */
            std::string getNumStatesShared() const;

        private:

            /** \brief Helper function to add a planner instance. */
            void addPlannerInstanceInternal(const base::PlannerPtr &planner);

            /** \brief Callback to be called everytime a new, better solution is found by a planner. */
            void newSolutionFound(const base::Planner *planner, const std::vector<const base::State *> &states, const base::SafetyCost cost);

        protected:

            /** \brief Manages the call to solve() for each individual planner. */
            void solve(base::Planner *planner, const base::PlannerTerminationCondition &ptc);

            /** \brief Optimization objective taken into account when planning. */
            base::OptimizationObjectivePtr               opt_;

            /** \brief The set of planners to be used. */
            std::vector<base::PlannerPtr>                planners_;

            /** \brief The set of sampler allocated by the planners */
            std::vector<base::StateSamplerPtr>           samplers_;

            /** \brief Stores the states already shared to check if a specific state has been shared. */
            boost::unordered_set<const base::State *>    statesShared_;

            /** \brief Number of paths shared among threads. */
            unsigned int                                 numPathsShared_;

            /** \brief Number of states shared among threads. */
            unsigned int                                 numStatesShared_;

            /** \brief Mutex to control the access to the newSolutionFound() method. */
            boost::mutex                                 newSolutionFoundMutex_;

            /** \brief Mutex to control the access to samplers_ */
            boost::mutex                                 addSamplerMutex_;

            /** \brief Flag to control the tree pruning. */
            bool                                         prune_;

            /** \brief Default number of threads to use when no planner instances are specified by the user */
            unsigned int                                 numThreads_;

            //STa
            base::SafetyCost                            bestCost_;
            base::SafeMultiOptimizationObjective* 	    safe_multi_opt_;

            /** \brief If this variable is disabled, the framework uses only standard parallelization   */
            bool cforestEnabled_;

            //STa test
            ompl::time::point init_;

        };
    }
}

#endif
