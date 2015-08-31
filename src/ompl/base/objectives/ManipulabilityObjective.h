/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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

/* Author: Luis G. Torres */

#ifndef OMPL_BASE_OBJECTIVES_MANIPULABILITY_OBJECTIVE_
#define OMPL_BASE_OBJECTIVES_MANIPULABILITY_OBJECTIVE_

#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/SafeStateValidityChecker.h"

namespace ompl
{
    namespace base
    {
        /** \brief Objective for attempting to maximize the safety along a path. */
        class ManipulabilityObjective : public OptimizationObjective
        {
        public:

            ManipulabilityObjective(const SpaceInformationPtr &si);

            ManipulabilityObjective(const SpaceInformationPtr &si, Cost cost_threshold);

            virtual bool isSymmetric() const;

            /** \brief Defined as the safety of the state \e s, which is computed using the StateValidityChecker in this objective's SpaceInformation */
            virtual Cost stateCost(const State *s) const;

            /** \brief Since we wish to maximize safety, and costs are equivalent to path safety, we return the greater of the two cost values. */
            virtual bool isCostBetterThan(Cost c1, Cost c2) const;

            virtual Cost motionCost(const State *s1, const State *s2) const;

            virtual Cost combineCosts(Cost c1, Cost c2) const;

            /** \brief Returns -infinity, since any cost combined with -infinity under this objective will always return the other cost. */
            virtual Cost identityCost() const;

            /** \brief Returns +infinity, since no path safety value can be considered worse than this. */
            virtual Cost infiniteCost() const;

        private:

            ompl::base::SafeStateValidityChecker* ssvc_;

        };
    }
}

#endif
