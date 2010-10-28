// Copyright (C) 2010 by Antonio El Khoury.
//
// This file is part of the kws-bezier-optimizer.
//
// kws-bezier-optimizer is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either
// version 3 of the License, or (at your option) any later version.
//
// kws-bezier-optimizer is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with kws-bezier-optimizer.  If not, see
// <http://www.gnu.org/licenses/>.


/**
 * \brief Declaration of Optimizer.
 */

#ifndef KWS_BEZIER_OPTIMIZER_OPTIMIZER_HH_
# define KWS_BEZIER_OPTIMIZER_OPTIMIZER_HH_

#include "KineoWorks2/kwsPathOptimizer.h"

namespace kws
{
  namespace bezieroptimizer
  {
    /// \brief This class has been generated automatically
    /// See Doxygen documentation to learn how to document your classes:
    /// http://www.stack.nl/~dimitri/doxygen/

    KIT_PREDEF_CLASS (Optimizer);

    class Optimizer : public CkwsPathOptimizer
    {
    public:
      virtual ktStatus init (const OptimizerWkPtr& i_weakPtr);

      /// \sa Optimizer ()
      static OptimizerShPtr create (unsigned int i_nbLoops,
					 unsigned int i_bezierTries,
					 double i_double);

      virtual ~Optimizer ();

    protected:
      /// \brief Non-atomic implementation of Bezier Optimizer
      ///
      /// This is the method that contains the optimization
      /// algorithm. The input path is a piecewise linear path, and is
      /// smoothed with Bezier curves to respect a nonholonomic
      /// constraint.
      ///
      /// \param io_path Input piecewise linear path. Optimized path
      /// is then put here, can be null if optimization fails.
      ///
      /// \return KD_OK | KD_ERROR
      virtual ktStatus doOptimizePath (const CkwsPathShPtr& io_path);

      /// \brief retrieves collision validators from input path.
      ///
      /// \param i_path Input path.
      ///
      /// \retval o_dpValidator Output direct path collision validator
      ///
      /// \retval o_cfgValidator Output configuration collision validator
      ///
      /// \return KD_OK | KD_ERROR
      virtual ktStatus
      retrieveValidators (const CkwsPathShPtr& i_path,
			  CkwsValidatorDPCollisionShPtr& o_dpValidator,
			  CkwsValidatorCfgCollisionShPtr& o_cfgValidator);

      /// \brief Find intermediate configuration between begin and end
      /// configuration of a direct path.
      ///
      /// Three cases present theselves:
      ///
      /// - If the direct path is in the beginning of a new section,
      /// output configuration is at distance of stepSize () away from
      /// begin configuration.
      ///
      /// - If the direct path is in the end of a section, output
      /// configuration is at distance of stepSize () away from end
      /// configuration.
      ///
      /// - In the general case output configuration will be in the
      /// exact middle of the direct path.
      ///
      /// \param i_beginConfig Begin configuration of direct path.
      ///
      /// \param i_endConfig End configuration of direct path.
      ///
      /// \param i_isStartOfSection Is set to True if the direct path is
      /// at the beginning of a new Section.
      ///
      /// \param i_isEndOfSection is set to False if the direct path is
      /// at the end of a section.
      ///
      /// \param i_cfgValidator Configuration Validator.
      ///
      /// \retval Output intermediate configuration.
      ///
      /// \return KD_OK | KD_ERROR
      virtual ktStatus
      intermediateConfig (const CkwsConfig& i_beginConfig,
			  const CkwsConfig& i_endConfig,
			  bool i_isStartOfSection,
			  bool i_isEndOfSection,
			  const CkwsValidatorCfgCollisionShPtr& i_cfgValidator,
			  CkwsConfig& o_config);

      /// \brief Smoothing of portion of path.
      ///
      /// This method tries to find a collision-free path that
      /// connects begin and configuration. This path is composed of
      /// linear direct and Bezier direct paths joined together to
      /// smooth the original path.  The algorithm runs recursively
      /// and stops:
      ///
      /// - If a collision-free path is found.
      ///
      /// - If a realigned configuration is in collision, or if the
      /// maximum number of recursion loops is reached. This means
      /// there is no collision-free Bezier path joining begin and end
      /// configuration, and output is NULL.
      ///
      /// \param i_beginConfig Begin configuration.
      ///
      /// \param i_middleConfig Middle configuration. It is used to
      /// give the direction of the two direct paths.
      ///
      /// \param i_endConfig End configuration.
      ///
      /// \param i_dpValidator Direct path collision validator.
      ///
      /// \param i_cfgValidator Configuration collision validator.
      ///
      /// \param i_int Integer that keeps track of the number of
      /// recursion loops.
      ///
      /// \return Output is a collision-free Linear+Bezier path, can
      /// be null if unsuccessful.
      CkwsPathShPtr
      makeBezierPath (const CkwsConfig& i_beginConfig,
		      const CkwsConfig& i_middleConfig,
		      const CkwsConfig& i_endConfig,
		      const CkwsValidatorDPCollisionShPtr& i_dpValidator,
		      const CkwsValidatorCfgCollisionShPtr& i_cfgValidator,
		      unsigned int i_int);

      /// \brief Validation of realigned configurations and
      /// corresponding direct paths.
      ///
      /// This method checks whether the orientation modifications
      /// result in collisions or not. If there are collisions then
      /// the Bezier path cannot be constructed and the whole path is
      /// discarded. If there are no collisions, KD_OK is returned and
      /// MakeBezierPath continues by trying to build a Bezier curve.
      ///
      /// \sa makeBezierPath
      ///
      /// \param i_beginConfig Begin configuration.
      ///
      /// \param i_middleConfig Middle configuration.
      ///
      /// \param i_endConfig End configuration.
      ///
      /// \param i_dpValidator Direct path collision validator.
      ///
      /// \param i_cfgValidator Configuration collision validator.
      ///
      /// \param i_int Integer that keeps track of the number of
      /// recursion loops.
      ///
      /// \retval io_bezierPath This path is constructed step by step
      /// in MakeBezierPath. This method adds here a linear direct
      /// path at the beginning of the path if it has been sucessfully
      /// validated, in preparation of further constructions.
      ///
      /// \return KD_OK | KD_ERROR
      ktStatus
      validateInterConfigs (const CkwsConfig& i_beginConfig,
			    const CkwsConfig& i_middleConfig,
			    const CkwsConfig& i_endConfig,
			    const CkwsValidatorCfgCollisionShPtr& i_cfgValidator,
			    unsigned int i_int,
			    CkwsConfig& o_interBeginConfig,
			    CkwsConfig& o_interEndConfig,
			    CkwsPathShPtr& io_bezierPath);

      //FIXME {doxygen}
      ktStatus
      tryAppendLinearBeginDP (const CkwsConfig& i_beginConfig,
			      const CkwsConfig& i_interBeginConfig,
			      const CkwsValidatorDPCollisionShPtr& i_dpValidator,
			      unsigned int i_int,
			      CkwsPathShPtr& o_path);

      //FIXME {doxygen}
      ktStatus
      tryMakeLinearEndDP (const CkwsConfig& i_interEndConfig,
			  const CkwsConfig& i_endConfig,
			  const CkwsValidatorDPCollisionShPtr& i_dpValidator,
			  unsigned int i_int,
			  CkwsPathShPtr& o_path);

      //FIXME {doxygen}
      ktStatus
      tryAppendRecursiveBezierPath (const CkwsConfig& i_interBeginConfig,
				    const CkwsConfig& i_middleConfig,
				    const CkwsConfig& i_interEndConfig,
				    const CkwsConfig& i_endConfig,
				    const CkwsValidatorDPCollisionShPtr& i_dpValidator,
				    const CkwsValidatorCfgCollisionShPtr& i_cfgValidator,
				    unsigned int i_int,
				    CkwsPathShPtr& o_path);

      /// \brief Constructor
      ///
      /// \param i_nbLoops Maximum number of loops to be run by
      /// Adaptive Shorcut Optimizer.
      ///
      /// \param i_bezierTries Maximum number of recursions to be run
      /// when replace a section of the path with a Bezier curve.
      ///
      /// \param i_double Human size.
      Optimizer (unsigned int i_nbLoops, unsigned int i_bezierTries,
		 double i_double);
      
      /// \brief Get value of max_nb_loop_optimizer_
      unsigned int NbOptimizationLoops ();

      /// \brief Get value of bezier_nb_tries_.
      unsigned int bezierNbTries ();

      /// \brief Get value of step_size_.
      double stepSize ();

    private:
      unsigned int max_nb_optimization_loops;

      unsigned int bezier_nb_tries_;

      double human_size_;

      double step_size_;

      OptimizerWkPtr optimizer_;
    };
  } // end of namespace bezieroptimizer.
} // end of namespace kws.

#endif //! KWS_BEZIER_OPTIMIZER_OPTIMIZER_HH_
