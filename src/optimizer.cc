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
 * \file src/optimizer.cc
 *
 * \brief Implementation of Optimizer.
 */

#include <iostream>

#include <KineoWorks2/kwsValidatorCfgCollision.h>
#include <KineoWorks2/kwsDevice.h>
#include <KineoWorks2/kwsValidatorDPCollision.h>
#include <KineoWorks2/kwsSMLinear.h>
#include <KineoWorks2/kwsAdaptiveShortcutOptimizer.h>

//FIXME {update later hpp-util in robotpkg to include sstream}
#include <sstream>
#include <hpp/util/debug.hh>
#include <kwsPlus/directPath/bezierSteeringMethod.h>
#include <kwsPlus/directPath/bezierDirectPath.h>

#include <kws/hash-optimizer/optimizer.hh>

#include "kws/bezier-optimizer/optimizer.hh"

namespace kws
{
  namespace bezieroptimizer
  {
    ktStatus Optimizer::init (const OptimizerWkPtr& i_weakPtr)
    {
      if (KD_ERROR == CkwsPathOptimizer::init (i_weakPtr))
      {
	hppDout (error, "Optimizer::init failed");
	return KD_ERROR;
      }
      else hppDout (info, "Optimizer::init successful");
      
      return KD_OK;
    }

    OptimizerShPtr Optimizer::create (unsigned int i_nbLoops,
				      unsigned int i_bezierTries,
				      double i_double)
    {
      Optimizer* ptr = new Optimizer (i_nbLoops, i_bezierTries, i_double);
      OptimizerShPtr shPtr (ptr);

      if (ptr->init (shPtr) != KD_OK)
	{
	  shPtr.reset ();
	}

      return shPtr;
    }

    Optimizer::~Optimizer ()
    {
    }

    ktStatus Optimizer::doOptimizePath (const CkwsPathShPtr& io_path)
    {
      CkwsAdaptiveShortcutOptimizerShPtr basicOptimizer
	= CkwsAdaptiveShortcutOptimizer::create ();
      basicOptimizer->maxNbLoop (NbOptimizationLoops ());
      CkwsPathShPtr copyPath = CkwsPath::createCopy (io_path);

      if (KD_ERROR == basicOptimizer->optimizePath (copyPath))
	{
	  hppDout(error, "Basic optimization could not be completed");
	  return KD_ERROR;
	}
 
      CkwsValidatorDPCollisionShPtr dpValidator;
      CkwsValidatorCfgCollisionShPtr cfgValidator;
      if (KD_ERROR == retrieveValidators (io_path, dpValidator, cfgValidator))
	return KD_ERROR;

      CkwsSMLinearShPtr linearSM = CkwsSMLinear::create ();
      CbezierSteeringMethodShPtr bezierSM = CbezierSteeringMethod::create ();
      CkwsPathShPtr bezierPath = CkwsPath::create (device ());

      unsigned int configsNumber = copyPath->countConfigurations ();
      CkwsPathShPtr outPath = CkwsPath::create (device ());
      if (configsNumber == 2)
	{
	  outPath = io_path;
	  return KD_OK;
	}
      
      bool holonomicStart = true; //DP starts with holonomic constraints
      bool holonomicEnd = false; //DP ends with holonomic constraints
      bool newPathSection = true; //new section starts when failed to append bezier
      bool endOfPath = false; //path's end has been reached
      bool canContinue = true;
      CkwsConfig interLeftCfg (device ());
      CkwsConfig interRightCfg (device ());
      CkwsConfig lastCfg (device ());
      CkwsConfig ithCfg (device ());
      CkwsConfig ithNextCfg (device ());
      CkwsConfig ithSecondNextCfg (device ());

      CkwsPathOptimizerShPtr hashOptim
	= kws::hashoptimizer::Optimizer::create (1, 6*stepSize (), 3);
      hashOptim->distance (distance ());
      CkwsPathShPtr hashPath = CkwsPath::create (device ());
      
      unsigned int i = 0;
      while (i < (configsNumber - 2) && canContinue)
	{
	  std::cout << i << std::endl;
	  copyPath->getConfiguration (i, ithCfg);
	  copyPath->getConfiguration (i + 1, ithNextCfg);
	  copyPath->getConfiguration (i + 2, ithSecondNextCfg);
	  if (i == configsNumber - 3)
	    endOfPath = true;

	  //try to reorient interLeftCfg
	  if (canContinue && newPathSection)
	    {
	      holonomicStart = true;
	      holonomicEnd = false;
	      if (KD_ERROR == intermediateConfig (ithCfg, ithNextCfg,
						  holonomicStart, holonomicEnd, 
						  cfgValidator,
						  interLeftCfg))
		{
		  hppDout (warning, "invalid interLeftCfg");
		  if (KD_ERROR
		      == hashPath->appendDirectPath (copyPath->directPath (i)))
		    hppDout (error, "could not append direct path " << i);
		  lastCfg = ithNextCfg;
		  if (endOfPath)
		    {
		      if (KD_ERROR == hashPath
			  ->appendDirectPath (copyPath->directPath (i + 1)))
			hppDout (error, "could not append direct path "
				 << i + 1);
		      // else std::cout << "appended DP " << std::endl;
		      lastCfg = ithSecondNextCfg;
		    }
		  newPathSection = true;
		  canContinue = false;
		  i++;
		}
	      // else 
	      //   std::cout << "intermediate left config valid" << std::endl;
	    }

	  //try to create linear left DP
	  if (canContinue && newPathSection)
	    {
	      CkwsDirectPathShPtr linearLeftDP 
		= linearSM->makeDirectPath (ithCfg, interLeftCfg);
	      dpValidator->validate (*linearLeftDP);
	      if (!linearLeftDP->isValid ())
		{
		  hppDout (warning, "invalid linearLeftDP");
		  if (KD_ERROR
		      == hashPath->appendDirectPath (copyPath->directPath (i)))
		    hppDout (error, "could not append direct path " << i);
		  lastCfg = ithNextCfg;
		  if (endOfPath)
		    {
		      if (KD_ERROR == hashPath
			  ->appendDirectPath (copyPath->directPath (i + 1)))
			hppDout (error, "could not append direct path "
				 << i + 1);
		      lastCfg = ithSecondNextCfg;
		    }
		  newPathSection = true;
		  canContinue = false;
		  i++;
		}
	      // else std::cout << "linear left DP valid" << std::endl;
	    }

	  //try to reorient interRightCfg
	  if (canContinue)
	    {
	      holonomicStart = false;
	      if (endOfPath)
		holonomicEnd = true;
	      else holonomicEnd = false;
	      if (newPathSection)
		{
		  if (KD_ERROR == intermediateConfig (ithNextCfg, ithSecondNextCfg,
						      holonomicStart, holonomicEnd,
						      cfgValidator,
						      interRightCfg))
		    {
		      hppDout (warning, "invalid interRightCfg");
		      if (KD_ERROR == hashPath
			  ->appendDirectPath (copyPath->directPath (i)))
			hppDout (error, "could not append direct path " << i);
		      lastCfg = ithNextCfg;
		      if (endOfPath)
			{
			  if (KD_ERROR == hashPath
			      ->appendDirectPath (copyPath->directPath (i + 1)))
			    hppDout (error, "could not append direct path "
				     << i + 1);
			  lastCfg = ithSecondNextCfg;
			}
		      newPathSection = true;
		      canContinue = false;
		      i++;
		    }
		  // else std::cout << "interRightCfg reoriented" << std::endl;
		}
	      else
		{
		  if (KD_ERROR == intermediateConfig (ithNextCfg, ithSecondNextCfg,
						      holonomicStart, holonomicEnd,
						      cfgValidator,
						      interRightCfg))
		    {
		      // std::cout << "intermediate right cfg invalid" << std::endl;
		      hppDout (warning, "invalid interRightCfg");
		      interLeftCfg = lastCfg;
		      CkwsDirectPathShPtr linearLeftDP
			= linearSM->makeDirectPath (interLeftCfg, ithNextCfg);
		      if (KD_ERROR == hashPath->appendDirectPath (linearLeftDP))
			hppDout (error, "could not append linera left DP" << i);
		      lastCfg = ithNextCfg;
		      if (endOfPath)
			{
			  if (KD_ERROR == hashPath
			      ->appendDirectPath (copyPath->directPath (i + 1)))
			    hppDout (error, "could not append direct path "
				     << i + 1);
			  lastCfg = ithSecondNextCfg;
			}
		      newPathSection = true;
		      canContinue = false;
		      i++;
		    }
		  // else std::cout << "interRightCfg reoriented" << std::endl;
		}
	    }
	
	  //try to make bezier path
	  if (canContinue)
	    {
	      if (!newPathSection)
		{
		  interLeftCfg = lastCfg;
		}
	      bezierPath = makeBezierPath (interLeftCfg, ithNextCfg, interRightCfg,
					   dpValidator, cfgValidator, 0);
	      if (!bezierPath)
		{
		  hppDout (warning, "invalid bezierPath");
		  // std::cout << "no valid bezier path" << std::endl;
		  if (newPathSection)
		    {
		      if (KD_ERROR == hashPath
			  ->appendDirectPath (copyPath->directPath (i)))
			hppDout (error, "could not append direct path " << i);
		      lastCfg = ithNextCfg;
		    }
		  else
		    {
		      CkwsDirectPathShPtr linearLeftDP
			= linearSM->makeDirectPath (interLeftCfg, ithNextCfg);
		      if (KD_ERROR
			  == hashPath->appendDirectPath (linearLeftDP))
			hppDout (error, "could not append linear left DP "
				 << i);
		      lastCfg = ithNextCfg;
		    }
		  if (endOfPath)
		    {
		      if (KD_ERROR == hashPath
			  ->appendDirectPath (copyPath->directPath (i + 1)))
			hppDout (error, "could not append direct path "
				 << i + 1);
		      lastCfg = ithSecondNextCfg;
		    }
		  newPathSection = true;
		  canContinue = false;
		  i++;
		}
	      // else std::cout << "bezier path made" << std::endl;
	    }

	  //try to make linear right DP
	  if (canContinue)
	    {
	      CkwsDirectPathShPtr linearRightDP 
		= linearSM->makeDirectPath (interRightCfg, ithSecondNextCfg);
	      dpValidator->validate (*linearRightDP);
	      if (!linearRightDP->isValid ())
		{
		  hppDout (warning, "invalid linearRightDP");
		  // std::cout << "linear right DP not valid"<< std::endl;
		  if (newPathSection)
		    {
		      if (KD_ERROR == hashPath
			  ->appendDirectPath (copyPath->directPath (i)))
			hppDout (error, "could not append direct path " << i);
		    }
		  else
		    {
		      linearRightDP 
			= linearSM->makeDirectPath (interLeftCfg, ithNextCfg);
		      if (KD_ERROR
			  == hashPath->appendDirectPath (linearRightDP))
			hppDout (error, "could not append linear right DP "
				 << i);
		    }
		  lastCfg = ithNextCfg;
		  if (endOfPath)
		    {
		      if (KD_ERROR == hashPath
			  ->appendDirectPath (copyPath->directPath (i + 1)))
			hppDout (error, "could not append direct path "
				 << i + 1);
		      lastCfg = ithSecondNextCfg;
		    }
		  newPathSection = true;
		  canContinue = false;
		  i++;
		}
	      // else std::cout << "linear right DP is valid" << std::endl;
	    }

	  // append paths to ouput
	  if (canContinue)
	    {
	      hppDout (notice, "input hashPath direct paths "
		       << hashPath->countDirectPaths ());
	      if (hashPath->countDirectPaths () > 0)
		{
		  if (KD_ERROR == hashOptim->optimizePath (hashPath))
		    hppDout (notice, "hash optimization incomplete");
		  if (KD_ERROR == outPath->appendPath (hashPath))
		    hppDout (error, "could not append optimized hash path");
		  hashPath->clear ();
		}

	      if (newPathSection)
		{
		  CkwsDirectPathShPtr linearLeftDP
		    = linearSM->makeDirectPath (ithCfg, interLeftCfg);
		  if (KD_ERROR == outPath->appendDirectPath (linearLeftDP))
		    std::cout << "linear left DP could not be appended" << std::endl;
		  // else std::cout << "linear left DP appended" << std::endl;
		  lastCfg = interLeftCfg;
		}
	      if (KD_ERROR == outPath->appendPath (bezierPath))
		{
		  std::cout << "bezier path could not be appended" << std::endl;
		}
	      else
		{ 
		  // std::cout << "bezier path appended" << std::endl;
		  lastCfg = interRightCfg;
		  newPathSection = false;
		}
	      if (endOfPath)
		{
		  CkwsDirectPathShPtr linearRightDP 
		    = linearSM->makeDirectPath (interRightCfg, ithSecondNextCfg);
		  if (KD_ERROR == outPath->appendDirectPath (linearRightDP))
		    std::cout << "linear right DP could not be appended" << std::endl;
		  // else std::cout << "linear right DP appended" << std::endl;
		  lastCfg = ithSecondNextCfg;
		}
	      i++;
	    }
	  else if (endOfPath)
	    {
	      hppDout (notice, "input hashPath direct paths "
		       << hashPath->countDirectPaths ());
      	      if (KD_ERROR == hashOptim->optimizePath (hashPath))
	      	hppDout (notice, "hash optimization incomplete");
	      hppDout (notice, "output hashPath direct paths "
		       << hashPath->countDirectPaths ());
	      if (KD_ERROR == outPath->appendPath (hashPath))
		hppDout (error, "could not append optimized hash path");
	      hashPath->clear ();
	    }
	  
	  canContinue = true;
	}
      // o_path = CkwsPath::create (device ());
      // *o_path = *outPath;
      *io_path = *outPath;
      
      return KD_OK;
    }

    ktStatus Optimizer::retrieveValidators
    (const CkwsPathShPtr& i_path,
     CkwsValidatorDPCollisionShPtr& o_dpValidator,
     CkwsValidatorCfgCollisionShPtr& o_cfgValidator)
    {
      ktStatus dpSuccess = KD_OK;
      ktStatus cfgSuccess = KD_OK;

      o_dpValidator = device ()->directPathValidators ()
	->retrieve<CkwsValidatorDPCollision> ();

      if (!o_dpValidator)
	{
	  hppDout (warning, "No DP Validator found in device");
	  o_dpValidator
	    = CkwsValidatorDPCollision::create (device (),
						i_path->maxPenetration (),
						CkwsValidatorDPCollision::
						PROGRESSIVE_FORWARD);
	  if (!o_dpValidator)
	    {
	      hppDout (error, "no DP Validator created");
	      dpSuccess = KD_ERROR;
	    }
	}

      o_dpValidator
	->algorithm (CkwsValidatorDPCollision::PROGRESSIVE_FORWARD);
      o_cfgValidator = device ()->configValidators ()
	->retrieve<CkwsValidatorCfgCollision> ();

      if (!o_cfgValidator)
	{
	  hppDout (warning, "No config Validator found in device");
	  o_cfgValidator = CkwsValidatorCfgCollision::create (device ());
	  if (!o_cfgValidator)
	    {
	      hppDout (error, "no config Validator created");
	      cfgSuccess = KD_ERROR;
	    }
	}

      if (dpSuccess != KD_ERROR && cfgSuccess != KD_ERROR)
	return KD_OK;
    }

    ktStatus
    Optimizer::intermediateConfig (const CkwsConfig& i_beginConfig,
				   const CkwsConfig& i_endConfig,
				   bool i_isStartOfSection,
				   bool i_isEndOfSection,
				   const CkwsValidatorCfgCollisionShPtr&
				   i_cfgValidator,
				   CkwsConfig& o_config)
    {
      if (i_beginConfig == i_endConfig)
	{
	  hppDout (error,
		   "Begin and end configuration of direct path are the same");
	  return KD_ERROR;
	}

      bool splitInHalf = false;
      double step = stepSize ();
      double deltaX = i_endConfig.dofValue (0) - i_beginConfig.dofValue (0);
      double deltaY = i_endConfig.dofValue (1) - i_beginConfig.dofValue (1);
      double vectorNorm = sqrt (pow (deltaX, 2) + pow (deltaY, 2));

      if (step < vectorNorm/2)
	{
	  if (i_isStartOfSection)
	    {
	      o_config.dofValue (0, i_beginConfig.dofValue (0)
				 + step * deltaX/vectorNorm);
	      o_config.dofValue (1, i_beginConfig.dofValue (1)
				 + step * deltaY/vectorNorm);
	    }

	  if (i_isEndOfSection)
	    {
	      o_config.dofValue (0, i_endConfig.dofValue (0)
				 - step * deltaX/vectorNorm);
	      o_config.dofValue (1, i_endConfig.dofValue (1)
				 - step * deltaY/vectorNorm);
	    }

	  if (!i_isStartOfSection && !i_isEndOfSection)
	    splitInHalf = true;
	}
      else splitInHalf = true;

      if (splitInHalf)
	{
	  o_config.dofValue
	    (0, (i_beginConfig.dofValue (0) + i_endConfig.dofValue (0))/2);
	  o_config.dofValue
	    (1, (i_beginConfig.dofValue (1) + i_endConfig.dofValue (1))/2);
	}

      o_config.dofValue (5, atan2 (deltaY, deltaX));
      i_cfgValidator->validate (o_config);

      if (!o_config.isValid ())
	{
	  hppDout (error, "Intermediate configuration is not valid");
	  return KD_ERROR;
	}
      else return KD_OK;
    }

    CkwsPathShPtr Optimizer::
    makeBezierPath (const CkwsConfig& i_beginConfig,
		    const CkwsConfig& i_middleConfig,
		    const CkwsConfig& i_endConfig,
		    const CkwsValidatorDPCollisionShPtr& i_dpValidator,
		    const CkwsValidatorCfgCollisionShPtr& i_cfgValidator,
		    unsigned int i_int)
    {
      CkwsDeviceShPtr device = i_beginConfig.device ();
      CkwsConfig interBeginConfig (device);
      CkwsConfig interEndConfig (device);
      CkwsPathShPtr bezierPath = CkwsPath::create (device);

      if (i_int == bezierNbTries ())
	{
	  hppDout (warning, "Could not compute a valid Bezier path.");
	  bezierPath.reset ();
	  return bezierPath;
	}

      CbezierSteeringMethodShPtr bezierSM
	= CbezierSteeringMethod::create ();
      CkwsDirectPathShPtr bezierDP
	= bezierSM->makeDirectPath (i_beginConfig, i_endConfig);

      if (!bezierDP)
	hppDout (warning, "makeBezierPath:: bezier DP could not be made.");
      else
	{
	  i_dpValidator->validate (*bezierDP);

	  if (!bezierDP->isValid ())
	    {
	      hppDout (warning,
		       "makeBezierPath(" << i_int
		       << ")::bezierDP is not valid.");

	      ktStatus validatedConfigs
		= validateInterConfigs (i_beginConfig, i_middleConfig,
					i_endConfig, i_cfgValidator, i_int,
					interBeginConfig, interEndConfig, bezierPath);
	      ktStatus appendedLinearBeginDP;

	      if (KD_OK == validatedConfigs)
		appendedLinearBeginDP
		  = tryAppendLinearBeginDP (i_beginConfig, interBeginConfig,
					    i_dpValidator, i_int, bezierPath);
	      ktStatus madeLinearEndDP;

	      if (KD_OK == appendedLinearBeginDP)
		madeLinearEndDP
		  = tryMakeLinearEndDP (interEndConfig, i_endConfig,
					i_dpValidator, i_int, bezierPath);
	      ktStatus appendedRecursiveBezierPath;

	      if (KD_OK == madeLinearEndDP)
		{
		  appendedRecursiveBezierPath
		    = tryAppendRecursiveBezierPath (interBeginConfig,
						    i_middleConfig, interEndConfig,
						    i_endConfig, i_dpValidator,
						    i_cfgValidator, i_int+1,
						    bezierPath);
		}
	    }
	  else bezierPath->appendDirectPath (bezierDP);
	}

      return bezierPath;
    }

    ktStatus Optimizer::
    validateInterConfigs (const CkwsConfig& i_beginConfig,
			  const CkwsConfig& i_middleConfig,
			  const CkwsConfig& i_endConfig,
			  const CkwsValidatorCfgCollisionShPtr& i_cfgValidator,
			  unsigned int i_int,
			  CkwsConfig& o_interBeginConfig,
			  CkwsConfig& o_interEndConfig,
			  CkwsPathShPtr& o_path)
    {
      //check inter begin config
      if (KD_ERROR == intermediateConfig (i_beginConfig, i_middleConfig,
					  false, false, i_cfgValidator,
					  o_interBeginConfig))
	{
	  hppDout (warning, "makeBezierPath(" << i_int
		   << "):: interBeginConfig invalid.");
	  o_path.reset ();
	  return KD_ERROR;
	}

      //check inter end config
      if (KD_ERROR == intermediateConfig (i_middleConfig, i_endConfig,
					  false, false, i_cfgValidator,
					  o_interEndConfig))
	{
	  hppDout (warning, "makeBezierPath(" << i_int
		   << ")::interEndConfig inValid.");
	  o_path.reset ();
	  return KD_ERROR;
	}
      return KD_OK;
    }

    ktStatus Optimizer::
    tryAppendLinearBeginDP (const CkwsConfig& i_beginConfig,
			    const CkwsConfig& i_interBeginConfig,
			    const CkwsValidatorDPCollisionShPtr& i_dpValidator,
			    unsigned int i_int,
			    CkwsPathShPtr& o_path)
    {
      //check linear begin DP
      if (i_beginConfig == i_interBeginConfig)
	{
	  hppDout (warning, "makeBezierPath(" << i_int
		   << "):: linear Begin DP could not be made because extrema configurations are the same.");
	  o_path.reset ();
	  return KD_ERROR;
	}
      else
	{
	  CkwsSMLinearShPtr linearSM = CkwsSMLinear::create ();
	  CkwsDirectPathShPtr linearBeginDP
	    = linearSM->makeDirectPath (i_beginConfig, i_interBeginConfig);
	  i_dpValidator->validate (*linearBeginDP);

	  if (!linearBeginDP->isValid ())
	    {
	      hppDout (error, "makeBezierPath(" << i_int
		       << ")::linear begin DP is not valid.");
	      return KD_ERROR;
	    }
	  else
	    {
	      if (KD_ERROR == o_path->appendDirectPath (linearBeginDP))
		{
		  hppDout (error, "makeBezierPath(" << i_int
			   << ")::could not append linear begin DP.");
		  return KD_ERROR;
		}
	    }
	}

      return KD_OK;
    }

    ktStatus Optimizer::
    tryMakeLinearEndDP (const CkwsConfig& i_interEndConfig,
			const CkwsConfig& i_endConfig,
			const CkwsValidatorDPCollisionShPtr& i_dpValidator,
			unsigned int i_int,
			CkwsPathShPtr& o_path)
    {
      //check linear end DP
      if (i_interEndConfig == i_endConfig)
	{
	  hppDout (warning, "makeBezierPath(" << i_int
		   << "):: linear End DP could not be made because extrema configurations are the same.");
	  o_path.reset ();
	  return KD_ERROR;
	}
      else
	{
	  CkwsSMLinearShPtr linearSM = CkwsSMLinear::create ();
	  CkwsDirectPathShPtr linearEndDP
	    = linearSM->makeDirectPath (i_interEndConfig, i_endConfig);
	  i_dpValidator->validate (*linearEndDP);

	  if (!linearEndDP->isValid ())
	    {
	      hppDout (error, "makeBezierPath(" << i_int
		       << ")::linear end DP is not valid.");
	      return KD_ERROR;
	    }
	  else return KD_OK;
	}
    }

    ktStatus Optimizer::
    tryAppendRecursiveBezierPath (const CkwsConfig& i_interBeginConfig,
				  const CkwsConfig& i_middleConfig,
				  const CkwsConfig& i_interEndConfig,
				  const CkwsConfig& i_endConfig,
				  const CkwsValidatorDPCollisionShPtr& i_dpValidator,
				  const CkwsValidatorCfgCollisionShPtr& i_cfgValidator,
				  unsigned int i_int,
				  CkwsPathShPtr& o_path)
    {
      //make and append recursive bezier path
      CkwsPathShPtr recursiveBezierPath
	= makeBezierPath (i_interBeginConfig, i_middleConfig,
				      i_interEndConfig, i_dpValidator, i_cfgValidator,
				      i_int + 1);

      if (!recursiveBezierPath)
	{
	  hppDout (error, "makeBezierPath(" << i_int
		   << ")::bezier Path could not be made.");
	  o_path.reset ();
	  return KD_ERROR;
	}
      else
	{
	  hppDout (notice, "makeBezierPath(" << i_int
		   << ")::bezier Path was found.");

	  if (KD_ERROR
	      == o_path->appendPath (recursiveBezierPath))
	    {
	      hppDout (error, "makeBezierPath(" << i_int
		       << ")::could not append bezier path.");
	      o_path.reset ();
	    }

	  CkwsSMLinearShPtr linearSM = CkwsSMLinear::create ();
	  CkwsDirectPathShPtr linearEndDP
	    = linearSM->makeDirectPath (i_interEndConfig, i_endConfig);

	  if (KD_ERROR
	      == o_path->appendDirectPath (linearEndDP))
	    {
	      hppDout (error, "makeBezierPath(" << i_int
		       << ")::could not append linear end DP.");
	      o_path.reset ();
	    }
	}

      return KD_OK;
    }

    Optimizer::Optimizer (unsigned int i_nbLoops, unsigned int i_bezierTries,
			  double i_double) : CkwsPathOptimizer ()
    {
      max_nb_optimization_loops = i_nbLoops;
      bezier_nb_tries_ = i_bezierTries;
      human_size_ = i_double;
      step_size_ = human_size_/6;
    }

    unsigned int Optimizer::NbOptimizationLoops ()
    {
      return max_nb_optimization_loops;
    }

    unsigned int Optimizer::bezierNbTries ()
    {
      return bezier_nb_tries_;
    }

    double Optimizer::stepSize ()
    {
      return step_size_;
    }

  } // end of namespace bezieroptimizer.
} // end of namespace kws.
