// RRR - Robust Loop Closing over Time
// Copyright (C) 2014 Y.Latif, C.Cadena, J.Neira
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef BACKEND_G2O_HPP_
#define BACKEND_G2O_HPP_

#include "backEndInterface.hpp"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

using namespace g2o;

G2O_USE_TYPE_GROUP(slam2d);

template <typename VertexType, typename EdgeType>
class G2O_Interface : public BaseBackend
{
	typedef std::map<IntPair, g2o::HyperGraph::Edge *>
		IntPairToEdgePtrMap;

	typedef std::vector<g2o::HyperGraph::Edge *>
		EdgePtrVector;

	typedef g2o::SparseOptimizer OptimizerType;

	EdgePtrVector odomteryEdges;
	IntPairToEdgePtrMap loopclosureEdges;

	g2o::SparseOptimizer *optimizer;
	g2o::OptimizableGraph::EdgeSet activeEdges;

	bool initialized;

public:
	G2O_Interface()
	{
		optimizer = NULL;
		initialized = false;
	}

	bool setOptimizer(void *opt)
	{
		if (optimizer == NULL)
		{
			optimizer = (OptimizerType *)opt;
			initialized = true;
			store();
		}
		else
		{
			std::cerr << "Already existing optimizer?" << std::endl;
			return false;
		}
		return true;
	}

	bool getLoopClosures(IntPairSet &loops)
	{
		if (optimizer == NULL)
		{
			std::cerr << "Please read in a g2o file or pass the pointer to an existing optimizer before calling getLoopClosures()" << std::endl;
			return false;
		}

		g2o::OptimizableGraph::EdgeSet::iterator
			eIt = optimizer->edges().begin(),
			eEnd = optimizer->edges().end();

		for (; eIt != eEnd; eIt++)
		{

			int e1 = (*eIt)->vertices()[0]->id();
			int e2 = (*eIt)->vertices()[1]->id();
			if (std::abs(e1 - e2) > 1)
			{
				loops.insert(IntPair(e1, e2));
				loopclosureEdges[IntPair(e1, e2)] = *eIt;
			}
			else
			{
				odomteryEdges.push_back(*eIt);
			}
		}
		//std::cout<<"# Odom: "<<odomteryEdges.size()<<std::endl;
		//std::cout<<"# LC :"<<loops.size()<<std::endl;
		return true;
	}

	virtual ~G2O_Interface() {}

	int vertexCount() { return optimizer->vertices().size(); };

	bool store() // store the current graph state
	{
		g2o::OptimizableGraph::VertexIDMap::iterator
			vIt = optimizer->vertices().begin(),
			vEnd = optimizer->vertices().end();

		for (; vIt != vEnd; vIt++)
			static_cast<VertexType *>(vIt->second)->push();

		return true;
	}

	bool restore() // restore from backup the previous estimate of the graph
	{
		g2o::OptimizableGraph::VertexIDMap::iterator
			vIt = optimizer->vertices().begin(),
			vEnd = optimizer->vertices().end();

		for (; vIt != vEnd; vIt++)
			static_cast<VertexType *>(vIt->second)->pop();

		// HACK : store again!
		store();
		return true;
	}

	bool read(
		const char *filename)
	{
		if (optimizer != NULL)
		{
			std::cerr << "An allocated optimizer already exists " << std::endl;
			return false;
		}
		optimizer = new g2o::SparseOptimizer;
		auto linearSolver = g2o::make_unique<LinearSolverEigen<BlockSolverX::PoseMatrixType>>();
		linearSolver->setBlockOrdering(false);
		auto blockSolver = g2o::make_unique<BlockSolverX>(std::move(linearSolver));

		OptimizationAlgorithmGaussNewton *optimizationAlgorithm =
			new OptimizationAlgorithmGaussNewton(std::move(blockSolver));
		optimizer->setAlgorithm(optimizationAlgorithm);

		if (!optimizer->load(filename))
		{
			std::cerr << "Can't find file to read : " << filename << std::endl;
			;
			return false;
		}
		store();
		initialized = true;
		return true;
	}

	// Optimize on a given set of loop closure links
	// @return: The error of each link + overall error of the active part of the graph as the last element of the vector

	IntPairDoubleMap optimize(
		const IntPairSet &activeLoops,
		const int nIterations)
	{

		activeEdges.clear();
		activeEdges.insert(
			odomteryEdges.begin(),
			odomteryEdges.end());

		for (
			IntPairSet::const_iterator it = activeLoops.begin(), end = activeLoops.end();
			it != end;
			it++)
		{
			activeEdges.insert(loopclosureEdges[*it]);
		}

		IntPairDoubleMap loopClosureLinkError;

		restore();
		//
		optimizer->setVerbose(false);
		optimizer->findGauge()->setFixed(true);
		//optimizer->vertex(0)->setFixed(true);
		optimizer->initializeOptimization(activeEdges);
		optimizer->optimize(nIterations, false);
		optimizer->computeActiveErrors();

		for (
			IntPairSet::const_iterator it = activeLoops.begin(), end = activeLoops.end();
			it != end;
			it++)
		{
			//dynamic_cast< EdgeType* >(loopclosureEdges[*it])->computeError();
			loopClosureLinkError[*it] = dynamic_cast<EdgeType *>(loopclosureEdges[*it])->chi2();
		}

		// NOTE : The number of edges involved is the last element
		loopClosureLinkError[IntPair(-1, 0)] = optimizer->activeChi2(); // There can be no links with negative IDS
		loopClosureLinkError[IntPair(-1, -1)] = optimizer->activeEdges().size();

		//
		return loopClosureLinkError;
	}

	bool write(const char *filename, const IntPairSet &correctLoops)
	{
		restore();
		std::ofstream out(filename);

		activeEdges.clear();
		activeEdges.insert(odomteryEdges.begin(), odomteryEdges.end());

		for (IntPairSet::const_iterator it = correctLoops.begin(), end = correctLoops.end();
			 it != end;
			 it++)
		{
			activeEdges.insert(loopclosureEdges[*it]);
		}
		optimizer->saveSubset(out, activeEdges);
		out.close();

		return true;
	}

	int edgeDimension()
	{
		return EdgeType::Dimension;
	}

	bool isInitialized()
	{
		return initialized;
	}

	bool removeEdges(const IntPairSet &falseLinks)
	{
		for (IntPairSet::const_iterator it = falseLinks.begin(), end = falseLinks.end(); it != end; ++it)
		{
			if (optimizer->removeEdge(loopclosureEdges[*it]))
			{
				loopclosureEdges.erase(*it); // Clear up from our records as well
			}
		}
		return true;
	}
};

#endif /* BACKEND_G2O_HPP_ */
