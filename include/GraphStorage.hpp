/*
 * GraphStorage.hpp
 *
 *  Created on: Feb 23, 2012
 *      Author: yasir
 */

#ifndef GRAPHSTORAGE_HPP_
#define GRAPHSTORAGE_HPP_

#include <g2o/core/graph_optimizer_sparse.h>
using namespace g2o;
template <typename VertexType>
class GraphStorage
{
	public:

	void store(g2o::SparseOptimizer& optimizer)
	{
		g2o::OptimizableGraph::VertexIDMap::iterator
			vIt = optimizer.vertices().begin(),
			vEnd = optimizer.vertices().end();

		for(; vIt!=vEnd ; vIt++ ) dynamic_cast<VertexType>(vIt->second)->push();

	}

	void load(SparseOptimizer& optimizer)
	{
		g2o::OptimizableGraph::VertexIDMap::iterator
			vIt = optimizer.vertices().begin(),
			vEnd = optimizer.vertices().end();

		for(; vIt!=vEnd ; vIt++ )	dynamic_cast<VertexType>(vIt->second)->pop();

		// HACK : store again!
		store(optimizer);
	}

	void reset(SparseOptimizer& optimizer)
	{
		g2o::OptimizableGraph::VertexIDMap::iterator
					vIt = optimizer.vertices().begin(),
					vEnd = optimizer.vertices().end();

				for(; vIt!=vEnd ; vIt++ ){
					dynamic_cast<VertexType>(vIt->second)->discardTop();
					dynamic_cast<VertexType>(vIt->second)->push();
				}
	}

};

#endif /* GRAPHSTORAGE_HPP_ */
