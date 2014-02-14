// RRR - Robust Loop Closing over Time
// Copyright (C) 2012 Y.Latif, C.Cadena, J.Neira
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

#ifndef GRAPHSTORAGE_HPP_
#define GRAPHSTORAGE_HPP_

#include <g2o/core/sparse_optimizer.h>
using namespace g2o;
template <typename VertexPtrType>
class GraphStorage
{
	public:

	void store(g2o::SparseOptimizer& optimizer)
	{
		g2o::OptimizableGraph::VertexIDMap::iterator
			vIt = optimizer.vertices().begin(),
			vEnd = optimizer.vertices().end();

		for(; vIt!=vEnd ; vIt++ ) dynamic_cast<VertexPtrType>(vIt->second)->push();

	}

	void load(SparseOptimizer& optimizer)
	{
		g2o::OptimizableGraph::VertexIDMap::iterator
			vIt = optimizer.vertices().begin(),
			vEnd = optimizer.vertices().end();

		for(; vIt!=vEnd ; vIt++ )	dynamic_cast<VertexPtrType>(vIt->second)->pop();

		// HACK : store again!
		store(optimizer);
	}

	void reset(SparseOptimizer& optimizer)
	{
		g2o::OptimizableGraph::VertexIDMap::iterator
					vIt = optimizer.vertices().begin(),
					vEnd = optimizer.vertices().end();

				for(; vIt!=vEnd ; vIt++ ){
					dynamic_cast<VertexPtrType>(vIt->second)->discardTop();
					dynamic_cast<VertexPtrType>(vIt->second)->push();
				}
	}

};

#endif /* GRAPHSTORAGE_HPP_ */
