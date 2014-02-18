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


#ifndef BACKENDINTERFACE_H_
#define BACKENDINTERFACE_H_

#include "types.hpp"

/*! \brief An interface class for backends
 *
 */

class BaseBackend
{
public:

	/*!
	* Read in the the given Graph and return all the _loop closing_ links
	* as a set consisting of  (start_vertex_id, end_vertex_id) for each link
	*
	* @param[in] filename name of the  graph file to read
	*/
	virtual bool read(const char* filename) = 0;

	/*!
	 *  Rather than reading a file, directly set the optimizer that is currently
	 *  being used. Assumes that the optimizer has a solver declared
	 *
	 *  @param[in] OptimizerPtr Pointer to the optimizer being used
	 *
	 */
	virtual bool setOptimizer(void* OptimizerPtr) = 0;

	/*!
	 *  Collect all the loop closures from the underlying back-end
	 *
	 *  @param[in,out] loops set of loopCloures as (head_vertex_id,tail_vertex_id) pairs
	 */
	virtual bool getLoopClosures(IntPairSet& loops) = 0;

	/*!
	 * Write the Graph to the given filename, only including the links specified in the Set
	 *
	 * @param[in] filename filename to write to
	 * @param[in] edges edges as pairs of (head_vertex_id,tail_vertex_id)
	 */
	virtual bool write(const char* filename, const IntPairSet& edges) = 0;

	/*! Take a back up of the current estimate of vertex positions
	 * 	(Back-end dependant)
	 */
	virtual bool store() = 0;

	/*! Restore the vertex positions from a backup taken earlier
	 *  (Back-end dependant)
	 */
	virtual bool restore() = 0;

	/*! Optimize with a set of links given for the number of iterations
	* return for each link, the chi2 on that link
	* link [-1,0] is reserved for the overallChi2 of the active graph
	* link [-1,-1] is reserved for the number of links in the active Graph
	*/
	virtual IntPairDoubleMap optimize(const IntPairSet&, const int) = 0;


	/*! Get the Degrees of Freedom for each edge for the chi2 test
	 *
	 */
	virtual int edgeDimension() = 0;

	/*! Return the number of vertices present in the graph
	 *
	 */
	virtual int vertexCount() = 0;

	/*! Check if all is well at the back end to proceed with optimization tasks
	 *
	 */
	virtual bool isInitialized() = 0;

	virtual bool removeEdges(const IntPairSet& falseLinks) = 0;
	//
	virtual ~BaseBackend() {};
};


#endif /* BACKENDINTERFACE_H_ */
