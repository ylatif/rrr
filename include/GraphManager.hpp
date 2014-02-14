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


#ifndef GRAPHMANAGER_HPP_
#define GRAPHMANAGER_HPP_

#include <g2o/core/sparse_optimizer.h>
#include "Cluster.hpp"
#include "GraphStorage.hpp"

typedef std::pair<int,int> intPair;

//template <typename VertexPtrType>

class GraphManager{

public:
	typedef std::set< intPair > LoopClosureSet;
	typedef std::map< intPair, int > LoopClosureClusterIDMap;
	typedef std::vector< std::vector< intPair > > ClusterCollection;

	//typedef g2o::VertexSE2* VertexPtrType;	//Changing this will be needed when Vertex types change.


private:

	std::vector<int> 		_limits;							// Specifying limits for sessions
	LoopClosureSet 			_loopClosures;						// stores only loop closures indices
	LoopClosureClusterIDMap _loopClosureClusterIDMap;			// Mapping from LoopClosure -> clusterIDs
	ClusterCollection 		_clusterCollection;					// Reverse of above , for getting links in each cluster

	// Things dealing with clusterizing the loop closures //
	Clusterizer _clusterizer;
	int _clusterCount;

public:
	GraphManager()
	{
		_clusterCount = -1;
	}

	g2o::SparseOptimizer optimizer;

	bool read(std::string filename)
	{
		if(!optimizer.load(filename.c_str())) return false;

		g2o::OptimizableGraph::EdgeSet::iterator eIt = optimizer.edges().begin(), eEnd = optimizer.edges().end();

		for( ; eIt!=eEnd ; eIt++)
		{
			if((*eIt)->vertices()[0]->id() > (*eIt)->vertices()[1]->id())
			{
				_loopClosures.insert(intPair((*eIt)->vertices()[0]->id(),(*eIt)->vertices()[1]->id()));

			}
		}
		std::cout<<"# LC :"<<_loopClosures.size()<<std::endl;

		return true;
	}


	bool write(const std::string filename)
		{ return optimizer.save(filename.c_str()); }

	const LoopClosureSet& loopClousures()
		{	return _loopClosures;}

	int clusterCount()
		{ return _clusterCount;	 }

	void clusterize(const int& threshold )
	{
		std::vector<int> loops , membership;

		LoopClosureSet::iterator
			it	= _loopClosures.begin(),
			end = _loopClosures.end();

		for( ; it!=end ; it++){	loops.push_back(it->first); loops.push_back(it->second); }

		_clusterizer.clusterize(loops,threshold,membership,_clusterCount);
		_clusterCollection.resize(_clusterCount);

		std::vector<int>::iterator mIter;
		for (mIter = membership.begin(),  it = _loopClosures.begin();
				it!=end ; it++, mIter++)
		{
			_loopClosureClusterIDMap.insert(
					std::pair< intPair, int >(*it,*mIter)
					);
			_clusterCollection[*mIter].push_back(*it);
		}

	}

	std::vector< intPair > getClusterbyID(const int& ID)
	{
		if(ID > -1 and ID < (int)_clusterCollection.size())
			return _clusterCollection[ID];
		else
		{
			std::cerr<<__PRETTY_FUNCTION__<<" ERROR : ID "<<ID<<" not found!!"<<std::endl;
			return std::vector< intPair >();
		}
	}

	// Given a (start,end), returns the clusterID of the cluster to which the link belongs
	int getClusterID(const intPair& loopClosure)
	{
		if(_loopClosureClusterIDMap.find(loopClosure)!=_loopClosureClusterIDMap.end())
			return _loopClosureClusterIDMap[loopClosure];
		else
			return -2;
	}

	void setClusterID(const intPair& loopClosure, int ID)
	{
		if(_loopClosureClusterIDMap.find(loopClosure)!=_loopClosureClusterIDMap.end())
		{
			int oldID = _loopClosureClusterIDMap[loopClosure];

			for(std::vector< intPair >::iterator it = _clusterCollection[oldID].begin()
					, end = _clusterCollection[oldID].end();
					it!=end;
					it++
					)
			{
				if (*it == loopClosure){
					_clusterCollection[oldID].erase(it);
					break;
				}
			}

			_loopClosureClusterIDMap[loopClosure] = ID;

		}
	}

	bool isLoopClosure(const intPair& link)
	{
		if(_loopClosures.find(link)==_loopClosures.end())
			return false;
		return true;
	}

};

#endif /* GRAPHMANAGER_HPP_ */
