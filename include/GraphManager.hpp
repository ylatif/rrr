/*
 * GraphManager.hpp
 *
 *  Created on: Feb 22, 2012
 *      Author: yasir
 */

#ifndef GRAPHMANAGER_HPP_
#define GRAPHMANAGER_HPP_

#include <g2o/core/graph_optimizer_sparse.h>
#include "Cluster.hpp"
#include "GraphStorage.hpp"

typedef std::pair<int,int> intPair;


class GraphManager{

public:
	typedef std::set< intPair > LoopClosureSet;
	typedef std::map< intPair, int > LoopClosureClusterIDMap;
	typedef std::vector< std::vector< intPair > > ClusterCollection;

	typedef g2o::VertexSE2* VertexPtrType;	//Changing this will be needed when Vertex types change.


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

	// To differentiate b/w intra- & inter-session loopClosures
	/*
	int getVertexSessionID(int vertexID)
	{
		if(_limits.empty()){
			std::cerr<<__PRETTY_FUNCTION__<<" WARNING : SESSION_LIMITS_NOT_DEFINED "<<std::endl;
			return -1;
		}
		size_t i=0, end = _limits.size();

		for( ; i<end; i++)
		{
			if(vertexID == _limits[i]){	return (int)i; }
			else if(vertexID  < _limits[i] ) {	return i;}

		}
		return _limits.size()-1;
	}
	*/
	/*
	intPair getEdgeSessionIDs(const intPair& loopClosure)
	{
		if(_limits.empty()){
			std::cerr<<__PRETTY_FUNCTION__<<" WARNING : SESSION_LIMITS_NOT_DEFINED "<<std::endl;
			return intPair(0,0);
		}
		intPair membership; membership.first = 0 ; membership.second=0;
		size_t i=1, end = _limits.size();
		for( ; i<end; i++)
		{
			if(loopClosure.first  < _limits[i] and loopClosure.first  > _limits[i-1]) { membership.first = i;}
			if(loopClosure.second < _limits[i] and loopClosure.second > _limits[i-1]) { membership.second = i;}
		}

		return membership;
	}
	*/

	/*
	bool setSessionLimits(const std::vector<int>& sessionLimits)	{	_limits = sessionLimits;	return true;	}
	*/

	bool write(const std::string filename){ return optimizer.save(filename.c_str()); }

	const LoopClosureSet& loopClousures(){	return _loopClosures;}

	int clusterCount()
	{
		return _clusterCount;
	}

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

	intPair getSessionIDfromClusterID(int clusterID)
	{
		return intPair(0,0);
		/*
		if(clusterID< (int)_clusterCollection.size())
		{
			return getEdgeSessionIDs(_clusterCollection[clusterID][0]);
		}
		else
		{
			std::cerr<<__PRETTY_FUNCTION__<<" Invalid clusterID :"<<clusterID<<std::endl;
			return intPair(-1,-1);
		}
		*/
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
