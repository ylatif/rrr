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

#ifndef CLUSTER_HPP_
#define CLUSTER_HPP_

#include <iostream>
#include <vector>
#include <cstdlib>

struct cluster
{
	int startLow, startHigh;
	int endLow, endHigh;
	int size;

	cluster(): startLow(-1), startHigh(-1), endLow(-1), endHigh(-1), size(0) {}
	cluster(int start, int end) : startLow(start), startHigh(start), endLow(end), endHigh(end), size(1){}

	bool contains(int start, int end, int threshold)
	{
		return
				(
					std::abs(start-startHigh) < threshold or	std::abs(start-startLow)  < threshold
				)
				and
				(
					std::abs(end-endHigh) < threshold or std::abs(end-endLow) < threshold
				);

	}
};

class Clusterizer
{
	typedef IntPairIDMap 			LoopToClusterIDMap;
	typedef IDintPairSetMap 		ClusterIDtoLoopsMap;

	std::vector<cluster> _clustersFound;
	ClusterIDtoLoopsMap	clusterIDtoLoopsMap;
	LoopToClusterIDMap  loopToClusterIDMap;

public:
	// Assumes that in the vector the values are passed as (start_1,end_1), (start_2,end_2), ...

	int getClusterID(const IntPair& loop)
	{
		return loopToClusterIDMap[loop];
	}

	void setClusterID(const IntPair& loopClosure, int ID)
	{
		if(loopToClusterIDMap.find(loopClosure)!=loopToClusterIDMap.end())
		{
			int oldID = loopToClusterIDMap[loopClosure];

//			for(IntPairSet::iterator
//					it = clusterIDtoLoopsMap[oldID].begin(),
//					end = clusterIDtoLoopsMap[oldID].end();
//					it!=end;
//					it++)
//
//			{
				//if (*it == loopClosure){
					clusterIDtoLoopsMap[oldID].erase(loopClosure);
				//	loopToClusterIDMap[*it] = ID;
				//	break;
				//}
//			}
			clusterIDtoLoopsMap[ID].insert(loopClosure);
			loopToClusterIDMap[loopClosure] = ID;

		}
	}

	void clusterize( const IntPairSet& loops , const int threshold)//, std::vector<int>& membership, int& clusterCount)
	{
		if(loops.empty())
		{
			std::cerr<<"clusterize(): "<<__LINE__<<" no loops to make clusters"<<std::endl;
			return;
		}
		_clustersFound.clear();
		for(IntPairSet::const_iterator it = loops.begin(), lend = loops.end();
				it!=lend;
				it++)
		{
			int start 	= std::max(it->first,it->second);
			int end 	= std::min(it->first,it->second);

			if(_clustersFound.empty())
			{
				cluster s(start,end);
				_clustersFound.push_back(s);
				clusterIDtoLoopsMap[_clustersFound.size()-1].insert(*it);
				loopToClusterIDMap[*it] = _clustersFound.size()-1;
			}
			else
			{
				cluster* currentCluster = NULL;
				//Search for a cluster where it can belong
				for(size_t i=0; i<_clustersFound.size(); i++)
				{
					if(_clustersFound[i].contains(start,end,threshold))
					{
						currentCluster = &_clustersFound[i];
						currentCluster->size++;
						//membership.push_back(i);
						clusterIDtoLoopsMap[i].insert(*it);
						loopToClusterIDMap[*it] = i;

						if(start<currentCluster->startLow)	currentCluster->startLow = start;
						if(start>currentCluster->startHigh)	currentCluster->startHigh = start;

						if(end<currentCluster->endLow) currentCluster->endLow = end;
						if(end>currentCluster->endHigh) currentCluster->endHigh = end;

						break;
					}
				}
				if(currentCluster == NULL) // Does not belong to any existing cluster.
				{
					cluster s(start,end);
					_clustersFound.push_back(s);
					//membership.push_back(_clustersFound.size()-1);
					clusterIDtoLoopsMap[_clustersFound.size()-1].insert(*it);
					loopToClusterIDMap[*it] = _clustersFound.size()-1;
				}
			}

		}

#if 0
		if(0)
		{
			std::cout<<" \% Clusters formed "<<_clustersFound.size()<<std::endl;
			std::cout<<"limits = [ "<<std::endl;
			for(size_t i=0 ; i< _clustersFound.size() ; i++)
			{
				std::cout<<i<<" -> sz "<<_clustersFound[i].size<<" :: ";
				std::cout<<" "<<_clustersFound[i].startLow<<" "<<_clustersFound[i].startHigh<<" ";
				std::cout<<" "<<_clustersFound[i].endLow<<" "<<_clustersFound[i].endHigh<<std::endl;;

			}
			std::cout<<std::endl;
			std::cout<<"]; "<<std::endl;


			std::cout<<"membership =[ ";
			for(size_t i=0; i<membership.size();i++)
			{
				std::cout<<membership[i]<<" ";
			}
			std::cout<<std::endl;
			std::cout<<"]; "<<std::endl;
		}
#endif

	}

	IntPairSet& getClusterByID(int id){
		return clusterIDtoLoopsMap[id];
	}

	size_t clusterCount()
	{
		return _clustersFound.size();
	}

	bool deleteCluster(int clusterID)
	{
		clusterIDtoLoopsMap.erase(clusterID);

		for(IntPairIDMap::iterator it= loopToClusterIDMap.begin();
				it!=loopToClusterIDMap.end(); it++)
		{
			if(it->second == clusterID)
				loopToClusterIDMap.erase(it->first);
		}

		return true;
	}

};

#endif /* CLUSTER_HPP_ */
