/*
 * Cluster.hpp
 *
 *  Created on: Dec 30, 2011
 *      Author: yasir
 *      Given a set of loop closuers, clusters them
 */

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
	std::vector<cluster> _clustersFound;
public:
	// Assumes that in the vector the values are passed as (start_1,end_1), (start_2,end_2), ...
	void clusterize( const std::vector<int> loops , const int threshold, std::vector<int>& membership, int& clusterCount)
	{
		if(loops.size() < 2)
		{
			std::cerr<<"clusterize(): "<<__LINE__<<" no loops to make clusters"<<std::endl;
			clusterCount = 0;
			membership = std::vector<int>();
			return;
		}
		_clustersFound.clear();
		membership.clear();

		for(size_t i=0; i<loops.size();i+=2)
		{
			int start 	= std::max(loops[i],loops[i+1]);
			int end 	= std::min(loops[i],loops[i+1]);

			if(_clustersFound.empty())
			{
				cluster s(start,end);
				_clustersFound.push_back(s);
				membership.push_back(_clustersFound.size()-1);

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
						membership.push_back(i);
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
					membership.push_back(_clustersFound.size()-1);
				}
			}
		}

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

		clusterCount = _clustersFound.size();
	}

};

#endif /* CLUSTER_HPP_ */
