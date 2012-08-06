/*
 * test.cpp
 *
 *  Created on: May 21, 2012
 *      Author: yasir
 */

#include "include/OptimizationManager.hpp"
#include "include/Timer.hpp"

typedef OptimizationManager<g2o::VertexSE2, g2o::EdgeSE2> OptimizationManager2D;
typedef std::pair<int,int> intPair;



int main(int argc, char** argv)
{

	if(argc!=4 )
	{
		std::cout<<"Usage: "<<argv[0]<<" filename.g2o odometry_rate_in_hertz place_recog_rate_in_hertz"<<std::endl;
		return -1;
	}

	float OdomRate = atof(argv[2]);
	float PrRate = atof(argv[3]);

	std::vector<intPair> goodLinks;

	std::cerr<<" Args in : "<<OdomRate<<" "<<PrRate<<std::endl;

	OptimizationManager2D optimizationManager;

	Timer t;

	t.start();

	optimizationManager.init(argv[1],OdomRate,PrRate);
	optimizationManager.RRR(goodLinks,true);

	t.printTime(t.stop());

	return 0;
}
