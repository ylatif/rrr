/*
 * test.cpp
 *
 *  Created on: May 21, 2012
 *      Author: yasir
 */

#include "include/OptimizationManager.hpp"

int main(int argc, char** argv)
{

	OptimizationManager<g2o::VertexSE2, g2o::EdgeSE2> optimizationManager;
	//OptimizationManager<g2o::VertexSE3, g2o::EdgeSE3> optimizationManager;
	optimizationManager.init(argv[1],std::vector<int>(),atoi(argv[3]),atoi(argv[2]));
	optimizationManager.RRR(std::set<int>());

	std::cin.get();

	return 0;
}
