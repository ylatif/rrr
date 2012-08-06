/*
 * OptimizationManager.hpp
 *
 *  Created on: Feb 23, 2012
 *      Author: yasir
 */

#ifndef OPTIMIZATIONMANAGER_HPP_
#define OPTIMIZATIONMANAGER_HPP_

//#include "Viewer/Viewer.hpp"

#include <g2o/core/factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/math_groups/se2.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/vertex_se2.h>

#include <g2o/types/slam3d/edge_se3_quat.h>
#include <g2o/types/slam3d/vertex_se3_quat.h>


#include <g2o/solvers/csparse/linear_solver_csparse.h>

#include <boost/math/distributions/chi_squared.hpp>

//#include "AnchorEdge.hpp"
#include "GraphManager.hpp"
#include "GraphStorage.hpp"

#include "Timer.hpp"



using namespace g2o;

template <typename vertexType, typename edgeType>
class OptimizationManager
{
	typedef edgeType EdgeAnchor ;

	typedef std::map< std::pair<int,int>, edgeType * > OdometryMap;
	typedef std::map< std::pair<int,int>, edgeType * > LoopClosureMap;

	typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
	typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

	GraphManager _graphManager;
	GraphStorage<vertexType*> _graphStorage;

	g2o::SparseOptimizer optimizer;
	SlamLinearSolver* linearSolver ;
	SlamBlockSolver* solver ;

	OdometryMap _odometryMap;
	LoopClosureMap _loopClosureMap;

	//std::vector<g2o::VertexSE2*> _anchors;

	int _iterations, _clusteringThrehold;

	bool _initialized;

	std::vector<bool> _computedIC_clusterID;

	//Viewer* //_viewer;

	double _chi2(int dof)
	{
		if(dof>0)
			return boost::math::quantile(boost::math::chi_squared(dof),0.99);
		else
		{
			std::cerr<<__LINE__<<" dof <= 0 ? "<<std::endl;
			return 0;
		}
	}



public :
	OptimizationManager(){
		//Factory * factory = Factory::instance();
		//factory->registerType("EDGE_ANCHOR", new HyperGraphElementCreator<EdgeAnchor>);

		linearSolver = new SlamLinearSolver();
		linearSolver->setBlockOrdering(false);
		solver = new SlamBlockSolver(&optimizer, linearSolver);
		optimizer.setSolver(solver);

		_initialized = false;

		//_viewer = new Viewer(optimizer);
	}

	~OptimizationManager(){

		//_odometryMap.clear();
		//_loopClosureMap.clear();
		//optimizer.clear();

	}

	bool read(	 // Read the Graph, calculate clustering and sessionIDs, convert to Anchored Graph and store
			const std::string filename,
			std::vector<int> sessionLimits,
			int clusteringThreshold = 50,
			int iterations = 4
	)
	{
		if(!_graphManager.read(filename))
		{
			//std::cerr<<"Could not read : "<<filename.c_str()<<std::endl;
			return false;
		}

		_iterations = iterations;

		if(sessionLimits.empty()) sessionLimits.push_back(_graphManager.optimizer.vertices().size());

//		_graphManager.setSessionLimits(sessionLimits);
		_graphManager.clusterize(clusteringThreshold);


		g2o::OptimizableGraph::VertexIDMap::const_iterator
		vIt  = _graphManager.optimizer.vertices().begin(),
		vEnd = _graphManager.optimizer.vertices().end();
		// Add all the edges!
		for( ; vIt!=vEnd ; vIt++)
		{
			vertexType* v = new vertexType ; vertexType* oldV = dynamic_cast<vertexType*>(vIt->second);

			v->setId(oldV->id());
			v->setEstimate(oldV->estimate());

			optimizer.addVertex(dynamic_cast<vertexType*>(v));
		}

		for (
				g2o::OptimizableGraph::EdgeSet::iterator
				eIt= _graphManager.optimizer.edges().begin(),
				eEnd = _graphManager.optimizer.edges().end() ;
				eIt!=eEnd ;
				eIt++
		)
		{
			intPair thisEdge((*eIt)->vertices()[0]->id(), (*eIt)->vertices()[1]->id());
			if(_graphManager.isLoopClosure( thisEdge ))
			{

				edgeType* anchoredEdge = new edgeType;

				anchoredEdge->vertices()[0] = optimizer.vertex(thisEdge.first);
				anchoredEdge->vertices()[1] = optimizer.vertex(thisEdge.second);


				anchoredEdge->setInformation(static_cast<edgeType*>(*eIt)->information());
				anchoredEdge->setMeasurement(static_cast<edgeType*>(*eIt)->measurement());

				optimizer.addEdge(static_cast<edgeType*>(anchoredEdge));;

				_loopClosureMap.insert( std::pair < intPair, edgeType* >(thisEdge, static_cast<edgeType*>(anchoredEdge)));

			}
			else
			{
				edgeType * e = new edgeType; edgeType* oldE = dynamic_cast<edgeType*>(*eIt);

				e->vertices()[0] = optimizer.vertex(oldE->vertices()[0]->id());
				e->vertices()[1] = optimizer.vertex(oldE->vertices()[1]->id());

				e->setMeasurement(oldE->measurement());
				e->setInformation(oldE->information());

				//e->setRobustKernel(true);
				//e->setHuberWidth(1.0);

				optimizer.addEdge(static_cast<edgeType*>(e));
				_odometryMap.insert(std::pair< intPair, edgeType* >(thisEdge, static_cast<edgeType*>(e)));
			}
		}

		_graphStorage.store(optimizer);

		optimizer.save("test.g2o");

		return true;
	}


	bool optimizeClusterList(const std::vector<int>& clusterList, int iterations = -1)
	{

		if( clusterList.size() == 0)
		{
			//std::cerr<<" WARN _ clusterList.size() is zero!" <<std::endl;
			return true;
		}

		//std::vector< std::set<int> > subsets;
		//std::vector< std::vector< intPair > > edges;

		_graphStorage.load(optimizer);

		OptimizableGraph::EdgeSet activeEdges;
		for(
				std::vector<int>::const_iterator
				it = clusterList.begin(),
				end = clusterList.end();
				it!=end;
				it++
		)
		{
			std::vector< intPair > loopEdges = _graphManager.getClusterbyID(*it);
			for(size_t j=0; j<loopEdges.size();j++)
			{
				activeEdges.insert(dynamic_cast<OptimizableGraph::Edge*>(_loopClosureMap[loopEdges[j]]));
			}
		}

		for(
				typename OdometryMap::iterator
				it= _odometryMap.begin(),
				end = _odometryMap.end() ;
				it!=end;
				it++
		)
		{
			activeEdges.insert(dynamic_cast<OptimizableGraph::Edge*>(it->second));
		}

		optimizer.setVerbose(false);
		g2o::OptimizableGraph::Vertex* v_fixed;

		std::cerr<<" GF "<<std::endl;

		//if(optimizer.gaugeFreedom())
		{
			v_fixed = optimizer.vertex(0);
			//std::cout<<v_fixed->id()<<" fixed "<<std::endl;
			v_fixed->setFixed(true);
		}

		optimizer.initializeOptimization(activeEdges);

		int optStatus;
		if(iterations<0)
			optStatus = optimizer.optimize(_iterations);
		else
			optStatus = optimizer.optimize(iterations);

		v_fixed->setFixed(false);


		return true;
	}

	std::vector<bool> individualCompatibility(int clusterID){

		_computedIC_clusterID[clusterID] = true;
		std::cerr<<"-- IC "<<clusterID<<" --"<<std::endl;
		std::vector<int> clusters;

		clusters.push_back(clusterID);

		optimizeClusterList(clusters);

		int 	minVertexID = std::numeric_limits<int>::max(),
				maxVertexID = std::numeric_limits<int>::min();

		std::vector<intPair> loopEdges = _graphManager.getClusterbyID(clusterID);

		for(std::vector< intPair >::iterator it = loopEdges.begin(), end = loopEdges.end();
				it!=end ; it++)
		{
			//		std::cerr<<it->first<<" "<<it->second<<std::endl;
			if(it->first > maxVertexID) maxVertexID = it->first;
			if(it->second < minVertexID ) minVertexID = it->second;
		}

		std::vector<bool> isOkay(loopEdges.size(),false);

		double errorSum = 0;
		double linkThreshold = _chi2(edgeType::Dimension-1);

		for(size_t i=0 ; i< loopEdges.size() ; i++)
		{
			intPair thisPair = loopEdges[i];
			_loopClosureMap[thisPair]->computeError();
			errorSum += _loopClosureMap[thisPair]->chi2();
			//std::cout<<"\t"<<_graphManager.getClusterID(loopEdges[0][i])<<" : "<<i<<" "<<loops[i]->chi2()<<std::endl;
			if(_loopClosureMap[thisPair]->chi2() < linkThreshold)
			{
				isOkay[i] = true;
				std::cout<<"[ ]";
			}
			else std::cout<<"[X]";
		}
		std::cout<<std::endl;

		//std::cout<<std::endl;

		double allThreshold = _chi2(vertexType::Dimension*(maxVertexID - minVertexID)-1); //The number of vertices involved
		//double allThreshold = _chi2(3*optimizer.edges().size()-1);

		std::cout<<"   "<<errorSum<<" / "<<allThreshold<<std::endl;

		if(errorSum > allThreshold)
		{
			std::cerr<<"All rejected "<<std::endl;
			return std::vector<bool>(loopEdges.size(),false);
		}
		else
		{
			int accepted = 0;
			for(size_t i=0 ; i< loopEdges.size(); i++)
			{
				//(isOkay[i])? //std::cout<<"[ ]"://std::cout<<"[X]";
				accepted +=isOkay[i];
				if(!isOkay[i]) _graphManager.setClusterID(loopEdges[i],-2);
			}
			//std::cout<<std::endl;
			 	std::cerr<<"  Accepted  : "<< accepted <<"/"<<loopEdges.size()<<std::endl;
			//std::cin.get();
			return isOkay;
		}

				}

	bool jointCompatibility(
			std::vector<int>& H,
			std::vector<int>& rejected,
			int checkLast,
			int iterations = -1
	)
	{
		//std::cout<<" H : "; for(size_t i=0 ; i< H.size(); i++)	//std::cout<<H[i]<<" "; //std::cout<<std::endl;

		if(H.empty() or (checkLast == 0))
		{
			//std::cerr<<"Warning H() empty"<<std::endl;
			return true;
		}

		optimizeClusterList(H,iterations);

		optimizer.computeActiveErrors();

		std::vector<double> errors( H.size(), 0.);
		std::vector<double>::iterator errorIt = errors.begin();



		int linkCount = 0;
		for( size_t c = 0 ; c < H.size(); c++, errorIt++ )
		{
			std::vector< intPair > thisCluster = _graphManager.getClusterbyID(H[c]);
			for( std::vector< intPair >::iterator
					cIt = thisCluster.begin(), cEnd = thisCluster.end() ;
					cIt!= cEnd ;
					cIt++
			)
			{
				_loopClosureMap[*cIt]->computeError();
				(*errorIt) += _loopClosureMap[*cIt]->chi2();
				linkCount++;
			}
		}

		double chi2Links = _chi2(3*linkCount -1);
		double chi2all   = _chi2(3*optimizer.edges().size()-1);
		double overallError = 0;

		for( errorIt = errors.begin(); errorIt!=errors.end() ; errorIt++) overallError+= (*errorIt);

		std::cout<< "("<<checkLast<<") JC : "<<overallError<<" / "<<chi2Links<<std::endl;
		if ( (overallError < chi2Links and optimizer.activeChi2() < chi2all) or
				checkLast==0)
		{
			return true;
		}
		else // Find who is  to be eliminated ..
		{
			int rejectedClusterID = -1;
			double  minConsistencyIndex = 0;
			for(size_t i= H.size() - checkLast; i<H.size();i++)
			{
				double consistencyIndex = errors[i]/(_chi2(_graphManager.getClusterbyID(H[i]).size()*3 -1 ));
				//std::cout<<consistencyIndex<<" ("<<H_now[i]<<") ";
				if(consistencyIndex > minConsistencyIndex)
				{
					minConsistencyIndex = consistencyIndex;
					rejectedClusterID = i;
				}
			}

			//std::cout<<std::endl;
			//std::cerr<<"rejectedClusterID "<<H_now[rejectedClusterID]<<std::endl;
			rejected.push_back(H[rejectedClusterID]);
			H.erase(H.begin()+rejectedClusterID);
			return jointCompatibility(H,rejected,checkLast-1);
		}
		return true;
	}

	void init(
			const std::string filename,
			std::vector<int> sessionLimits,
			int clusteringThreshold = 50,
			int iterations = 4
	)
	{
		read(filename,sessionLimits,clusteringThreshold,iterations);
		//std::cerr<<"Done reading ... "<<std::endl;

		_computedIC_clusterID = std::vector<bool>(_graphManager.clusterCount(),false);

		_clusteringThrehold = clusteringThreshold;

		_initialized = true;
	}

	void RRR(std::set<int> sessionsIn = std::set<int>())
	{
		if(!_initialized)
		{
			std::cerr<<"Please initialized by calling init() first "<<std::endl;
			return ;
		}

		std::vector<int> clustersToExplore;					//Clusters Associated with the included Sessions

		std::set< int > sessionsIncluded;

		for(int x=0 ; x< _graphManager.clusterCount() ; x++)
		{
			individualCompatibility(x);
			clustersToExplore.push_back(x);
		}


		std::set<int> goodClusters,			// Clusters selected in every iteration
		selectedClusters, 					// Overall set of JC sets that are selected
		rejectedClusters;					// Clusters that have been rejected (NOT JC) in the prev iteration.

		std::vector< int > H;
		bool done = false;

		while(!done)
		{
			std::set< int > toConsider; // Clusters not in the goodSet
			for(size_t i=0 ; i<clustersToExplore.size(); i++)
			{
				if(selectedClusters.find(clustersToExplore[i])==selectedClusters.end()
						and rejectedClusters.find(clustersToExplore[i])==rejectedClusters.end())
				{ // Not in the selected Cluster nor in reject
					toConsider.insert(clustersToExplore[i]);
				}
			}

			done = true;

			std::cout<<"-----------> "<<_iterations<<std::endl;
			optimizeClusterList(std::vector<int>(toConsider.begin(),toConsider.end()),_iterations);

			optimizer.computeActiveErrors();
			for( typename LoopClosureMap::iterator
					cIt = _loopClosureMap.begin(),
					cEnd = _loopClosureMap.end() ;
				cIt!=cEnd ;
				cIt++
			)
			{
				double myClusterID = _graphManager.getClusterID( cIt->first );
				cIt->second->computeError();
				double myError = cIt->second->chi2();
				if (selectedClusters.find(myClusterID)==selectedClusters.end()
						and rejectedClusters.find(myClusterID)==rejectedClusters.end()
						and	myError < _chi2(3-1)
						and myClusterID>=0
				)
				{
					goodClusters.insert(myClusterID);
					//std::cout<<"["<<myClusterID<<"]";
					done = false;
				}
				//else	std::cout<<"[X]";

			}

			//if( goodClusters.size() == 0 ) continue;

			//std::cout<<" Good Set "; display(selectedClusters) ; std::cout<<std::endl;
			std::cout<<" Candidates "; display(goodClusters) ; std::cout<<std::endl;

			H.clear();
			H.insert(H.end(), selectedClusters.begin(), selectedClusters.end());
			H.insert(H.end(), goodClusters.begin(), goodClusters.end());

			std::vector<int> rejected;

			int oldSize = selectedClusters.size();

			jointCompatibility(H,rejected, goodClusters.size());

			//std::cerr<<" JC returned "<<std::endl;

			for(size_t i=0 ; i< rejected.size(); i++)
			{
				H.erase(std::remove(H.begin(),H.end(),rejected[i]),H.end());
			}

			//std::cerr<<" Done with updating H "<<std::endl;

			goodClusters.clear();
			selectedClusters.clear();

			selectedClusters.insert(H.begin(),H.end());

			if((int)selectedClusters.size() > oldSize)
			{
				rejectedClusters.clear();
			}
			rejectedClusters.insert(rejected.begin(), rejected.end());

			//std::cout<<std::endl;

			std::cout<<"Current Good Set "; display(selectedClusters); std::cout<<std::endl;
			std::cout<<"Rejected List" << std::endl ; display(rejectedClusters); std::cout<<std::endl;

			//	std::cin.get();

		}



		_graphStorage.load(optimizer);

		write("out_unopt.g2o", selectedClusters, sessionsIncluded);

		for(size_t i=0; i<clustersToExplore.size();i++)
		{
			if(selectedClusters.find(clustersToExplore[i])==selectedClusters.end())
			{
				rejectedClusters.insert(clustersToExplore[i]);
			}
		}

		write("out_unopt_reject.g2o", rejectedClusters, sessionsIncluded);

		std::vector< std::vector < intPair > > e;
		std::vector< std::set<int> > subs;

		optimizeClusterList(std::vector<int>(selectedClusters.begin(),selectedClusters.end()));

		//_graphStorage.store(optimizer);

		//std::stringstream filename;

		//filename<<"out_"<<x<<".g2o";

		//write(filename.str(), selectedClusters, sessionsIncluded);

		optimizer.save("out_anchor.g2o");

	}

	void display(const std::set<int>& s)
	{
		for( std::set<int>::iterator it = s.begin(), end = s.end() ; it!=end; it++)
		{
			std::cout<< *it <<" ";
		}
	}

	void write(std::string filename, std::set<int>& clusters, std::set<int>& sessionsIncluded)
	{
		int maxVertexID = -1;

		for(std::set<int>::iterator
				it = clusters.begin(), end = clusters.end();
				it!=end;
				it++
		)
		{
			std::vector< intPair > m =_graphManager.getClusterbyID((*it));

			for(size_t i=0; i< m.size() ; i++)
			{
				if(m[i].first > maxVertexID ) maxVertexID = m[i].first;
				if(m[i].second > maxVertexID ) maxVertexID = m[i].second;
			}
		}

		if(maxVertexID<0)
		{
			//std::cerr<<"I refuse to write! "<<std::endl;
			return;
		}

		std::ofstream out(filename.c_str());
		for(
				g2o::OptimizableGraph::VertexIDMap::iterator
				vIt = optimizer.vertices().begin(), vEnd = optimizer.vertices().end();
				vIt!=vEnd ;
				vIt++)
		{
			//if(sessionsIncluded.empty()){
				out<<"VERTEX_SE2 "<<vIt->second->id()<<" "; dynamic_cast<vertexType*>(vIt->second)->write(out); out<<std::endl;
			//}
			/*else if(
					sessionsIncluded.find(_graphManager.getVertexSessionID(vIt->second->id()))!= sessionsIncluded.end() and
					_graphManager.getVertexSessionID(vIt->second->id()) <= maxVertexID
			)
			{
				out<<"VERTEX_SE2 "<<vIt->second->id()<<" "; dynamic_cast<vertexType*>(vIt->second)->write(out); out<<std::endl;
			}*/
		}

		for( typename OdometryMap::iterator
				it = _odometryMap.begin(),
				end = _odometryMap.end();
				it!=end;
				it++
		)
		{
			//if(sessionsIncluded.empty()){
				out<<"EDGE_SE2 "<<it->second->vertices()[0]->id()<<" "
						<<it->second->vertices()[1]->id()<<" "; it->second->write(out); out<<std::endl;
			//}
				/*
			else if(
					sessionsIncluded.find(_graphManager.getVertexSessionID(it->second->vertices()[0]->id())) != sessionsIncluded.end() and
					sessionsIncluded.find(_graphManager.getVertexSessionID(it->second->vertices()[1]->id())) != sessionsIncluded.end()
			)
			{
				out<<"EDGE_SE2 "<<it->second->vertices()[0]->id()<<" "
						<<it->second->vertices()[1]->id()<<" "; it->second->write(out); out<<std::endl;
			}
			*/
		}

		for(std::set<int>::iterator
				it = clusters.begin(), end = clusters.end();
				it!=end;
				it++
		)
		{
			std::vector< intPair > m =_graphManager.getClusterbyID((*it));

			for(size_t i=0; i< m.size() ; i++)
			{
				out<<"EDGE_SE2 "<<m[i].first<<" "<<m[i].second<<" ";
				_loopClosureMap[m[i]]->write(out); out<<std::endl;
			}
		}

	}

};

#endif /* OPTIMIZATIONMANAGER_HPP_ */