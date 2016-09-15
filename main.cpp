#include <iostream>
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include "g2o/types/slam2d/parameter_se2_offset.h"
#include "g2o/types/slam2d/edge_link_xy.h"
// #include "types_tutorial_slam2d.h"


#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"


void printGraph(const g2o::OptimizableGraph& optimizable){

	auto idmap = optimizable.vertices();
	auto idmapedges = optimizable.edges();
	
	std::cout << std::endl;
	for ( auto it = idmap.begin(); it != idmap.end(); ++it ){
		std::cout << " " << it->first << " : " << it->second << std::endl;
	}
	std::cout << std::endl;
	for ( auto ite = idmapedges.begin(); ite != idmapedges.end(); ++ite ){
		std::cout << " " << *ite << " connected";
		for(auto ite2 = (*ite)->vertices().begin(); ite2 != (*ite)->vertices().end() ; ++ite2){
			std::cout << " " << *ite2;
		}
		std::cout << std::endl;
	}		
}

void found(const g2o::OptimizableGraph& optimizable, const g2o::HyperGraph::Vertex* vert){

	auto idmap = optimizable.vertices();
	auto idmapedges = optimizable.edges();
	
	std::cout << std::endl;
	for ( auto it = idmap.begin(); it != idmap.end(); ++it ){
		std::cout << " " << it->first << " : " << it->second << std::endl;
		if(vert == it->second){
			std::cout << "FOUND" << std::endl;
		}
		else{
			std::cout << "Not the same" << std::endl;
		}
	}
	
	
	
}



int main(int argc, char **argv) {
    std::cout << "Hello, world!" << std::endl;
	
	g2o::OptimizableGraph optimizable;
	
	g2o::EdgeLinkXY_malcolm* edgelink = new g2o::EdgeLinkXY_malcolm();
	g2o::EdgeLinkXY_malcolm* edgelink2 = new g2o::EdgeLinkXY_malcolm();
	g2o::EdgeSE2PointXY* edgese2 = new g2o::EdgeSE2PointXY();
	
	g2o::VertexPointXY* landmark = new g2o::VertexPointXY();
	landmark->setId(1);
	
	std::cout << "Landmark : " << landmark << std::endl;
	std::cout << "edge : " << edgelink << std::endl;
	std::cout << "edge : " << edgelink2 << std::endl;
	std::cout << "edge : " << edgese2 << std::endl;
	
	std::cout << std::endl;
	
	edgelink->vertices()[0] = landmark;
	edgelink->vertices()[1] = landmark;
	
	optimizable.addVertex(landmark);
	optimizable.addEdge(edgelink);
// 	optimizable.addEdge(edgelink2);
// 	optimizable.addEdge(edgese2);
	
	printGraph(optimizable);
	
// 	std::cout << std::endl;
	g2o::VertexPointXY* landmark2 = new g2o::VertexPointXY();
	landmark2->setId(10);
	
	edgelink->vertices()[1] = landmark2;
	
	optimizable.addVertex(landmark2);
	optimizable.addEdge(edgelink);
	
	found(optimizable, landmark);
	
	//TRYING TO UPDATE THE NODE
	std::cout << "Updating the graph" << std::endl;
// 	landmark->setId(100);
	optimizable.changeId(landmark, 100);
	printGraph(optimizable);
	g2o::VertexPointXY* landmark3 = dynamic_cast<g2o::VertexPointXY*>(optimizable.vertex(100));
	std::cout << "the new id " << landmark3 << " with " << landmark3->id() << std::endl;
	
	
	std::cout << "Removing the vertex" << std::endl;
	optimizable.removeVertex(landmark2, true);
	std::cout << "Done the vertex" << std::endl;
	
	printGraph(optimizable);
	
	
	
    return 0;
}
