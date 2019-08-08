//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#ifndef NETWORKS2019_BIDIRECTIONAL_LABELING_H
#define NETWORKS2019_BIDIRECTIONAL_LABELING_H

#include <vector>
#include <tuple>

#include "goc/goc.h"

#include "vrp_instance.h"
#include "label.h"
#include "lazy_label.h"
#include "monodirectional_labeling.h"

namespace networks2019
{
class BidirectionalLabeling
{
public:
	int solution_limit; // Maximum number of solutions to obtain.
	goc::Duration time_limit; // Maximum execution time.
	std::ostream* screen_output; // Output log to this stream (if nullptr, then no output is available).
	bool closing_state; // true if Closing state (last-edge merge), false if Opening state (iterative merge).
	
	BidirectionalLabeling();
	
	// Sets the problem to use for the labeling algorithm.
	void SetProblem(const VRPInstance& vrp, const std::vector<ProfitUnit>& profits);
	
	// Runs the bidirectional labeling algorithm and leaves the negative reduced cost routes on the parameter R.
	// Returns: the execution information log.
	goc::BLBExecutionLog Run(std::vector<goc::Route>* R);

private:
	// Attempts to merge label l against all the labels in the opposite direction dominance structure.
	// 	w: 	l will be merged with all labels m in L such that v(m) == v(l) and v(parent(m)) == w.
	// 		if w == -1, then the check v(parent(m)) == w is ignored.
	void Process(Label* l, const MonodirectionalLabeling::DominanceStructure& L, goc::Vertex w=-1);
	
	// Adds a solution to the pool S if it is the best yet found with those visited vertices.
	void AddSolution(const goc::GraphPath& p, double min_duration);
	
	VRPInstance vrp_;
	std::vector<ProfitUnit> profits_;
	
	// Pool of negative reduced cost solutions found (indexed by their visited vertices).
	// We only keep the best solution for each set of visited vertices.
	std::unordered_map<VertexSet, goc::Route> S;
};
} // namespace networks2019

#endif //NETWORKS2019_BIDIRECTIONAL_LABELING_H
