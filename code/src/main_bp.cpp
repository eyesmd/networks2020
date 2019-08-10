//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include <iostream>
#include <vector>
#include <climits>

#include <goc/goc.h>

#include "vrp_instance.h"
#include "preprocess_travel_times.h"
#include "preprocess_capacity.h"
#include "preprocess_time_windows.h"
#include "preprocess_service_waiting.h"
#include "preprocess_triangle_depot.h"

#include "bcp.h"
#include "spf.h"
#include "pricing_problem.h"
#include "bidirectional_labeling.h"

using namespace std;
using namespace goc;
using namespace nlohmann;
using namespace networks2019;

double path_cost(const VRPInstance& vrp, PricingProblem pp, GraphPath p)
{
	return vrp.BestDurationRoute(p).duration - sum<Vertex>(p, [&] (Vertex v) { return pp.P[v]; });
}

int main()
{
	json output; // STDOUT output will go into this JSON.
	
	simulate_input_in_debug("instances/dabia_et_al_2013", "C104_25", "experiments/bp.json", "BP-PART");
	
	json experiment, instance, solutions;
	cin >> experiment >> instance >> solutions;
	
	// Parse experiment.
	Duration time_limit = value_or_default(experiment, "time_limit", 2.0_hr);
	int cut_limit = value_or_default(experiment, "cut_limit", 0);
	int node_limit = value_or_default(experiment, "node_limit", INT_MAX);
	
	// Show experiment details.
	clog << "Time limit: " << time_limit << "s." << endl;
	clog << "Cut limit: " << cut_limit << endl;
	clog << "Node limit: " << node_limit << endl;
	
	// Preprocess instance JSON.
	clog << "Preprocessing..." << endl;
	preprocess_capacity(instance);
	preprocess_travel_times(instance);
	preprocess_service_waiting(instance);
	preprocess_time_windows(instance);
	preprocess_triangle_depot(instance);
	
	// Parse instance.
	VRPInstance vrp = instance;
	
	// Run BCP.
	clog << "Running BCP algorithm..." << endl;
	
	// Create SPF and add initial routes (o, i, d).
	SPF spf(vrp.D.VertexCount());
	for (Vertex i: exclude(vrp.D.Vertices(), {vrp.o, vrp.d})) spf.AddRoute(vrp.BestDurationRoute({vrp.o, i, vrp.d}));
	
	BCP bcp(vrp.D, &spf);
	bcp.time_limit = time_limit;
	bcp.cut_limit = cut_limit;
	bcp.node_limit = node_limit;
	
	BidirectionalLabeling lbl(vrp);
	lbl.solution_limit = 3000;
	lbl.closing_state = false;
	bcp.pricing_solver = [&] (const PricingProblem& pricing_problem, int node_number, Duration tlimit, CGExecutionLog* cg_execution_log)
	{
		lbl.time_limit = tlimit;
		vector<Route> R;
		auto lbl_log = lbl.Run(pricing_problem, &R);
		
		// Add negative reduced cost routes.
		for (auto& r: R) spf.AddRoute(r);
		
		// Add iteration log.
		cg_execution_log->iterations->push_back(lbl_log);
		
		// Update merge_start and closing_state.
		if (lbl_log.status == BLBStatus::Finished) lbl.closing_state = true;
		lbl.merge_start = (lbl.merge_start + lbl_log.forward_log->processed_count) / 2;
	};
	VRPSolution solution(INFTY, {});
	auto log = bcp.Run(&solution);
	
	output["Exact"] = log;
	output["Best solution"] = solution;
	
	clog << "Time: " << log.time << endl;
	clog << "#Nodes: " << log.nodes_closed << endl;
	clog << "Status: " << log.status << endl;
	if (solution.value == INFTY) clog << "No solution found." << endl;
	if (solution.value != INFTY)
	{
		clog << "Best solution:" << endl;
		clog << "\tValue: " << solution.value << endl;
		clog << "\tRoutes:" << endl;
		for (auto& r: solution.routes) clog << "\t\t" << r << endl;
	}
	
	// Send JSON output to cout.
	cout << output << endl;
	return 0;
}