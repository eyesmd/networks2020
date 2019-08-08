//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include <iostream>
#include <vector>
#include <goc/goc.h>

#include "vrp_instance.h"
#include "preprocess_travel_times.h"
#include "preprocess_capacity.h"
#include "preprocess_time_windows.h"
#include "preprocess_service_waiting.h"
#include "preprocess_triangle_depot.h"

#include "bidirectional_labeling.h"

using namespace std;
using namespace goc;
using namespace nlohmann;
using namespace networks2019;

namespace
{
// Returns: the sum of all profits in p.
ProfitUnit path_profit(const GraphPath& p, const vector<ProfitUnit>& profits)
{
	return sum<Vertex>(p, [&] (Vertex v) { return profits[v]; });
}
}

int main()
{
	json output; // STDOUT output will go into this JSON.
	
	simulate_input_in_debug("instances/networks_2019b", "C207_50_b", "experiments/pricing.json", "PART");
	
	json experiment, instance, solutions;
	cin >> experiment >> instance >> solutions;
	
	// Parse experiment.
	Duration time_limit = value_or_default(experiment, "time_limit", 2.0_hr);
	
	// Show experiment details.
	clog << "Time limit: " << time_limit << "s." << endl;
	
	// Preprocess instance JSON.
	clog << "Preprocessing..." << endl;
	preprocess_capacity(instance);
	preprocess_travel_times(instance);
	preprocess_service_waiting(instance);
	preprocess_time_windows(instance);
	preprocess_triangle_depot(instance);
	
	// Parse instance.
	VRPInstance vrp = instance;
	
	// Read profits if present.
	vector<ProfitUnit> profits(instance["profits"].begin(), instance["profits"].end());
	clog << "Running pricing algorithm..." << endl;
	BidirectionalLabeling lbl;
	lbl.screen_output = &clog;
	lbl.SetProblem(vrp, profits);
	vector<Route> R;
	BLBExecutionLog log = lbl.Run(&R);
	
	clog << "Total time: " << log.time << endl;
	// Get best route.
	if (!R.empty())
	{
		Route best = R[0];
		for (auto& r: R)
			if (r.duration-path_profit(r.path, profits) < best.duration-path_profit(best.path, profits))
				best = r;
		clog << "Best solution: " << endl;
		clog << "\tpath: " << best.path << endl;
		clog << "\tt0: " << best.t0 << endl;
		clog << "\tduration: " << best.duration << endl;
		clog << "\tcost: " << best.duration-path_profit(best.path, profits) << endl;
		output["Best solution"] = VRPSolution(best.duration-path_profit(best.path, profits), {best});
	}
	output["Exact"] = log;
	
	// Send JSON output to cout.
	cout << output << endl;
	return 0;
}