//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "tdcarp/transform_problem.h"

using namespace std;
using namespace goc;
using namespace nlohmann;

namespace networks2019
{
namespace
{

// Calculates the time to depart to traverse arc e arriving at tf.
// Returns: INFTY if it is infeasible to depart inside the horizon.
double departing_time(double d, const PWLFunction& speed_function, double tf)
{
	double t = tf;
	for (int k = (int)speed_function.PieceCount()-1; k >= 0; --k)
	{
		Interval domain = speed_function[k].domain;
		double speed = speed_function[k].image.left;
		if (epsilon_equal(d, 0.0)) break;
		if (epsilon_bigger(domain.left, tf)) continue;
		double remaining_time_in_k = min(domain.right, tf) - domain.left;
		double time_to_complete_d_in_k = d / speed;
		double time_in_k = min(remaining_time_in_k, time_to_complete_d_in_k);
		t -= time_in_k;
		d -= time_in_k * speed;
	}
	if (epsilon_bigger(d, 0.0)) return INFTY;
	return t;
}


// Calculates the travel time to traverse arc e departing at t0.
// Returns: INFTY if it is infeasible to arrive inside the horizon.
double travel_time(double d, const PWLFunction& speed_function, double t0)
{
	double t = t0;
	for (int k = 0; k < speed_function.PieceCount(); ++k)
	{
		Interval domain = speed_function[k].domain;
		double speed = speed_function[k].image.left;
		if (epsilon_equal(d, 0.0)) break;
		if (epsilon_smaller(domain.right, t0)) continue;
		double remaining_time_in_k = domain.right - max(domain.left, t0);
		double time_to_complete_d_in_k = d / speed;
		double time_in_k = min(remaining_time_in_k, time_to_complete_d_in_k);
		t += time_in_k;
		d -= time_in_k * speed;
	}
	if (epsilon_bigger(d, 0.0)) return INFTY;
	return t-t0;
}

// Returns the time when we arrive at the end of arc e if departing at t0.
double ready_time(double d, const PWLFunction& speed_function, double t0)
{
	double tt = travel_time(d, speed_function, t0);
	return tt == INFTY ? tt : t0 + tt;
}

// Precondition: no speeds are 0.
PWLFunction compute_travel_time_function(double dist, const PWLFunction& speed_function)
{
	// Calculate speed breakpoints.
	vector<double> speed_breakpoints;
	for (auto& p: speed_function.Pieces()) speed_breakpoints.push_back(p.domain.left);
	speed_breakpoints.push_back(speed_function.LastPiece().domain.right);
	
	// Travel time breakpoints are two sets
	// 	- B1: speed breakpoints which are feasible to depart
	// 	- B2: times t such that we arrive to head(e) at a speed breakpoint.
	vector<double> B1;
	for (double t: speed_breakpoints)
		if (travel_time(dist, speed_function, t) != INFTY)
			B1.push_back(t);
		
	vector<double> B2;
	for (double t: speed_breakpoints)
		if (departing_time(dist, speed_function, t) != INFTY)
			B2.push_back(departing_time(dist, speed_function, t));
	
	// Merge breakpoints in order in a set B.
	vector<double> B(B1.size()+B2.size());
	merge(B1.begin(), B1.end(), B2.begin(), B2.end(), B.begin());
	
	// Remove duplicates from B.
	B.resize(distance(B.begin(), unique(B.begin(), B.end())));
	
	// Calculate travel times for each t \in B.
	vector<double> T;
	for (double t: B) T.push_back(travel_time(dist, speed_function, t));
	
	// Create travel time function.
	PWLFunction tau;
	for (int i = 0; i < (int)B.size()-1; ++i)
		tau.AddPiece(LinearFunction(Point2D(B[i], T[i]), Point2D(B[i+1], T[i+1])));
	
	return tau;
}

Matrix<PWLFunction> quickest_paths(Digraph D, Matrix<PWLFunction> travel_times, Interval horizon)
{
	// Calculate arriving times
	Matrix<PWLFunction> arriving_times;
	for (Arc e: D.Arcs()) {
		PWLFunction& tau = travel_times[e.tail][e.head];
		arriving_times[e.tail][e.head] = tau + PWLFunction::IdentityFunction(tau.Domain());
	}

	// Quickest paths algorithm
	Matrix<PWLFunction> xxxx(D.VertexCount(), D.VertexCount());
	for (Vertex v : D.Vertices()) {
		auto& xx = xxxx[v];
		for (Vertex w : D.Vertices()) {
			if (w == v) {
				xx[w] = PWLFunction::IdentityFunction(horizon);
			} else {
				// Empty PWLFunction
			}
		}

		for (int i = 0; i <= D.VertexCount(); i++) { // Could be done better
			for (Vertex from : D.Vertices()) {
				for (Arc e : D.OutboundArcs(from)) {
					auto yy = arriving_times[e.tail][e.head].Compose(xx[e.tail]);
					xx[e.head] = Min(xx[e.head], yy);
				}
			}
		}
	}

	Matrix<PWLFunction> zzzz(D.VertexCount(), D.VertexCount());
	for (Vertex v : D.Vertices()) {
		for (Vertex w : D.Vertices()) {
			zzzz[v][w] = xxxx[v][w] - PWLFunction::IdentityFunction(xxxx[v][w].Domain());
		}
	}

	return zzzz;
}

}

nlohmann::json transform_problem(nlohmann::json& instance)
{
	json res;

	// Input digraph and travel times
    Digraph original_digraph((int) instance["graph"]["vertex_count"]);
    Matrix<PWLFunction> original_travel_times(original_digraph.VertexCount(), original_digraph.VertexCount());
    for (auto& e : instance["graph"]["edges"]) {
        original_digraph.AddArc({e["tail"], e["head"]});

		PWLFunction speed_function;
        int piece_start = instance["horizon"];
        for (auto& piece : e["travel_time"]) {
			speed_function.AddPiece(LinearFunction({(double) piece_start, e["speed"]}, {e["piece_end"], e["speed"]}));
			piece_start = e["piece_end"];
        }
        original_travel_times[e["tail"]][e["head"]] = compute_travel_time_function(e["distance"], speed_function);
    }

	Matrix<PWLFunction> quickest = quickest_paths(original_digraph, original_travel_times, instance["horizon"]);

	// Group edges with demand by their incident node set
    map<pair<int, int>, vector<json>> xxx;
    for (auto& e : instance["graph"]["edges"]) {
        if (e["demand"] > 0) {
            int tail = e["tail"];
            int head = e["head"];
            xxx[make_pair(min(tail, head), max(tail, head))].push_back(e);
        }
    }

	// Arbitrarily pick edges to serve
	vector<json> serviced_edges;
	for (auto& kv : xxx) {
		int node_a = kv.first.first;
		vector<json> edges = kv.second;
		serviced_edges.push_back((node_a % 3 == 0) ? edges[0] : edges[1]);
	}

	// Make digraph
	int n = serviced_edges.size() + 2; // start_depot = 0; end_depot = n-1
	Digraph D(n);
	for (int i = 1; i < n-1; i++) {
		D.AddArc({0, i});
		D.AddArc({i, n-1});
		for (int j = 1; j < n-1; j++) if (i != j) {
			D.AddArc({i, j});
		}
	}

	// Travel times (with service time included)
	Matrix<PWLFunction> travel_times(D.VertexCount(), D.VertexCount());
	for (int i = 1; i < n-1; i++) {
		json& e = serviced_edges[i-1];
		auto service_time = [=] (json& ed) { return original_travel_times[e["tail"]][e["head"]] * (double) instance["service_speed_factor"]; }; 
		travel_times[0][i] = original_travel_times[instance["depot"]][e["tail"]] + service_time(e);
		travel_times[i][n-1] = original_travel_times[e["head"]][instance["depot"]]; 
		for (int j = 1; j < n-1; j++) if (i != j) {
			json& to = serviced_edges[j-1];
			travel_times[i][j] = quickest[e["head"]][to["tail"]] + service_time(to);
		}
	}

	// Marshalling
    res["capacity"] = instance["capacity"];
    res["instance_name"] = instance["instance_name"];
    res["vehicle_count"] = instance["vehicle_count"];
    res["horizon"] = instance["horizon"];
	res["start_depot"] = 0;
    res["end_depot"] = n-1;
	res["time_windows"] = json::array();
    for (int i = 0; i < n; i++) {
        res["time_windows"].push_back( json::array({res["horizon"][0], res["horizon"][1]}) );
    }
	res["digraph"]["vertex_count"] = D.VertexCount();
	res["digraph"]["arc_count"] = D.ArcCount();
	res["digraph"]["arcs"] = Matrix<int>(D.VertexCount(), D.VertexCount());
	for(int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			res["digraph"]["arcs"][i][j] = (D.IncludesArc({i, j})) ? 1 : 0;
		}
	}
	res["travel_times"] = travel_times;
	res["demands"] = json::array();
	res["demands"].push_back(0); // start_depot
	for (int i = 1; i < n-1; i++) {
		res["demands"].push_back(serviced_edges[i-1]["demand"]);
	}
	res["demands"].push_back(0); // end_depot
	res["service_times"] = json::array(); // already included in travel_times
	for (int i = 0; i < n; i++) {
		res["service_times"].push_back(0);
	}

	//res["digraph"]["coordinates"] // unused
    //res["distances"] // already included in travel_times

    return res;
}
} // namespace networks2019