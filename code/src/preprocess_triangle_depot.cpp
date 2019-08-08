//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "preprocess_triangle_depot.h"

#include "vrp_instance.h"

using namespace std;
using namespace goc;
using namespace nlohmann;

namespace networks2019
{
namespace
{
// Removes the arc ij from the instance.
void remove_arc(Vertex i, Vertex j, json& instance)
{
	if (instance["digraph"]["arcs"][i][j] == 0) return;
	instance["digraph"]["arcs"][i][j] = 0;
	int arc_count = instance["digraph"]["arc_count"];
	instance["digraph"]["arc_count"] = arc_count - 1;
	if (has_key(instance, "travel_times")) instance["travel_times"][i][j] = vector<json>({});
}
}

void preprocess_triangle_depot(json& instance)
{
	VRPInstance vrp = instance;
	for (Vertex i: vrp.D.Vertices())
	{
		for (Vertex j: vrp.D.Successors(i))
		{
			if (i == vrp.o || j == vrp.d) continue;
			double t0_ij = vrp.TravelTime({i, vrp.d}, vrp.LDT[i][j]) + vrp.PreTravelTime({vrp.o, j}, vrp.ArrivalTime({i, vrp.d}, vrp.LDT[i][j]));
			if (epsilon_smaller_equal(t0_ij, vrp.TravelTime({i,j}, vrp.LDT[i][j])))
				remove_arc(i, j, instance);
		}
	}
}
} // namespace networks2019