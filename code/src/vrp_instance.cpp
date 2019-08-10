//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "vrp_instance.h"

using namespace std;
using namespace goc;
using namespace nlohmann;

namespace networks2019
{
TimeUnit VRPInstance::TravelTime(goc::Arc e, TimeUnit t0) const
{
	auto& tau_e = tau[e.tail][e.head];
	if (epsilon_bigger(t0, max(dom(tau_e)))) return INFTY;
	else if (epsilon_smaller(t0, min(dom(tau_e)))) return min(dom(tau_e))+tau_e.Value(min(dom(tau_e)))-t0;
	return tau_e.Value(t0);
}

TimeUnit VRPInstance::PreTravelTime(goc::Arc e, TimeUnit tf) const
{
	auto& pretau_e = pretau[e.tail][e.head];
	if (epsilon_smaller(tf, min(dom(pretau_e)))) return INFTY;
	else if (epsilon_bigger(tf, max(dom(pretau_e)))) return tf-max(dom(pretau_e))+pretau_e.Value(max(dom(pretau_e)));
	return pretau_e.Value(tf);
}

TimeUnit VRPInstance::ArrivalTime(goc::Arc e, TimeUnit t0) const
{
	auto& arr_e = arr[e.tail][e.head];
	if (epsilon_bigger(t0, max(dom(arr_e)))) return INFTY;
	else if (epsilon_smaller(t0, min(dom(arr_e)))) return min(img(arr_e));
	return arr_e.Value(t0);
}

TimeUnit VRPInstance::DepartureTime(goc::Arc e, TimeUnit tf) const
{
	auto& dep_e = dep[e.tail][e.head];
	if (epsilon_smaller(tf, min(dom(dep_e)))) return INFTY;
	else if (epsilon_bigger(tf, max(dom(dep_e)))) return max(img(dep_e));
	return dep_e.Value(tf);
}

TimeUnit VRPInstance::ReadyTime(const GraphPath& p, TimeUnit t0) const
{
	CapacityUnit qq = q[0];
	TimeUnit t = t0;
	for (int k = 0; k < (int)p.size()-1; ++k)
	{
		Vertex i = p[k], j = p[k+1];
		if (!tau[i][j].Domain().Includes(t)) return INFTY;
		t += tau[i][j](t);
		qq += q[j];
	}
	if (epsilon_bigger(qq, Q)) return INFTY;
	return t;
}

Route VRPInstance::BestDurationRoute(const GraphPath& p) const
{
	PWLFunction Delta = arr[p[0]][p[0]];
	if (Delta.Empty()) return {{}, 0.0, INFTY};
	for (int k = 0; k < (int)p.size()-1; ++k)
	{
		Vertex i = p[k], j = p[k+1];
		Delta = arr[i][j].Compose(Delta);
		if (Delta.Empty()) return {{}, 0.0, INFTY};
	}
	Delta = Delta - PWLFunction::IdentityFunction(dom(Delta));
	return Route(p, Delta.PreValue(min(img(Delta))), min(img(Delta)));
}

VertexSet VRPInstance::Unreachable(Vertex v, TimeUnit t0) const
{
	VertexSet U;
	for (Vertex w: D.Vertices()) if (epsilon_bigger(t0, LDT[v][w])) U.set(w);
	return U;
}

void VRPInstance::Print(ostream& os) const
{
	os << json(*this);
}

void to_json(json& j, const VRPInstance& instance)
{
	j["digraph"] = instance.D;
	j["start_depot"] = instance.o;
	j["end_depot"] = instance.d;
	j["horizon"] = vector<TimeUnit>({0, instance.T});
	j["time_windows"] = instance.tw;
	j["capacity"] = instance.Q;
	j["demands"] = instance.q;
	j["travel_times"] = instance.tau;
}

void from_json(const json& j, VRPInstance& instance)
{
	int n = j["digraph"]["vertex_count"];
	instance.D = j["digraph"];
	instance.o = j["start_depot"];
	instance.d = j["end_depot"];
	instance.T = j["horizon"][1];
	instance.tw = vector<Interval>(j["time_windows"].begin(), j["time_windows"].end());
	instance.Q = value_or_default(j, "capacity", 1.0);
	instance.q = vector<CapacityUnit>(j["demands"].begin(), j["demands"].end());
	// Add travel time functions.
	instance.tau = instance.arr = instance.dep = instance.pretau = Matrix<PWLFunction>(n, n);
	for (Vertex u: instance.D.Vertices())
	{
		for (Vertex v: instance.D.Successors(u))
		{
			instance.tau[u][v] = j["travel_times"][u][v];
			instance.arr[u][v] = instance.tau[u][v] + PWLFunction::IdentityFunction(instance.tau[u][v].Domain());
			instance.dep[u][v] = instance.arr[u][v].Inverse();
			instance.pretau[u][v] = PWLFunction::IdentityFunction(instance.dep[u][v].Domain()) - instance.dep[u][v];
		}
	}
	// Add travel functions for (i, i) (for boundary reasons).
	for (Vertex u: instance.D.Vertices())
	{
		instance.tau[u][u] = instance.pretau[u][u] = PWLFunction::ConstantFunction(0.0, instance.tw[u]);
		instance.dep[u][u] = instance.arr[u][u] = PWLFunction::IdentityFunction(instance.tw[u]);
	}
	// Set LDT.
	instance.LDT = Matrix<TimeUnit>(n, n);
	for (Vertex i: instance.D.Vertices())
	{
		vector<TimeUnit> LDT_i = compute_latest_departure_time(instance.D, i, instance.tw[i].right, [&] (Vertex u, Vertex v, double tf) { return instance.DepartureTime({u,v}, tf); });
		for (Vertex k: instance.D.Vertices()) instance.LDT[k][i] = LDT_i[k];
	}
}
} // namespace networks2019