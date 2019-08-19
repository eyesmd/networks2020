//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "labeling/bidirectional_labeling.h"

#include "bcp/pricing_problem.h"

#include <climits>

using namespace std;
using namespace goc;

namespace networks2019
{
namespace
{
// Reverses a VRP instance.
// o' := d
// d' := o
// D' := reverse(D)
// tw'(v) := [T-b(v), T-a(v)]
// arr'_vu(t) := T-dep_uv(T-t)
VRPInstance reverse_instance(const VRPInstance& vrp)
{
	VRPInstance r = vrp;
	swap(r.o, r.d);
	r.D = vrp.D.Reverse();
	for (Vertex v: r.D.Vertices()) r.tw[v] = {vrp.T - vrp.tw[v].right, vrp.T - vrp.tw[v].left};
	for (Vertex u: vrp.D.Vertices())
	{
		for (Vertex v: vrp.D.Successors(u))
		{
			// Compute reverse travel functions.
			r.arr[v][u] = vrp.T - vrp.dep[u][v].Compose(vrp.T - PWLFunction::IdentityFunction({0.0, vrp.T}));
			r.arr[v][u] = Min(PWLFunction::ConstantFunction(min(img(r.arr[v][u])), {min(r.tw[v]), min(dom(r.arr[v][u]))}), r.arr[v][u]);
			r.tau[v][u] = r.arr[v][u] - PWLFunction::IdentityFunction({0.0, vrp.T});
			r.dep[v][u] = r.arr[v][u].Inverse();
			r.pretau[v][u] = PWLFunction::IdentityFunction(dom(r.dep[v][u])) - r.dep[v][u];
		}
	}
	// Add travel functions for (i, i) (for boundary reasons).
	for (Vertex u: r.D.Vertices())
	{
		r.tau[u][u] = r.pretau[u][u] = PWLFunction::ConstantFunction(0.0, r.tw[u]);
		r.dep[u][u] = r.arr[u][u] = PWLFunction::IdentityFunction(r.tw[u]);
	}
	// Set LDT.
	for (Vertex i: r.D.Vertices())
	{
		vector<TimeUnit> LDT_i = compute_latest_departure_time(r.D, i, r.tw[i].right, [&] (Vertex u, Vertex v, double tf) { return r.DepartureTime({u,v}, tf); });
		for (Vertex k: r.D.Vertices()) r.LDT[k][i] = LDT_i[k];
	}
	return r;
}

PricingProblem reverse_pricing_problem(const PricingProblem& pp)
{
	PricingProblem rpp = pp;
	rpp.A.clear();
	for (Arc e: pp.A) rpp.A.push_back(e.Reverse());
	return rpp;
}
}

BidirectionalLabeling::BidirectionalLabeling(const VRPInstance& vrp)
	: vrp_(vrp), lbl_{MonodirectionalLabeling(vrp_), MonodirectionalLabeling(reverse_instance(vrp_))}
{
	solution_limit = INT_MAX;
	time_limit = Duration::Max();
	screen_output = nullptr;
	closing_state = true;
	merge_start = 0;
	lbl_[0].process_limit = lbl_[1].process_limit = 10;
	lbl_[0].cross = false, lbl_[1].cross = true;
	partial = limited_extension = lazy_extension = unreachable_strengthened = sort_by_cost = true;
	relax_elementary_check = relax_cost_check = false;
}

BLBExecutionLog BidirectionalLabeling::Run(const PricingProblem& pricing_problem, vector<Route>* R)
{
	// Clean solution pool.
	S.clear();
	M[0] = M[1] = vector<MonodirectionalLabeling::DemandLevel>(vrp_.D.VertexCount());
	
	// Set pricing problem.
	vrp_.D.AddArcs(pp_.A); // Add previously forbidden arcs.
	pp_ = pricing_problem;
	vrp_.D.RemoveArcs(pp_.A); // Remove pricing problem forbidden arcs.
	
	// Init forward and backward labeling.
	lbl_[0].SetProblem(pp_);
	lbl_[1].SetProblem(reverse_pricing_problem(pp_));
	lbl_[0].t_m = lbl_[1].t_m = symmetric ? vrp_.T / 2 : vrp_.T;
	
	lbl_[0].partial = lbl_[1].partial = partial;
	lbl_[0].relax_elementary_check = lbl_[1].relax_elementary_check = relax_elementary_check;
	lbl_[0].relax_cost_check = lbl_[1].relax_cost_check = relax_cost_check;
	lbl_[0].limited_extension = lbl_[1].limited_extension = limited_extension;
	lbl_[0].lazy_extension = lbl_[1].lazy_extension = lazy_extension;
	lbl_[0].sort_by_cost = lbl_[1].sort_by_cost = sort_by_cost;
	lbl_[0].unreachable_strengthened = lbl_[1].unreachable_strengthened = unreachable_strengthened;
	lbl_[0].correcting = lbl_[1].correcting = correcting;
	
	BLBExecutionLog log(true);
	Stopwatch rolex(false), merge_rolex(false);
	
	// Init queues with initial labels.
	LBQueue q[2];
	q[0].push(lbl_[0].Init());
	q[1].push(lbl_[1].Init());
	
	// Index monodirectional labeling logs by direction.
	MLBExecutionLog* mlb_log[2] { &*log.forward_log, &*log.backward_log };
	
	// Initialize output.
	TableStream tstream(screen_output, 2.0);
	tstream.AddColumn("time", 10).AddColumn("fw-time", 10).AddColumn("bw-time", 10).AddColumn("fw-proc", 10).
		AddColumn("bw-proc", 10).AddColumn("#sol", 6).AddColumn("fw-t_m", 8).AddColumn("bw-t_m", 8).
		AddColumn("#q-f", 10).AddColumn("#q-b", 10);
	tstream.WriteHeader();
	
	rolex.Resume();
	
	// While there are labels to extend, do it.
	bool processed = true;
	while (processed)
	{
		processed = false;
		// For each direction of the labeling (0=Forward, 1=Backward).
		for (int d: {0, 1})
		{
			int od = (d+1)%2; // opposite direction.
			
			if (q[d].empty()) continue;
			if (rolex.Peek() >= time_limit) { log.status = BLBStatus::TimeLimitReached; break; } // Check if TLim is reached.
			if (S.size() >= solution_limit) { log.status = BLBStatus::SolutionLimitReached; break; } // Check if SLim is reached.
			lbl_[d].time_limit = time_limit - rolex.Peek(); // Set time limit.
			auto P = lbl_[d].Run(&q[d], mlb_log[d]);
			
			// If iterative-merge is enabled, then add the labels to the structure.
			if (!closing_state)
				for (Label* l: P)
					insert_sorted(M[d][l->v].Insert(floor(l->q), {}), l, [] (Label* l, Label* m) { return l->min_cost < m->min_cost; });
			
			// If iterative-merge is enabled, then try to merge.
			if (!closing_state && log.forward_log->processed_count >= merge_start)
			{
				merge_rolex.Reset().Resume();
				for (Label* l: P) IterativeMerge(l, M[od]);
				*log.merge_time += merge_rolex.Pause();
			}
			
			// Check if any full route was generated.
			for (Label* l: P)
				if (d == 0 && l->v == vrp_.d && epsilon_smaller(l->min_cost, 0.0))
					AddSolution(l->Path(), min(img(l->duration)));
			
			// Update t_m.
			if (q[d].empty()) lbl_[d].t_m = vrp_.T - lbl_[od].t_m; // If d has no more labels in the queue, the middle is t_m
			else lbl_[od].t_m = min(lbl_[od].t_m, max(vrp_.T-lbl_[d].t_m, vrp_.T-q[d].top().makespan));
			
			// Check if any label was processed.
			processed |= !P.empty();
		}
		
		// Output to screen.
		if (tstream.RegisterAttempt() || !processed)
		{
			tstream.WriteRow({STR(rolex.Peek()), STR(mlb_log[0]->time), STR(mlb_log[1]->time),
					 STR(mlb_log[0]->processed_count), STR(mlb_log[1]->processed_count), STR(S.size()),
					 STR(lbl_[0].t_m), STR(vrp_.T-lbl_[1].t_m), STR(q[0].size()), STR(q[1].size())});
		}
	}
	
	// Last-edge merge.
	if (S.size() < solution_limit && rolex.Peek() < time_limit)
	{
		merge_rolex.Reset().Resume();
		LastArcMerge(q[0], lbl_[1].U);
		*log.merge_time += merge_rolex.Pause();
	}
	
	if (S.size() >= solution_limit) log.status = BLBStatus::SolutionLimitReached;
	else if (log.status == BLBStatus::DidNotStart) log.status = BLBStatus::Finished;
	*log.time += rolex.Pause();
	
	// Add solutions from the pool to the return vector R.
	for (auto& V_r: S)
	{
		Route& r = V_r.second;
		// Compute r actual duration.
		R->push_back(vrp_.BestDurationRoute(r.path));
	}
	
	return log;
}

void BidirectionalLabeling::IterativeMerge(Label* l, const MonodirectionalLabeling::DominanceStructure& L)
{
	TimeUnit T = vrp_.T;
	for (auto& demand_entry : L[l->v])
	{
		if (S.size() >= solution_limit) break; // Do not exceed solution limit.
		if (epsilon_bigger(demand_entry.first+l->q-vrp_.q[l->v], vrp_.Q)) break;
		for (auto& m: demand_entry.second)
		{
			if (S.size() >= solution_limit) break; // Do not exceed solution limit.
			if (epsilon_bigger_equal(m->min_cost+l->min_cost+pp_.P[l->v] + l->cut_cost - l->parent->cut_cost, 0.0)) break;
			Merge(l, m);
		}
	}
}

void BidirectionalLabeling::LastArcMerge(LBQueue& qf, const MonodirectionalLabeling::DominanceStructure& Lb)
{
	TimeUnit T = vrp_.T;
	
	// Create M_ijq structure.
	Matrix<VectorMap<CapacityUnit, vector<Label*>>> M(vrp_.D.VertexCount(), vrp_.D.VertexCount());
	for (Vertex v: vrp_.D.Vertices())
		for (auto& entry: Lb[v])
			for (auto& m: entry.second)
				insert_sorted(M[m->v][m->parent->v].Insert(entry.first, {}), m, [] (Label* m1, Label* m2) { return m1->min_cost < m2->min_cost; });
	
	while (!qf.empty())
	{
		LazyLabel ll = qf.top();
		qf.pop();
		Label* l = ll.parent;
		if (S.size() >= solution_limit) continue;
		
		for (auto& entry: M[ll.parent->v][ll.v])
		{
			if (epsilon_bigger(entry.first + l->q - vrp_.q[l->v], vrp_.Q)) break;
			if (S.size() >= solution_limit) break; // Do not exceed solution limit.
			for (Label* m: entry.second)
			{
				if (S.size() >= solution_limit) break; // Do not exceed solution limit.
				if (epsilon_bigger_equal(m->min_cost+l->min_cost+pp_.P[l->v] + l->cut_cost - l->parent->cut_cost, 0.0)) break;
				Merge(l, m);
			}
		}
	}
}

void BidirectionalLabeling::Merge(Label* l, Label* m)
{
	TimeUnit T = vrp_.T;
	
	if (epsilon_bigger(min(l->rw), T-min(m->rw))) return;
	if (intersection(l->S, m->S) != create_bitset<MAX_N>({l->v})) return;
	
	Route r;
	// Merge l and m duration functions lm_d(t) = l_d(t) + m_d(T-t).
	if (epsilon_bigger_equal(T-max(m->rw), max(l->rw)))
	{
		r.duration = l->duration(max(l->rw)) + m->duration(max(m->rw)) + (T-max(m->rw)) - max(l->rw);
	}
	else
	{
		PWLFunction lm_duration = l->duration + m->duration.Compose(T - PWLFunction::IdentityFunction({0.0, T}));
		if (lm_duration.Empty()) return;
		r.duration = min(img(lm_duration));
	}
	
	double merge_cut_cost = 0.0;
	for (int i = 0; i < pp_.S.size(); ++i) if (l->parent->cut_visited[i]+m->cut_visited[i] >= 2) merge_cut_cost += pp_.sigma[i];
	double merge_cost = r.duration - l->p - m->p + pp_.P[l->v] - merge_cut_cost;
	if (epsilon_bigger_equal(merge_cost, 0.0)) return;
	
	// Merge l and m paths.
	r.path = l->Path();
	for (Label* x = m->parent; x->parent != nullptr; x = x->parent) r.path.push_back(x->v);
	if (r.path[0] != vrp_.o) r.path = reverse(r.path);
	
	// We have a negative reduced cost route r.
	AddSolution(r.path, r.duration);
}

void BidirectionalLabeling::AddSolution(const goc::GraphPath& p, double min_duration)
{
	VertexSet V = create_bitset<MAX_N>(p);
	if (!includes_key(S, V)) S[V] = Route({}, 0.0, INFTY);
	if (S[V].duration > min_duration) S[V] = Route(p, 0.0, min_duration);
}
} // namespace networks2019