//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "monodirectional_labeling.h"

#include <climits>

#include "pwl_domination_function.h"

using namespace std;
using namespace goc;

namespace networks2019
{
MonodirectionalLabeling::MonodirectionalLabeling()
{
	t_m = INFTY;
	cross = true;
	process_limit = INT_MAX;
	time_limit = 2.0_hr;
	partial = true;
	processed_count = 0;
}

MonodirectionalLabeling::~MonodirectionalLabeling()
{
}

void MonodirectionalLabeling::SetProblem(const VRPInstance& vrp, const vector<ProfitUnit>& profits)
{
	vrp_ = vrp;
	profits_ = profits;
	t_m = vrp.T;
	U = vector<DemandLevel>(vrp.D.VertexCount());
	
	// no-label is a label that represents the empty path.
	no_label.parent = nullptr;
	no_label.p = no_label.q = no_label.min_cost = 0.0;
	no_label.duration = vrp.tau[vrp.o][vrp.o];
	no_label.rw = dom(no_label.duration);
	no_label.length = 0;
	no_label.S = no_label.U = {};
	no_label.v = vrp.o;
}

vector<Label*> MonodirectionalLabeling::Run(LBQueue* q, MLBExecutionLog* log)
{
	// Use rolex to measure whole run time, and rolex2 to measure steps time.
	Stopwatch rolex(true), rolex2(false);
	vector<Label*> P; // Processed labels.
	while (!q->empty())
	{
		if (P.size() >= process_limit) { log->status = MLBStatus::ProcessLimitReached; break; }
		if (rolex.Peek() >= time_limit) { log->status = MLBStatus::TimeLimitReached; break; }
		
		// If label crossed t_m, but should not, then stop processing queue.
		// This extensions will be used for last-edge merge strategy.
		if (!cross && epsilon_bigger(q->top().makespan, t_m)) break;
		
		// Pop label.
		rolex2.Reset().Resume();
		LazyLabel ll = q->top();
		q->pop();
		*log->queuing_time += rolex2.Pause();
		
		// Turn lazy label into complete label.
		rolex2.Reset().Resume();
		Label* l = ExtensionStep(ll);
		*log->extension_time += rolex2.Pause();
		if (!l) continue; // Label could not be extended.
		(*log->extended_count)++;
		
		// Check domination.
		rolex2.Reset().Resume();
		bool is_dominated = DominationStep(l);
		*log->domination_time += rolex2.Pause();
		if (is_dominated) *log->positive_domination_time += rolex2.Pause();
		if (!is_dominated) *log->negative_domination_time += rolex2.Pause();
		if (is_dominated) { (*log->dominated_count)++; delete l; continue; } // Label is dominated, ignore.

		// Correct existing labels.
		rolex2.Reset().Resume();
		*log->corrected_count += CorrectionStep(l);
		(*log->correction_time) += rolex2.Pause();
		
		// If min(rw(l)) > t_m, then l should not be extended and ll should have been preserved in the queue with the
		// new makespan.
		if (!cross && epsilon_bigger(min(l->rw), t_m))
		{
			q->push(LazyLabel(l->parent, l->v, min(l->rw)));
			continue;
		}
		
		// Otherwise, label l should be processed and extended.
		// Process label.
		rolex2.Reset().Resume();
		ProcessStep(l);
		(*log->processed_count)++;
		*log->process_time += rolex2.Pause();
		P.push_back(l);
		processed_count++;
		
		// Get feasible extensions.
		if (epsilon_smaller_equal(min(l->rw), t_m))
		{
			rolex2.Reset().Resume();
			auto extensions = EnumerationStep(l);
			*log->enumeration_time += rolex2.Pause();
			*log->enumerated_count += extensions.size();
			
			// Add feasible extensions to the queue.
			rolex2.Reset().Resume();
			for (LazyLabel& ll: extensions) q->push(ll);
			*log->queuing_time += rolex2.Pause();
		}
	}
	
	if (q->empty()) log->status = MLBStatus::Finished;
	*log->time += rolex.Pause();
	
	return P;
}

LazyLabel MonodirectionalLabeling::Init() const
{
	return {(Label*)&no_label, vrp_.o, vrp_.tw[vrp_.o].left};
}

Label* MonodirectionalLabeling::ExtensionStep(const LazyLabel& ll) const
{
	auto& l = ll.parent;
	Vertex v = ll.v;
	Vertex u = l->v;
	
	// Check if depot triangle inequality holds.
	// If max(rw(lv)) < a_v and tau_u0v(max(rw(lv))) <= a_v - max(rw(lv)) then ignore label.
	if (epsilon_smaller(l->rw.right, vrp_.tw[v].left) && vrp_.D.IncludesArc({u, vrp_.d}) && vrp_.D.IncludesArc({vrp_.o, v}))
	{
		TimeUnit tau_u0v = vrp_.TravelTime({u, vrp_.d}, max(l->rw)) + vrp_.PreTravelTime({vrp_.o, v}, vrp_.tw[v].left);
		if (epsilon_smaller(tau_u0v, vrp_.tw[v].left - l->rw.right)) return nullptr;
	}
	
	auto lv = new Label();
	lv->parent = l;
	lv->v = v;
	lv->q = l->q + vrp_.q[v];
	lv->p = l->p + profits_[v];
	lv->length = l->length+1;
	// If max(rw(l)) > min(img(dep_uv)) then no matter when we depart we reach v before its time window.
	// We have to wait, therefore the duration function is a single point.
	if (epsilon_smaller(max(l->rw), min(img(vrp_.dep[u][v]))))
		lv->duration = PWLFunction::ConstantFunction(l->duration(max(l->rw))+min(vrp_.tw[v])-max(l->rw), {min(vrp_.tw[v]), min(vrp_.tw[v])});
	// Otherwise, we do the classic extension D_lv(t) = D_l(\dep_uv(t)) + \tau_uv(\dep_uv(t)).
	else
		lv->duration = (l->duration + vrp_.tau[u][v]).Compose(vrp_.dep[u][v]);
	
	if (lv->duration.Empty()) { delete lv; return nullptr; } // If no duration pieces exist, then ignore.
	lv->rw = dom(lv->duration);
	lv->min_cost = min(img(lv->duration)) - lv->p;
	lv->S = unite(l->S, {v});
	lv->U = unite(lv->S, vrp_.Unreachable(v, lv->rw.left));
	return lv;
}

bool MonodirectionalLabeling::DominationStep(Label* l) const
{
	// If full route, then it is dominated if the reduced cost is bigger than or equal to zero.
	if (l->v == vrp_.d) return epsilon_bigger_equal(l->min_cost, 0.0);
	
	// Create function Delta which will be dominated.
	PWLDominationFunction Delta = l->duration;
	double l_max_cost = max(img(l->duration))-l->p;
	
	for (auto& demand_entry : U[l->v])
	{
		if (epsilon_bigger(demand_entry.first, l->q)) break;
		for (auto& m: demand_entry.second)
		{
			// We know that q(m) <= q(l), v(m) = v(l).
			if (epsilon_bigger(m->min_cost, l_max_cost)) break;
			if (!is_subset(m->U, l->U)) continue;
			if (!partial && !Delta.IsAlwaysDominated(m->duration, l->p - m->p)) continue;
			else if (partial && !Delta.DominatePieces(m->duration, l->p - m->p)) continue;
			return true;
		}
	}
	l->duration = (PWLFunction)Delta;
	l->rw = l->duration.Domain();
	l->min_cost = min(img(l->duration)) - l->p;
	return false;
}

int MonodirectionalLabeling::CorrectionStep(Label* l) const
{
	return 0;
}

void MonodirectionalLabeling::ProcessStep(Label* l)
{
	insert_sorted(U[l->v].Insert(floor(l->q), {}), l, [] (Label* l1, Label* l2) { return l1->min_cost < l2->min_cost; });
}

vector<LazyLabel> MonodirectionalLabeling::EnumerationStep(Label* l) const
{
	vector<LazyLabel> E;
	if (l->v == vrp_.d) return E; // End depot has no extensions.
	for (Vertex v: vrp_.D.Successors(l->v))
	{
		if (l->U.test(v)) continue;
		if (epsilon_bigger(l->q + vrp_.q[v], vrp_.Q)) continue;
		if (epsilon_bigger(min(l->rw), max(dom(vrp_.arr[l->v][v])))) continue;
		double makespan = vrp_.arr[l->v][v](max(min(l->rw), min(dom(vrp_.arr[l->v][v]))));
		E.push_back({l, v, cross ? min(l->rw) : makespan});
	}
	return E;
}
} // networks2019