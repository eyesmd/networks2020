//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#ifndef NETWORKS2019_MONODIRECTIONAL_LABELING_H
#define NETWORKS2019_MONODIRECTIONAL_LABELING_H

#include <vector>
#include <tuple>

#include "goc/goc.h"

#include "vrp_instance.h"
#include "label.h"
#include "lazy_label.h"
#include "pricing_problem.h"

namespace networks2019
{

// LBQueue is the queue to use in the labeling algorithm.
struct LabelExtensionComparator
{
	bool operator() (const LazyLabel& l1, const LazyLabel& l2) { return std::make_tuple(l1.makespan, l1.parent->length+1, l1.parent->q) > std::make_tuple(l2.makespan, l2.parent->length+1, l2.parent->q); }
};
typedef std::priority_queue<LazyLabel, std::vector<LazyLabel>, LabelExtensionComparator> LBQueue;

class MonodirectionalLabeling
{
public:
	int process_limit; // Maximum number of labels to process for each run.
	goc::Duration time_limit; // Maximum execution time.
	TimeUnit t_m; // Only extend labels that have min(rw(.)) <= t_m.
	bool cross; // Indicates if labels are allowed to cross t_m only one step.
	bool partial; // Indicates if partial domination should be used.
	
	// Dominance structure.
	typedef std::vector<Label*> BoundLevel;
	typedef goc::VectorMap<CapacityUnit, BoundLevel> DemandLevel;
	typedef std::vector<DemandLevel> DominanceStructure;
	DominanceStructure U; // Indexed by last vertex, demand and sorted by c_min.
	int processed_count; // Number of labels in the dominance structure.
	
	MonodirectionalLabeling(const VRPInstance& vrp);
	
	~MonodirectionalLabeling();
	
	// Sets the problem to use for the labeling algorithm.
	void SetProblem(const PricingProblem& pricing_problem);
	
	// Runs the labeling algorithm using the labels in the queue q, and outputs the execution information on log.
	// Returns: a vector of the labels that were not dominated (processed) during the proccess and time limits.
	std::vector<Label*> Run(LBQueue* q, goc::MLBExecutionLog* log);
	
	// Returns: a lazy label with the initial vertex only (start depot).
	LazyLabel Init() const;
	
private:
	// The extension step consists in taking a lazy label and building the complete label.
	// Returns: the extended full label (notice that it may be infeasible, so it may be nullptr).
	Label* ExtensionStep(const LazyLabel& ll) const;
	
	// The domination step checks if l is dominated by any other processed label in U. And if partial domination
	// is active, then it removes the dominated parts.
	// Returns: if label l has all pieces dominated.
	bool DominationStep(Label* l) const;
	
	// The correction step consists in removing all dominated parts of labels m in U by label l.
	// Returns: the number of labels that were fully dominated.
	int CorrectionStep(Label* l) const;
	
	// The process step is where label l gets added to the domination structure U.
	void ProcessStep(Label* l);
	
	// The enumeration step is where label l is attempted to be extended to all its successors and the feasible
	// extensions are returned.
	// Returns: the feasible extensions
	std::vector<LazyLabel> EnumerationStep(Label* l) const;
	
	// Resets the dominance structures and counters.
	void Clean();
	
	VRPInstance vrp_;
	PricingProblem pp_;
	Label no_label; // null object pattern of the label to avoid using ifs.
};
} // namespace networks2019

#endif //NETWORKS2019_MONODIRECTIONAL_LABELING_H
