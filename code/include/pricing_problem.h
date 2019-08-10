//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#ifndef NETWORKS2019_PRICING_PROBLEM_H
#define NETWORKS2019_PRICING_PROBLEM_H

#include <vector>
#include <iostream>

#include "goc/goc.h"

namespace networks2019
{
class PricingProblem : public goc::Printable
{
public:
	std::vector<goc::Arc> A; // Forbidden arcs.
	std::vector<double> P; // Profits of the vertices.
	
	virtual void Print(std::ostream& os) const;
};

void to_json(nlohmann::json& j, const PricingProblem& p);

} // namespace networks2019

#endif //NETWORKS2019_PRICING_PROBLEM_H
