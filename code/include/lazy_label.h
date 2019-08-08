//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#ifndef NETWORKS2019_LAZY_LABEL_H
#define NETWORKS2019_LAZY_LABEL_H

#include "label.h"
#include "vrp_instance.h"

namespace networks2019
{
class LazyLabel
{
public:
	Label* parent; // label to extend.
	goc::Vertex v; // extending to vertex v.
	TimeUnit makespan; // earliest arrival time to v, for queuing purposes.
	
	LazyLabel();
	
	LazyLabel(Label* parent, goc::Vertex v, TimeUnit makespan);
};
} // namespace networks2019

#endif //NETWORKS2019_LAZY_LABEL_H
