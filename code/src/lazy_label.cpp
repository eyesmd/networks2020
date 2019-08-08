//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "lazy_label.h"

namespace networks2019
{
LazyLabel::LazyLabel() : parent(nullptr), v(-1), makespan(-1)
{}

LazyLabel::LazyLabel(Label* parent, goc::Vertex v, TimeUnit makespan) : parent(parent), v(v), makespan(makespan)
{}
} // namespace networks2019