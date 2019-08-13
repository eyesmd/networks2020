//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#ifndef NETWORKS2019_PREPROCESS_TIME_WINDOWS_H
#define NETWORKS2019_PREPROCESS_TIME_WINDOWS_H

#include <goc/goc.h>

namespace networks2019
{
// Takes a JSON instance of the vehicle routing problems with the following attributes:
//	- digraph
//	- travel_times
//	- time_windows
//	- start_depot
//	- end_depot
// Assumes preprocess_service_waiting was called (i.e. instance has no service nor waiting times).
// Shrinks time windows [a_i, b_i] according to the techinques introduced in:
//	Desrosiers, J., Dumas, Y., Solomon, M. M., & Soumis, F. (1995).
// and removes infeasible arcs.
// Only applies preprocessing techniques that do not require that all vertices all visited in one route.
void preprocess_time_windows(nlohmann::json& instance);
} // namespace networks2019

#endif //NETWORKS2019_PREPROCESS_TIME_WINDOWS_H
