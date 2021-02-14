
# ROADMAP

## Objetivo
Hacer el algoritmo de 'Quickest Path Algorithm'

## Pasos

1. Ver qué ya hay desarrollado de funciones lineales en el proyecto
 [ ] Ver 'Compose'
2. Hacer pseudo-código de los algoritmos relacionados
3. Plantear interfaz
4. Hacer test unitarios
5. Desarrollar

6. ¿Por qué la cantidad de piezas es a lo sumo n^theta(log n)?

# NOTES

LinearFunction::PreValue
 - Si el slope es 0, te devuelve el extremo derecho del dominio

PWLFunction
 - Mantiene su dominio e imagen (son ambos intervalos cerrados)

PWLFunction::AddPiece
 - Es O(1). Asume que la nueva pieza se agrega al final
 - Normaliza, AKA, mergea la pieza agregada con la última pieza de ser posible
 - (No mergea al final si la pieza agregada es un punto)
 - (No chequea que la nueva pieza se solape con alguna pieza actual, ni que efectivamente esté a la derecha del resto)

PWLFunction::PopPiece
 - Es O(n) porque llama a 'UpdateImage', que actualiza la imagen linealmente.

PWLFunction::Operators
 - They're all O(n + m)
 - Sum and Product intersect the domains
 - Max and Min unify the domains
 - The domain of the Composition f(g(x)) are the x in the domain of g for which g(x) is in the domain of f
