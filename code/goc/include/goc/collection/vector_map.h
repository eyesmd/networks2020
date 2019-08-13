//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#ifndef NETWORKS2019_VECTOR_MAP_H
#define NETWORKS2019_VECTOR_MAP_H

#include <iostream>
#include <vector>

#include "goc/print/printable.h"

namespace goc
{
template<typename K, typename V>
class VectorMap : public Printable
{
public:
	V& Insert(const K& key, const V& value)
	{
		// Try to see if the element already existed.
		for (auto& e: S)
		{
			if (e.first == key) return e.second;
			else if (e.first > key) break;
		}
		
		// Element does not exist, then insert.
		S.push_back(make_pair(key, value));
		int i = S.size()-1;
		while (i > 0 && S[i-1].first > S[i].first)
		{
			swap(S[i-1], S[i]);
			--i;
		}
		return S[i].second;
	}
	
	std::pair<K,V>& operator[](int index)
	{
		return S[index];
	}
	
	const std::pair<K,V>& operator[](int index) const
	{
		return S[index];
	}
	
	int Erase(int index)
	{
		S.erase(index);
		return std::max(0, index-1);
	}
	
	typename std::vector<std::pair<K, V>>::iterator begin()
	{
		return S.begin();
	}
	
	typename std::vector<std::pair<K, V>>::const_iterator begin() const
	{
		return S.begin();
	}
	
	typename std::vector<std::pair<K, V>>::iterator end()
	{
		return S.end();
	}
	
	typename std::vector<std::pair<K, V>>::const_iterator end() const
	{
		return S.end();
	}
	
	typename std::vector<std::pair<K, V>>::reverse_iterator rbegin()
	{
		return S.rbegin();
	}
	
	typename std::vector<std::pair<K, V>>::const_reverse_iterator rbegin() const
	{
		return S.rbegin();
	}
	
	typename std::vector<std::pair<K, V>>::reverse_iterator rend()
	{
		return S.rend();
	}
	
	typename std::vector<std::pair<K, V>>::const_reverse_iterator rend() const
	{
		return S.rend();
	}
	
	virtual void Print(std::ostream& os) const
	{
		os << S;
	}
	
private:
	std::vector<std::pair<K, V>> S;
};
} // namespace goc

#endif //NETWORKS2019_VECTOR_MAP_H
