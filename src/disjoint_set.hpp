#pragma once

#include "vertex.hpp"

#include <iostream>
#include <exception>
#include <stdexcept>
#include <array>
#include <vector>
#include <queue>
#include <stack>
#include <algorithm>

namespace gtor 
{
  template <typename T, int N>
  class DisjointSet
  {
    private:
      std::array<Vertex<T>, N> _vertices;
      std::array<int, N> _parent_idxs;
      std::array<int, N> _ranks;
    public:
      DisjointSet(std::array<Vertex<T>, N>& vertices):
        _vertices { vertices },
        _parent_idxs { std::iota(std::begin(_parent_idxs), std::end(_parent_idxs), 0) },
        _ranks { }
      {
        _ranks.fill(1);
      }

      int find_set(int idx_u) {
        if (_parent_idxs[idx_u] == idx_u) {
          return idx_u;
        }

        return _parent_idxs[idx_u] = find_set(_parent_idxs); 
      }

      void union_set(int idx_u, int idx_v) {
        idx_u = find_set(idx_u);
        idx_v = find_set(idx_v);

        if (idx_u != idx_v) {
          if (_ranks[idx_u] < _ranks[idx_v]) {
            std::swap(idx_u, idx_v);
          }
          
          _parent_idxs[idx_v] = idx_u;

          if (_ranks[idx_u] == _ranks[idx_v]) {
            _ranks[idx_u] ++;
          }
        }
      }
  };
}