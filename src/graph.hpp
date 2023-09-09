#pragma once

#include "vertex.hpp"

#include <iostream>
#include <exception>
#include <stdexcept>
#include <array>
#include <vector>
#include <queue>
#include <stack>
#include <functional>
#include <optional>
// #include <cstdarg>
#include <type_traits>

namespace gtor {
  template <typename T, int N>
  class Graph {
    private:
    std::array<std::array<int, N>, N> _adj_mat;
    std::array<Vertex<T>, N> _vertices;
    bool _directed;
    public:
    Graph(std::array<Vertex<T>, N>& vertices, std::array<std::array<int, N>, N>& adj_mat, bool directed=false):
    _adj_mat  {  adj_mat },
    _vertices { vertices },
    _directed { directed } { 
      if (!directed) {
        // Assert that the expected graph is undirected.
        for (int idx { 0 }; idx < _adj_mat.size(); idx++) {
          for(int jdx { idx + 1 }; jdx < _adj_mat.size(); jdx ++) {
            if (_adj_mat.at(idx).at(jdx) != _adj_mat.at(jdx).at(idx)) {
              throw std::runtime_error ( "Expected undirected graph, got directed graph." );
            }
          }
        } 
      }
    }

    Graph(std::array<Vertex<T>, N>& vertices, bool directed=false):
    _adj_mat           { },
    _vertices { vertices },
    _directed { directed } {

    }

    ~Graph(void) {

    }
    
    std::vector<int> neighbors(int vert_idx) {
      std::vector<int> nbs { };

      if (vert_idx > _adj_mat.size()) {
        throw std::runtime_error { "Index specified exceeds graph width." };
      }

      for (int i { 0 }; i < _adj_mat.at(vert_idx).size(); i++) {
        if (_adj_mat.at(vert_idx).at(i) == 1) {
          nbs.emplace_back(i);
        }
      }

      return nbs;
    }

    int is_neighbor(const std::array<int, N>& v_e) {
      for (int i { 0 }; i < N; i++) {
        if (v_e.at(i) != 0 && _vertices.at(i).mark() == 0 ) {
          return i;
        }
      }

      return -1;
    }

    Vertex<T> bfs_all(void) {
      for (Vertex<T>& vtx: _vertices) {
        vtx.set_mark(0);
      }

      int initial_node { 0 };

      Vertex<T> last { };

      for (int i { initial_node }; i < _vertices.size(); i++) {
        if (_vertices.at(i).mark() == 0) {
          last = bfs(initial_node);
        }
      }

      return last;
    }

    template<typename U = std::nullptr_t>
    Vertex<T> bfs_all(U callback=nullptr) {
      for (Vertex<T>& vtx: _vertices) {
        vtx.set_mark(0);
      }

      int initial_node { 0 };
      Vertex<T> last { };

      for (int i { initial_node }; i < _vertices.size(); i++) {
        if (_vertices.at(i).mark() == 0) {
          last = bfs(initial_node, callback);
        }
      }

      return last;
    } 

    Vertex<T> bfs(void) {
      int initial_node { 0 };

      for (Vertex<T>& v: _vertices) {
        v.set_mark(0);
      }

      std::queue<int> bfs_queue {};
      bfs_queue.emplace(initial_node);
      _vertices[initial_node].set_mark(1);

      Vertex<T> last { _vertices.at(initial_node) };

      while (!bfs_queue.empty()) {
        int idx { bfs_queue.front() };
        bfs_queue.pop();

        std::vector<int> neighs { neighbors(idx) };

        for (int x: neighs) {
          if (_vertices.at(x).mark() == 0) {
            bfs_queue.emplace(x);
            _vertices[x].set_mark(1);
            last = _vertices.at(x);
          }
        }
      }

      return last;
    }

    template<typename U = std::nullptr_t>
    Vertex<T> bfs(U callback=nullptr) {
      int initial_node { 0 };

      for (Vertex<T>& v: _vertices) {
        v.set_mark(0);
      }

      std::queue<int> bfs_queue {};
      bfs_queue.emplace(initial_node);

      if constexpr (std::is_void_v<decltype(callback(_vertices.at(0)))>) {
        callback(_vertices.at(initial_node));
      } else {
        auto z { callback(_vertices.at(initial_node)) };
        if (z) {
          return _vertices.at(initial_node);
        }
      }

      _vertices[initial_node].set_mark(1);

      Vertex<T> last { _vertices.at(initial_node) };

      while (!bfs_queue.empty()) {
        int idx { bfs_queue.front() };
        bfs_queue.pop();

        std::vector<int> neighs { neighbors(idx) };

        for (int x: neighs) {
          if (_vertices.at(x).mark() == 0) {
            bfs_queue.emplace(x);

            // TODO - Do any processing you want here.
            if constexpr (std::is_void_v<decltype(callback(_vertices.at(0)))>) {
              callback(_vertices.at(x));
            } else {
              auto output { callback(_vertices.at(x)) };
              if (output) {
                return _vertices.at(x);
              }
            }
            
            _vertices[x].set_mark(1);
            last = _vertices.at(x);
          }
        }
      }

      return last;
    }

    Vertex<T> bfs(int initial_node_idx=0) {
      std::queue<int> bfs_queue {};
      bfs_queue.emplace(initial_node_idx);

      _vertices[initial_node_idx].set_mark(1);

      Vertex<T> last { _vertices.at(initial_node_idx) };

      while (!bfs_queue.empty()) {
        int idx { bfs_queue.front() };
        bfs_queue.pop();

        std::vector<int> neigh { neighbors(idx) };

        for (int x: neigh) {
          if (_vertices.at(x).mark() == 0) {
            bfs_queue.emplace(x);
            _vertices[x].set_mark(1);
            last = _vertices.at(x);
          }
        }
      }

      return last;
    }

    template<typename U = std::nullptr_t>
    Vertex<T> bfs(int initial_node_idx=0, U callback=nullptr) {
      std::queue<int> bfs_queue {};
      bfs_queue.emplace(initial_node_idx);

      _vertices[initial_node_idx].set_mark(1);

      Vertex<T> last { _vertices.at(initial_node_idx) };

      while (!bfs_queue.empty()) {
        int idx { bfs_queue.front() };
        bfs_queue.pop();

        std::vector<int> neigh { neighbors(idx) };

        for (int x: neigh) {
          if (_vertices.at(x).mark() == 0) {
            bfs_queue.emplace(x);

            // TODO - Do any processing you want here.
            if constexpr (std::is_void_v<decltype(callback(_vertices.at(0)))>) {
              callback(_vertices.at(x));
            } else {
              auto output { callback(_vertices.at(x)) };
              if (output) {
                return _vertices.at(x);
              }
            }

            _vertices[x].set_mark(1);
            last = _vertices.at(x);
          }
        }
      }

      return last;
    }

    // Vertex<T> dfs(std::function<bool(T, std::optional<T>)> callback, const std::optional<T>& datum) {
    //   int initial_node { 0 };

    //   for (Vertex<T>& v: _vertices) {
    //     v.set_mark(0);
    //   }

    //   std::stack<int> dfs_stack {};
    //   dfs_stack.emplace(initial_node);
    //   _vertices[initial_node].set_mark(1);

    //   Vertex<T> last { _vertices.at(initial_node) };

    //   while (!dfs_stack.empty()) {
    //     int idx { dfs_stack.top() };
    //     int next { is_neighbor(_adj_mat.at(idx)) };

    //     if (next != -1) {
    //         _vertices[next].set_mark(1);
    //         dfs_stack.emplace(next);

    //         if (callback && datum) {
    //           if (callback(_vertices.at(next).datum(), datum)) {
    //             return _vertices.at(next);
    //           }
    //         }
    //         // TODO - Do any processing you want here.
    //     } else {
    //       int l { dfs_stack.top() };
    //       dfs_stack.pop();
    //       // TODO - Do any postprocessing you want here.
    //       last = _vertices.at(l);
    //     }
    //   }

    //   return last;
    // }

  };
}