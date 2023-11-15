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
#include <unordered_map>
#include <algorithm>
#include <type_traits>

namespace gtor {
  struct EdgeList
  {
    int idx_u;
    int idx_v;
    int weight;

    bool operator >(const EdgeList& el) const {
      return weight > el.weight;
    }

    bool operator >=(const EdgeList& el) const {
      return weight >= el.weight;
    }
    
    bool operator <(const EdgeList& el) const {
      return weight < el.weight;
    }

    bool operator <=(const EdgeList& el) const {
      return weight <= el.weight;
    }

    bool operator ==(const EdgeList& el) const {
      return weight == el.weight;
    }

    bool operator !=(const EdgeList& el) const {
      return weight != el.weight;
    }
  };

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

          for (int i { 0 }; i < N; i++) {
            _vertices[i].set_order(i);
          }
        }
      }

      Graph(std::array<Vertex<T>, N>& vertices, bool directed=false):
      _adj_mat           { },
      _vertices { vertices },
      _directed { directed } { }

      ~Graph(void) { }

      std::array<Vertex<T>, N> vertices(void) const {
        return _vertices;
      } 

      std::vector<int> neighbors(int vert_idx) {
        std::vector<int> nbs { };

        if (vert_idx > _adj_mat.size()) {
          throw std::runtime_error { "Index specified exceeds graph width." };
        }

        for (int i { 0 }; i < _adj_mat.at(vert_idx).size(); i++) {
          if (_adj_mat.at(vert_idx).at(i) != 0) {
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

      void connect(int i, int j, int weight=1) {
        _adj_mat[i][j] = weight;

        if (!_directed) {
          _adj_mat[j][i] = weight;
        }
      }

      void disconnect(int i, int j) {
        _adj_mat[i][j] = 0;

        if (!_directed) {
          _adj_mat[j][i] = 0;
        }
      }

      template<typename U = std::nullptr_t>
      std::optional<Vertex<T>> bfs_all(U callback=nullptr) {
        for (Vertex<T>& vtx: _vertices) {
          vtx.set_mark(0);
        }

        int initial_node { _vertices.at(0).order() };
        std::optional<Vertex<T>> last { std::nullopt };

        for (int i { initial_node }; i < _vertices.size(); i++) {
          if (_vertices.at(i).mark() == 0) {
            last = bfs(i, callback, true);
            
            if (last) {
              return last;
            }
          }
        }

        return last;
      } 

      template<typename U = std::nullptr_t>
      std::optional<Vertex<T>> bfs(int initial_node_idx=0, U callback=nullptr, bool searching_all=false) {
        if (!searching_all) {
          for (Vertex<T>& vtx: _vertices) {
            vtx.set_mark(0);
          }
        }

        std::queue<int> bfs_queue {};
        bfs_queue.emplace(initial_node_idx);

        if constexpr (callback != nullptr) {
          if constexpr (std::is_void_v<decltype(callback(_vertices.at(initial_node_idx)))>) {
            callback(_vertices.at(initial_node_idx));
          } else {
            auto output { callback(_vertices.at(initial_node_idx)) };
            if (output) {
              _vertices[initial_node_idx].set_mark(1);
              return _vertices.at(initial_node_idx);
            }
          }
        }
        _vertices[initial_node_idx].set_mark(1);

        while (!bfs_queue.empty()) {
          int idx { bfs_queue.front() };
          bfs_queue.pop();

          std::vector<int> neigh { neighbors(idx) };

          for (int x: neigh) {
            if (_vertices.at(x).mark() == 0) {
              bfs_queue.emplace(x);

              // TODO - Do any processing you want here.
              if constexpr (callback != nullptr) {
                if constexpr (std::is_void_v<decltype(callback(_vertices.at(x)))>) {
                  callback(_vertices.at(x));
                } else {
                  auto output { callback(_vertices.at(x)) };
                  if (output) {
                    _vertices[x].set_mark(1);
                    return _vertices.at(x);
                  }
                }
              }
              
              _vertices[x].set_mark(1);
            }
          }
        }

        return std::nullopt;
      }

      void shortest_path_dijkstra(std::array<float, N>& distances, Vertex<T>& vtx) {
        std::priority_queue<int> spd_queue { };
        spd_queue.emplace( vtx.order() );

        distances[vtx.order()] = 0;

        while (!spd_queue.empty()) {
          int idx { spd_queue.top() };
          spd_queue.pop();

          std::vector<int> neigh { neighbors(idx) };

          for (int x: neigh) {
            float dist { distances.at(idx) + _adj_mat.at(idx).at(x) };
            if (distances.at(x) > dist) {
              spd_queue.emplace(x);
              distances[x] = dist;
            }
          }
        }
      }

      template<typename U = std::nullptr_t, typename V = std::nullptr_t>
      std::optional<Vertex<T>> dfs_all(U pre_callback=nullptr, V post_callback=nullptr) {
        for (Vertex<T>& vtx: _vertices) {
          vtx.set_mark(0);
        }

        int initial_node { _vertices.at(0).order() };
        std::optional<Vertex<T>> last { std::nullopt };

        for (int i { initial_node }; i < _vertices.size(); i++) {
          if (_vertices.at(i).mark() == 0) {
            last = dfs(i, pre_callback, post_callback, true);
            
            if (last) {
              return last;
            }
          }
        }

        return last;
      } 

      template<typename U = std::nullptr_t, typename V = std::nullptr_t>
      std::optional<Vertex<T>> dfs(int initial_node_idx=0, U pre_callback=nullptr, V post_callback=nullptr, bool searching_all=false) {
        std::cout<< "dfs\n";
        if (!searching_all) {
          for (Vertex<T>& vtx: _vertices) {
            vtx.set_mark(0);
          }
        }

        std::stack<int> dfs_stack {};
        dfs_stack.emplace(initial_node_idx);
        _vertices[initial_node_idx].set_mark(1);

        if constexpr (pre_callback != nullptr) {
          if constexpr (std::is_void_v<decltype(pre_callback(_vertices.at(initial_node_idx)))>) {
            pre_callback(_vertices.at(initial_node_idx));
          } else {
            auto output { pre_callback(_vertices.at(initial_node_idx)) };
            if (output) {
              _vertices[initial_node_idx].set_mark(1);
              return _vertices.at(initial_node_idx);
            }
          }
        }

        Vertex<T> last { _vertices.at(initial_node_idx) };

        while (!dfs_stack.empty()) {
          int idx { dfs_stack.top() };
          int next { is_neighbor(_adj_mat.at(idx)) };

          if (next != -1) {
            _vertices[next].set_mark(1);
            dfs_stack.emplace(next);

            // TODO - Do any preprocessing you want here.
            if constexpr (pre_callback != nullptr) {
              if constexpr (std::is_void_v<decltype(pre_callback(_vertices.at(next)))>) {
                pre_callback(_vertices.at(next));
              } else {
                auto output { pre_callback(_vertices.at(next)) };
                if (output) {
                  _vertices[next].set_mark(1);
                  return _vertices.at(next);
                }
              }
            }
          } else {
            int l { dfs_stack.top() };
            dfs_stack.pop();
            
            // TODO - Do any postprocessing you want here.
            if constexpr (post_callback != nullptr) {
              if constexpr (std::is_void_v<decltype(post_callback(_vertices.at(l)))>) {
                post_callback(_vertices.at(l));
              } else {
                auto output { post_callback(_vertices.at(l)) };
                if (output) {
                  _vertices[l].set_mark(1);
                  return _vertices.at(l);
                }
              }
            }

            last = _vertices.at(l);
          }
        }

        return last;
      }

      std::vector<EdgeList> to_edge_list(void) {
        std::vector<EdgeList> edgeList { };
        
        for (size_t i { 0 }; i < N; i++) {
          for (size_t j { i+1 }; j < N; j++) {
            if (_adj_mat.at(i)(j)) {
              edgeList.emplace_back(EdgeList { i, j, _adj_mat.at(i)(j) });
            }
          }
        }

        return edgeList;
      }
  };
}