#pragma once

#include "vertex.hpp"
#include "edge.hpp"
#include "edge_list.hpp"

#include <iostream>
#include <exception>
#include <stdexcept>
#include <array>
#include <vector>
#include <queue>
#include <stack>
#include <unordered_set>
#include <functional>
#include <optional>
#include <algorithm>
#include <type_traits>

namespace gtor {
  template <typename T, size_t N>
  class Graph {
    private:
      std::array<std::array<size_t, N>, N> _adj_mat;
      std::array<Vertex<T>, N> _vertices;
      bool _directed;
    public:
      Graph(std::array<Vertex<T>, N>& vertices, std::array<std::array<size_t, N>, N>& adj_mat, bool directed=false):
      _adj_mat  {  adj_mat },
      _vertices { vertices },
      _directed { directed } { 
        if (!directed) {
          // Assert that the expected graph is undirected.
          for (size_t idx { 0 }; idx < _adj_mat.size(); idx++) {
            for(size_t jdx { idx + 1 }; jdx < _adj_mat.size(); jdx ++) {
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
      _directed { directed } { }

      ~Graph(void) { }

      size_t index_of_vtx(const Vertex<T>& vtx) const {
        auto it = std::find(_vertices.begin(), _vertices.end(), vtx);

        if (it != _vertices.end()) {
          return std::distance(_vertices.begin(), it);
        } else {
          return _vertices.size();
        }
      }
      std::array<Vertex<T>, N> vertices(void) const {
        return _vertices;
      } 

      std::vector<size_t> neighbors(size_t vert_idx) {
        std::vector<size_t> nbs { };

        if (vert_idx > _adj_mat.size()) {
          throw std::runtime_error { "Index specified exceeds graph width." };
        }

        for (size_t i { 0 }; i < _adj_mat.at(vert_idx).size(); i++) {
          if (_adj_mat.at(vert_idx).at(i) != 0) {
            nbs.emplace_back(i);
          }
        }

        return nbs;
      }

      size_t is_neighbor(const std::array<size_t, N>& v_e, const std::unordered_set<size_t>& visited) {
        for (size_t i { 0 }; i < N; i++) {
          if (v_e.at(i) != 0 && visited.find(i) == visited.end() ) {
            return i;
          }
        }

        return N;
      }

      void connect(size_t i, size_t j, size_t weight=1) {
        _adj_mat[i][j] = weight;

        if (!_directed) {
          _adj_mat[j][i] = weight;
        }
      }

      void disconnect(size_t i, size_t j) {
        _adj_mat[i][j] = 0;

        if (!_directed) {
          _adj_mat[j][i] = 0;
        }
      }

      template<typename U = std::nullptr_t>
      std::optional<Vertex<T>> bfs
      (
        std::unordered_set<size_t>& visited,
        size_t initial_node_idx=0, 
        U callback=nullptr
      ) {
        std::queue<size_t> bfs_queue {};
        bfs_queue.emplace(initial_node_idx);

        if constexpr (std::is_void_v<decltype(callback(_vertices.at(initial_node_idx)))>) {
          callback(_vertices.at(initial_node_idx));
        } else {
          auto output { callback(_vertices.at(initial_node_idx)) };
          if (output) {
            return _vertices.at(initial_node_idx);
          }
        }
        
        visited.insert(initial_node_idx);

        while (!bfs_queue.empty()) {
          size_t idx { bfs_queue.front() };
          bfs_queue.pop();

          std::vector<size_t> neigh { neighbors(idx) };

          for (size_t x: neigh) {
            if (visited.find(x) == visited.end()) {
              bfs_queue.emplace(x);

              // TODO - Do any processing you want here.
              if constexpr (std::is_void_v<decltype(callback(_vertices.at(x)))>) {
                callback(_vertices.at(x));
              } else {
                auto output { callback(_vertices.at(x)) };
                if (output) {
                  return _vertices.at(x);
                }
              }
              
              visited.insert(x);
            }
          }
        }

        return std::nullopt;
      }

      template<typename U = std::nullptr_t>
      std::optional<Vertex<T>> bfs_all(U callback=nullptr) {
        std::unordered_set<size_t> visited { };
        size_t initial_node { 0 };
        std::optional<Vertex<T>> last { std::nullopt };

        for (size_t i { initial_node }; i < _vertices.size(); i++) {
          if (visited.find(i) == visited.end()) {
            last = bfs(visited, i, callback);
            if (last) {
              return last;
            }
          }
        }

        return last;
      } 

      template<typename U = std::nullptr_t>
      std::optional<Vertex<T>> bfs(size_t initial_node_idx=0, U callback=nullptr) {
        std::unordered_set<size_t> visited {};
        return bfs(visited, 0, callback);
      }

      std::optional<Vertex<T>> bfs(std::unordered_set<size_t>& visited, size_t initial_node_idx=0) {
        std::queue<size_t> bfs_queue {};
        bfs_queue.emplace(initial_node_idx);
        
        visited.insert(initial_node_idx);

        while (!bfs_queue.empty()) {
          size_t idx { bfs_queue.front() };
          bfs_queue.pop();

          std::vector<size_t> neigh { neighbors(idx) };

          for (size_t x: neigh) {
            if (visited.find(x) == visited.end()) {
              bfs_queue.emplace(x);              
              visited.insert(x);
            }
          }
        }

        return std::nullopt;
      }

      std::optional<Vertex<T>> bfs_all(void) {
        std::unordered_set<size_t> visited { };

        size_t initial_node { 0 };
        std::optional<Vertex<T>> last { std::nullopt };

        for (size_t i { initial_node }; i < _vertices.size(); i++) {
          if (visited.find(i) != visited.end()) {
            last = bfs(visited, i);
            
            if (last) {
              return last;
            }
          }
        }

        return last;
      } 

      std::optional<Vertex<T>> bfs(size_t initial_node_idx=0) {
        std::unordered_set<size_t> visited { };
        return bfs(visited, initial_node_idx);
      }

      void shortest_path_dijkstra(std::array<float, N>& distances, Vertex<T>& vtx) {
        std::priority_queue<size_t> spd_queue { };
        size_t initial_idx { index_of_vtx(vtx) };

        spd_queue.emplace(initial_idx);
        distances[initial_idx] = 0;

        while (!spd_queue.empty()) {
          size_t idx { spd_queue.top() };
          spd_queue.pop();

          std::vector<size_t> neigh { neighbors(idx) };

          for (size_t x: neigh) {
            float dist { distances.at(idx) + _adj_mat.at(idx).at(x) };
            if (distances.at(x) > dist) {
              spd_queue.emplace(x);
              distances[x] = dist;
            }
          }
        }
      }

      template<typename U = std::nullptr_t, typename V = std::nullptr_t>
      std::optional<Vertex<T>> dfs
      (
        std::unordered_set<size_t>& visited,
        size_t initial_node_idx=0, 
        U pre_callback=nullptr, 
        V post_callback=nullptr
      ) {
        

        std::stack<size_t> dfs_stack {};
        dfs_stack.emplace(initial_node_idx);
        visited.insert(initial_node_idx);

        if constexpr (std::is_void_v<decltype(pre_callback(_vertices.at(initial_node_idx)))>) {
          pre_callback(_vertices.at(initial_node_idx));
        } else {
          auto output { pre_callback(_vertices.at(initial_node_idx)) };
          if (output) {
            return _vertices.at(initial_node_idx);
          }
        }

        Vertex<T> last { _vertices.at(initial_node_idx) };

        while (!dfs_stack.empty()) {
          size_t idx { dfs_stack.top() };
          size_t next { is_neighbor(_adj_mat.at(idx), visited) };

          if (next != N) {
            visited.insert(next);
            dfs_stack.emplace(next);

            // TODO - Do any preprocessing you want here.
            if constexpr (std::is_void_v<decltype(pre_callback(_vertices.at(next)))>) {
              pre_callback(_vertices.at(next));
            } else {
              auto output { pre_callback(_vertices.at(next)) };
              if (output) {
                return _vertices.at(next);
              }
            }
          } else {
            size_t l { dfs_stack.top() };
            dfs_stack.pop();
            
            // TODO - Do any postprocessing you want here.
            if constexpr (std::is_void_v<decltype(post_callback(_vertices.at(l)))>) {
              post_callback(_vertices.at(l));
            } else {
              auto output { post_callback(_vertices.at(l)) };
              if (output) {
                return _vertices.at(l);
              }
            }
          }
        }

        return std::nullopt;
      }

      template<typename U = std::nullptr_t, typename V = std::nullptr_t>
      std::optional<Vertex<T>> dfs_all(U pre_callback=nullptr, V post_callback=nullptr) {
        size_t initial_node { 0 };
        std::optional<Vertex<T>> last { std::nullopt };
        std::unordered_set<size_t> visited {};

        for (size_t i { initial_node }; i < _vertices.size(); i++) {
          if (visited.find(i) == visited.end()) {
            last = dfs(visited, i, pre_callback, post_callback);
            if (last) {
              return last;
            }
          }
        }

        return last;
      }

      template<typename U = std::nullptr_t, typename V = std::nullptr_t>
      std::optional<Vertex<T>> dfs(size_t initial_node_idx=0, U pre_callback=nullptr, V post_callback=nullptr) {
        std::unordered_set<size_t> visited {};
        return dfs(visited, 0, pre_callback, post_callback);
      } 

      std::optional<Vertex<T>> dfs(std::unordered_set<size_t>& visited, size_t initial_node_idx=0) {
        std::stack<size_t> dfs_stack {};
        dfs_stack.emplace(initial_node_idx);
        visited.insert(initial_node_idx);

        Vertex<T> last { _vertices.at(initial_node_idx) };

        while (!dfs_stack.empty()) {
          size_t idx { dfs_stack.top() };
          size_t next { is_neighbor(_adj_mat.at(idx), visited) };

          if (next != N) {
            visited.insert(next);
            dfs_stack.emplace(next);
          } else {
            size_t l { dfs_stack.top() };
            dfs_stack.pop();
            
            last = _vertices.at(l);
          }
        }

        return last;
      }

      std::optional<Vertex<T>> dfs_all(void) {
        size_t initial_node { _vertices.at(0).order() };
        std::optional<Vertex<T>> last { std::nullopt };
        std::unordered_set<size_t> visited { };
        for (size_t i { initial_node }; i < _vertices.size(); i++) {
          if (visited.find(i) == visited.end()) {
            last = dfs(visited, i);
            
            if (last) {
              return last;
            }
          }
        }

        return last;
      }

      std::optional<Vertex<T>> dfs(size_t initial_node_idx=0) {
        std::unordered_set<size_t> visited {};
        return dfs(visited, 0);
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