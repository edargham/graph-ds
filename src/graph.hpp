#pragma once

#include "vertex.hpp"
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

      size_t is_neighbor(const std::array<size_t, N>& v_e) {
        for (size_t i { 0 }; i < N; i++) {
          if (v_e.at(i) != 0 && _vertices.at(i).mark() == 0 ) {
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
      std::optional<Vertex<T>> bfs_all(U callback=nullptr) {
        std::optional<std::reference_wrapper<std::unordered_set<size_t>>> visited { };
        // std::unordered_set<size_t> visited { };
        size_t initial_node { 0 };
        std::optional<Vertex<T>> last { std::nullopt };

        for (size_t i { initial_node }; i < _vertices.size(); i++) {
          if (visited->get().find(i) == visited->get().end()) {
            last = bfs(i, callback, true, visited);
            
            if (last) {
              return last;
            }
          }
        }

        return last;
      } 

      template<typename U = std::nullptr_t>
      std::optional<Vertex<T>> bfs
      (
        size_t initial_node_idx=0, 
        U callback=nullptr, 
        bool searching_all=false,
        std::optional<std::reference_wrapper<std::unordered_set<size_t>>> visited=std::nullopt
      ) {
        std::cout << "here.";
        if (!searching_all || !visited.has_value()) {
          visited = std::optional<std::reference_wrapper<std::unordered_set<size_t>>> {};
        }

        std::queue<size_t> bfs_queue {};
        bfs_queue.emplace(initial_node_idx);

        if constexpr (std::is_void_v<decltype(callback(_vertices.at(initial_node_idx)))>) {
          std::cout << "sda";
          callback(_vertices.at(initial_node_idx));
        } else {
          auto output { callback(_vertices.at(initial_node_idx)) };
          if (output) {
            return _vertices.at(initial_node_idx);
          }
        }
        
        visited->get().insert(initial_node_idx);

        while (!bfs_queue.empty()) {
          size_t idx { bfs_queue.front() };
          bfs_queue.pop();

          std::vector<size_t> neigh { neighbors(idx) };

          for (size_t x: neigh) {
            if (visited->get().find(x) != visited->get().end()) {
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
              
              visited->get().insert(x);
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
            last = bfs(i, true);
            
            if (last) {
              return last;
            }
          }
        }

        return last;
      } 

      std::optional<Vertex<T>> bfs
      (
        size_t initial_node_idx=0, 
        bool searching_all=false,
        std::optional<std::reference_wrapper<std::unordered_set<size_t>>> visited=std::nullopt
      ) {
        if (!searching_all || !visited.has_value()) {
          visited = std::optional<std::reference_wrapper<std::unordered_set<size_t>>> {};
        }

        std::queue<size_t> bfs_queue {};
        bfs_queue.emplace(initial_node_idx);
        
        visited->get().insert(initial_node_idx);

        while (!bfs_queue.empty()) {
          size_t idx { bfs_queue.front() };
          bfs_queue.pop();

          std::vector<size_t> neigh { neighbors(idx) };

          for (size_t x: neigh) {
            if (visited->get().find(x) != visited->get().end()) {
              bfs_queue.emplace(x);              
              visited->get().insert(x);
            }
          }
        }

        return std::nullopt;
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
      std::optional<Vertex<T>> dfs_all(U pre_callback=nullptr, V post_callback=nullptr) {
        for (Vertex<T>& vtx: _vertices) {
          vtx.set_mark(0);
        }

        size_t initial_node { _vertices.at(0).order() };
        std::optional<Vertex<T>> last { std::nullopt };

        for (size_t i { initial_node }; i < _vertices.size(); i++) {
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
      std::optional<Vertex<T>> dfs
      (
        size_t initial_node_idx=0, 
        U pre_callback=nullptr, 
        V post_callback=nullptr, 
        bool searching_all=false
      ) {
        if (!searching_all) {
          for (Vertex<T>& vtx: _vertices) {
            vtx.set_mark(0);
          }
        }

        std::stack<size_t> dfs_stack {};
        dfs_stack.emplace(initial_node_idx);
        _vertices[initial_node_idx].set_mark(1);

        if constexpr (std::is_void_v<decltype(pre_callback(_vertices.at(initial_node_idx)))>) {
          pre_callback(_vertices.at(initial_node_idx));
        } else {
          auto output { pre_callback(_vertices.at(initial_node_idx)) };
          if (output) {
            _vertices[initial_node_idx].set_mark(1);
            return _vertices.at(initial_node_idx);
          }
        }

        Vertex<T> last { _vertices.at(initial_node_idx) };

        while (!dfs_stack.empty()) {
          size_t idx { dfs_stack.top() };
          size_t next { is_neighbor(_adj_mat.at(idx)) };

          if (next != N) {
            _vertices[next].set_mark(1);
            dfs_stack.emplace(next);

            // TODO - Do any preprocessing you want here.
            if constexpr (std::is_void_v<decltype(pre_callback(_vertices.at(next)))>) {
              pre_callback(_vertices.at(next));
            } else {
              auto output { pre_callback(_vertices.at(next)) };
              if (output) {
                _vertices[next].set_mark(1);
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
                _vertices[l].set_mark(1);
                return _vertices.at(l);
              }
            }
            
            last = _vertices.at(l);
          }
        }

        return last;
      }
      
      std::optional<Vertex<T>> dfs_all(void) {
        for (Vertex<T>& vtx: _vertices) {
          vtx.set_mark(0);
        }

        size_t initial_node { _vertices.at(0).order() };
        std::optional<Vertex<T>> last { std::nullopt };

        for (size_t i { initial_node }; i < _vertices.size(); i++) {
          if (_vertices.at(i).mark() == 0) {
            last = dfs(i, true);
            
            if (last) {
              return last;
            }
          }
        }

        return last;
      } 

      std::optional<Vertex<T>> dfs(size_t initial_node_idx=0, bool searching_all=false) {
        if (!searching_all) {
          for (Vertex<T>& vtx: _vertices) {
            vtx.set_mark(0);
          }
        }

        std::stack<size_t> dfs_stack {};
        dfs_stack.emplace(initial_node_idx);
        _vertices[initial_node_idx].set_mark(1);

        Vertex<T> last { _vertices.at(initial_node_idx) };

        while (!dfs_stack.empty()) {
          size_t idx { dfs_stack.top() };
          size_t next { is_neighbor(_adj_mat.at(idx)) };

          if (next != N) {
            _vertices[next].set_mark(1);
            dfs_stack.emplace(next);
          } else {
            size_t l { dfs_stack.top() };
            dfs_stack.pop();
            
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