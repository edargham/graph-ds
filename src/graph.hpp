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
    
  struct MstList {
    std::vector<EdgeList> mst;
    size_t weight;
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
    _directed { directed } {

    }

    ~Graph(void) {

    }

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

    Vertex<T> bfs_all(void) {
      for (Vertex<T>& vtx: _vertices) {
        vtx.set_mark(0);
      }

      int initial_node { _vertices.at(0).order() };
      Vertex<T> last { };

      for (int i { initial_node }; i < _vertices.size(); i++) {
        if (_vertices.at(i).mark() == 0) {
          last = bfs(i);
        }
      }

      return last;
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
          last = bfs(i, callback);
          
          if (last) {
            return last;
          }
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
    std::optional<Vertex<T>> bfs(U callback=nullptr) {
      int initial_node { 0 };

      for (Vertex<T>& v: _vertices) {
        v.set_mark(0);
      }

      std::queue<int> bfs_queue {};
      bfs_queue.emplace(initial_node);

      if constexpr (std::is_void_v<decltype(callback(_vertices.at(0)))>) {
        callback(_vertices.at(initial_node));
      } else {
        auto output { callback(_vertices.at(initial_node)) };
        if (output) {
          _vertices[initial_node].set_mark(1);
          return _vertices.at(initial_node);
        }
      }

      _vertices[initial_node].set_mark(1);

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
                _vertices[x].set_mark(1);
                return _vertices.at(x);
              }
            }
            
            _vertices[x].set_mark(1);
          }
        }
      }

      return std::nullopt;
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
    std::optional<Vertex<T>> bfs(int initial_node_idx=0, U callback=nullptr) {
      std::queue<int> bfs_queue {};
      bfs_queue.emplace(initial_node_idx);

      if constexpr (std::is_void_v<decltype(callback(_vertices.at(0)))>) {
        callback(_vertices.at(initial_node_idx));
      } else {
        auto output { callback(_vertices.at(initial_node_idx)) };
        if (output) {
          _vertices[initial_node_idx].set_mark(1);
          return _vertices.at(initial_node_idx);
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
            if constexpr (std::is_void_v<decltype(callback(_vertices.at(0)))>) {
              callback(_vertices.at(x));
            } else {
              auto output { callback(_vertices.at(x)) };
              if (output) {
                _vertices[x].set_mark(1);
                return _vertices.at(x);
              }
            }

            _vertices[x].set_mark(1);
          }
        }
      }

      return std::nullopt;
    }

    void shortest_path_dijkstra(std::array<float, N>& distances, Vertex<T>& vtx) {
      std::queue<int> spd_queue { };
      spd_queue.emplace( vtx.order() );

      distances[vtx.order()] = 0;

      while (!spd_queue.empty()) {
        int idx { spd_queue.front() };
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

    int find_set(std::unordered_map<int, int>& parent, int idx_u) {
      if (parent.find(idx_u) == parent.end()) {  // Key not found
        return -1;  // or handle it in another way suitable for your program
      }
      if (parent[idx_u] != idx_u) {
        parent[idx_u] = find_set(parent, parent[idx_u]);
      }
      return parent[idx_u]; 
    }

    void union_set(std::unordered_map<int, int>& parent, std::unordered_map<int, int>& rank, int idx_u, int idx_v) {
      idx_u = find_set(idx_u);
      idx_v = find_set(idx_v);

      if (rank[idx_u] > rank[idx_v]) {
        parent[idx_v] = idx_u;
      } else {
        parent[idx_u] = idx_v;
        if (rank[idx_u] == rank[idx_v]) {
          rank[idx_v] += 1;
        }
      }
    }

    MstList union_find_krus(void) {
      std::vector<EdgeList> edges { to_edge_list() };
      std::sort(edges.begin(), edges.end(), [](const EdgeList& a, const EdgeList& b) { return a > b; });

      std::unordered_map<int, int> parent { };
      std::unordered_map<int, int>   rank { };

      for (const EdgeList& el: edges) {
        parent[el.idx_u] = el.idx_u;
        parent[el.idx_v] = el.idx_v;

        rank[el.idx_u] = 0;
        rank[el.idx_v] = 0;
      }

      std::vector<EdgeList> mst { };
      size_t mst_weight { 0 };

      MstList mst_list { };
      mst_list.mst    = mst;
      mst_list.weight = mst_weight;

      for (const EdgeList& el: edges) {
        if (find_set(parent[el.idx_u]) != find_set(parent[el.idx_v])) {
          mst_list.mst.emplace_back(el);
          mst_list.weight += el.weight;
          union_set(parent, rank, el.idx_u, el.idx_v); 
        }
      }

      return mst_list;
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