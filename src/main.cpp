#include "vertex.hpp"
#include "graph.hpp"

#include <array>
#include <optional>
#include <iostream>
#include <limits>

bool is_equal(char a, char b) {
  return a == b;
}


int main(void) {
  const size_t graph_size { 5 };
  std::array<gtor::Vertex<char>, graph_size> vtxs { 'A', 'B', 'C', 'D', 'E' };
  std::array<std::array<size_t, graph_size>, graph_size> edges {
    //                            A  B  C  D  E
    std::array<size_t, graph_size> { 0, 5, 0, 8, 0 }, // A
    std::array<size_t, graph_size> { 5, 0, 0, 2, 0 }, // B
    std::array<size_t, graph_size> { 0, 0, 1, 0, 6 }, // C
    std::array<size_t, graph_size> { 8, 2, 0, 0, 0 }, // D
    std::array<size_t, graph_size> { 0, 0, 6, 0, 0 }  // E
  };

  gtor::Graph<char, graph_size> g_v_e { vtxs, edges };

  char tgt { 'E' };

  std::array<float, graph_size> distances {  };

  for (size_t i { 0 }; i < distances.size(); i++) {
    distances[i] = std::numeric_limits<float>::infinity();
  }

  auto search_lambda = [&g_v_e, graph_size, &tgt] (gtor::Vertex<char>& vrt) {
    std::cout << vrt.datum() << " ";
    return tgt == vrt.datum();
  };

  std::optional<gtor::Vertex<char>> target { g_v_e.bfs_all(search_lambda) };

  if (target) {
    std::cout << "Target found: " << target.value().datum() << "\n";
  } else {
    std::cout << "Target not found.\n";
  }

  size_t vtx_at { 0 };
  g_v_e.shortest_path_dijkstra(distances, g_v_e.vertices().at(vtx_at));

  std::cout << "\nShortest path to all connected vertices from " 
            << g_v_e.vertices().at(vtx_at).datum()
            << ":\n";
  for (int i { 0 }; i < graph_size; i++) {
    std::cout << g_v_e.vertices().at(i).datum() << ": " << distances.at(i) << "\n";  
  }

  return 0;
}