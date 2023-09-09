#include "vertex.hpp"
#include "graph.hpp"

#include <array>
#include <optional>
#include <iostream>

bool is_equal(char a, char b) {
  return a == b;
}


int main(void) {
  const int graph_size { 5 };
  std::array<gtor::Vertex<char>, graph_size> vtxs { 'A', 'B', 'C', 'D', 'E' };
  std::array<std::array<int, graph_size>, graph_size> edges {
    //                            A  B  C  D  E
    std::array<int, graph_size> { 0, 1, 0, 1, 0 }, // A
    std::array<int, graph_size> { 1, 0, 0, 1, 0 }, // B
    std::array<int, graph_size> { 0, 0, 1, 0, 1 }, // C
    std::array<int, graph_size> { 1, 1, 0, 0, 0 }, // D
    std::array<int, graph_size> { 0, 0, 1, 0, 0 }  // E
  };

  gtor::Graph<char, graph_size> g_v_e { vtxs, edges };

  char tgt { 'F' };

  auto lambd = [&g_v_e, graph_size, &tgt] (gtor::Vertex<char>& vrt) {
    std::cout << vrt.datum() << " ";
    return tgt == vrt.datum();
  };

  std::optional<gtor::Vertex<char>> target { g_v_e.bfs_all(lambd) };
  if (target) {
    std::cout << "Target found: " << target.value().datum() << "\n";
  } else {
    std::cout << "Target not found.\n";
  }

  return 0;
}