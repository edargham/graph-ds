#include "edge_list.hpp"

using namespace gtor;

bool EdgeList::operator >(const EdgeList& el) const {
  return weight > el.weight;
}

bool EdgeList::operator >=(const EdgeList& el) const {
  return weight >= el.weight;
}

bool EdgeList::operator <(const EdgeList& el) const {
  return weight < el.weight;
}

bool EdgeList::operator <=(const EdgeList& el) const {
  return weight <= el.weight;
}

bool EdgeList::operator ==(const EdgeList& el) const {
  return weight == el.weight;
}

bool EdgeList::operator !=(const EdgeList& el) const {
  return weight != el.weight;
}