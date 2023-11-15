#pragma once

namespace gtor {
  struct EdgeList
  {
    int idx_u;
    int idx_v;
    int weight;

    bool operator >(const EdgeList& el) const;
    bool operator >=(const EdgeList& el) const;
    bool operator <(const EdgeList& el) const;
    bool operator <=(const EdgeList& el) const;
    bool operator ==(const EdgeList& el) const;
    bool operator !=(const EdgeList& el) const;
  };
}