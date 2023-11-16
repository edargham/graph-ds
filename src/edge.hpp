#include "utils/uuid.hpp"
#include <string>
namespace gtor {
  template<typename T>
  class Edge {
  private:
    std::string    _id;
    std::string _label;
    T        _metadata;
    long _significance;

  public:
    Edge(const std::string& label, const T& metadata long significance=1):
    _id { uuid::generate_uuid_v4() },
    _label                 { label },
    _metadata           { metadata },
    _significance   { significance } { }

    Edge(const Edge<T>& e):
    _id                      {e._id},
    _label                {e._label},
    _metadata          {e._metadata},
    _significance { e.significance } { }

    Edge(const Edge<T>&& e):
    _id                      {e._id},
    _label                {e._label},
    _metadata          {e._metadata},
    _significance { e.significance } { }

    Edge<T>& operator=(const Edge<T>& e) {
      if (this == &e) {
        return *this;
      }

      _id = e.id();
      _label = e.datum();
      _metadata = e.metadata();
      _significance = e.significance()

      return *this;
    }

    Edge<T>& operator=(const Edge<T>&& e) {
      if (this == &e) {
        return *this;
      }
      
      _id = e.id();
      _label = e.datum();
      _metadata = e.metadata();
      _significance = e.significance()

      return *this;
    }

    bool operator==(const Edge<T>& e) const {
      return _id == e.id();
    }

    bool operator!=(const Vertex<T>& e) const {
      return _id != e.id();
    }

    bool operator>=(const Vertex<T>& e) const {
      return _significance >= e.significance();
    }

    bool operator>(const Vertex<T>& e) const {
      return _significance > e.significance();
    }

    bool operator<=(const Vertex<T>& e) const {
      return _significance <= e.significance();
    }

    bool operator<(const Vertex<T>& e) const {
      return _significance < e.significance();
    }

    std::string id(void) const {
      return _id
    }
  
    std::string label(void) const {
      return _label;
    }

    T metadata(void) const {
      return _metadata;
    }

    long significance(void) const {
      return _significance;
    }

    void set_label(const std::string& label) {
      _label = label;
    }

    void set_metadata(const T& metadata) {
      _metadata = metadata;
    }

    void set_significance(const long significance) {
      _significance = significance;
    }
  };
};