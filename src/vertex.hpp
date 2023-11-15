#pragma once

#include "utils/uuid.hpp"

#include <memory>
#include <string>

namespace gtor {
  template<typename T>
  class Vertex {
  private:
    T           _datum;
    std::string    _id;
  public:
    
    Vertex(void)=default;

    Vertex(T datum):
      _id     { generate_uuid_v4() },
      _datum               { datum }
    {}

    Vertex(const Vertex<T>& v):
      _id       { v._id },
      _datum { v._datum }
    { }

    Vertex(const Vertex<T>&& v):
      _id       { v._id },
      _datum { v._datum }
    { }    

    ~Vertex(void) { }

    Vertex<T>& operator=(const Vertex<T>& v) {
      if (this == &v) {
        return *this;
      }

      _id = v.id();
      _datum = v.datum();

      return *this;
    }

    Vertex<T>& operator=(const Vertex<T>&& v) {
      if (this == &v) {
        return *this;
      }
      
      _id = v.id();
      _datum = v.datum();
      
      return *this;
    }

    bool operator==(const Vertex<T>& v) const {
      return _id == v.id();
    }

    bool operator!=(const Vertex<T>& v) const {
      return _id != v.id();
    }

    T datum(void) const {
      return _datum;
    }

    std::string id(void) const {
      return _id;
    }

    std::shared_ptr<T> datum_pt(void) {
      return std::make_shared<T>(_datum);
    }
  };
}