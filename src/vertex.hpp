#pragma once
#include <memory>

namespace gtor {
  template<typename T>
  class Vertex {
  private:
    T       _datum;
    int      _mark;
    int _order_idx;

  public:
    
    Vertex(void)=default;

    Vertex(T datum, int order=0):
      _datum         { datum },
      _mark              { 0 },
      _order_idx      { order }
    {}

    Vertex(const Vertex<T>& v):
      _datum        { v._datum },
      _mark         {  v._mark },
      _order_idx { v._order_idx }
    { }

    Vertex(const Vertex<T>&& v):
      _datum { v._datum },
      _mark  {  v._mark }
    { }    

    ~Vertex(void) { }

    Vertex<T>& operator=(const Vertex<T>& v) {
      if (this == &v) {
          return *this;
      }

      _datum = v.datum();
      _mark = v.mark();

      return *this;
    }

    Vertex<T>& operator=(const Vertex<T>&& v) {
      if (this == &v) {
          return *this;
      }
      
      _datum = v.datum();
      _mark = v.mark();
      
      return *this;
    }

    T datum(void) const {
      return _datum;
    }

    int mark(void) const {
      return _mark;
    }

    int order(void) const {
      return _order_idx;
    }

    std::shared_ptr<T> datum_pt(void) {
      return std::make_shared<T>(_datum);
    }

    void set_mark(int mark) {
      _mark = mark;
    }

    void set_order(int order) {
      _order_idx = order;
    }
  };
}