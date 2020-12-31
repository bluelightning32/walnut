#ifndef WALNUT_NO_VERTEX_DATA_H__
#define WALNUT_NO_VERTEX_DATA_H__

namespace walnut {

// This is the default data that is attached to every vertex of a
// ConvexPolygon.
struct NoVertexData {
  constexpr NoVertexData() = default;
  template <typename Other>
  constexpr explicit NoVertexData(const Other&) { }

  template <typename Other>
  constexpr bool operator!=(const Other& other) const {
    return false;
  }

  template <typename Other>
  constexpr NoVertexData& operator=(const Other& other) {
    return *this;
  }
};

}  // walnut

#endif // WALNUT_NO_VERTEX_DATA_H__
