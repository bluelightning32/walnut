#ifndef WALNUT_VERTEX_DOUBLE_POINT3_MAPPER_H__
#define WALNUT_VERTEX_DOUBLE_POINT3_MAPPER_H__

#include <array>
// For remove_reference
#include <type_traits>
#include <unordered_map>
// For declval
#include <utility>

// For ConnectedEdge
#include "walnut/connected_polygon.h"
#include "walnut/double_point3.h"
#include "walnut/redirectable_value.h"

namespace walnut {

// Converts the vertices of `ConnectedEdge`s into `DoublePoint3`s with
// auxiliary data.
//
// Due to small rounding errors, there can be multiple DoublePoint3
// representations for different `HomoPoint3`s that are considered equivalent.
// However, when exporting a polyhedron into a double format, the exported
// format expects the `DoublePoint3`s to be exactly the same if the
// `HomoPoint3`s are equivalent. VertexDoublePoint3Mapper maintains that
// invariant by merging together different DoublePoint3 representations for the
// same vertex, as they are discovered.
//
// VertexDoublePoint3Mapper may temporarily have unmerged vertices, until the
// point where it has been called with every edge at least once.
//
// The mappings are stored in the `map` field. The field is publically
// accessible so that the caller may take the results out of it after the
// merging is complete.
//
// `ValueFactory` is a functor that accepts a DoublePoint3 and produces a value
// type for the map. The value type is inferred from the return type of the
// factory.
//
// `ValueMerger` is a functor that accepts two RedirectableValue<Value>
// parameters, and merges the first into the second. Afterwards one of the
// redirectable values should point to the other. The return value of the
// functor is ignored.
template <typename ValueFactory, typename ValueMerger>
struct VertexDoublePoint3Mapper {
  using Value = std::remove_reference_t<
    decltype(std::declval<ValueFactory>()(std::declval<DoublePoint3>()))>;
  using MapType = std::unordered_map<DoublePoint3, RedirectableValue<Value>>;
  using MapIterator = typename MapType::iterator;

  VertexDoublePoint3Mapper() = default;

  VertexDoublePoint3Mapper(ValueFactory factory, ValueMerger merger)
    : factory(std::move(factory)), merger(std::move(merger)) { }

  template <typename FinalPolygon, typename EdgeParent>
  MapIterator Map(const ConnectedEdge<FinalPolygon, EdgeParent>& edge) {
    DoublePoint3 first_point = edge.vertex().GetDoublePoint3();
    MapIterator it = map.find(first_point);
    if (it == map.end()) {
      auto emplace_result = map.emplace(first_point, factory(first_point));
      assert(emplace_result.second);
      it = emplace_result.first;
    }

    // The point is in the map. However, there can be multiple DoublePoint3
    // representations of the same HomoPoint3. So check the next edge that
    // originates from the same vertex, and if it has a different
    // representation, merge them.
    const ConnectedEdge<FinalPolygon, EdgeParent>* next =
      edge.GetNextAroundVertex();
    if (next == nullptr) return it;

    DoublePoint3 next_point = next->vertex().GetDoublePoint3();
    if (next_point != first_point) {
      MapIterator next_it = map.find(next_point);
      if (next_it == map.end()) {
        map.emplace(next_point, it->second);
      } else if (next_it->second != it->second) {
        merger(it->second, next_it->second);
      }
    }
    return it;
  }

  ValueFactory factory;
  ValueMerger merger;
  MapType map;
};

}  // walnut

#endif // WALNUT_VERTEX_DOUBLE_POINT3_MAPPER_H__
