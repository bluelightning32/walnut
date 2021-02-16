#ifndef WALNUT_CONVEX_VERTEX_AABB_TRACKER_H__
#define WALNUT_CONVEX_VERTEX_AABB_TRACKER_H__

#include <array>
#include <iterator>

#include "walnut/aabb.h"
#include "walnut/rational.h"

namespace walnut {

// Builds an approximate axis-aligned bounding box from a range of convex
// vertices.
//
// The range of vertices is specified by a begin and end iterator. The range of
// vertices must form a convex polygon.
//
// Specifically the vertices in the range are `HomoPoint3`. Out of all of the
// extreme vertices that define the AABB (the vertices which are on the AABB
// boundary), this class takes the vertex denominator with the largest
// magnitude and uses that as the overall denominator for the entire AABB. Some
// of the extreme vertices may not be exact multiples of the chosen
// denominator. The resultant AABB is enlarged so that it contains all of the
// input vertices.
template <size_t point3_bits_template = 32>
class ConvexVertexAABBTracker {
 public:
  using AABBRep = AABB<(point3_bits_template - 1)*7 + 10,
                       (point3_bits_template - 1)*6 + 10>;
  using DenomInt = typename AABBRep::DenomInt;

  static constexpr size_t point3_bits = point3_bits_template;

  // Creates the AABB from the input vertices
  template <typename VertexIterator>
  ConvexVertexAABBTracker(const VertexIterator& begin,
                          const VertexIterator& end) :
      min_indices_{0, 0, 0},
      max_indices_{0, 0, 0} {
    using HomoPoint3Rep =
      typename std::iterator_traits<VertexIterator>::value_type;
    VertexIterator it = begin;
    if (it != end) {
      ++it;
      for (size_t pos = 1; it != end; ++it, ++pos) {
        for (size_t i = 0; i < 3; ++i) {
          const HomoPoint3Rep& old_min = begin[min_indices_[i]];
          if (it->CompareComponent(i, old_min) < 0) {
            // new_min is lower.
            min_indices_[i] = pos;
          }

          const HomoPoint3Rep& old_max = begin[max_indices_[i]];
          if (it->CompareComponent(i, old_max) > 0) {
            // new_max is greater.
            max_indices_[i] = pos;
          }
        }
      }
      ApproximateExtremes(begin);
    }
  }

  // Creates an empty tracker
  ConvexVertexAABBTracker() :
    min_indices_{0, 0, 0},
    max_indices_{0, 0, 0} { }

  // Returns false if the extreme vertex indices are invalid.
  bool IsValidState(size_t vertex_count) const {
    if (vertex_count == 0) {
      for (size_t i = 0; i < 3; ++i) {
        if (min_indices_[i] != 0) return false;
        if (max_indices_[i] != 0) return false;
      }
    } else {
      for (size_t i = 0; i < 3; ++i) {
        if (min_indices_[i] >= vertex_count) return false;
        if (max_indices_[i] >= vertex_count) return false;
      }
    }
    return true;
  }

  // Rotates all of the extreme vertex indices to the left by `offset`.
  //
  // The caller must ensure:
  //   0 <= offset <= vertex_count
  void RotateIndices(size_t offset, size_t vertex_count) {
    for (size_t i = 0; i < 3; ++i) {
      min_indices_[i] =
        (min_indices_[i] + vertex_count - offset) % vertex_count;
      max_indices_[i] =
        (max_indices_[i] + vertex_count - offset) % vertex_count;
    }
  }

  const std::array<size_t, 3>& min_indices() const {
    return min_indices_;
  }

  const std::array<size_t, 3>& max_indices() const {
    return max_indices_;
  }

  const AABBRep& aabb() const {
    return aabb_;
  }

 private:
  // Out of all of the extreme indices, returns the denominator with the
  // highest absolute value.
  template <typename VertexIterator>
  DenomInt GetMaxDenominator(const VertexIterator& begin) const;

  // Builds `aabb_` from `min_indices_` and `max_indices_`.
  template <typename VertexIterator>
  void ApproximateExtremes(const VertexIterator& begin) {
    ApproximateExtremes(GetMaxDenominator(begin), begin);
  }

  // Builds `aabb_` from `min_indices_` and `max_indices_`.
  template <typename VertexIterator>
  void ApproximateExtremes(DenomInt denom, const VertexIterator& begin);

  std::array<size_t, 3> min_indices_;
  std::array<size_t, 3> max_indices_;
  AABBRep aabb_;
};

template <size_t point3_bits>
template <typename VertexIterator>
typename ConvexVertexAABBTracker<point3_bits>::DenomInt
ConvexVertexAABBTracker<point3_bits>::GetMaxDenominator(
    const VertexIterator& begin) const {
  DenomInt denom = begin[min_indices_[0]].w().abs();
  for (int i = 1; i < 3; ++i) {
    denom = std::max<DenomInt>(denom, begin[min_indices_[i]].w().abs());
  }
  for (int i = 0; i < 3; ++i) {
    denom = std::max<DenomInt>(denom, begin[max_indices_[i]].w().abs());
  }
  return denom;
}

template <size_t point3_bits>
template <typename VertexIterator>
void ConvexVertexAABBTracker<point3_bits>::ApproximateExtremes(
    DenomInt denom, const VertexIterator& begin) {
  aabb_ = AABBRep(
      rational::RoundDown(begin[min_indices_[0]].x(),
                          begin[min_indices_[0]].w(), denom),
      rational::RoundDown(begin[min_indices_[1]].y(),
                          begin[min_indices_[1]].w(), denom),
      rational::RoundDown(begin[min_indices_[2]].z(),
                          begin[min_indices_[2]].w(), denom),
      rational::RoundUp(begin[max_indices_[0]].x(),
                        begin[max_indices_[0]].w(), denom),
      rational::RoundUp(begin[max_indices_[1]].y(),
                        begin[max_indices_[1]].w(), denom),
      rational::RoundUp(begin[max_indices_[2]].z(),
                        begin[max_indices_[2]].w(), denom),
      std::move(denom));
}

template <size_t point3_bits>
std::ostream& operator<<(
    std::ostream& out,
    const ConvexVertexAABBTracker<point3_bits>& tracker) {
  out << "min_indices=[";
  for (size_t i = 0; i < 3; ++i) {
    out << tracker.min_indices()[i];
    if (i != 2) {
      out << ", ";
    }
  }
  out << "] max_indices=[";
  for (size_t i = 0; i < 3; ++i) {
    out << tracker.max_indices()[i];
    if (i != 2) {
      out << ", ";
    }
  }
  out << "] " << tracker.aabb();
  return out;
}

}  // walnut

#endif // WALNUT_CONVEX_VERTEX_AABB_TRACKER_H__