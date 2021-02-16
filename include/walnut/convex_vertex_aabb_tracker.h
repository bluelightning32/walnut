#ifndef WALNUT_CONVEX_VERTEX_AABB_TRACKER_H__
#define WALNUT_CONVEX_VERTEX_AABB_TRACKER_H__

#include <array>
#include <iterator>

#include "walnut/aabb.h"
#include "walnut/convex_polygon_split_info.h"
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
  using SplitInfoRep = ConvexPolygonSplitInfo<point3_bits_template>;
  using HomoPoint3Rep = typename SplitInfoRep::HomoPoint3Rep;

  static constexpr size_t point3_bits = point3_bits_template;

  // Creates the AABB from the input vertices
  template <typename VertexIterator>
  ConvexVertexAABBTracker(const VertexIterator& begin,
                          const VertexIterator& end) :
      min_indices_{0, 0, 0},
      max_indices_{0, 0, 0} {
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

  // Creates both split children.
  //
  // This function may only be called if `split.ShouldEmitNegativeChild()`
  // `split.ShouldEmitPositiveChild()` are both true.
  //
  // `parent_begin` refers to the start of the range of vertices of the parent.
  // That is the range of vertices before the split. `parent_count` describes
  // how many vertices are available from `parent_begin`.
  //
  // `neg_begin` is the range of the negative child after the split.
  // `pos_begin` is the range of the negative child after the split. Refer to
  // ConvexPolygon::CreateSplitChildren for the order that the vertices should
  // appear in `neg_begin` and `pos_begin`.
  //
  // The first entry of the returned pair is the negative child, and the second
  // entry is the positive child.
  //
  // The vertex indices are rotated on the returned trackers so that the last 2
  // vertices of the negative child will be on the split plane. Whereas for the
  // postive child, the first and last vertices will be on the split plane.
  // This ordering is chosen to match `ConvexPolygon::CreateSplitChildren`.
  template <typename VertexIterator>
  std::pair<ConvexVertexAABBTracker, ConvexVertexAABBTracker>
  CreateSplitChildren(const VertexIterator& parent_begin, size_t parent_count,
                      const VertexIterator& neg_begin,
                      const VertexIterator& pos_begin,
                      const SplitInfoRep& split) const;

  const std::array<size_t, 3>& min_indices() const {
    return min_indices_;
  }

  const std::array<size_t, 3>& max_indices() const {
    return max_indices_;
  }

  const AABBRep& aabb() const {
    return aabb_;
  }

  // Returns true if this tracker is equal to `other`
  //
  // The extreme indices and bounding box must be equal for the trackers to be
  // considered equal.
  template <size_t other_point3_bits>
  bool operator==(
      const ConvexVertexAABBTracker<other_point3_bits>& other) const {
    return min_indices() == other.min_indices() &&
           max_indices() == other.max_indices() &&
           aabb() == other.aabb();
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

  static void SplitComponent(size_t component, int min_max_mult,
                             const HomoPoint3Rep& shared1,
                             const HomoPoint3Rep& shared2,
                             size_t parent_shifted, size_t neg_count,
                             size_t pos_count, size_t pos_range_start_shifted,
                             size_t pos_range_end_shifted, size_t& neg_output,
                             size_t& pos_output);

  std::array<size_t, 3> min_indices_;
  std::array<size_t, 3> max_indices_;
  AABBRep aabb_;
};

template <size_t point3_bits>
void ConvexVertexAABBTracker<point3_bits>::SplitComponent(
    size_t component, int min_max_mult,
    const HomoPoint3Rep& shared1, const HomoPoint3Rep& shared2,
    size_t parent_shifted, size_t neg_count, size_t pos_count,
    size_t pos_range_start_shifted, size_t pos_range_end_shifted,
    size_t& neg_output, size_t& pos_output) {
  if (parent_shifted < neg_count) {
    // Vertex is only on the negative side.
    neg_output = parent_shifted;
    if (shared1.CompareComponent(component, shared2) * min_max_mult > 0) {
      // shared1 becomes the extreme for the positive child.
      pos_output = pos_count + 1;
    } else {
      // shared2 becomes the extreme for the positive child.
      pos_output = 0;
    }
  } else if (parent_shifted >= pos_range_start_shifted) {
    if (parent_shifted < pos_range_end_shifted) {
      // Vertex is only on the positive side.
      pos_output = parent_shifted - pos_range_start_shifted + 1;
      if (shared1.CompareComponent(component, shared2) * min_max_mult > 0) {
        // shared1 becomes the extreme for the negative child.
        neg_output = neg_count + 1;
      } else {
        // shared2 becomes the extreme for the negative child.
        neg_output = neg_count;
      }
    } else {
      assert(parent_shifted == pos_range_end_shifted);
      // The vertex is after the range of the positive only child and before
      // the negative only range. So the vertex is shared1.
      neg_output = neg_count + 1;
      pos_output = pos_count + 1;
    }
  } else {
    assert(parent_shifted == neg_count);
    // The vertex is after the range of the negative only child and before
    // the positive only range. So the vertex is shared2.
    neg_output = neg_count;
    pos_output = 0;
  }
}

template <size_t point3_bits>
template <typename VertexIterator>
std::pair<ConvexVertexAABBTracker<point3_bits>,
          ConvexVertexAABBTracker<point3_bits>>
ConvexVertexAABBTracker<point3_bits>::CreateSplitChildren(
    const VertexIterator& parent_begin, size_t parent_count,
    const VertexIterator& neg_begin, const VertexIterator& pos_begin, 
    const SplitInfoRep& split) const {
  assert(split.ShouldEmitNegativeChild());
  assert(split.ShouldEmitPositiveChild());
  std::pair<ConvexVertexAABBTracker, ConvexVertexAABBTracker> result;
  size_t neg_shift = parent_count - split.neg_range().first;
  // Number of vertices that belong to only the negative side.
  size_t neg_count = split.neg_range().second - split.neg_range().first;
  size_t pos_count = split.pos_range().second - split.pos_range().first;
  // For the negative child, shared1 is located at neg_count + 1.
  // For the positive child, shared1 is located at pos_count + 1.
  const HomoPoint3Rep& shared1 = neg_begin[neg_count + 1];
  // For the negative child, shared2 is located at neg_count.
  // For the positive child, shared2 is located at 0.
  const HomoPoint3Rep& shared2 = pos_begin[0];
  size_t pos_range_start_shifted =
    (split.pos_range().first + neg_shift) % parent_count;
  size_t pos_range_end_shifted = pos_range_start_shifted + pos_count;
  assert(pos_range_start_shifted < pos_range_end_shifted);
  for (size_t component = 0; component < 3; component++) {
    // Shift the extreme index so that a value of neg_range.first becomes 0.
    size_t parent_shifted =
      (min_indices()[component] + neg_shift) % parent_count;
    SplitComponent(component, /*min_max_mult=*/-1, shared1, shared2,
                   parent_shifted, neg_count, pos_count,
                   pos_range_start_shifted, pos_range_end_shifted,
                   result.first.min_indices_[component],
                   result.second.min_indices_[component]);
  }
  for (size_t component = 0; component < 3; component++) {
    // Shift the extreme index so that a value of neg_range.first becomes 0.
    size_t parent_shifted =
      (max_indices()[component] + neg_shift) % parent_count;
    SplitComponent(component, /*min_max_mult=*/1, shared1, shared2,
                   parent_shifted, neg_count, pos_count,
                   pos_range_start_shifted, pos_range_end_shifted,
                   result.first.max_indices_[component],
                   result.second.max_indices_[component]);
  }
  result.first.ApproximateExtremes(neg_begin);
  result.second.ApproximateExtremes(pos_begin);
  return result;
}

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
