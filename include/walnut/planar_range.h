#ifndef WALNUT_PLANAR_RANGE_H__
#define WALNUT_PLANAR_RANGE_H__

#include <iterator>

#include "walnut/concat_range.h"
#include "walnut/half_space3.h"
#include "walnut/point3.h"

namespace walnut {

// Represents a subrange of a parent range of R^3 vertices, such that all of
// the vertices in the subrange are in a plane. Most of the logic in the class
// is around finding the planar subrange within the parent range.
template <typename Point3Iterator>
class PlanarRange {
 public:
  using Point3Rep = typename std::iterator_traits<Point3Iterator>::value_type;
  using ConcatRangeRep = ConcatRange<Point3Iterator>;
  using OutputIterator = typename ConcatRangeRep::const_iterator;

  // Find the next planar range from an iterator range of `Point3Rep`s.
  //
  // First the function finds the plane normal. This is calculated from one
  // vertex at the end of the input range and typically 2 vertices from the
  // beginning of the input range, although more are taken from the beginning
  // range if they are collinear.
  //
  // After the plane normal is found, more vertices are taken from the
  // beginning and end of the input range, as long as they are in the same
  // plane.
  //
  // Splitting out a planar range essentially removes a range from the
  // input iterator range. The first and last vertices of the planar output
  // must also stay in the remaining range so that as this function is repeated
  // called on the same input range, the resulting polygons are connected with
  // edges.
  //
  // The input vertices represent a cycle, but they are stored as an iterator
  // range, where the end of the iterator range is logically connected back to
  // the beginning again. This function pulls vertices from the start and end
  // of the iterator range. That way the existing break in the iterator range
  // is enlarged instead of introducing another break.
  //
  // remaining_begin: on input, this points to the list of vertices to take
  //    from. On output, this is the first point to consider in the next input
  //    range.
  // remaining_end: on input, this points to one past the last vertex to
  //    in the input range. On output, this points to one past the last vertex
  //    to use for the next input range.
  void Build(Point3Iterator& remaining_begin,
             Point3Iterator& remaining_end);

  // Returns the start of the output range from the last `Build` call.
  OutputIterator begin() const {
    return range_.begin();
  }

  // Returns the end of the output range from the last `Build` call.
  OutputIterator end() const {
    return range_.end();
  }

  // Returns the number of vertices in the output range.
  size_t size() const {
    return size_;
  }

  // Returns the plane that all of the output vertices are in.
  //
  // If all of the vertices in the input range were collinear, a plane with a 0
  // normal vector will be returned.
  const HalfSpace3& plane() const {
    return plane_;
  }

 private:
  ConcatRangeRep range_;
  size_t size_;
  HalfSpace3 plane_;
};

template <typename Point3Iterator>
inline void PlanarRange<Point3Iterator>::Build(
    Point3Iterator& remaining_begin,
    Point3Iterator& remaining_end) {
  range_.Clear();
  if (remaining_begin == remaining_end) {
    plane_ = HalfSpace3::Zero();
    size_ = 0;
    return;
  }
  Point3Iterator range_begin = remaining_begin;
  Point3Iterator range_end = remaining_end;
  --remaining_end;

  const Point3Rep& p1 = *remaining_end;
  if (remaining_begin == remaining_end) {
    range_.Append(range_begin, range_end);
    plane_ = HalfSpace3::Zero();
    size_ = 1;
    return;
  }

  const Point3Rep& p2 = *remaining_begin;
  size_ = 2;
  ++remaining_begin;

  // Keep p1 and p2 pointing to where they are. Use the next soonest point for
  // p3 such that p1, p2, and p3 are not coincident.
  //
  // After the loop ends, `plane_` holds the plane formed from p1, p2, and p3.
  do {
    if (remaining_begin == remaining_end) {
      range_.Append(range_begin, range_end);
      plane_ = HalfSpace3::Zero();
      return;
    }
    const Point3Rep& p3 = *remaining_begin;
    plane_ = HalfSpace3(p1, p2, p3);
    ++size_;
    ++remaining_begin;
  } while (!plane_.IsValid());

  // Keep picking more points from the beginning of the input range, as long as
  // they are in the same plane and the input range does not run out.
  while (true) {
    if (remaining_begin == remaining_end) {
      range_.Append(range_begin, range_end);
      return;
    }
    if (!plane_.IsCoincident(*remaining_begin)) {
      break;
    }
    ++size_;
    ++remaining_begin;
  }

  // Keep picking more points from the end of the input range, as long as they
  // are in the plane. The input range is guaranteed not to run out here,
  // because if all of the input points were in the plane, the previous loop
  // would have already accepted them.
  while (true) {
    assert(remaining_begin != remaining_end);
    --remaining_end;
    if (!plane_.IsCoincident(*remaining_end)) {
      ++remaining_end;
      break;
    }
    ++size_;
  }
  range_.Append(range_begin, remaining_begin);
  range_.Append(remaining_end, range_end);

  // Since not all of the vertices were used from the input range, add back the
  // two vertices from the ends of the output range.
  --remaining_begin;
  ++remaining_end;
}

}  // walnut

#endif // WALNUT_PLANAR_RANGE_H__
