#ifndef WALNUT_MONOTONE_RANGE_H__
#define WALNUT_MONOTONE_RANGE_H__

#include <iterator>

#include "walnut/concat_range.h"
#include "walnut/point3.h"

namespace walnut {

// Represents a monotone piece of an R^3 polygon. The vertices are a contiguous
// range of a parent polygon (not necessarily monotone). Most of the logic in
// the class is around finding a monotone range within the parent polygon.
template <typename Point3Iterator>
class MonotoneRange {
 public:
  using Point3Rep = typename std::iterator_traits<Point3Iterator>::value_type;
  using ConcatRangeRep = ConcatRange<Point3Iterator>;

  // Find the next monotone polygon from an iterator range of `Point3Rep`s in
  // the same plane.
  //
  // Abstractly, given an input list of vertices, all in the same plane, this
  // function grabs as many vertices as possible from the beginning and end of
  // the input range such that it forms a monotone polygon. Although iterators
  // are used instead of lists, and the function also takes the end of the
  // input list into account.
  //
  // Splitting out a monotone polygon essentially removes a range from the
  // input iterator range. The first and last vertices of the monotone output
  // must also stay in the remaining range so that spatially all of the initial
  // input polygon is covered by the outputted monotone polygon and remaining
  // polygon.
  //
  // The input vertices represent a cycle, but they are stored as an iterator
  // range, where the end of the iterator range is logically connected back to
  // the beginning again. This function pulls vertices from the start and end
  // of the iterator range. That way the existing break in the iterator range
  // is enlarged instead of introducing another break.
  //
  // Note that if the input polygon is not already monotone, this function may
  // return a self intersecting (but monotone) polygon. This is a tradeoff for
  // never adding new vertices to the input.
  //
  // monotone_dimension: the dimension that the output polygon should be
  //    monotone in. This is in the range [0, 3).
  // remaining_begin: on input, this points to the list of vertices to take
  //    from for the monotone polygon. On output, this is the first point to
  //    consider in the next monotone polygon.
  // remaining_end: on input, this points to one past the last vertex to
  //    consider for the monotone polygon. On output, this points to one past
  //    the last vertex to use for the next monotone polygon.
  void Build(int monotone_dimension,
             Point3Iterator& remaining_begin,
             Point3Iterator& remaining_end);

  // Gets the resulting chains from the last call to `Build`. The next call to
  // `Build` will invalidate the resulting iterators.
  //
  // The minimum and maximum vertices are included in both chains.
  //
  // Both chains are increasing in the monotone_dimension.
  //
  // If the vertices taken from the input range were in counter-clockwise
  // order, then chain1 will be less than chain2 in the compare dimension.
  void GetChains(typename ConcatRangeRep::const_iterator& chain1_start,
                 typename ConcatRangeRep::const_iterator& chain1_end,
                 typename ConcatRangeRep::const_reverse_iterator& chain2_begin,
                 typename ConcatRangeRep::const_reverse_iterator& chain2_end)
    const;

 private:
  // Gets the direction of the first vertices of remaining_begin relative to
  // `*prev`, in the monontone dimension.
  //
  // `remaining_begin` will be advanced until it has a different component
  // than `prev` in the monotone dimension. If `remaining_begin` reaches
  // `remaining_end` before finding a vertex different from `prev`, then 0 is
  // returned instead.
  //
  // On output, `remaining_begin` points to the first vertex with a different
  // value in the monotone dimension, or it points to `remaining_end` if it
  // does not have such a vertex.
  //
  // Returns:
  // * 1 if the first vertices of `remaining_begin` are increasing in the
  //   monotone dimension.
  // * 0 if all vertices of `remaining_begin` have the same component as prev
  //   in the monotone dimension.
  // * -1 if the first vertices of `remaining_begin` are decreasing in the
  //   monotone dimension.
  static int GetDir(int monotone_dimension, const Point3Rep* prev,
      Point3Iterator& remaining_begin, Point3Iterator remaining_end);

  // Decrements `remaining_end` as long as the vertices follow `dir` in the
  // monotone dimension. Initially `*remaining_end` is compared against
  // `*next`, but after one vertex is accepted, it is compared against the
  // previously accepted vertex.
  //
  // Precondition: there must be a vertex somewhere at or before
  // `remaining_end` such that it moves in the opposite direction, and can
  // termintate this algorithm before it deferences an invalid iterator.
  //
  // On output, `remaining_end` points to the last vertex that was accepted.
  static void FollowDirReverse(int monotone_dimension, int dir,
      const Point3Rep* next, Point3Iterator& remaining_end);

  // Increments `remaining_begin` as long as the vertices follow `dir` in the
  // monotone dimension. Initially `*remaining_begin` is compared against
  // `*prev`, but after one vertex is accepted, it is compared against the
  // previously accepted vertex.
  //
  // Precondition: there must be a vertex somewhere at or after
  // `remaining_begin` such that it moves in the opposite direction, and can
  // termintate this algorithm before it deferences an invalid iterator.
  //
  // On output, `remaining_begin` points to the first vertex that was not
  // accepted.
  static void FollowDir(int monotone_dimension, int dir,
      const Point3Rep* prev, Point3Iterator& remaining_begin);

  // Increments `remaining_begin` as long as the vertices follow `dir` in the
  // monotone dimension, and they are before `up_to` in the monotone_dimension.
  // Initially `*remaining_begin` is compared against `*prev`, but after one
  // vertex is accepted, it is compared against the previously accepted vertex.
  //
  // On output, `remaining_begin` points to the first vertex that was not
  // accepted.
  static void FollowDirUpTo(int monotone_dimension, int dir,
      const Point3Rep* prev, Point3Iterator& remaining_begin,
      const Point3Rep* up_to);

  // Decrements `remaining_end` as long as the vertices follow `dir` in the
  // monotone dimension, and they are before `up_to` in the monotone_dimension.
  // Initially `*remaining_end` is compared against `*next`, but after one
  // vertex is accepted, it is compared against the
  // previously accepted vertex.
  //
  // Precondition: there must be a vertex somewhere at or before
  // `remaining_end` such that it moves in the opposite direction, and can
  // termintate this algorithm before it deferences an invalid iterator.
  //
  // On output, `remaining_end` points to the last vertex that was accepted.
  static void FollowDirUpToReverse(int monotone_dimension, int dir,
      const Point3Rep* next, Point3Iterator& remaining_end,
      const Point3Rep* up_to);

  // 1 if chain1 is increasing in the monotone dimension and chain2 is
  // decreasing in the monotone dimension.
  //
  // 0 if all of vertices in both chains all have the same monotone component.
  //
  // -1 if chain1 is decreasing in the monotone dimension and chain2 is
  // increasing in the monotone dimension.
  int chain_dir_;

  ConcatRangeRep chain1_;
  ConcatRangeRep chain2_;
};

template <typename Point3Iterator>
inline int MonotoneRange<Point3Iterator>::GetDir(
    int monotone_dimension, const Point3Rep* prev,
    Point3Iterator& remaining_begin, Point3Iterator remaining_end) {
  if (remaining_begin == remaining_end) {
    return 0;
  }

  // Find the first vertex that has a different value in the monotone dimension
  // than *remaining_end. This is necessary to find out if the vertices are
  // initially increasing or decreasing in the monotone dimension.
  while (remaining_begin->components()[monotone_dimension] ==
         prev->components()[monotone_dimension]) {
    ++remaining_begin;
    if (remaining_begin == remaining_end) {
      return 0;
    }
  }

  // Verified by previous loop
  assert(remaining_begin != remaining_end);
  return prev->components()[monotone_dimension] <
         remaining_begin->components()[monotone_dimension] ? 1 : -1;
}

template <typename Point3Iterator>
inline void MonotoneRange<Point3Iterator>::FollowDirReverse(
    int monotone_dimension, int dir, const Point3Rep* next,
    Point3Iterator& remaining_end) {
  while (next->components()[monotone_dimension].Compare(
          remaining_end->components()[monotone_dimension]) * dir >= 0) {
    next = &*remaining_end;
    --remaining_end;
  }
  // At this point remaining_end is pointing to the last vertex not accepted in
  // the above loop.
  ++remaining_end;
  // Now remaining_end is the last vertex accepted in the above loop.
}

template <typename Point3Iterator>
inline void MonotoneRange<Point3Iterator>::FollowDir(
    int monotone_dimension, int dir, const Point3Rep* prev,
    Point3Iterator& remaining_begin) {
  while (remaining_begin->components()[monotone_dimension].Compare(
          prev->components()[monotone_dimension]) * dir >= 0) {
    prev = &*remaining_begin;
    ++remaining_begin;
  }
}

template <typename Point3Iterator>
inline void MonotoneRange<Point3Iterator>::FollowDirUpTo(
    int monotone_dimension, int dir, const Point3Rep* prev,
    Point3Iterator& remaining_begin, const Point3Rep* up_to) {
  while (up_to->components()[monotone_dimension].Compare(
          remaining_begin->components()[monotone_dimension]) * dir > 0 &&
         remaining_begin->components()[monotone_dimension].Compare(
           prev->components()[monotone_dimension]) * dir >= 0) {
    prev = &*remaining_begin;
    ++remaining_begin;
  }
}

template <typename Point3Iterator>
inline void MonotoneRange<Point3Iterator>::FollowDirUpToReverse(
    int monotone_dimension, int dir, const Point3Rep* next,
    Point3Iterator& remaining_end, const Point3Rep* up_to) {
  while (next->components()[monotone_dimension].Compare(
          remaining_end->components()[monotone_dimension]) * dir >= 0 &&
         remaining_end->components()[monotone_dimension].Compare(
          up_to->components()[monotone_dimension]) * dir > 0) {
    next = &*remaining_end;
    --remaining_end;
  }
  // At this point remaining_end is pointing to the last vertex not accepted in
  // the above loop.
  ++remaining_end;
  // Now remaining_end is the last vertex accepted in the above loop.
}

template <typename Point3Iterator>
inline void MonotoneRange<Point3Iterator>::Build(
    int monotone_dimension, Point3Iterator& remaining_begin,
    Point3Iterator& remaining_end) {
  chain1_.Clear();
  chain2_.Clear();
  if (remaining_begin == remaining_end) {
    chain_dir_ = 0;
    return;
  }
  Point3Iterator range_end = remaining_end;
  --remaining_end;

  // Step 1. determine chain_dir_.
  Point3Iterator range_begin = remaining_begin;
  chain_dir_ = MonotoneRange<Point3Iterator>::GetDir(monotone_dimension,
                                                     /*prev=*/&*remaining_end,
                                                     remaining_begin,
                                                     remaining_end);
  if (chain_dir_ == 0) {
    // All of the vertices are collinear.
    chain1_.Append(range_begin, range_end);
    // The minimum vertex must be either at the beginning of chain1_ (and at
    // the end of chain2_) or at the end of chain1_ (and at the beginning of
    // chain1_). The maximum vertex must be on the opposite end of chain1_ and
    // chain2_. Since chain_dir_ == 0, all of the input vertices have the same
    // component in the monotone dimension. So any 2 of them could be declared
    // the minimum or maximum. For simplicity, we'll choose the start of
    // chain1_ as the minimum and the end of chain1_ as the maximum.
    //
    // Now that minimum and maximum needs to be added to chain2. Again, the
    // minimum and maximum are at the ends of chain1_. The ends must be swapped
    // when they are copied to chain2_.
    chain2_.Append(remaining_end, range_end);
    if (remaining_end != range_begin) {
      // Only random access iterators have a +n operator. To support
      // bidirectional iterators, use ++ with a temporary iterator instead.
      Point3Iterator temp = range_begin;
      ++temp;
      chain2_.Append(range_begin, temp);
    }
    return;
  }

  // chain1_ will contain part of the end of the input range, followed by part
  // of the beginning of the input range. Somewhere in the middle of chain1_,
  // it will contain *remaining_end, possibly followed by some vertices from the
  // beginning of the input range.

  // Post condition of `GetDir`.
  assert(remaining_begin != remaining_end);

  // Step 2. add vertices from the end of the input range to chain1_.
  //
  // Keep accepting vertices from the end of the input range while they
  // continue to increase/decrease (depending on dir) relative to the next
  // vertex.
  Point3Rep* next = &*remaining_end;
  --remaining_end;
  FollowDirReverse(monotone_dimension, /*dir=*/chain_dir_, next, remaining_end);
  // remaining_end points to the last vertex accepted by FollowDirReverse.
  // Invariant:
  //   range_begin <= remaining_begin < remaining_end < range_end
  assert(remaining_end != remaining_begin);
  assert(remaining_end != range_end);
  chain1_.Append(remaining_end, range_end);

  // Step 3. add vertices from the beginning of the input range to chain1_.
  //
  // Keep accepting vertices from the beginning of the input range while they
  // continue to increase/decrease (depending on dir) relative to the previous
  // vertex.
  Point3Rep* prev = &*remaining_begin;
  ++remaining_begin;
  FollowDir(monotone_dimension, /*dir=*/chain_dir_, prev, remaining_begin);
  // At this point remaining_begin is the first vertex not accepted for the end
  // of chain1_. Invariant:
  //   range_begin < remaining_begin <= remaining_end
  assert(remaining_begin != range_begin);
  chain1_.Append(range_begin, remaining_begin);

  // Step 4. add vertices from the beginning of the input range to chain2_.
  //
  // Keep accepting vertices from the beginning of the input range while they
  // continue to increase/decrease (depending on -dir) relative to the previous
  // vertex. The last vertex from step 3 is also included in step 4, because
  // the min and max vertices should be added to both chains.
  range_begin = remaining_begin;
  // Go back to the last vertex added in step 3.
  --range_begin;
  prev = &*range_begin;
  FollowDirUpTo(monotone_dimension, /*dir=*/-chain_dir_, prev, remaining_begin,
                /*up_to=*/&*remaining_end);
  // At this point remaining_begin is the last vertex not accepted in the above
  // loop. Invariant:
  //   range_begin < remaining_begin <= remaining_end
  assert(remaining_begin != range_begin);
  while (remaining_begin != remaining_end &&
         (remaining_begin->components()[monotone_dimension] ==
          remaining_end->components()[monotone_dimension])) {
    ++remaining_begin;
  }
  chain2_.Append(range_begin, remaining_begin);

  // Step 5. add vertices from the end of the input range to chain2_.
  //
  // Keep accepting vertices from the end of the input range while they
  // continue to increase/decrease (depending on dir) relative to the next
  // vertex.
  range_end = remaining_end;
  // Include the last vertex added in step 2.
  ++range_end;
  next = &*remaining_end;
  --remaining_end;
  Point3Iterator last_accepted = remaining_begin;
  --last_accepted;
  FollowDirUpToReverse(monotone_dimension, /*dir=*/-chain_dir_, next,
                       remaining_end, /*up_to=*/&*last_accepted);
  if (remaining_begin != remaining_end) {
    --remaining_end;
    while (remaining_begin != remaining_end &&
           (last_accepted->components()[monotone_dimension] ==
            remaining_end->components()[monotone_dimension])) {
      --remaining_end;
    }
    ++remaining_end;
  }
  chain2_.Append(remaining_end, range_end);

  if (remaining_begin != remaining_end) {
    --remaining_begin;
    ++remaining_end;
  }
}

template <typename Point3Iterator>
inline void MonotoneRange<Point3Iterator>::GetChains(
    typename ConcatRangeRep::const_iterator& chain1_start,
    typename ConcatRangeRep::const_iterator& chain1_end,
    typename ConcatRangeRep::const_reverse_iterator& chain2_begin,
    typename ConcatRangeRep::const_reverse_iterator& chain2_end) const {
  if (chain_dir_ >= 0) {
    chain1_start = chain1_.begin();
    chain1_end = chain1_.end();
    chain2_begin = chain2_.rbegin();
    chain2_end = chain2_.rend();
  } else {
    chain1_start = chain2_.begin();
    chain1_end = chain2_.end();
    chain2_begin = chain1_.rbegin();
    chain2_end = chain1_.rend();
  }
}

}  // walnut

#endif // WALNUT_MONOTONE_RANGE_H__
