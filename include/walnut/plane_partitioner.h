#ifndef WALNUT_PLANE_PARTITIONER_H__
#define WALNUT_PLANE_PARTITIONER_H__

#include <iterator>
#include <stack>

#include "walnut/bsp_content_info.h"
#include "walnut/bsp_polygon.h"
#include "walnut/bsp_visitor.h"

namespace walnut {

template <typename PolygonTemplate = BSPPolygon<>>
class PlanePartitioner {
 public:
  using Polygon = PolygonTemplate;
  using BSPPolygonBase = typename Polygon::BSPPolygonRep;
  static_assert(
      std::is_base_of<BSPPolygon<typename BSPPolygonBase::UnspecializedParent,
                                 typename BSPPolygonBase::EdgeParent>,
                      Polygon>::value,
      "Polygon must inherit from BSPPolygon.");
  using EdgeRep = typename Polygon::EdgeRep;
  using ContentInfo = std::vector<BSPContentInfo>;

  // Performs a 2D binary space partition on the input range of 3D polygons,
  // and outputs the result to `visitor`.
  //
  // `drop_dimension` refers to the dimension of the input polygons that should
  // be dropped in order to make them 2D. Every input polygon must have a
  // normal vector with a non-zero `drop_dimension` index component.
  //
  // If `pos_normal` is true, then any polygons that are encountered with a
  // normal that's positive in the `drop_dimension` component will be
  // considered as having the inside of their plane on the inside of the BSP
  // cell. If `pos_normal` is false, then the polygons with a negative normal
  // are considered as having their inside pointed towards the inside of the
  // cell. Polygons with a normal in the opposite direction are considered as
  // having their outside point towards the inside of the cell.
  //
  // `initial_content_info` should describe the PWN of the inside of the cell.
  // The input polygons are like stickers on the inside of the cell. The
  // internal content info is updated as the stickers are peeled off the inside
  // of the cell. That is, given the PWN of the inside of the cell, the
  // function incrementally calculates the PWN of the outside of the cell, in
  // order to determine which input polygons should be sent to the visitor.
  //
  // The caller must also ensure that the `has_border_polygons` field of
  // `initial_content_info` is true of any polygon id in the input range.
  //
  // At leaf nodes, `visitor` is given any polygon where one side is inside of
  // the set defined by the `visitor` and the other side is outside of the
  // set.
  //
  // The BSP will search the polygons for edges where the
  // `edge_first_coincident_` is not set. All such discovered edges will be
  // converted into a 3D half-space and the polygons will be split by that
  // half-space.
  template <typename Iterator>
  void Run(Iterator begin, Iterator end, int drop_dimension, bool pos_normal,
           const ContentInfo& initial_content_info,
           BSPVisitor<Polygon>& visitor) {
    using InputPolygon = typename std::iterator_traits<Iterator>::value_type;
    using InputEdge = typename InputPolygon::EdgeRep;
    assert(ContentInfoMatches(begin, end, initial_content_info));
    std::pair<bool, bool> match = visitor.IsInside(initial_content_info);
    if (!match.second) {
      // Only polygons that change the sideness should be emitted. The sideness
      // is fixed for this set of polygons, which means that none of them will
      // be emitted.
      return;
    }
    Iterator pos = begin;
    const InputEdge* split_edge = nullptr;
    while (pos != end) {
      split_edge = SearchForSplitEdge(*pos);
      if (split_edge != nullptr) {
        break;
      }
      ++pos;
    }
    current_content_info_ = initial_content_info;
    if (split_edge == nullptr) {
      ProcessLeaf(begin, end, drop_dimension, pos_normal, match, visitor);
      return;
    }

    HalfSpace3 split_plane(split_edge->line().Project2D(drop_dimension),
                           /*add_dimension=*/drop_dimension);
    visitor.EnterInteriorNode(/*from_partitioner=*/true, split_plane);

    PolygonVector neg_children = AllocatePolygonVector();
    PolygonVector pos_children = AllocatePolygonVector();
    PushToChildren(begin, end, split_plane, neg_children, pos_children);

    SetCurrentContentInfoPolygons(neg_children);
    Run(std::move(neg_children), drop_dimension, pos_normal,
        initial_content_info, visitor);
    SetCurrentContentInfoPolygons(pos_children);
    Run(std::move(pos_children), drop_dimension, pos_normal,
        initial_content_info, visitor);

    visitor.LeaveInteriorNode(/*from_partitioner=*/true, split_plane);
  }

  // Returns true if `has_border_polygons` is true in the content_info for
  // every polygon in the given range.
  template <typename Iterator>
  static bool ContentInfoMatches(Iterator begin, Iterator end,
                                 const ContentInfo& content_info) {
    while (begin != end) {
      if (begin->id >= content_info.size()) return false;
      if (!content_info[begin->id].has_border_polygons) return false;
      ++begin;
    }
    return true;
  }

 private:
  using PolygonVector = std::vector<Polygon>;

  // Returns the first edge of `polygon` where `edge_first_coincident_` equals
  // `edge_last_coincident`, or nullptr if there is no such edge.
  //
  // `polygon` is expected to be on a split plane already, so all of its edges
  // will be coincident with at least 1 BSP split. However, if the edge is not
  // at the corner of two BSP splits, then its first and last coincident BSPs
  // will be the same.
  template <typename InputPolygon>
  const typename InputPolygon::EdgeRep*
  SearchForSplitEdge(const InputPolygon& polygon) {
    for (size_t i = 0; i < polygon.vertex_count(); ++i) {
      if (polygon.edge(i).edge_first_coincident.split ==
          polygon.edge(i).edge_last_coincident.split) {
        return &polygon.edge(i);
      }
    }
    return nullptr;
  }

  // Does the same thing as the public Run, but it takes a vector of input
  // polygons instead of an iterator range.
  void Run(PolygonVector&& polygons, int drop_dimension, bool pos_normal,
           const ContentInfo& initial_content_info,
           BSPVisitor<Polygon>& visitor) {
    std::pair<bool, bool> match = visitor.IsInside(current_content_info_);
    if (!match.second) {
      // Only polygons that change the sideness should be emitted. The sideness
      // is fixed for this set of polygons, which means that none of them will
      // be emitted.
      ReturnPolygonVector(std::move(polygons));
      return;
    }
    const EdgeRep* split_edge = nullptr;
    size_t mid = polygons.size() / 2;
    size_t offset = 0;
    // Search for an edge that needs to be split, starting from the polygon in
    // the middle of `polygons` and working towards the ends of the vector,
    // alternating directions.
    while (true) {
      if (offset == mid) {
        if (polygons.size() & 1) {
          split_edge = SearchForSplitEdge(polygons[mid + offset]);
        }
        break;
      }
      split_edge = SearchForSplitEdge(polygons[mid + offset]);
      if (split_edge != nullptr) {
        break;
      }
      split_edge = SearchForSplitEdge(polygons[mid - offset - 1]);
      if (split_edge != nullptr) {
        break;
      }
      ++offset;
    }
    if (split_edge == nullptr) {
      ProcessLeaf(std::make_move_iterator(polygons.begin()),
                  std::make_move_iterator(polygons.end()), drop_dimension,
                  pos_normal, match, visitor);
      RestoreCurrentContentInfoPWN(initial_content_info);
      ReturnPolygonVector(std::move(polygons));
      return;
    }

    HalfSpace3 split_plane(split_edge->line().Project2D(drop_dimension),
                           /*add_dimension=*/drop_dimension);
    visitor.EnterInteriorNode(/*from_partitioner=*/true, split_plane);

    PolygonVector neg_children = AllocatePolygonVector();
    PolygonVector pos_children = AllocatePolygonVector();
    PushToChildren(std::make_move_iterator(polygons.begin()),
                   std::make_move_iterator(polygons.end()), split_plane,
                   neg_children, pos_children);
    ReturnPolygonVector(std::move(polygons));

    SetCurrentContentInfoPolygons(neg_children);
    Run(std::move(neg_children), drop_dimension, pos_normal,
        initial_content_info, visitor);
    SetCurrentContentInfoPolygons(pos_children);
    Run(std::move(pos_children), drop_dimension, pos_normal,
        initial_content_info, visitor);

    visitor.LeaveInteriorNode(/*from_partitioner=*/true, split_plane);
  }

  // Splits the input range along `split_plane` and put the result in
  // `neg_children` and `pos_children`.
  template <typename Iterator>
  static void PushToChildren(Iterator begin, Iterator end,
                             const HalfSpace3& split_plane,
                             PolygonVector& neg_children,
                             PolygonVector& pos_children) {
    while (begin != end) {
      ConvexPolygonSplitInfo split_info = begin->GetSplitInfo(split_plane);
      assert(!split_info.ShouldEmitOnPlane());
      if (split_info.ShouldEmitNegativeChild()) {
        if (split_info.ShouldEmitPositiveChild()) {
          auto children = begin->CreateSplitChildren(std::move(split_info));
          Polygon::SetChildBoundaryAngles(children, split_plane);
          neg_children.emplace_back(std::move(children.first));
          pos_children.emplace_back(std::move(children.second));
        } else {
          neg_children.emplace_back(*begin);
          neg_children.back().SetBoundaryAngles(
              SplitSide{&split_plane, /*pos_child=*/false},
              /*exclude_range=*/split_info.neg_range());
        }
      } else {
        pos_children.emplace_back(*begin);
        pos_children.back().SetBoundaryAngles(
            SplitSide{&split_plane, /*pos_child=*/true},
            /*exclude_range=*/split_info.pos_range());
      }
      ++begin;
    }
  }

  // Copies the `pwn` field from `initial_content_info` into
  // `current_content_info_`, but leaves the `has_border_polygons` field alone.
  void RestoreCurrentContentInfoPWN(const ContentInfo& initial_content_info) {
    assert(current_content_info_.size() == initial_content_info.size());
    for (size_t i = 0; i < current_content_info_.size(); ++i) {
      current_content_info_[i].pwn = initial_content_info[i].pwn;
    }
  }

  // Sets the `has_border_polygons` field to true iff. `polygons` contains one
  // or more entries with that id.
  void SetCurrentContentInfoPolygons(const PolygonVector& polygons) {
    for (size_t i = 0; i < current_content_info_.size(); ++i) {
      assert(!current_content_info_[i].has_interior_polygons);
      current_content_info_[i].has_border_polygons = false;
    }
    for (const Polygon& polygon : polygons) {
      current_content_info_[polygon.id].has_border_polygons = true;
    }
  }

  template <typename InputPolygon>
  static Polygon ConvertPolygon(const InputPolygon& input) {
    return Polygon(input);
  }

  static Polygon&& ConvertPolygon(Polygon&& input) {
    return std::move(input);
  }

  template <typename Iterator>
  void ProcessLeaf(Iterator begin, Iterator end, int drop_dimension,
                   bool pos_normal, std::pair<bool, bool> inner_match,
                   BSPVisitor<Polygon>& visitor) {
    visitor.EnterLeafNode();
    int flip = pos_normal ? 1 : -1;
    while (begin != end) {
      assert(!begin->normal().components()[drop_dimension].IsZero());
      // `current_content_info_` reflects the inside of the cell. If the
      // polygons are stickers that are applied on the inside of the cell, then
      // `current_content_info_` accounts for all of the remaining stickers
      // applied. Now one of the stickers will be peeled off the inside of the
      // cell.
      int pwn_change;
      if ((begin->normal().components()[drop_dimension].GetSign() ^
           flip) < 0) {
        pwn_change = -1;
      } else {
        pwn_change = 1;
      }
      current_content_info_[begin->id].pwn += pwn_change;
      std::pair<bool, bool> peeled_match =
        visitor.IsInside(current_content_info_);
      if (peeled_match.first != inner_match.first) {
        visitor.Accept(ConvertPolygon(*begin));
      }
      inner_match = peeled_match;
      ++begin;
    }
  }

  // Allocates a PolygonVector from `polygon_vector_freelist_`.
  //
  // The benefit vectors from the freelist is that they often come with
  // preallocated capacity.
  PolygonVector AllocatePolygonVector() {
    if (polygon_vector_freelist_.empty()) {
      return PolygonVector();
    }
    PolygonVector result(std::move(polygon_vector_freelist_.top()));
    polygon_vector_freelist_.pop();
    return result;
  }

  // Returns a PolygonVector to `polygon_vector_freelist_`.
  void ReturnPolygonVector(PolygonVector&& v) {
    v.clear();
    polygon_vector_freelist_.push(std::move(v));
  }

  std::stack<PolygonVector> polygon_vector_freelist_;
  ContentInfo current_content_info_;
};

}  // walnut

#endif // WALNUT_PLANE_PARTITIONER_H__
