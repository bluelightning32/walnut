#ifndef WALNUT_BSP_NODE_H__
#define WALNUT_BSP_NODE_H__

// for std::unique_ptr
#include <memory>
// for std::pair and std::move
#include <utility>
#include <vector>

#include "walnut/aabb_convex_polygon.h"
#include "walnut/bsp_content_info.h"
#include "walnut/bsp_edge_info.h"
#include "walnut/bsp_polygon.h"
#include "walnut/half_space3.h"
#include "walnut/polygon_event_point.h"
#include "walnut/r_transformation.h"

namespace walnut {

// This is a node within a binary space partition tree.
//
// `OutputPolygonParentTemplate` must inherit from ConvexPolygon.
template <typename OutputPolygonParentTemplate = AABBConvexPolygon<>>
class BSPNode {
 public:
  using OutputPolygonParent = OutputPolygonParentTemplate;
  using PolygonRep = BSPPolygon<OutputPolygonParent>;
  using EdgeParent = typename PolygonRep::EdgeParent;
  using EdgeRep = typename PolygonRep::EdgeRep;
  using BSPEdgeInfoRep = typename PolygonRep::BSPEdgeInfoRep;

  BSPNode() = default;

  // Convert a leaf node into an interior node.
  //
  // This may only be called on a leaf node.
  //
  // The plane of `half_space` must be different from the any of the planes
  // that ancestor nodes were split on.
  //
  // The contents of this node will be pushed into the new child nodes.
  void Split(const HalfSpace3& half_space) {
    Split(HalfSpace3(half_space));
  }

  void Split(HalfSpace3&& half_space) {
    assert(half_space.IsValid());
    MakeInterior(std::move(half_space), new BSPNode(), new BSPNode());
    PushContentsToChildren();
  }

  void Split() {
    assert(!contents_.empty());
    HalfSpace3 half_space = PickSplitPlane();
    assert(half_space.IsValid());
    Split(std::move(half_space));
  }

  // Split this polygon based on `partition` in `dimension`.
  //
  // On input, `event_points` must be the event points from the same call to
  // `GetLowestCost` that produced `partition`.
  //
  // On output, the positive child event points are stored in `event_points`,
  // and the negative child event points are stored in `neg_event_points`.
  //
  // On output, `polygon_index_map` contains the mapping from parent polygon
  // index to child polygon index.
  void Split(const PolygonEventPointPartition& partition, int dimension,
             std::vector<PolygonEventPoint>& event_points,
             std::vector<size_t>& polygon_index_map,
             std::vector<PolygonEventPoint>& neg_event_points) {
    negative_child_ = std::make_unique<BSPNode>();
    positive_child_ = std::make_unique<BSPNode>();
    negative_child_->SetPWN(content_info_by_id_);
    positive_child_->SetPWN(content_info_by_id_);
    split_ = partition.GetSplitPlane(dimension, event_points.data(),
                                     contents_);
    PushContentPWNToChildren();

    polygon_index_map.resize(contents_.size());
    neg_event_points.resize(partition.GetNegEventPointCount());
    partition.ApplyPrimary(dimension, event_points.data(), &split_,
                           contents_, polygon_index_map.data(),
                           neg_event_points.data(), negative_child_->contents_,
                           positive_child_->contents_,
                           negative_child_->border_contents_,
                           positive_child_->border_contents_);
    contents_.shrink_to_fit();

    negative_child_->SetPolygonCountFromContents();
    positive_child_->SetPolygonCountFromContents();

    PushBorderContentsToChildren();
  }

  // Returns a plane that can divide contents_. The chosen plane is guaranteed
  // to produce leaf nodes where at least one polygon from contents_ is added
  // to border_contents_, and/or contents_ is split between the two children.
  //
  // Currently this function picks an entry from contents_ that belongs to the
  // polyhedron with the fewest facets in this node. However, this algorithm
  // could be changed in the future.
  HalfSpace3 PickSplitPlane() const;

  // Given the index of a polygon from contents_, returns a plane with a
  // normal that is a quarter turn away from the input. A distance along that
  // normal is chosen such that about half of the polygons of the same content
  // type are on the same side, or at least straddling.
  //
  // This plane selection tends to better subdivide a large group of polygons,
  // but it also introduces more split polygons.
  //
  // This may only be called if there is more than 1 content polygon with the
  // same id.
  HalfSpace3 PickQuarterTurnSplitPlane(size_t first_content_index) const;

  bool IsLeaf() const {
    return !split().IsValid();
  }

  const HalfSpace3& split() const {
    return split_;
  }

  const BSPNode* negative_child() const {
    return negative_child_.get();
  }
  BSPNode* negative_child() {
    return negative_child_.get();
  }
  const BSPNode* positive_child() const {
    return positive_child_.get();
  }
  BSPNode* positive_child() {
    return positive_child_.get();
  }

  const std::vector<PolygonRep>& contents() const {
    return contents_;
  }

  const std::vector<PolygonRep>& border_contents() const {
    return border_contents_;
  }

  const std::vector<BSPContentInfo>& content_info_by_id() const {
    return content_info_by_id_;
  }

  BSPContentInfo GetContentInfoForId(BSPContentId id) const {
    if (id < content_info_by_id_.size()) {
      return content_info_by_id_[id];
    } else {
      return BSPContentInfo{};
    }
  }

  int64_t GetPWNForId(BSPContentId id) const {
    return GetContentInfoForId(id).pwn;
  }

  // Push the contents of an interior node to the children.
  //
  // This is a no-op for leaves.
  void PushContentsToChildren();

  // Push the contents all the way down to descendant leaf nodes.
  //
  // This may be called on leaf or interior nodes.
  void PushContentsToLeaves();

  // Adds a polygon to a root node.
  //
  // Even if the node is an interior node, the polygon is not pushed to the
  // child nodes. After all of the content is added, `PushContentsToLeaves`
  // must be called afterwards if the node is interior.
  template <typename InputConvexPolygon>
  void AddRootContent(BSPContentId id, InputConvexPolygon&& polygon) {
    contents_.emplace_back(id, /*on_node_plane=*/nullptr, /*pos_side=*/false,
                           std::forward<InputConvexPolygon>(polygon));
    using InputConvexPolygonNoRef =
      typename std::remove_reference<InputConvexPolygon>::type;
    using InputEdgeParent = typename InputConvexPolygonNoRef::EdgeParent;
    if (std::is_base_of<EdgeParent, InputEdgeParent>::value) {
      contents_.back().ResetBSPInfo();
    }

    if (id >= content_info_by_id_.size()) {
      content_info_by_id_.resize(id + 1);
    }
    content_info_by_id_[id].has_interior_polygons++;
  }

  // Determine if a crossing occurs at a vertex.
  //
  // `edge_comparison` should be -1 if the edge boundary angle is clockwise
  // from the split_.normal(), or it should be 1 if it is counter-clockwise.
  // This function should not be called if the vectors are the same.
  //
  // The first field of the returned value is 1 if an entrance crossing
  // occurred, 0 if no crossing occurred, or -1 if an exit crossing occurred.
  // The entrance/exit status is flipped if `vertex_edge` refers to the vertex
  // after `edge_comparison`, instead of before.
  //
  // The second field of the returned value is true if the PWN of the positive
  // child should be updated, or it is false if the negative child's PWN should
  // be updated.
  std::pair<int, bool> GetPWNEffectAtVertex(
      int edge_comparison, const SplitSide& edge_last_coincident,
      const EdgeRep& vertex_edge) const;

  // If the node has ids for more than one mesh, the most populous mesh is
  // returned along with its count.
  std::pair<BSPContentId, size_t> GetCostExclude() const {
    std::pair<BSPContentId, size_t> best(-1, 0);
    size_t count = 0;
    for (size_t id = 0; id < content_info_by_id_.size(); ++id) {
      if (content_info_by_id_[id].has_interior_polygons) {
        ++count;

        if (content_info_by_id_[id].has_interior_polygons > best.second) {
          best.first = id;
          best.second = content_info_by_id_[id].has_interior_polygons;
        }
      }
    }
    if (count > 1) {
      return best;
    } else {
      return std::pair<BSPContentId, size_t>(-1, 0);
    }
  }

 protected:
  void MakeInterior(HalfSpace3&& half_space, BSPNode* negative_child,
                    BSPNode* positive_child) {
    assert(IsLeaf());
    split_ = std::move(half_space);
    negative_child_ = std::unique_ptr<BSPNode>(negative_child);
    positive_child_ = std::unique_ptr<BSPNode>(positive_child);

    negative_child->SetPWN(content_info_by_id_);
    positive_child->SetPWN(content_info_by_id_);
  }

  void Reset() {
    contents_.clear();
    border_contents_.clear();
    split_ = HalfSpace3();
    negative_child_.reset();
    positive_child_.reset();
    content_info_by_id_.clear();
  }

 private:
  template <typename OutputPolygonParent>
  friend class BSPTree;

  // Copies only the `pwn` field from `content_info_by_id`, and clears
  // `has_interior_polygons` and `has_border_polygons`.
  void SetPWN(const std::vector<BSPContentInfo>& content_info_by_id) {
    content_info_by_id_.resize(content_info_by_id.size());
    for (size_t i = 0; i < content_info_by_id.size(); ++i) {
      content_info_by_id_[i].has_interior_polygons = 0;
      content_info_by_id_[i].has_border_polygons = 0;
      content_info_by_id_[i].pwn = content_info_by_id[i].pwn;
    }
  }

  void SetPolygonCountFromContents() {
    for (PolygonRep& polygon : contents_) {
      content_info_by_id_[polygon.id].has_interior_polygons++;
    }
    for (PolygonRep& polygon : border_contents_) {
      content_info_by_id_[polygon.id].has_border_polygons++;
    }
  }

  // Update the children's PWN based on this node's contents.
  //
  // This may only be called on an interior node.
  void PushContentPWNToChildren();

  // Push the border_contents_ of an interior node to the children.
  void PushBorderContentsToChildren();

  // For a leaf node, these are the polygons that are inside the cell. They may
  // possibly be touching the cell border, but they are not on the border.
  //
  // For a finished interior node, this will be empty. When new contents are
  // added to the tree, this will be temporarily non-empty, until the new
  // contents are psuhed to the children.
  std::vector<PolygonRep> contents_;

  // For a leaf node, these are the polygons that are on the cell border.
  //
  // For a finished interior node, this will be empty. When new contents are
  // added to the tree, this will be temporarily non-empty, until the new
  // contents are psuhed to the children.
  std::vector<PolygonRep> border_contents_;

  // The plane that splits an interior node.
  //
  // For a leaf node, !split_.IsValid().
  HalfSpace3 split_;

  std::unique_ptr<BSPNode> negative_child_;
  std::unique_ptr<BSPNode> positive_child_;

  std::vector<BSPContentInfo> content_info_by_id_;
};

template <typename OutputPolygonParent>
HalfSpace3 BSPNode<OutputPolygonParent>::PickSplitPlane() const {
  assert(IsLeaf());
  if (contents_.empty()) {
    return HalfSpace3();
  }

  // Search for the polygon with the lowest count. If there are multiple
  // polygons with the same low count, pick the one closest to the middle. The
  // search starts from the middle and goes outward (alternating directions),
  // so picking the middle most one means picking the first polygon found with
  // that low count.
  size_t mid = contents_.size() / 2;
  const PolygonRep* split = &contents_[mid];
  if (contents_.size() == 1) {
    return split->plane();
  }
  size_t offset = 1;
  auto consider_split = [this, &split](const PolygonRep &polygon) {
    if (content_info_by_id_[polygon.id].has_interior_polygons <
        content_info_by_id_[split->id].has_interior_polygons) {
      split = &polygon;
    }
  };
  while (offset < mid) {
    consider_split(contents_[mid + offset]);
    consider_split(contents_[mid - offset - 1]);
    ++offset;
  }
  if (contents_.size() & 1) {
    consider_split(contents_[mid + offset]);
  }
  if (content_info_by_id_[split->id].has_interior_polygons > 40) {
    // There are enough polygons remaining that it is worth trying to more
    // evenly divide them, at the cost of creating more split polygons.
    return PickQuarterTurnSplitPlane(split - contents_.data());
  }
  return split->plane();
}

template <typename OutputPolygonParent>
HalfSpace3 BSPNode<OutputPolygonParent>::PickQuarterTurnSplitPlane(
    size_t first_content_index) const {
  const PolygonRep& seed = contents_[first_content_index];
  const PolygonRep* second_seed;
  for (size_t i = 0; ; ++i) {
    if (i == contents_.size() - 1) {
      // All of the polygons for this content id are on the same plane.
      return seed.plane();
    }
    second_seed = &contents_[(first_content_index + i + 1) % contents_.size()];
    if (second_seed->id == seed.id &&
        !second_seed->plane().normal().IsSameOrOppositeDir(
          seed.plane().normal())) {
      break;
    }
  }
  const Vector3& a = seed.plane().normal();
  const Vector3& b = second_seed->plane().normal();
  Vector3 turned = a.Scale(a.Dot(b)) - b.Scale(a.Dot(a));
  turned.Reduce();

  std::vector<std::pair<BigInt, const BigInt*>> distances;

  for (const PolygonRep& content : contents_) {
    if (content.id == seed.id) {
      assert(content.vertex_count() >= 3);
      distances.emplace_back(
          turned.Dot(content.vertex(0).vector_from_origin()),
          &content.vertex(0).w());
    }
  }
  assert(distances.size() >= 1);

  auto compare = [](const std::pair<BigInt, const BigInt*>& a,
                    const std::pair<BigInt, const BigInt*>& b) {
    return rational::IsLessThan(/*num1=*/a.first, /*denom1=*/*a.second,
                                /*num2=*/b.first, /*denom2=*/*b.second);
  };
  auto middle = distances.begin() + distances.size()/2;

  auto equals_middle = [&middle](const std::pair<BigInt, const BigInt*>& a) {
    return rational::Equals(/*num1=*/a.first, /*denom1=*/*a.second,
                            /*num2=*/middle->first,
                            /*denom2=*/*middle->second);
  };

  std::nth_element(distances.begin(), middle, distances.end(), compare);
  if (std::all_of(distances.begin(), middle, equals_middle)) {
    // The middle is really a minimum element. Give up.
    return seed.plane();
  }
  if (std::all_of(middle + 1, distances.end(), equals_middle)) {
    // The middle is really a maximum element. Give up.
    return seed.plane();
  }

  Vector3 scaled_vector = turned.Scale(*middle->second);
  if (middle->second->SignExtension() < 0) {
    middle->first.Negate();
    scaled_vector.Negate();
  }
  HalfSpace3 plane(std::move(scaled_vector), std::move(middle->first));
  plane.Reduce();
  return plane;
}

template <typename OutputPolygonParent>
std::pair<int, bool> BSPNode<OutputPolygonParent>::GetPWNEffectAtVertex(
    int edge_comparison, const SplitSide& edge_last_coincident,
    const EdgeRep& vertex_edge) const {
  const SplitSide& vertex_last_coincident =
    vertex_edge.vertex_last_coincident;
  const int vertex_comparison = RXYCompareBivector(split_.normal(),
                                                   vertex_last_coincident);

  // For a crossing at the vertex, the edge boundary angle and the
  // vertex angle must be on opposite sides of the split normal. That
  // means that `edge_comparison` and `vertex_comparison` will have
  // different signs.
  if (edge_comparison + vertex_comparison == 0) {
    assert((edge_comparison < 0 && vertex_comparison > 0) ||
           (edge_comparison > 0 && vertex_comparison < 0));
    const int vertex_edge_side_mult =
      (vertex_last_coincident.pos_side ^ edge_last_coincident.pos_side) ?
      -1 : 1;
    const int vertex_to_edge =
      RXYCompareBivector(vertex_last_coincident.split->normal(),
                         edge_last_coincident.split->normal()) *
      vertex_edge_side_mult;
    // Since the vertex boundary angle and edge boundary angle are on
    // opposite sides of the split normal,
    //   vertex_boundary_angle != edge_boundary_angle
    //   Compare(vertex_boundary_angle, edge_boundary_angle) != 0
    assert(vertex_to_edge != 0);
    int side_comparison = vertex_comparison * vertex_to_edge;
    if (side_comparison >= 0) {
      assert (side_comparison == 1);
      // The split normal to vertex rotation is the same direction as
      // the vertex to current edge rotation.
      assert((vertex_comparison >= 0) == (vertex_to_edge >= 0));
      assert((edge_comparison >= 0) != (vertex_to_edge >= 0));
    } else {
      assert (side_comparison == -1);
      // The split normal to vertex rotation is the opposite direction
      // as the vertex to current edge rotation.
      assert((vertex_comparison >= 0) != (vertex_to_edge >= 0));
      assert((edge_comparison >= 0) == (vertex_to_edge >= 0));
    }
    assert (vertex_to_edge == 1 || vertex_to_edge == -1);
    int side_comparison2 = split_.Compare(vertex_edge.vertex());
    if (side_comparison * side_comparison2 >= 0) {
      // min_max_comparison will be positive if the following vectors are
      // arranged in counter-clockwise order:
      // * vertex_boundary_angle
      // * edge_boundary_angle
      // * split_.normal()
      //
      BigIntWord min_max_comparison = split_.normal().Dot(
          vertex_last_coincident.split->normal().Cross(
            edge_last_coincident.split->normal())).GetSign();
      // * Starting the list of vectors at a different offset does not
      //   affect the sign.
      // * Negating split_.normal() negates the sign.
      // * swapping vertex_boundary_angle and edge_boundary_angle
      //   negates the sign.
      //
      // split_.normal() is negated if (vertex_comparison ^
      // vertex_to_edge) < 0.
      //
      // vertex_boundary_angle and edge_boundary_angle are swapped if
      // vertex_to_edge < 0.
      //
      // So the adjusted sign of min_max_comparison is:
      //   min_max_comparison ^ (vertex_comparison ^
      //                         vertex_to_edge) ^ vertex_to_edge
      // = min_max_comparison ^ vertex_comparison
      if ((min_max_comparison ^ vertex_comparison ^
           vertex_edge_side_mult) >= 0) {
        return std::make_pair(vertex_to_edge, side_comparison == -1);
      }
    }
  }
  return std::make_pair(0, false);
}

template <typename OutputPolygonParent>
void BSPNode<OutputPolygonParent>::PushContentPWNToChildren() {
  assert(split_.IsValid());
  for (PolygonRep& polygon : contents_) {
    for (size_t i = 0; i < polygon.vertex_count(); ++i) {
      const EdgeRep& current_edge = polygon.const_edge(i);

      const SplitSide& edge_last_coincident =
        current_edge.edge_last_coincident;
      if (edge_last_coincident.split == nullptr) continue;
      int edge_comparison = RXYCompareBivector(split_.normal(),
                                               edge_last_coincident);
      if (edge_comparison == 0) continue;

      std::pair<int, bool> push_info =
        GetPWNEffectAtVertex(edge_comparison, edge_last_coincident,
                             current_edge);
      if (push_info.first != 0) {
        BSPNode<OutputPolygonParent>* push_to_child =
          push_info.second ? positive_child() : negative_child();
        assert(push_to_child->content_info_by_id_.size() >= polygon.id);
        push_to_child->content_info_by_id_[polygon.id].pwn +=
          push_info.first;
      }

      const EdgeRep& next_edge =
        polygon.const_edge((i + 1)%polygon.vertex_count());
      push_info = GetPWNEffectAtVertex(edge_comparison, edge_last_coincident,
                                       next_edge);
      if (push_info.first != 0) {
        BSPNode<OutputPolygonParent>* push_to_child =
          push_info.second ? positive_child() : negative_child();
        assert(push_to_child->content_info_by_id_.size() >= polygon.id);
        push_to_child->content_info_by_id_[polygon.id].pwn +=
          -push_info.first;
      }
    }
  }
}

template <typename OutputPolygonParent>
void BSPNode<OutputPolygonParent>::PushBorderContentsToChildren() {
  assert(!IsLeaf());

  for (PolygonRep& polygon : border_contents_) {
    assert(polygon.vertex_count() > 0);
    ConvexPolygonSplitInfo info = polygon.GetSplitInfo(split_);

    if (info.ShouldEmitNegativeChild()) {
      if (info.ShouldEmitPositiveChild()) {
        std::pair<PolygonRep, PolygonRep> children =
          std::move(polygon).CreateSplitChildren(std::move(info));
        PolygonRep::SetChildBoundaryAngles(children, split_);
        negative_child_->content_info_by_id_[polygon.id].has_border_polygons++;
        negative_child_->border_contents_.push_back(std::move(children.first));
        positive_child_->content_info_by_id_[polygon.id].has_border_polygons++;
        positive_child_->border_contents_.push_back(
            std::move(children.second));
      } else {
        polygon.SetBoundaryAngles(SplitSide{&split_, /*pos_child=*/false},
                                  /*exclude_range=*/info.neg_range());
        negative_child_->content_info_by_id_[polygon.id].has_border_polygons++;
        negative_child_->border_contents_.push_back(std::move(polygon));
      }
    } else if (info.ShouldEmitPositiveChild()) {
      polygon.SetBoundaryAngles(SplitSide{&split_, /*pos_child=*/true},
                                /*exclude_range=*/info.pos_range());
      positive_child_->content_info_by_id_[polygon.id].has_border_polygons++;
      positive_child_->border_contents_.push_back(std::move(polygon));
    } else {
      assert(info.ShouldEmitOnPlane());
      // This branch only runs if this node is being split on the same plane as
      // an ancestor node, which should not happen.
      assert(false);
      // If polygon.plane().normal() and split_.normal() point in the same
      // direction, put polygon in the negative child.
      const int drop_dimension = polygon.drop_dimension();
      bool pos_child =
        polygon.plane().normal().components()[drop_dimension].HasDifferentSign(
            split_.normal().components()[drop_dimension]);
      polygon.SetBoundaryAngles(SplitSide{&split_, pos_child},
                                /*coincident_begin=*/0,
                                /*coincident_end=*/polygon.vertex_count() + 1);
      polygon.on_node_plane.split = &split_;
      polygon.on_node_plane.pos_side = pos_child;
      if (pos_child) {
        positive_child_->content_info_by_id_[polygon.id].has_border_polygons++;
        positive_child_->border_contents_.push_back(std::move(polygon));
      } else {
        negative_child_->content_info_by_id_[polygon.id].has_border_polygons++;
        negative_child_->border_contents_.push_back(std::move(polygon));
      }
    }
  }
  border_contents_.clear();
  border_contents_.shrink_to_fit();
}

template <typename OutputPolygonParent>
void BSPNode<OutputPolygonParent>::PushContentsToChildren() {
  if (IsLeaf()) {
    return;
  }
  PushContentPWNToChildren();

  for (PolygonRep& polygon : contents_) {
    assert(polygon.vertex_count() > 0);
    ConvexPolygonSplitInfo info = polygon.GetSplitInfo(split_);
    assert(info.IsValid(polygon.vertex_count()));

    if (info.ShouldEmitNegativeChild()) {
      if (info.ShouldEmitPositiveChild()) {
        std::pair<PolygonRep, PolygonRep> children =
          std::move(polygon).CreateSplitChildren(std::move(info));
        PolygonRep::SetChildBoundaryAngles(children, split_);

        negative_child_->content_info_by_id_[polygon.id].
          has_interior_polygons++;
        negative_child_->contents_.push_back(std::move(children.first));
        positive_child_->content_info_by_id_[polygon.id].
          has_interior_polygons++;
        positive_child_->contents_.push_back(std::move(children.second));
      } else {
        polygon.SetBoundaryAngles(SplitSide{&split_, /*pos_child=*/false},
                                  /*exclude_range=*/info.neg_range());
        negative_child_->content_info_by_id_[polygon.id].
          has_interior_polygons++;
        negative_child_->contents_.push_back(std::move(polygon));
      }
    } else if (info.ShouldEmitPositiveChild()) {
      polygon.SetBoundaryAngles(SplitSide{&split_, /*pos_child=*/true},
                                /*exclude_range=*/info.pos_range());
      positive_child_->content_info_by_id_[polygon.id].has_interior_polygons++;
      positive_child_->contents_.push_back(std::move(polygon));
    } else {
      assert(info.ShouldEmitOnPlane());
      // If polygon.plane().normal() and split_.normal() point in the same
      // direction, put polygon in the negative child.
      const int drop_dimension = polygon.drop_dimension();
      bool pos_child =
        polygon.plane().normal().components()[drop_dimension].HasDifferentSign(
            split_.normal().components()[drop_dimension]);
      polygon.SetBoundaryAngles(SplitSide{&split_, pos_child},
                                /*coincident_begin=*/0,
                                /*coincident_end=*/polygon.vertex_count() + 1);
      if (pos_child) {
        positive_child_->content_info_by_id_[polygon.id].has_border_polygons++;
        positive_child_->border_contents_.emplace_back(&split_,
                                                       /*pos_side=*/true,
                                                       std::move(polygon));
      } else {
        negative_child_->content_info_by_id_[polygon.id].has_border_polygons++;
        negative_child_->border_contents_.emplace_back(&split_,
                                                       /*pos_side=*/false,
                                                       std::move(polygon));
      }
    }
  }
  contents_.clear();
  contents_.shrink_to_fit();

  PushBorderContentsToChildren();
}

template <typename OutputPolygonParent>
void BSPNode<OutputPolygonParent>::PushContentsToLeaves() {
  if (contents_.empty() && border_contents_.empty()) {
    return;
  }
  if (!IsLeaf()) {
    PushContentsToChildren();
    negative_child_->PushContentsToLeaves();
    positive_child_->PushContentsToLeaves();
  }
}

}  // walnut

#endif // WALNUT_BSP_NODE_H__
