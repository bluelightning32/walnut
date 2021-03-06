#ifndef WALNUT_BSP_VISITOR_H__
#define WALNUT_BSP_VISITOR_H__

#include <deque>
#include <unordered_map>

namespace walnut {

// Filters out all polygons except for the ones that have the requested id.
//
// All points with a PWN greater than or equal to 1 for the given id are
// considered as inside the shape. In practice this means that only the
// polygons for the transition from 0 to 1 for the given id are considered part
// of the border that is accepted.
class PolygonFilter {
 public:
  PolygonFilter(BSPContentId accept_id) : accept_id_(accept_id) { }

  // Specifically the first returned bool is true if the PWN is inside the
  // polyhedron defined by this visitor. The second returned bool is true if
  // the sideness could change as subnodes of this branch of the BSP tree is
  // visited, given which polygons have not been visited yet in this subtree
  // (the `has_polygons` field).
  std::pair<bool, bool> operator()(
      const std::vector<BSPContentInfo>& content_info_by_id) {
    if (accept_id_ >= content_info_by_id.size()) {
      return std::make_pair(false, false);
    } else {
      return std::make_pair(content_info_by_id[accept_id_].pwn > 0,
                            content_info_by_id[accept_id_].has_polygons());
    }
  }

 private:
  BSPContentId accept_id_;
};

// Filters out all polygons except for the ones that have the requested id.
//
// All points with a PWN greater than or equal to 0 for the given id are
// considered as inside the shape. In practice this means that only the
// polygons for the transition from -1 to 0 for the given id are considered part
// of the border that is accepted.
class InvertedPolygonFilter {
 public:
  InvertedPolygonFilter(BSPContentId accept_id) : accept_id_(accept_id) { }

  // Specifically the first returned bool is true if the PWN is inside the
  // polyhedron defined by this visitor. The second returned bool is true if
  // the sideness could change as subnodes of this branch of the BSP tree is
  // visited, given which polygons have not been visited yet in this subtree
  // (the `has_polygons` field).
  std::pair<bool, bool> operator()(
      const std::vector<BSPContentInfo>& content_info_by_id) {
    if (accept_id_ >= content_info_by_id.size()) {
      return std::make_pair(false, false);
    } else {
      return std::make_pair(content_info_by_id[accept_id_].pwn >= 0,
                            content_info_by_id[accept_id_].has_polygons());
    }
  }

 private:
  BSPContentId accept_id_;
};

// Filters out all polygons except for the ones that have the requested id.
//
// All points with an odd PWN for the given id are considered as inside the
// shape.
class OddPolygonFilter {
 public:
  OddPolygonFilter(BSPContentId accept_id) : accept_id_(accept_id) { }

  // Specifically the first returned bool is true if the PWN is inside the
  // polyhedron defined by this visitor. The second returned bool is true if
  // the sideness could change as subnodes of this branch of the BSP tree is
  // visited, given which polygons have not been visited yet in this subtree
  // (the `has_polygons` field).
  std::pair<bool, bool> operator()(
      const std::vector<BSPContentInfo>& content_info_by_id) {
    if (accept_id_ >= content_info_by_id.size()) {
      return std::make_pair(false, false);
    } else {
      return std::make_pair(content_info_by_id[accept_id_].pwn & 1,
                            content_info_by_id[accept_id_].has_polygons());
    }
  }

 private:
  BSPContentId accept_id_;
};

template <typename Filter1, typename Filter2>
class IntersectionFilter {
 public:
  IntersectionFilter(Filter1 filter1, Filter2 filter2)
    : filter1_(std::move(filter1)), filter2_(std::move(filter2)) { }

  std::pair<bool, bool> operator()(
      const std::vector<BSPContentInfo>& content_info_by_id) {
    std::pair<bool, bool> result1 = filter1_(content_info_by_id);
    std::pair<bool, bool> result2 = filter2_(content_info_by_id);
    return std::make_pair(result1.first && result2.first,
                          (result1.first || result1.second) &&
                          (result2.first || result2.second) &&
                          (result1.second || result2.second));
  }

 private:
  Filter1 filter1_;
  Filter2 filter2_;
};

template <typename Filter1, typename Filter2>
IntersectionFilter<Filter1, Filter2> MakeIntersectionFilter(Filter1 filter1,
                                                            Filter2 filter2) {
  return IntersectionFilter<Filter1, Filter2>(std::move(filter1),
                                              std::move(filter2));
}

template <typename Filter1, typename Filter2>
class UnionFilter {
 public:
  UnionFilter(Filter1 filter1, Filter2 filter2)
    : filter1_(std::move(filter1)), filter2_(std::move(filter2)) { }

  std::pair<bool, bool> operator()(
      const std::vector<BSPContentInfo>& content_info_by_id) {
    std::pair<bool, bool> result1 = filter1_(content_info_by_id);
    std::pair<bool, bool> result2 = filter2_(content_info_by_id);
    return std::make_pair(result1.first || result2.first,
                          (!result1.first || result1.second) &&
                          (!result2.first || result2.second) &&
                          (result1.second || result2.second));
  }

 private:
  Filter1 filter1_;
  Filter2 filter2_;
};

template <typename Filter1, typename Filter2>
UnionFilter<Filter1, Filter2> MakeUnionFilter(Filter1 filter1,
                                              Filter2 filter2) {
  return UnionFilter<Filter1, Filter2>(std::move(filter1), std::move(filter2));
}

template <typename Filter1, typename Filter2>
class SubtractionFilter {
 public:
  // Filter2 must produce an inverted mesh. For example, to subtract a cube,
  // all of the faces of the inverted cube should be in clockwise order.
  SubtractionFilter(Filter1 filter1, Filter2 filter2)
    : filter1_(std::move(filter1)), filter2_(std::move(filter2)) { }

  std::pair<bool, bool> operator()(
      const std::vector<BSPContentInfo>& content_info_by_id) {
    std::pair<bool, bool> result1 = filter1_(content_info_by_id);
    std::pair<bool, bool> result2 = filter2_(content_info_by_id);
    return std::make_pair(result1.first && result2.first,
                          (result1.first || result1.second) &&
                          (result2.first || result2.second) &&
                          (result1.second || result2.second));
  }

 private:
  Filter1 filter1_;
  Filter2 filter2_;
};

template <typename Filter1, typename Filter2>
SubtractionFilter<Filter1, Filter2> MakeSubtractionFilter(Filter1 filter1,
                                                          Filter2 filter2) {
  return SubtractionFilter<Filter1, Filter2>(std::move(filter1),
	                                         std::move(filter2));
}

template <typename PolygonRepTemplate>
class BSPVisitor {
 public:
  using PolygonRep = PolygonRepTemplate;

  // Returns whether the given PWN is inside the boolean operation defined by
  // this visitor.
  //
  // Specifically the first returned bool is true if the PWN is inside the
  // polyhedron defined by this visitor. The second returned bool is true if
  // the sideness could change as subnodes of this branch of the BSP tree is
  // visited, given which polygons have not been visited yet in this subtree
  // (the `has_polygons` field).
  virtual std::pair<bool, bool> IsInside(
      const std::vector<BSPContentInfo>& content_info_by_id) = 0;

  // The pointer to `split` is guaranteed to be valid until the same reference
  // is given to `LeaveInteriorNode`. The same pointer may appear in
  // `SplitSide` objects in the results. If `from_partitioner` is false, the
  // pointer will be valid past when it is passed to `LeaveInteriorNode`.
  virtual void EnterInteriorNode(bool from_partitioner,
                                 const HalfSpace3& split) { }

  virtual void LeaveInteriorNode(bool from_partitioner,
                                 const HalfSpace3& split) { }

  virtual void EnterLeafNode() { }

  virtual void Accept(const PolygonRep& polygon) {
    Accept(PolygonRep(polygon));
  }

  virtual void Accept(PolygonRep&& polygon) { }
};

template <typename Filter1, typename Filter2>
class XORFilter {
 public:
  XORFilter(Filter1 filter1, Filter2 filter2)
    : filter1_(std::move(filter1)), filter2_(std::move(filter2)) { }

  std::pair<bool, bool> operator()(
      const std::vector<BSPContentInfo>& content_info_by_id) {
    std::pair<bool, bool> result1 = filter1_(content_info_by_id);
    std::pair<bool, bool> result2 = filter2_(content_info_by_id);
    return std::make_pair(result1.first ^ result2.first,
                          result1.second || result2.second);
  }

 private:
  Filter1 filter1_;
  Filter2 filter2_;
};

template <typename Filter1, typename Filter2>
XORFilter<Filter1, Filter2> MakeXORFilter(Filter1 filter1, Filter2 filter2) {
  return XORFilter<Filter1, Filter2>(std::move(filter1), std::move(filter2));
}

template <typename PolygonRep, typename FilterTemplate>
class CollectorVisitor : public BSPVisitor<PolygonRep> {
 public:
  using Filter = FilterTemplate;
  using Parent = BSPVisitor<PolygonRep>;

  explicit CollectorVisitor(Filter filter) : filter_(std::move(filter)) { }

  std::pair<bool, bool> IsInside(
      const std::vector<BSPContentInfo>& content_info_by_id) override {
    return filter_(content_info_by_id);
  }

  const std::vector<PolygonRep>& polygons() const {
    return polygons_;
  }

  std::vector<PolygonRep>& polygons() {
    return polygons_;
  }

  std::vector<PolygonRep> TakePolygons(std::deque<HalfSpace3>& split_planes) {
    std::vector<PolygonRep> result(std::move(polygons_));
    polygons_.clear();
    split_planes = std::move(split_planes_);
    split_planes_.clear();
    assert(split_plane_map_.empty());
    return result;
  }

  void Accept(PolygonRep&& polygon) override {
    for (size_t i = 0; i < polygon.vertex_count(); ++i) {
      auto& edge = polygon.bsp_edge_info(i);
      auto it = split_plane_map_.find(edge.edge_first_coincident.split);
      if (it != split_plane_map_.end()) {
        edge.edge_first_coincident.split = it->second;
      }

      it = split_plane_map_.find(edge.vertex_last_coincident.split);
      if (it != split_plane_map_.end()) {
        edge.vertex_last_coincident.split = it->second;
      }

      it = split_plane_map_.find(edge.edge_last_coincident.split);
      if (it != split_plane_map_.end()) {
        edge.edge_last_coincident.split = it->second;
      }
    }
    polygons_.push_back(std::move(polygon));
  }

  using Parent::Accept;

  void EnterInteriorNode(bool from_partitioner,
                         const HalfSpace3& split) override {
    if (from_partitioner) {
      split_planes_.push_back(split);
      bool inserted =
        split_plane_map_.insert({&split, &split_planes_.back()}).second;
      // Fix an unused variable warning in release builds.
      (void)(inserted);
      assert(inserted);
    }
  }

  void LeaveInteriorNode(bool from_partitioner,
                         const HalfSpace3& split) override {
    if (from_partitioner) {
      size_t removed = split_plane_map_.erase(&split);
      // Fix an unused variable warning in release builds.
      (void)(removed);
      assert(removed == 1);
    }
  }

 private:
  Filter filter_;
  std::vector<PolygonRep> polygons_;
  std::deque<HalfSpace3> split_planes_;
  std::unordered_map<const HalfSpace3*, HalfSpace3*> split_plane_map_;
};

}  // walnut

#endif // WALNUT_BSP_VISITOR_H__
