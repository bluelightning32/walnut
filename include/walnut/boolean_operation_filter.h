#ifndef WALNUT_BOOLEAN_OPERATION_FILTER_H__
#define WALNUT_BOOLEAN_OPERATION_FILTER_H__

// for std::pair
#include <utility>
#include <vector>

#include "walnut/bsp_content_info.h"
#include "walnut/bsp_polygon.h"

namespace walnut {

class BooleanOperationFilter {
 public:
  virtual std::pair<bool, bool> operator()(
      const std::vector<BSPContentInfo>& content_info_by_id) = 0;
};

// Calculates the union of multiple polyhedrons. A point is considered part of
// one of the unioned polyhedrons if its pwn >= 1.
class UnionIdsFilter : public BooleanOperationFilter {
 public:
  UnionIdsFilter() = default;

  UnionIdsFilter(std::vector<BSPContentId> ids) : ids_(std::move(ids)) { }

  void SetIds(std::vector<BSPContentId> ids) {
    ids_ = std::move(ids);
  }

  std::pair<bool, bool> operator()(
      const std::vector<BSPContentInfo>& content_info_by_id) override {

    std::pair<bool, bool> total(false, false);

    for (BSPContentId id : ids_) {
      std::pair<bool, bool> id_result;

      if (id >= content_info_by_id.size()) {
        id_result = std::make_pair(false, false);
      } else {
        id_result =  std::make_pair(content_info_by_id[id].pwn > 0,
                                    content_info_by_id[id].has_polygons());
      }

      total.second = (!total.first || total.second) &&
                     (!id_result.first || id_result.second) &&
                     (total.second || id_result.second);
      total.first |= id_result.first;
    }

    return total;
  }

 private:
  std::vector<BSPContentId> ids_;
};

// Calculates the intersection of multiple polyhedrons. A point is considered
// part of one of the intersected polyhedrons if its pwn >= 1.
class IntersectIdsFilter : public BooleanOperationFilter {
 public:
  IntersectIdsFilter() = default;

  IntersectIdsFilter(std::vector<BSPContentId> ids) : ids_(std::move(ids)) { }

  void SetIds(std::vector<BSPContentId> ids) {
    ids_ = std::move(ids);
  }

  std::pair<bool, bool> operator()(
      const std::vector<BSPContentInfo>& content_info_by_id) override {

    std::pair<bool, bool> total(true, false);

    for (BSPContentId id : ids_) {
      std::pair<bool, bool> id_result;

      if (id >= content_info_by_id.size()) {
        id_result = std::make_pair(false, false);
      } else {
        id_result =  std::make_pair(content_info_by_id[id].pwn > 0,
                                    content_info_by_id[id].has_polygons());
      }

      total.second = (total.first || total.second) &&
                     (id_result.first || id_result.second) &&
                     (total.second || id_result.second);
      total.first &= id_result.first;
    }

    return total;
  }

 private:
  std::vector<BSPContentId> ids_;
};

// Calculates the difference of the first polyhedron from the remaining
// polyhedrons. The remaining polyhedrons must be flipped. The parts where the
// remaining polyhedrons have pwn < 0 are removed.
class SubtractIdsFilter : public BooleanOperationFilter {
 public:
  SubtractIdsFilter() = default;

  SubtractIdsFilter(std::vector<BSPContentId> ids) : ids_(std::move(ids)) { }

  void SetIds(std::vector<BSPContentId> ids) {
    ids_ = std::move(ids);
  }

  std::pair<bool, bool> operator()(
      const std::vector<BSPContentInfo>& content_info_by_id) override {

    if (ids_.empty()) return std::make_pair(false, false);

    if (ids_[0] >= content_info_by_id.size()) {
      return std::make_pair(false, false);
    } 

    std::pair<bool, bool> total(content_info_by_id[0].pwn > 0,
                                content_info_by_id[0].has_polygons());

    for (size_t i = 1; i < ids_.size(); ++i) {
      BSPContentId id = ids_[i];

      std::pair<bool, bool> id_result;

      if (id >= content_info_by_id.size()) {
        id_result = std::make_pair(false, false);
      } else {
        id_result =  std::make_pair(content_info_by_id[id].pwn >= 0,
                                    content_info_by_id[id].has_polygons());
      }

      total.second = (total.first || total.second) &&
                     (id_result.first || id_result.second) &&
                     (total.second || id_result.second);
      total.first &= id_result.first;
    }

    return total;
  }

 private:
  std::vector<BSPContentId> ids_;
};

}  // walnut

#endif // WALNUT_BOOLEAN_OPERATION_FILTER_H__
