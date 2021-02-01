#ifndef WALNUT_CONVEX_POLYGON_VERTEX_ITERATOR_H__
#define WALNUT_CONVEX_POLYGON_VERTEX_ITERATOR_H__

#include <type_traits>

namespace walnut {

// An iterator that produces just the vertex field from an iterator that
// produces a `ConvexPolygonEdge`.
template <typename EdgeIterator>
class ConvexPolygonVertexIterator {
 public:
  using difference_type = typename EdgeIterator::difference_type;
  // The extra set of parenthesis around the argument to decltype are necessary
  // so that the argument is considered an expression instead of an entity.
  using value_type = typename std::remove_reference<decltype((
        std::declval<typename EdgeIterator::value_type>().vertex))>::type;
  using pointer = value_type*;
  using reference = value_type&;
  using iterator_category = typename EdgeIterator::iterator_category;

  ConvexPolygonVertexIterator() = default;

  explicit ConvexPolygonVertexIterator(const EdgeIterator& edge_iterator) :
    edge_iterator_(edge_iterator) { }

  template <typename OtherEdgeIterator,
            typename = std::enable_if_t<
              std::is_constructible<EdgeIterator, OtherEdgeIterator>::value>>
  ConvexPolygonVertexIterator(
      const ConvexPolygonVertexIterator<OtherEdgeIterator>& other) :
    edge_iterator_(other.edge_iterator_) { }

  template <typename OtherEdgeIterator>
  auto operator==(
      const ConvexPolygonVertexIterator<OtherEdgeIterator>& other) const {
    return edge_iterator_ == other.edge_iterator_;
  }

  template <typename OtherEdgeIterator>
  auto operator!=(
      const ConvexPolygonVertexIterator<OtherEdgeIterator>& other) const {
    return edge_iterator_ != other.edge_iterator_;
  }

  reference operator*() const {
    return edge_iterator_->vertex;
  }

  pointer operator->() const {
    return &edge_iterator_->vertex;
  }

  ConvexPolygonVertexIterator& operator++() {
    ++edge_iterator_;
    return *this;
  }

  ConvexPolygonVertexIterator operator++(int) {
    return ConvexPolygonVertexIterator(edge_iterator_++);
  }

  auto& operator--() {
    --edge_iterator_;
    return *this;
  }

  auto operator--(int) {
    return ConvexPolygonVertexIterator(edge_iterator_--);
  }

  auto operator+(difference_type offset) const {
    return ConvexPolygonVertexIterator(edge_iterator_ + offset);
  }

  auto operator-(difference_type offset) const {
    return ConvexPolygonVertexIterator(edge_iterator_ - offset);
  }

  template <typename OtherEdgeIterator>
  auto operator-(
      const ConvexPolygonVertexIterator<OtherEdgeIterator>& other) const {
    return edge_iterator_ - other.edge_iterator_;
  }

  auto& operator+=(difference_type offset) {
    return edge_iterator_ += offset, *this;
  }

  auto& operator-=(difference_type offset) {
    return edge_iterator_ -= offset, *this;
  }

  auto& operator[](difference_type offset) const {
    return edge_iterator_[offset].vertex;
  }

  template <typename OtherEdgeIterator>
  auto operator<(
      const ConvexPolygonVertexIterator<OtherEdgeIterator>& other) const {
    return edge_iterator_ < other.edge_iterator_;
  }

  template <typename OtherEdgeIterator>
  auto operator<=(
      const ConvexPolygonVertexIterator<OtherEdgeIterator>& other) const {
    return edge_iterator_ <= other.edge_iterator_;
  }

  template <typename OtherEdgeIterator>
  auto operator>=(
      const ConvexPolygonVertexIterator<OtherEdgeIterator>& other) const {
    return edge_iterator_ >= other.edge_iterator_;
  }

  template <typename OtherEdgeIterator>
  auto operator>(
      const ConvexPolygonVertexIterator<OtherEdgeIterator>& other) const {
    return edge_iterator_ > other.edge_iterator_;
  }

 private:
  template <typename OtherEdgeIterator>
  friend class ConvexPolygonVertexIterator;
  EdgeIterator edge_iterator_;
};

}  // walnut

#endif // WALNUT_CONVEX_POLYGON_VERTEX_ITERATOR_H__
