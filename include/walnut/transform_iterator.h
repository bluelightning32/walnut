#ifndef WALNUT_TRANSFORM_ITERATOR_H__
#define WALNUT_TRANSFORM_ITERATOR_H__

#include <type_traits>
// For std::move
#include <utility>

namespace walnut {

// An iterator that produces a reference to a subvalue of an aggregate type,
// using a user provided function.
template <typename InputIterator, typename Transformer>
class TransformIterator {
 public:
  using difference_type = typename InputIterator::difference_type;
  // The extra set of parenthesis around the argument to decltype are necessary
  // so that the argument is considered an expression instead of an entity.
  using value_type = typename std::remove_reference<decltype(
        std::declval<Transformer>()(
          std::declval<typename InputIterator::reference>()))>::type;
  using pointer = value_type*;
  using reference = value_type&;
  using iterator_category = typename InputIterator::iterator_category;

  TransformIterator() = default;

  explicit TransformIterator(InputIterator input_iterator) :
    input_iterator_(std::move(input_iterator)) { }

  TransformIterator(InputIterator input_iterator, Transformer transform) :
    input_iterator_(std::move(input_iterator)),
    transform_(std::move(transform)) { }

  template <typename OtherInputIterator,
            typename = std::enable_if_t<
              std::is_constructible<InputIterator, OtherInputIterator>::value>>
  TransformIterator(
      const TransformIterator<OtherInputIterator, Transformer>& other) :
    input_iterator_(other.input_iterator_),
    transform_(other.transform_) { }

  // Many of these functions return auto so that SFINAE kicks in and removes
  // any operators that are not valid due to the input iterator type.

  template <typename OtherInputIterator>
  auto operator==(
      const TransformIterator<OtherInputIterator, Transformer>& other) const {
    return input_iterator_ == other.input_iterator_;
  }

  template <typename OtherInputIterator>
  auto operator!=(
      const TransformIterator<OtherInputIterator, Transformer>& other) const {
    return input_iterator_ != other.input_iterator_;
  }

  reference operator*() const {
    return transform_(*input_iterator_);
  }

  pointer operator->() const {
    return &transform_(*input_iterator_);
  }

  TransformIterator& operator++() {
    ++input_iterator_;
    return *this;
  }

  TransformIterator operator++(int) {
    return TransformIterator(input_iterator_++);
  }

  auto& operator--() {
    --input_iterator_;
    return *this;
  }

  auto operator--(int) {
    return TransformIterator(input_iterator_--);
  }

  auto operator+(difference_type offset) const {
    return TransformIterator(input_iterator_ + offset);
  }

  auto operator-(difference_type offset) const {
    return TransformIterator(input_iterator_ - offset);
  }

  template <typename OtherInputIterator>
  auto operator-(
      const TransformIterator<OtherInputIterator, Transformer>& other) const {
    return input_iterator_ - other.input_iterator_;
  }

  auto& operator+=(difference_type offset) {
    return input_iterator_ += offset, *this;
  }

  auto& operator-=(difference_type offset) {
    return input_iterator_ -= offset, *this;
  }

  auto& operator[](difference_type offset) const {
    return transform_(input_iterator_[offset]);
  }

  template <typename OtherInputIterator>
  auto operator<(
      const TransformIterator<OtherInputIterator, Transformer>& other) const {
    return input_iterator_ < other.input_iterator_;
  }

  template <typename OtherInputIterator>
  auto operator<=(
      const TransformIterator<OtherInputIterator, Transformer>& other) const {
    return input_iterator_ <= other.input_iterator_;
  }

  template <typename OtherInputIterator>
  auto operator>=(
      const TransformIterator<OtherInputIterator, Transformer>& other) const {
    return input_iterator_ >= other.input_iterator_;
  }

  template <typename OtherInputIterator>
  auto operator>(
      const TransformIterator<OtherInputIterator, Transformer>& other) const {
    return input_iterator_ > other.input_iterator_;
  }

 private:
  template <typename OtherInputIterator, typename OtherTransformer>
  friend class TransformIterator;
  InputIterator input_iterator_;
  Transformer transform_;
};

}  // walnut

#endif // WALNUT_TRANSFORM_ITERATOR_H__
