#ifndef WALNUT_CONCAT_RANGE_H__
#define WALNUT_CONCAT_RANGE_H__

#include <iterator>
#include <type_traits>
#include <vector>

namespace walnut {

// ConcatRange is a view of ranges of iterators concatenated together. For
// example, it could represent 2 ranges of a std::vector concatenated together.
// As it is a view, the elements are referenced from their original iterators
// instead of copying the elements.
//
// The `InputIterator` must be at least bidirectional.
template <typename InputIterator>
class ConcatRange {
 private:
  template <typename ValueType>
  class IteratorHelper;

 public:
  using iterator = IteratorHelper<
    typename std::iterator_traits<InputIterator>::value_type>;
  using const_iterator = IteratorHelper<
    const typename std::iterator_traits<InputIterator>::value_type>;
  using reverse_iterator = std::reverse_iterator<iterator>;
  using const_reverse_iterator = std::reverse_iterator<const_iterator>;

  using value_type = typename std::iterator_traits<InputIterator>::value_type;

  ConcatRange() : ranges_(1) { }

  void Append(InputIterator start, InputIterator end) {
    // Replace the terminator at the end with the new range.
    ranges_.back().first = start;
    ranges_.back().second = end;
    // Readd the terminator.
    ranges_.emplace_back();
  }

  void Prepend(InputIterator start, InputIterator end) {
    ranges_.emplace(ranges_.begin(), start, end);
  }

  iterator begin() {
    return iterator(ranges_.begin(), ranges_.begin()->first);
  }

  const_iterator begin() const {
    return cbegin();
  }

  const_iterator cbegin() const {
    return const_iterator(ranges_.begin(), ranges_.begin()->first);
  }

  iterator end() {
    return iterator(ranges_.end() - 1, ranges_.end()[-1].first);
  }

  const_iterator end() const {
    return cend();
  }

  const_iterator cend() const {
    return const_iterator(ranges_.end() - 1, ranges_.end()[-1].first);
  }

  reverse_iterator rbegin() {
    return std::make_reverse_iterator(end());
  }

  const_reverse_iterator rbegin() const {
    return crbegin();
  }

  const_reverse_iterator crbegin() const {
    return std::make_reverse_iterator(end());
  }

  reverse_iterator rend() {
    return std::make_reverse_iterator(begin());
  }

  const_reverse_iterator rend() const {
    return crend();
  }

  const_reverse_iterator crend() const {
    return std::make_reverse_iterator(begin());
  }

 private:
  // All but the last element point to valid ranges of elements. The last
  // element is a terminator so that end().pos_ can be constructed from
  // ranges_.end()[-1].first.
  std::vector<std::pair<InputIterator, InputIterator>> ranges_;
};

template <typename InputIterator>
template <typename ValueType>
class ConcatRange<InputIterator>::IteratorHelper {
 private:
  using RangeIterator = typename
    std::vector<std::pair<InputIterator, InputIterator>>::const_iterator;

 public:
  // Defines NonconstIterator only if ValueType is const qualified.
  template <typename ValueType_ = ValueType, typename T =
    typename std::enable_if<std::is_const<ValueType_>::value,
                            ConcatRange<InputIterator>::iterator>::type>
  using NonconstIterator = T;

  // Let the const version of IteratorHelper access private fields from the
  // non-const version.
  friend IteratorHelper<const ValueType>;

  // These declarations are necessary for std::iterator_traits<IteratorHelper>
  // to have its fields defined, which is necessary for IteratorHelper to
  // satisfy the LegacyIterator requirements.
  using value_type = ValueType;
  using difference_type = typename
    std::iterator_traits<InputIterator>::difference_type;
  using pointer = ValueType*;
  using reference = ValueType&;
  using iterator_category = std::bidirectional_iterator_tag;

  IteratorHelper() = default;

  template <typename ValueType_ = ValueType>
  explicit IteratorHelper(const NonconstIterator<ValueType_>& other) :
    IteratorHelper(other.range_pos_, other.pos_) { }

  IteratorHelper(const IteratorHelper& other) :
    IteratorHelper(other.range_pos_, other.pos_) { }

  IteratorHelper(const RangeIterator& range_pos, const InputIterator& pos) :
    range_pos_(range_pos), pos_(pos) { }

  template <typename ValueType_ = ValueType>
  IteratorHelper& operator=(const NonconstIterator<ValueType_>& other) {
    range_pos_ = other.range_pos_;
    pos_ = other.pos_;
    return *this;
  }

  IteratorHelper& operator=(IteratorHelper& other) {
    range_pos_ = other.range_pos_;
    pos_ = other.pos_;
  }

  ValueType& operator*() const {
    return *pos_;
  }

  // prefix increment
  IteratorHelper& operator++() {
    ++pos_;
    if (pos_ == range_pos_->second) {
      ++range_pos_;
      pos_ = range_pos_->first;
    }
    return *this;
  }

  // postfix increment
  IteratorHelper operator++(int) {
    IteratorHelper copy(*this);
    ++*this;
    return copy;
  }

  bool operator==(const IteratorHelper& other) const {
    return range_pos_ == other.range_pos_ && pos_ == other.pos_;
  }

  bool operator!=(const IteratorHelper& other) const {
    return !(*this == other);
  }

  template <typename ValueType_ = ValueType>
  bool operator==(const NonconstIterator<ValueType_>& other) const {
    return *this == IteratorHelper(other);
  }

  template <typename ValueType_ = ValueType>
  bool operator!=(const NonconstIterator<ValueType_>& other) const {
    return *this != IteratorHelper(other);
  }

  ValueType* operator->() const {
    return *&pos_;
  }

  // prefix decrement
  IteratorHelper& operator--() {
    if (pos_ == range_pos_->first) {
      --range_pos_;
      pos_ = range_pos_->second;
    }
    --pos_;
    return *this;
  }

  // postfix decrement
  IteratorHelper operator--(int) {
    IteratorHelper copy(*this);
    --*this;
    return copy;
  }

 private:
  RangeIterator range_pos_;
  InputIterator pos_;
};

}  // walnut

#endif // WALNUT_CONCAT_RANGE_H__
