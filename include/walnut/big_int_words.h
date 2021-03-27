#ifndef WALNUT_BIG_INT_WORDS_H__
#define WALNUT_BIG_INT_WORDS_H__

// For std::min
#include <algorithm>
#include <cassert>

#include "walnut/big_uint_word.h"

namespace walnut {

// Common base for BigIntImpl and BigUIntImpl
template <size_t max_words_template>
class BigIntWords {
  template <size_t other_max_words>
  friend class BigIntWords;

 public:
  static constexpr size_t max_words = max_words_template;
  static constexpr size_t bits_per_word = BigUIntWord::bits_per_word;
  static constexpr size_t bytes_per_word = BigUIntWord::bytes_per_word;
  static constexpr size_t max_bits = max_words * bits_per_word;

  constexpr size_t size() const {
    return size_;
  }

  constexpr const BigUIntWord& operator[](size_t i) const {
    assert(i < size());
    return words_[i];
  }

  constexpr BigUIntWord& operator[](size_t i) {
    assert(i < size());
    return words_[i];
  }

  constexpr BigIntWords(int used_words) : size_(used_words) { }

  template <size_t other_words>
  constexpr BigIntWords(const BigIntWords<other_words>& words, size_t used_words) :
      size_(used_words) {
    assert(used_words <= max_words);
    for (size_t i = 0; i < used_words; ++i) {
      words_[i] = words[i];
    }
  }

  template <size_t other_max_words>
  constexpr BigIntWords(size_t used_words, size_t copy_words,
                       const BigIntWords<other_max_words>& from) :
      size_(std::min(size_t(max_words), used_words)) {
    for (size_t i = 0; i < copy_words; ++i) {
      words_[i] = from.words_[i];
    }
  }

  template <size_t other_max_words>
  constexpr BigIntWords(const BigIntWords<other_max_words>& other) :
      size_(max_words < other_max_words ?
                  std::min(other.size(), size_t(max_words)) :
                  other.size()) {
    assert(other.size() <= max_words);
    for (size_t i = 0; i < size_; ++i) {
      words_[i] = other.words_[i];
    }
  }

  // Gets the lowest word after shifting this to the right `shift` bits.
  //
  // The caller must ensure:
  // a. `shift` is greater than or equal to 0.
  // b. shift / bits_per_word + 1 < max_words
  BigUIntWord GetAtBitOffset(unsigned shift) const {
    int word_index = shift / bits_per_word;
    int word_offset = shift % bits_per_word;
    return words_[word_index].ShiftRight(words_[word_index+1], word_offset);
  }

  template <size_t other_max_words>
  constexpr void Assign(
      const BigIntWords<other_max_words>& other, size_t used) {
    assert(used <= max_words);
    resize(used);
    for (size_t i = 0; i < used; ++i) {
      words_[i] = other.words_[i];
    }
  }

  constexpr void push_back(BigUIntWord word) {
    const size_t used = size();
    assert(used < max_words);
    if (used < max_words) {
      resize(size() + 1);
      words_[used] = word;
    }
  }

  constexpr void resize(size_t used_words) {
    size_ = used_words;
  }

 private:
  // The number of words used in words_.
  //
  // Invariant:
  //   size_ >= 1
  size_t size_ = 0;

  // words_[0] holds the lowest significant bits. Within each element of
  // words_, the bit order is in the machine native order.
  BigUIntWord words_[max_words];
};

}  // walnut

#endif // WALNUT_BIG_INT_WORDS_H__
