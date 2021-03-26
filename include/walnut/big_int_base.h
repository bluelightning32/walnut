#ifndef WALNUT_BIG_INT_BASE_H__
#define WALNUT_BIG_INT_BASE_H__

#include <cassert>

#include "walnut/big_uint_word.h"

namespace walnut {

// Common base for BigIntImpl and BigUIntImpl
template <size_t max_words_template>
class BigIntBase {
  template <size_t other_max_words>
  friend class BigIntBase;

 public:
  static constexpr size_t max_words = max_words_template;
  static constexpr size_t bits_per_word = BigUIntWord::bits_per_word;
  static constexpr size_t bits_per_byte = 8;
  static constexpr size_t bytes_per_word = BigUIntWord::bytes_per_word;
  static constexpr size_t max_bits = max_words * bits_per_word;
  static constexpr size_t max_bytes = max_words * bytes_per_word;

  constexpr uint32_t low_uint32() const {
    return words_[0].low_uint32();
  }

  constexpr uint64_t low_uint64() const {
    return words_[0].low_uint64();
  }

  constexpr int ToInt() const {
    assert(used_words() == 1);
    return words_[0].ToInt();
  }

  constexpr size_t used_words() const {
    return (used_bytes() + bytes_per_word - 1) / bytes_per_word;
  }

  constexpr BigUIntWord word(size_t i) const {
    return words_[i];
  }

 protected:
  constexpr BigIntBase(int used) : used_(used) { }

  constexpr BigIntBase(const BigUIntWord* words, size_t used) :
      used_(used * bytes_per_word) {
    assert(used_words() <= max_words);
    for (size_t i = 0; i < used; ++i) {
      words_[i] = words[i];
    }
  }

  template <size_t other_max_words>
  constexpr BigIntBase(size_t used_words, size_t copy_words,
                       const BigIntBase<other_max_words>& from) :
      used_(std::min(size_t(max_bytes), used_words * bytes_per_word)) {
    for (size_t i = 0; i < copy_words; ++i) {
      words_[i] = from.words_[i];
    }
  }

  template <size_t other_max_words>
  constexpr BigIntBase(const BigIntBase<other_max_words>& other) :
      used_(max_words < other_max_words ?
                  std::min(other.used_bytes(),
                           size_t(max_words * bytes_per_word)) :
                  other.used_bytes()) {
    assert(other.used_words() <= max_words);
    AssignWithoutTrim(other, used_words());
  }

  constexpr void set_word(size_t i, BigUIntWord v) {
    words_[i] = v;
  }

  constexpr size_t used_bytes() const {
    return used_;
  }

  template <size_t other_words>
  constexpr size_t GetCommonWordCount(
      const BigIntBase<other_words>& other) const {
    return std::min(used_words(), other.used_words());
  }

  // Gets the lowest word after shifting this to the right `shift` bits.
  //
  // The caller must ensure:
  // a. `shift` is greater than or equal to 0.
  // b. shift / bits_per_word + 1 < max_words
  BigUIntWord GetWordAtBitOffset(unsigned shift) const {
    int word_index = shift / bits_per_word;
    int word_offset = shift % bits_per_word;
    return words_[word_index].ShiftRight(words_[word_index+1], word_offset);
  }

  template <size_t other_max_words>
  constexpr void AssignWithoutTrim(
      const BigIntBase<other_max_words>& other, size_t used) {
    assert(used <= max_words);
    AllocateWords(used);
    for (size_t i = 0; i < used; ++i) {
      words_[i] = other.words_[i];
    }
  }

  constexpr void AddHighWord(BigUIntWord word) {
    const size_t used = used_words();
    assert(used < max_words);
    if (used < max_words) {
      AllocateWords(used_words() + 1);
      words_[used] = word;
    }
  }

  constexpr void AllocateWords(size_t words) {
    used_ = words * bytes_per_word;
  }

  // words_[0] holds the lowest significant bits. Within each element of
  // words_, the bit order is in the machine native order.
  BigUIntWord words_[max_words];

 private:
  // The number of bytes used in words_.
  //
  // Invariant:
  //   Either used_ == sizeof(BigUIntHalfWord), or used_ is a multiple of
  //   bytes_per_word.
  //
  // Even if used_ == sizeof(BigUIntHalfWord), the rest of the bits in
  // words_[0] are still initialized.
  size_t used_;
};

}  // walnut

#endif // WALNUT_BIG_INT_BASE_H__
