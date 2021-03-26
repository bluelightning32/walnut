#ifndef WALNUT_BIG_INT_WORDS_H__
#define WALNUT_BIG_INT_WORDS_H__

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

  constexpr size_t used_words() const {
    return used_words_;
  }

  constexpr BigUIntWord word(size_t i) const {
    return words_[i];
  }

  constexpr BigIntWords(int used_words) : used_words_(used_words) { }

  constexpr BigIntWords(const BigUIntWord* words, size_t used_words) :
      used_words_(used_words) {
    assert(used_words <= max_words);
    for (size_t i = 0; i < used_words; ++i) {
      words_[i] = words[i];
    }
  }

  template <size_t other_max_words>
  constexpr BigIntWords(size_t used_words, size_t copy_words,
                       const BigIntWords<other_max_words>& from) :
      used_words_(std::min(size_t(max_words), used_words)) {
    for (size_t i = 0; i < copy_words; ++i) {
      words_[i] = from.words_[i];
    }
  }

  template <size_t other_max_words>
  constexpr BigIntWords(const BigIntWords<other_max_words>& other) :
      used_words_(max_words < other_max_words ?
                  std::min(other.used_words(), size_t(max_words)) :
                  other.used_words()) {
    assert(other.used_words() <= max_words);
    AssignWithoutTrim(other, used_words());
  }

  constexpr void set_word(size_t i, BigUIntWord v) {
    words_[i] = v;
  }

  template <size_t other_words>
  constexpr size_t GetCommonWordCount(
      const BigIntWords<other_words>& other) const {
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
      const BigIntWords<other_max_words>& other, size_t used) {
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

  constexpr void AllocateWords(size_t used_words) {
    used_words_ = used_words;
  }

  // words_[0] holds the lowest significant bits. Within each element of
  // words_, the bit order is in the machine native order.
  BigUIntWord words_[max_words];

 private:
  // The number of words used in words_.
  //
  // Invariant:
  //   used_words_ >= 1
  size_t used_words_;
};

}  // walnut

#endif // WALNUT_BIG_INT_WORDS_H__
