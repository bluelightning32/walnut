#ifndef WALNUT_BIG_INT_BASE_H__
#define WALNUT_BIG_INT_BASE_H__

#include <cassert>

#include "walnut/big_uint_word.h"

namespace walnut {

// Common base for BigIntImpl and BigUIntImpl
template <size_t max_words_template, typename TrimPolicyTemplate>
class BigIntBase {
  template <size_t other_max_words, typename OtherTrimPolicy>
  friend class BigIntBase;

 public:
  using TrimPolicy = TrimPolicyTemplate;
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

  constexpr size_t used_bytes() const {
    return used_;
  }

  constexpr BigUIntWord word(size_t i) const {
    return words_[i];
  }

 protected:
  constexpr BigIntBase(int used) : used_(used) { }

  constexpr BigIntBase(const BigUIntWord* words, size_t used) :
      used_(used) {
    assert(used_bytes() <= max_bytes);
    for (size_t i = 0; i < (used + bytes_per_word - 1) / bytes_per_word; ++i) {
      words_[i] = words[i];
    }
    this->Trim();
  }

  template <size_t other_max_words, typename OtherPolicy>
  constexpr BigIntBase(size_t used, size_t copy,
                       const BigIntBase<other_max_words, OtherPolicy>& from) :
      used_(std::min(size_t(max_bytes), used)) {
    size_t copy_words = std::min(size_t(max_words),
                                 (copy + bytes_per_word - 1) / bytes_per_word);
    for (size_t i = 0; i < copy_words; ++i) {
      words_[i] = from.words_[i];
    }
    Trim();
  }

  template <size_t other_max_words, typename OtherPolicy>
  constexpr BigIntBase(const BigIntBase<other_max_words, OtherPolicy>& other) :
      used_(max_words < other_max_words ?
                  std::min(other.used_bytes(),
                           size_t(max_words * bytes_per_word)) :
                  other.used_bytes()) {
    assert(other.used_bytes() <= max_bytes);
    AssignWithoutTrim(other, used_bytes());
    this->Trim();
  }

  constexpr void set_word(size_t i, BigUIntWord v) {
    words_[i] = v;
  }

  constexpr size_t used_words() const {
    return (used_bytes() + bytes_per_word - 1) / bytes_per_word;
  }

  template <size_t other_words, typename OtherTrimPolicy>
  constexpr size_t GetCommonWordCount(
      const BigIntBase<other_words, OtherTrimPolicy>& other) const {
    return (std::min(used_bytes(), other.used_bytes()) +
            bytes_per_word - 1) / bytes_per_word;
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

  constexpr void Trim() {
    int i = used_bytes() / bytes_per_word - 1;
    if (i > 0) {
      BigUIntWord check = words_[i];
      BigUIntWord next;
      do {
        --i;
        next = words_[i];

        if (!TrimPolicy::CanTrim(/*low=*/next, /*high=*/check)) break;

        check = next;
        used_ -= bytes_per_word;
      } while (i > 0);
    }
    if (used_bytes() == bytes_per_word &&
        TrimPolicy::CanTrimLastHalf(words_[0])) {
      used_ = sizeof(BigUIntHalfWord);
    }
  }

  template <size_t other_max_words, typename OtherTrimPolicy>
  constexpr void AssignWithoutTrim(
      const BigIntBase<other_max_words, OtherTrimPolicy>& other, size_t used) {
    assert(used <= max_bytes);
    Allocate(used);
    for (size_t i = 0; i < (used + bytes_per_word - 1) / bytes_per_word; ++i) {
      words_[i] = other.words_[i];
    }
  }

  template <size_t other_max_words, typename OtherPolicy>
  constexpr void AssignIgnoreOverflow(
      const BigIntBase<other_max_words, OtherPolicy>& other) {
    AssignWithoutTrim(other,
        max_words < other_max_words ?
        std::min(other.used_bytes(), size_t(max_words * bytes_per_word)) :
        other.used_bytes());
    if (max_words < other_max_words) {
      this->Trim();
    }
  }

  template <size_t other_max_words, typename OtherPolicy>
  constexpr BigIntBase& operator=(
      const BigIntBase<other_max_words, OtherPolicy>& other) {
    assert(other.used_bytes() <= max_bytes);
    AssignIgnoreOverflow(other);
    return *this;
  }

  constexpr BigIntBase& operator=(BigUIntWord value) {
    words_[0] = value;
    used_ = CanTrimLastHalf(words_[0]) ?
      sizeof(BigUIntHalfWord) : bytes_per_word;
    return *this;
  }

  constexpr void AddHighWord(BigUIntWord word) {
    const size_t used = used_words();
    assert(used < max_words);
    if (used < max_words) {
      Allocate((used + 1) * bytes_per_word);
      words_[used] = word;
    }
  }

  constexpr void Allocate(size_t used) {
    used_ = used;
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
