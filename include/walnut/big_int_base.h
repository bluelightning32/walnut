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

  constexpr const BigUIntWord* words() const {
    return words_;
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
  constexpr BigIntBase(const BigIntBase<other_max_words, OtherPolicy>& other) :
      used_(max_words < other_max_words ?
                  std::min(other.used_bytes(),
                           size_t(max_words * bytes_per_word)) :
                  other.used_bytes()) {
    assert(other.used_bytes() <= max_bytes);
    AssignWithoutTrim(other.words(), used_bytes());
    this->Trim();
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

  constexpr void AssignWithoutTrim(const BigUIntWord* words, size_t used) {
    assert(used <= max_bytes);
    used_ = used;
    for (size_t i = 0; i < (used + bytes_per_word - 1) / bytes_per_word; ++i) {
      words_[i] = words[i];
    }
  }

  template <size_t other_max_words, typename OtherPolicy>
  constexpr void AssignIgnoreOverflow(
      const BigIntBase<other_max_words, OtherPolicy>& other) {
    AssignWithoutTrim(other.words(),
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

  template <typename Result, size_t other_max_words, typename OtherPolicy>
  constexpr Result MultiplySlow(
      const BigIntBase<other_max_words, OtherPolicy>& other) const {
    if (used_bytes() < other.used_bytes()) {
      return other.template MultiplySlow<Result>(*this);
    }
    Result result;
    int k = 0;
    {
      BigUIntWord add;
      for (size_t i = 0; i < this->used_words(); ++i, ++k) {
        result.words_[k] = other.words_[0].MultiplyAdd(words_[i], add, /*carry_in=*/false, &add);
      }
      result.words_[k] = add;
    }
    for (size_t j = 1; j < other.used_words(); j++) {
      k = j;
      BigUIntWord add;
      bool carry = false;
      for (size_t i = 0; i < this->used_words(); ++i, ++k) {
        add = add.Add(result.words_[k], carry, &carry);
        result.words_[k] = other.words_[j].MultiplyAdd(words_[i], add, /*carry_in=*/false, &add);
      }
      result.words_[k] = add.Add(carry, &carry);
    }
    k++;
    result.used_ = k * BigUIntWord::bytes_per_word;
    return result;
  }

  constexpr void AddHighWord(BigUIntWord word) {
    const size_t used = used_words();
    assert(used < max_words);
    if (used < max_words) {
      words_[used] = word;
      used_ = (used + 1) * bytes_per_word;
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
