#ifndef WALNUT_BIG_INT_BASE_H__
#define WALNUT_BIG_INT_BASE_H__

#include <cassert>

#include "walnut/big_uint_word.h"

namespace walnut {

// Common base for BigIntImpl and BigUIntImpl
template <int max_words_template>
class BigIntBase {
  template <typename TrimMixin>
  friend class BigIntBaseOperations;

 public:
  static constexpr int max_words = max_words_template;
  static constexpr int bits_per_word = BigUIntWord::bits_per_word;
  static constexpr int bits_per_byte = 8;
  static constexpr int bytes_per_word = BigUIntWord::bytes_per_word;
  static constexpr int max_bits = max_words * bits_per_word;
  static constexpr int max_bytes = max_words * bytes_per_word;

  constexpr uint32_t low_uint32() const {
    return words_[0].low_uint32();
  }

  constexpr uint64_t low_uint64() const {
    return words_[0].low_uint64();
  }

  constexpr int used_bytes() const {
    return used_;
  }

  constexpr const BigUIntWord* words() const {
    return words_;
  }

 protected:
  constexpr BigIntBase(int used) : used_(used) { }

  constexpr int used_words() const {
    return (used_ + bytes_per_word - 1) / bytes_per_word;
  }

  template <int other_words>
  constexpr int GetCommonWordCount(
      const BigIntBase<other_words>& other) const {
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

  // The number of bytes used in words_.
  //
  // Invariant:
  //   Either used_ == sizeof(BigUIntHalfWord), or used_ is a multiple of
  //   bytes_per_word.
  //
  // Even if used_ == sizeof(BigUIntHalfWord), the rest of the bits in words_[0] are
  // still initialized.
  int used_;

  // words_[0] holds the lowest significant bits. Within each element of
  // words_, the bit order is in the machine native order.
  BigUIntWord words_[max_words];
};

// Adds operations that require Trim.
//
// TrimMixin must implement Trim, and TrimMixin must inherit from BigIntBase.
template <typename TrimMixin>
class BigIntBaseOperations : public TrimMixin {
  template <typename OtherTrimBase>
  friend class BigIntBaseOperations;

 public:
  using TrimMixin::max_words;
  using TrimMixin::bits_per_word;
  using TrimMixin::bits_per_byte;
  using TrimMixin::bytes_per_word;
  using TrimMixin::max_bits;
  using TrimMixin::max_bytes;

 protected:
  using TrimMixin::TrimMixin;
  using TrimMixin::used_;
  using TrimMixin::words_;

  constexpr BigIntBaseOperations(const BigUIntWord* words, int used) :
      TrimMixin(used) {
    assert(("overflow", used_ <= max_bytes));
    for (int i = 0; i < (used + bytes_per_word - 1) / bytes_per_word; ++i) {
      words_[i] = words[i];
    }
    this->Trim();
  }

  template <typename OtherMixin>
  constexpr BigIntBaseOperations(
      const BigIntBaseOperations<OtherMixin>& other) :
      TrimMixin(max_words < OtherMixin::max_words ?
                  std::min(other.used_, int(max_words * bytes_per_word)) :
                  other.used_) {
    assert(("overflow", other.used_ <= max_bytes));
    int copy_words = (used_ + bytes_per_word - 1) / bytes_per_word;
    for (int i = 0; i < copy_words; i++) {
      words_[i] = other.words_[i];
    }
    this->Trim();
  }

  template <typename OtherMixin>
  constexpr BigIntBaseOperations operator=(
      const BigIntBaseOperations<OtherMixin>& other) {
    assert(other.used_ <= max_bytes);
    AssignIgnoreOverflow(other);
    return *this;
  }

  template <typename OtherMixin>
  constexpr void AssignIgnoreOverflow(
      const BigIntBaseOperations<OtherMixin>& other) {
    int copy_bytes = max_words < OtherMixin::max_words ?
                     std::min(other.used_, int(max_words * bytes_per_word)) :
                     other.used_;
    int copy_words = (copy_bytes + bytes_per_word - 1) / bytes_per_word;
    for (int i = 0; i < copy_words; i++) {
      words_[i] = other.words_[i];
    }
    used_ = copy_bytes;
    if (max_words < OtherMixin::max_words) {
      this->Trim();
    }
  }

  template <typename Result, typename Other>
  constexpr Result operator&(const Other& other) const {
    Result result;
    result.used_ = std::min(used_, other.used_);
    for (int i = 0; i < result.used_words(); i++) {
      result.words_[i] = words_[i] & other.words_[i];
    }
    result.Trim();
    return result;
  }

  template <typename Result, typename OtherMixin>
  constexpr Result MultiplySlow(
      const BigIntBaseOperations<OtherMixin>& other) const {
    if (used_ < other.used_) {
      return other.template MultiplySlow<Result>(*this);
    }
    Result result;
    int k = 0;
    {
      BigUIntWord add;
      for (int i = 0; i < this->used_words(); ++i, ++k) {
        result.words_[k] = other.words_[0].MultiplyAdd(words_[i], add, /*carry_in=*/false, &add);
      }
      result.words_[k] = add;
    }
    for (int j = 1; j < other.used_words(); j++) {
      k = j;
      BigUIntWord add;
      bool carry = false;
      for (int i = 0; i < this->used_words(); ++i, ++k) {
        add = add.Add(result.words_[k], carry, &carry);
        result.words_[k] = other.words_[j].MultiplyAdd(words_[i], add, /*carry_in=*/false, &add);
      }
      result.words_[k] = add.Add(carry, &carry);
    }
    k++;
    result.used_ = k * BigUIntWord::bytes_per_word;
    return result;
  }
};

}  // walnut

#endif // WALNUT_BIG_INT_BASE_H__