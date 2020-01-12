#ifndef WALNUT_BIG_UINT_H__
#define WALNUT_BIG_UINT_H__

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <type_traits>

#include "walnut/big_uint_word.h"

namespace walnut {

template <int max_words>
class BigUIntImpl
{
  template <int other_words>
  friend class BigUIntImpl;

 public:
  static constexpr int bits_per_word = BigUIntWord::bits_per_word;
  static constexpr int bits_per_byte = 8;
  static constexpr int bytes_per_word = BigUIntWord::bytes_per_word;
  static constexpr int max_bits = max_words * bits_per_word;
  static constexpr int max_bytes = max_words * bytes_per_word;

  constexpr BigUIntImpl() : BigUIntImpl(static_cast<BigUIntHalfWord>(0)) {
  }

  explicit constexpr BigUIntImpl(BigUIntHalfWord value) : used_(sizeof(BigUIntHalfWord)) {
    words_[0] = value;
  }

  explicit constexpr BigUIntImpl(BigUIntWord value) :
    used_(value <= std::numeric_limits<BigUIntHalfWord>::max() ?
          sizeof(BigUIntHalfWord) : BigUIntWord::bytes_per_word) {
    words_[0] = value;
  }

  explicit constexpr BigUIntImpl(uint64_t value) : BigUIntImpl(BigUIntWord(value)) { }

  explicit constexpr BigUIntImpl(int value) {
    if (value >= 0) {
      words_[0] = value;
      used_ = sizeof(BigUIntHalfWord);
    } else {
      words_[0] = value;
      for (int i = 1; i < max_words; ++i) {
        words_[i] = -1;
      }
      used_ = max_words * BigUIntWord::bytes_per_word;
    }
  }

  template <int other_max_words,
            typename std::enable_if<max_words >= other_max_words, int>::type
              enable = 1
  >
  constexpr BigUIntImpl(const BigUIntImpl<other_max_words>& other)
   : used_(other.used_) {
    static_assert(max_words >= other_max_words);
    for (int i = 0; i < other_max_words; i++) {
      words_[i] = other.words_[i];
    }
  }

  template <int other_max_words,
            typename std::enable_if<max_words < other_max_words, int>::type
              enable = 1
  >
  constexpr BigUIntImpl(const BigUIntImpl<other_max_words>& other) {
    static_assert(max_words < other_max_words);
    for (int i = 0; i < max_words; i++) {
      words_[i] = other.words_[i];
    }
    if (other.used_ < max_words * bytes_per_word) {
      used_ = other.used_;
    } else {
      used_ = max_words * bytes_per_word;
      Trim();
    }
  }

  template <int other_max_words,
           typename std::enable_if<max_words >= other_max_words, int>::type
             enable = 1>
  constexpr BigUIntImpl<max_words>& operator = (
      const BigUIntImpl<other_max_words>& other) {
    used_= other.used_;
    int i = 0;
    for (; i < other_max_words; i++) {
      words_[i] = other.words_[i];
    }
    for (; i < max_words; i++) {
      words_[i] = 0;
    }
    return *this;
  }

  template <int other_max_words,
           typename std::enable_if<max_words < other_max_words, int>::type
             enable = 1>
  constexpr BigUIntImpl<max_words>& operator = (
      const BigUIntImpl<other_max_words>& other) {
    for (int i = 0; i < max_words; i++) {
      words_[i] = other.words_[i];
    }
    if (other.used_ <= max_words * bytes_per_word) {
      used_= other.used_;
    } else {
      used_= max_words * bytes_per_word;
      Trim();
    }
    return *this;
  }

  uint32_t low_uint32() const {
    return words_[0].low_uint32();
  }

  uint64_t low_uint64() const {
    return words_[0].low_uint64();
  }

  template <int result_words=max_words>
  constexpr BigUIntImpl<result_words> operator << (int shift) const {
    if (used_  == sizeof(BigUIntHalfWord) && shift <= 32) {
      return BigUIntImpl<result_words>(words_[0].low_uint64() << shift);
    }

    BigUIntImpl<result_words> result;
    if (shift >= result.max_bits) return result;

    int in = 0;
    int out = shift / bits_per_word;
    const int word_left_shift = shift % bits_per_word;
    // The if statement is necessary to avoid shifting by bits_per_word.
    if (word_left_shift > 0) {
      BigUIntWord prev(0);
      const int prev_right_shift = bits_per_word - word_left_shift;
      for (; in < used_words() && out < result_words; in++, out++) {
        result.words_[out] = words_[in] << word_left_shift |
                               prev >> prev_right_shift;
        prev = words_[in];
      }
      if (out < result_words) {
        result.words_[out] = prev >> prev_right_shift;
        out++;
      }
    } else {
      for (; in < used_words() && out < result_words; in++, out++) {
        result.words_[out] = words_[in];
      }
    }
    result.used_ = out * BigUIntWord::bytes_per_word;
    result.Trim();
    return result;
  }

  template <int result_words = 0, int other_words,
            int rw = result_words == 0 ?
              std::max(max_words, other_words) : result_words>
  constexpr BigUIntImpl<rw> Add(const BigUIntImpl<other_words>& other) const {
    if (used_ == sizeof(BigUIntHalfWord) && other.used_ == sizeof(BigUIntHalfWord)) {
      return BigUIntImpl<rw>(words_[0].AddAsUInt32(other.words_[0]));
    }
    BigUIntImpl<rw> result;
    int i = 0;
    bool carry = false;
    int common_words = GetCommonWordCount(other);
    for (; i < common_words && i < rw; i++) {
      result.words_[i] = words_[i].Add(other.words_[i], carry, &carry);
    }
    for (; i < std::min(used_ / BigUIntWord::bytes_per_word, rw); i++) {
      result.words_[i] = words_[i].Add(carry, &carry);
    }
    for (; i < std::min(other.used_ / BigUIntWord::bytes_per_word, rw); i++) {
      result.words_[i] = other.words_[i].Add(carry, &carry);
    }
    if (carry && i < rw) {
      result.words_[i] = 1;
      i++;
    }
    result.used_ = i * BigUIntWord::bytes_per_word;
    result.Trim();
    return result;
  }

  template <int other_words>
  constexpr BigUIntImpl<std::max(max_words, other_words)> operator+(
      const BigUIntImpl<other_words>& other) const {
    return Add(other);
  }

  template <int result_words = 0, int other_words,
            int rw = result_words == 0 ?
              std::max(max_words, other_words) : result_words>
  constexpr BigUIntImpl<rw> Subtract(const BigUIntImpl<other_words>& other) const {
    if (used_ == sizeof(BigUIntHalfWord) && other.used_ == sizeof(BigUIntHalfWord) &&
        words_[0].low_uint32() >= other.words_[0].low_uint32()) {
      return BigUIntImpl<rw>(words_[0].SubtractAsUInt32(other.words_[0]));
    }
    BigUIntImpl<rw> result;
    int i = 0;
    bool carry = false;
    int common_words = GetCommonWordCount(other);
    for (; i < common_words && i < rw; i++) {
      result.words_[i] = words_[i].Subtract(other.words_[i], carry, &carry);
    }
    for (; i < std::min(used_ / BigUIntWord::bytes_per_word, rw); i++) {
      result.words_[i] = words_[i].Subtract(carry, &carry);
    }
    for (; i < std::min(other.used_ / BigUIntWord::bytes_per_word, rw); i++) {
      result.words_[i] = BigUIntWord(0).Subtract(other.words_[i], carry, &carry);
    }
    if (carry) {
      while (i < rw) {
        result.words_[i] = -1;
        i++;
      }
    }
    result.used_ = i * BigUIntWord::bytes_per_word;
    result.Trim();
    return result;
  }

  template <int result_words = 0, int other_words,
            int rw = result_words == 0 ?
              max_words + other_words : result_words>
  constexpr BigUIntImpl<rw> Multiply(const BigUIntImpl<other_words>& other) const {
    if (used_ == sizeof(BigUIntHalfWord) && other.used_ == sizeof(BigUIntHalfWord)) {
      return BigUIntImpl<rw>(words_[0].MultiplyAsHalfWord(other.words_[0]));
    }
    if (used_ <= BigUIntWord::bytes_per_word && other.used_ <= BigUIntWord::bytes_per_word && rw >= 2) {
      BigUIntImpl<rw> result;
      result.words_[0] = words_[0].Multiply(other.words_[0], &result.words_[1]);
      result.used_ = BigUIntWord::bytes_per_word * 2;
      result.Trim();
      return result;
    }
    if (used_ < other.used_) {
      return other.template Multiply<result_words>(*this);
    }
    BigUIntImpl<rw> result;
    int k = 0;
    {
      BigUIntWord add;
      for (int i = 0; i < used_words(); ++i, ++k) {
        result.words_[k] = other.words_[0].MultiplyAdd(words_[i], add, /*carry_in=*/false, &add);
      }
      if (k < rw) {
        result.words_[k] = add;
      }
    }
    for (int j = 1; j < other.used_words(); j++) {
      k = j;
      BigUIntWord add;
      for (int i = 0; i < used_words() && k < rw; ++i, ++k) {
        bool carry = false;
        add = add.Add(result.words_[k], &carry);
        result.words_[k] = other.words_[j].MultiplyAdd(words_[i], add, carry, &add);
      }
      if (k < rw) {
        result.words_[k] = add;
      }
    }
    result.used_ = std::min(used_words() + other.used_words(), rw) * BigUIntWord::bytes_per_word;
    result.Trim();
    return result;
  }

  template <int result_words = 0,
            int rw = result_words == 0 ?
              max_words + 1 : result_words>
  constexpr BigUIntImpl<rw> Multiply(BigUIntWord other) const {
    if (used_ == sizeof(BigUIntHalfWord) &&
        other <= std::numeric_limits<BigUIntHalfWord>::max()) {
      return BigUIntImpl<rw>(words_[0].MultiplyAsHalfWord(other));
    }
    BigUIntImpl<rw> result;
    int k = 0;
    BigUIntWord add;
    for (int i = 0; i < used_ / bytes_per_word; ++i, ++k) {
      result.words_[k] = other.MultiplyAdd(words_[i], add, /*carry_in=*/false, &add);
    }
    if (k < rw) {
      result.words_[k] = add;
      k++;
    }
    result.used_ = k * BigUIntWord::bytes_per_word;
    result.Trim();
    return result;
  }

  // Divide `this` by `other`. Return the quotient.
  template <int other_words>
  constexpr BigUIntImpl<max_words> operator/(const BigUIntImpl<other_words>& other) const {
    if (used_ <= bytes_per_word && other.used_ <= bytes_per_word) {
      return BigUIntImpl<max_words>{words_[0] / other.words_[0]};
    }
    BigUIntImpl<std::min(max_words, other_words)> unused;
    return DivideRemainderSlow(other, &unused);
  }

  // Divide `this` by `other`. Return the quotient and store the remainder in `remainder_out`.
  template <int other_words>
  constexpr BigUIntImpl<max_words> DivideRemainder(const BigUIntImpl<other_words>& other,
      BigUIntImpl<std::min(max_words, other_words)>* remainder_out) const {
    if (used_ <= bytes_per_word && other.used_ <= bytes_per_word) {
      *remainder_out = BigUIntImpl<std::min(max_words, other_words)>{words_[0] % other.words_[0]};
      return BigUIntImpl<max_words>{words_[0] / other.words_[0]};
    }
    return DivideRemainderSlow(other, remainder_out);
  }

  // Divide `this` by `other`. Return the remainder.
  template <int other_words>
  constexpr BigUIntImpl<std::min(max_words, other_words)> operator%(
      const BigUIntImpl<other_words>& other) const {
    if (used_ <= bytes_per_word && other.used_ <= bytes_per_word) {
      return BigUIntImpl<std::min(max_words, other_words)>{words_[0] % other.words_[0]};
    }
    BigUIntImpl<std::min(max_words, other_words)> remainder;
    DivideRemainderSlow(other, &remainder);
    return remainder;
  }

  template <int other_max_words>
  constexpr bool operator < (const BigUIntImpl<other_max_words>& other) const {
    if (used_ < other.used_) return true;
    if (used_ > other.used_) return false;

    for (int i = used_ / BigUIntWord::bytes_per_word - 1; i > 0; i--) {
      if (words_[i] < other.words_[i]) return true;
      if (words_[i] > other.words_[i]) return false;
    }
    return words_[0] < other.words_[0];
  }

  template <int other_max_words>
  constexpr bool operator <= (const BigUIntImpl<other_max_words>& other) const {
    if (used_ < other.used_) return true;
    if (used_ > other.used_) return false;

    for (int i = used_ / BigUIntWord::bytes_per_word - 1; i > 0; i--) {
      if (words_[i] < other.words_[i]) return true;
      if (words_[i] > other.words_[i]) return false;
    }
    return words_[0] <= other.words_[0];
  }

  template <int other_max_words>
  constexpr bool operator > (const BigUIntImpl<other_max_words>& other) const {
    if (used_ > other.used_) return true;
    if (used_ < other.used_) return false;

    for (int i = used_ / BigUIntWord::bytes_per_word - 1; i > 0; i--) {
      if (words_[i] > other.words_[i]) return true;
      if (words_[i] < other.words_[i]) return false;
    }
    return words_[0] > other.words_[0];
  }

  template <int other_max_words>
  constexpr bool operator >= (const BigUIntImpl<other_max_words>& other) const {
    if (used_ > other.used_) return true;
    if (used_ < other.used_) return false;

    for (int i = used_ / BigUIntWord::bytes_per_word - 1; i > 0; i--) {
      if (words_[i] > other.words_[i]) return true;
      if (words_[i] < other.words_[i]) return false;
    }
    return words_[0] >= other.words_[0];
  }

  template <int other_max_words>
  constexpr bool operator == (const BigUIntImpl<other_max_words>& other) const {
    if (used_ != other.used_) return false;

    for (int i = used_ / BigUIntWord::bytes_per_word - 1; i > 0; i--) {
      if (words_[i] != other.words_[i]) return false;
    }
    return words_[0] == other.words_[0];
  }

  // Adds (add << shift) to this. The caller must ensure:
  //   shift/bits_per_word < max_words
  constexpr BigUIntImpl& AddLeftShifted(BigUIntWord add, unsigned shift) {
    bool carry = false;
    int pos = shift / bits_per_word;
    unsigned shift_mod = shift % bits_per_word;
    words_[pos] = words_[pos].Add(add << shift_mod, &carry);
    pos++;
    if (pos < max_words && shift_mod) {
      words_[pos] = words_[pos].Add(add >> (bits_per_word - shift_mod),
                                    carry, &carry);
      pos++;
    }
    for (; pos < max_words && carry; pos++) {
      words_[pos] = words_[pos].Add(carry, &carry);
    }
    used_ = pos * bytes_per_word;
    Trim();
    return *this;
  }

  // Subtracts (other << shift) from this. The caller must ensure:
  //   shift/bits_per_word < max_words
  template <int other_words>
  constexpr BigUIntImpl& SubtractLeftShifted(const BigUIntImpl<other_words>& other, unsigned shift) {
    int in = 0;
    int out = shift / bits_per_word;
    const int word_left_shift = shift % bits_per_word;
    bool carry = false;
    // The if statement is necessary to avoid shifting by bits_per_word.
    if (word_left_shift > 0) {
      BigUIntWord prev(0);
      const int prev_right_shift = bits_per_word - word_left_shift;
      for (; in < other.used_words() && out < max_words; in++, out++) {
        BigUIntWord subtract = other.words_[in] << word_left_shift |
                                           prev >> prev_right_shift;
        words_[out] = words_[out].Subtract(subtract, carry, &carry);
        prev = other.words_[in];
      }
      if (out < max_words) {
        words_[out] = words_[out].Subtract(prev >> prev_right_shift, carry, &carry);
        out++;
      }
    } else {
      for (; in < other.used_words() && out < max_words; in++, out++) {
        words_[out] = words_[out].Subtract(other.words_[in], carry, &carry);
      }
    }
    for (; carry && out < max_words; out++) {
      words_[out] = words_[out].Subtract(carry, &carry);
    }
    used_ = std::max(out * bytes_per_word, used_);
    Trim();
    return *this;
  }

  // Subtracts (other << shift) from this. The caller must ensure:
  //   shift/bits_per_word < max_words
  constexpr BigUIntImpl& Subtract(BigUIntWord other) {
    int out = 0;
    bool carry = false;
    words_[out] = words_[out].Subtract(other, &carry);
    out++;
    for (; carry && out < max_words; out++) {
      words_[out] = words_[out].Subtract(carry, &carry);
    }
    used_ = std::max(out * bytes_per_word, used_);
    Trim();
    return *this;
  }

  constexpr BigUIntImpl& ShiftRightWord() {
    BigUIntWord prev_word{0};
    for (int i = used_words() - 1; i >= 0; i--) {
      std::swap(words_[i], prev_word);
    }
    Trim();
    return *this;
  }

 private:

  // The number of bytes used in words_.
  //
  // Invariant:
  //   Either used_ == sizeof(BigUIntHalfWord), or used_ is a multiple of
  //   BigUIntWord::bytes_per_word.
  //
  // Even if used_ == sizeof(BigUIntHalfWord), the rest of the bits in words_[0] are
  // still initialized.
  int used_;

  // words_[0] holds the lowest significant bits. Within each element of
  // words_, the bit order is in the machine native order.
  BigUIntWord words_[max_words];

  constexpr void Trim() {
    while (used_ > BigUIntWord::bytes_per_word) {
      if (words_[used_ / BigUIntWord::bytes_per_word - 1] != 0)
        break;
      used_-= BigUIntWord::bytes_per_word;
    }
    if (used_ == BigUIntWord::bytes_per_word &&
        words_[0] <= std::numeric_limits<BigUIntHalfWord>::max()) {
      used_ = sizeof(BigUIntHalfWord);
    }
  }

  constexpr int used_words() const {
    return (used_ + BigUIntWord::bytes_per_word - 1) / BigUIntWord::bytes_per_word;
  }

  template <int other_words>
  constexpr int GetCommonWordCount(const BigUIntImpl<other_words>& other) const {
    return (std::min(used_, other.used_) +
            BigUIntWord::bytes_per_word - 1) / BigUIntWord::bytes_per_word;
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

  // Divide `this` by `other`. Return the quotient and store the remainder in `remainder_out`.
  //
  // This function does not have any small word shortcuts.
  template <int other_words>
  constexpr BigUIntImpl<max_words> DivideRemainderSlow(const BigUIntImpl<other_words>& other,
      BigUIntImpl<std::min(max_words, other_words)>* remainder_out) const {
    // Dividing a 64 bit number by a 32 bit number (with the top bit set)
    // creates a remainder with at most 33 integer bits, with trailing fraction
    // bits.
    //
    // 0 <= a < 2^64
    // 2^31 <= b + 1 < 2^32
    // 2^31 - 1 <= b < 2^32 - 1
    //
    // remainder = a - ((a-1)/(b+1))*b
    //           = a - (b*(a-1)/(b+1))
    //           = (a*(b+1))/(b+1) - (b*(a-1)/(b+1))
    //           = (a*(b+1) - b*(a-1))/(b+1)
    //           = (a*b + a - b*a - b)/(b+1)
    //           = (a - b)/(b+1)
    //           = a/(b+1) - b/(b+1)
    //           < 2^64 / (2^31 - 1 + 1) - 0.9
    //           = 2^33 - 0.9
    // 
    // So dividing by a 32 bit number (with the top bit set) maximizes the
    // progress of the algorithm. Unfortunately it progresses at about 31 bits
    // on each round.

    int other_highest_word_index = other.used_words() - 1;
    BigUIntWord other_highest_word = other.words_[other_highest_word_index];
    int other_highest_bit = other_highest_word.GetHighestSetBit();
    // `other_shifted` is `other` either shifted to the left or the right:
    //   other_shifted = other * 2^x
    //
    // such that:
    //   0 < other_shifted < 2^(bits_per_word/2)
    BigUIntWord other_shifted;
    const int other_word_shift_right = other_highest_bit - bits_per_word/2;
    if (other_highest_bit < bits_per_word/2) {
      other_shifted = (other_highest_word << -other_word_shift_right);
      if (other_highest_word_index > 0) {
        other_shifted |= other.words_[other_highest_word_index - 1] >> (bits_per_word + other_word_shift_right);
      }
    } else {
      other_shifted = other_highest_word >> other_word_shift_right;
    }
    int other_shift_right = other_highest_word_index * bits_per_word + other_word_shift_right;
    if (other_shifted.low_half_word() == static_cast<BigUIntHalfWord>(-1)) {
      other_shifted = BigUIntWord{1} << (bits_per_word/2 - 1);
      other_shift_right++;
    } else {
      ++other_shifted;
    }

    BigUIntImpl<max_words> quotient;
    BigUIntImpl<max_words + 2> remainder = this->operator<< <max_words+2>(bits_per_word);
    // Number of bits to shift (*this) to the right, such that:
    // 0 < (*this >> this_shift_right_bits) < 2^bits_per_word
    int this_shift_right_bits = (used_ - bytes_per_word) * bits_per_byte;
    for (;
          this_shift_right_bits >= other_shift_right;
        this_shift_right_bits -= bits_per_word/2 - 1) {
      BigUIntWord this_shifted = remainder.GetWordAtBitOffset(this_shift_right_bits + bits_per_word);

      int shift_result_left = this_shift_right_bits - other_shift_right;
      BigUIntWord result = this_shifted / other_shifted;
      quotient.AddLeftShifted(result, shift_result_left);
      remainder.SubtractLeftShifted(other.Multiply(result), shift_result_left + bits_per_word);
    }
    for (;
          this_shift_right_bits + bits_per_word/2 >= other_shift_right;
        this_shift_right_bits -= bits_per_word/2 - 1) {
      BigUIntWord this_shifted = remainder.GetWordAtBitOffset(this_shift_right_bits + bits_per_word);

      int shift_result_right = other_shift_right - this_shift_right_bits;
      BigUIntWord result = this_shifted / other_shifted;
      quotient.AddLeftShifted(result >> shift_result_right, 0);
      remainder = remainder.SubtractLeftShifted(other.Multiply(result >> shift_result_right), bits_per_word);
    }
    assert(remainder.words_[0].low_half_word() == 0);
    remainder.ShiftRightWord();
    if (remainder >= other) {
      remainder = remainder.Subtract(other);
      quotient.AddLeftShifted(BigUIntWord{1}, 0);
    }
#if 0
    if (other_shift_right <= 0) {
      BigUIntWord result = remainder.words_[0] / other.words_[0];
      quotient.AddLeftShifted(result, 0);
      BigUIntWord unused_high;
      remainder.Subtract(other.words_[0].Multiply(result, &unused_high));
    }
#endif
    remainder.Trim();
    *remainder_out = remainder;
    return quotient;
  }

};

// `max_bits` is the maximum number of bits BigUInt can hold. For example
// BigUInt<128> can hold 128 bit unsigned integers.
template <int max_bits>
using BigUInt = BigUIntImpl<(max_bits + BigUIntWord::bits_per_word - 1) / BigUIntWord::bits_per_word>;

}  // walnut

#endif // WALNUT_BIG_UINT_H__
