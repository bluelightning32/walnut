#ifndef WALNUT_BIG_UINT_H__
#define WALNUT_BIG_UINT_H__

// For std::min
#include <algorithm>
// For assert
#include <cassert>

#include "walnut/big_int_words.h"
#include "walnut/big_uint_word.h"

namespace walnut {

class BigUInt {
 public:
  static constexpr size_t bits_per_word = BigIntWords::bits_per_word;

  constexpr BigUInt() : BigUInt(static_cast<BigUIntHalfWord>(0)) {
  }

  explicit constexpr BigUInt(BigUIntHalfWord value) : words_(1) {
    words_[0] = value;
  }

  explicit constexpr BigUInt(BigUIntWord value) : words_(1) {
    words_[0] = value;
  }

  explicit constexpr BigUInt(uint64_t value) : BigUInt(BigUIntWord(value)) { }

  constexpr BigUInt(const BigUInt& other) :
      words_(other.words_) {
    Trim();
  }

  constexpr BigUInt(const BigIntWords& words, size_t used) :
      words_(words, used) {
    Trim();
  }

  constexpr const BigIntWords& words() const {
    return words_;
  }

  constexpr size_t used_words() const {
    return words_.size();
  }

  constexpr BigUIntWord word(size_t i) const {
    return words_[i];
  }

  constexpr bool IsHalfWord() const {
    return used_words() == 1 && CanTrimLastHalf(word(0));
  }

  BigUInt operator << (size_t shift) const {
    if (IsHalfWord() && shift <= 32) {
      return BigUInt(words_[0].low_uint64() << shift);
    }

    BigUInt result;
    size_t in = 0;
    size_t out = shift / bits_per_word;
    const int word_left_shift = shift % bits_per_word;
    // The if statement is necessary to avoid shifting by bits_per_word.
    if (word_left_shift > 0) {
      size_t copy = used_words();
      size_t allocate = copy + out + 1;
      result.words_.resize(allocate);
      BigUIntWord prev(0);
      const int prev_right_shift = bits_per_word - word_left_shift;
      for (; in < copy; in++, out++) {
        result.words_[out] = words_[in] << word_left_shift |
                               prev >> prev_right_shift;
        prev = words_[in];
      }
      result.words_[out] = prev >> prev_right_shift;
      out++;
    } else {
      size_t copy = used_words();
      result.words_.resize(copy + out);
      for (; in < copy; in++, out++) {
        result.words_[out] = words_[in];
      }
    }
    assert(result.used_words() == out);
    result.Trim();
    return result;
  }

  BigUInt Subtract(const BigUInt& other) const {
    if (IsHalfWord() && other.IsHalfWord() &&
        words_[0].low_uint32() >= other.words_[0].low_uint32()) {
      return BigUInt(words_[0].Subtract(other.words_[0]));
    }
    BigUInt result;
    result.words_.resize(std::max(used_words(), other.used_words()));
    size_t i = 0;
    bool carry = false;
    size_t common_words = std::min(words_.size(), other.words_.size());
    for (; i < common_words; i++) {
      result.words_[i] = words_[i].Subtract(other.words_[i], carry, &carry);
    }
    for (; i < used_words(); i++) {
      result.words_[i] = words_[i].Subtract(carry, &carry);
    }
    for (; i < other.used_words(); i++) {
      result.words_[i] = BigUIntWord(0).Subtract(other.words_[i], carry, &carry);
    }
    assert(!carry);
    assert(used_words() == i);
    result.Trim();
    return result;
  }

  BigUInt Multiply(BigUIntWord other) const {
    if (IsHalfWord() &&
        other <= std::numeric_limits<BigUIntHalfWord>::max()) {
      return BigUInt(words_[0].MultiplyAsHalfWord(other));
    }
    BigUInt result;
    result.words_.resize(used_words() + 1);
    size_t k = 0;
    BigUIntWord add;
    for (size_t i = 0; i < used_words(); ++i, ++k) {
      result.words_[k] = other.MultiplyAdd(words_[i], add, /*carry_in=*/false,
                                           &add);
    }
    result.words_[k] = add;
    k++;
    assert(result.used_words() == k);
    result.Trim();
    return result;
  }

  // Divide `this` by `other`. Return the quotient and store the remainder in `remainder_out`.
  BigUInt DivideRemainder(const BigUInt& other, BigUInt* remainder_out) const {
    if (used_words() == 1 && other.used_words() == 1) {
      *remainder_out = BigUInt{words_[0] % other.words_[0]};
      return BigUInt{words_[0] / other.words_[0]};
    }
    return DivideRemainderSlow(other, remainder_out);
  }

  constexpr bool operator >= (const BigUInt& other) const {
    if (used_words() > other.used_words()) return true;
    if (used_words() < other.used_words()) return false;

    for (size_t i = used_words() - 1; i > 0; i--) {
      if (words_[i] > other.words_[i]) return true;
      if (words_[i] < other.words_[i]) return false;
    }
    return words_[0] >= other.words_[0];
  }

  // Adds (add << shift) to this.
  constexpr BigUInt& AddLeftShifted(BigUIntWord add, unsigned shift) {
    bool carry = false;
    size_t pos = shift / bits_per_word;
    unsigned shift_mod = shift % bits_per_word;
    size_t old_used = used_words();
    // Fix an unused variable warning in release builds.
    (void)(old_used);
    words_.resize(std::max(used_words(),
                           (pos + 1 + (shift_mod != 0))),
                  BigUIntWord{0});
    words_[pos] = words_[pos].Add(add << shift_mod, &carry);
    pos++;
    if (shift_mod) {
      words_[pos] = words_[pos].Add(add >> (bits_per_word - shift_mod),
                                    carry, &carry);
      pos++;
    }
    for (; carry; pos++) {
      words_.resize(std::max(used_words(), (pos + 1)));
      words_[pos] = words_[pos].Add(carry, &carry);
    }
    assert(used_words() == std::max(old_used, pos));
    Trim();
    return *this;
  }

  // Subtracts (other << shift) from this.
  constexpr BigUInt& SubtractLeftShifted(const BigUInt& other, unsigned shift) {
    size_t in = 0;
    size_t out = shift / bits_per_word;
    const int word_left_shift = shift % bits_per_word;
    bool carry = false;
    // The if statement is necessary to avoid shifting by bits_per_word.
    if (word_left_shift > 0) {
      BigUIntWord prev(0);
      const size_t prev_right_shift = bits_per_word - word_left_shift;
      for (; in < other.used_words(); in++, out++) {
        BigUIntWord subtract = other.words_[in] << word_left_shift |
                                           prev >> prev_right_shift;
        words_[out] = words_[out].Subtract(subtract, carry, &carry);
        prev = other.words_[in];
      }
      if (out < used_words()) {
        words_[out] = words_[out].Subtract(prev >> prev_right_shift, carry, &carry);
        out++;
      } else {
        assert((prev >> prev_right_shift) == 0);
      }
    } else {
      for (; in < other.used_words(); in++, out++) {
        words_[out] = words_[out].Subtract(other.words_[in], carry, &carry);
      }
    }
    for (; carry && out < used_words(); out++) {
      words_[out] = words_[out].Subtract(carry, &carry);
    }
    assert(!carry);
    assert(out <= used_words());
    Trim();
    return *this;
  }

  constexpr BigUInt& ShiftRightWord() {
    BigUIntWord prev_word{0};
    for (int i = used_words() - 1; i >= 0; i--) {
      std::swap(words_[i], prev_word);
    }
    Trim();
    return *this;
  }

 protected:
  static constexpr bool CanTrim(BigUIntWord low, BigUIntWord high) {
    return high == 0;
  }

  static constexpr bool CanTrimLastHalf(BigUIntWord last) {
    return last <= std::numeric_limits<BigUIntHalfWord>::max();
  }

  constexpr void Trim() {
    int i = used_words() - 1;
    if (i > 0) {
      BigUIntWord check = words_[i];
      BigUIntWord next;
      do {
        --i;
        next = words_[i];

        if (!CanTrim(/*low=*/next, /*high=*/check)) break;

        check = next;
        words_.resize(i + 1);
      } while (i > 0);
    }
  }

  // Gets the lowest word after shifting this to the right `shift` bits.
  //
  // The caller must ensure:
  // `shift` is greater than or equal to 0.
  BigUIntWord GetAtBitOffset(unsigned shift) const {
    size_t word_index = shift / bits_per_word;
    size_t word_offset = shift % bits_per_word;
    BigUIntWord next{0};
    if (word_index + 1 < words_.size()) {
      next = words_[word_index+1];
    }
    return words_[word_index].ShiftRight(next, word_offset);
  }

  // Divide `this` by `other`. Return the quotient and store the remainder in `remainder_out`.
  //
  // This function does not have any small word shortcuts.
  BigUInt DivideRemainderSlow(const BigUInt& other,
                              BigUInt* remainder_out) const {
    // Dividing a 64 bit number by a 32 bit number (with the top bit set)
    // creates a remainder with at most 34 integer bits, with trailing fraction
    // bits.
    //
    // 0 <= a < 2^64
    // 2^31 <= b + 1 < 2^32
    //
    // remainder =  a - b * floor(a/(b+1))
    //           <= a - b * (a/(b+1) - 1)
    //           =  a - b * (a/(b+1) + b
    //           =  a - a*b/(b+1) + b
    //           =  a * (1 - b/(b+1)) + b
    //           =  a * ( (b+1)/(b+1) - b/(b+1) ) + b
    //           =  a * (1/(b+1)) + b
    //           =  a/(b+1) + b
    //
    // 
    // So dividing by a 32 bit number (with the top bit set) maximizes the
    // progress of the algorithm. Unfortunately it progresses at about 30 bits
    // on each round.

    int other_highest_word_index = other.used_words() - 1;
    BigUIntWord other_highest_word = other.words_[other_highest_word_index];
    const size_t other_highest_bit = other_highest_word.GetHighestSetBit();
    // `other_shifted` is `other` either shifted to the left or the right:
    //   other_shifted = other * 2^x
    //
    // such that:
    //   0 < other_shifted < 2^(bits_per_word/2)
    BigUIntWord other_shifted;
    const int other_word_shift_right = other_highest_bit - bits_per_word/2;
    if (other_highest_bit < int(bits_per_word)/2) {
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
    // other_shift_right may be negative, 0, or positive.

    BigUInt quotient;
    BigUInt remainder = operator<< (bits_per_word);
    // Number of bits to shift (*this) to the right, such that:
    // 0 < (*this >> this_shift_right_bits) < 2^bits_per_word
    int this_shift_right_bits = (used_words() - 1) * bits_per_word;
    for (;
          this_shift_right_bits >= other_shift_right;
          this_shift_right_bits -= bits_per_word/2 - 2) {
      BigUIntWord this_shifted = remainder.GetAtBitOffset(this_shift_right_bits + bits_per_word);

      int shift_result_left = this_shift_right_bits - other_shift_right;
      BigUIntWord result = this_shifted / other_shifted;
      quotient.AddLeftShifted(result, shift_result_left);
      remainder.SubtractLeftShifted(other.Multiply(result), shift_result_left + bits_per_word);
      assert(remainder.GetAtBitOffset(this_shift_right_bits + bits_per_word) <= BigUIntWord{1}<<34);
    }
    for (;
          this_shift_right_bits + int(bits_per_word)/2 >= other_shift_right;
          this_shift_right_bits -= bits_per_word/2 - 2) {
      BigUIntWord this_shifted = remainder.GetAtBitOffset(this_shift_right_bits + bits_per_word);

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
    *remainder_out = remainder;
    return quotient;
  }

  BigIntWords words_;
};

}  // walnut

#endif // WALNUT_BIG_UINT_H__
