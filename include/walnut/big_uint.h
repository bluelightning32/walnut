#ifndef WALNUT_BIG_UINT_H__
#define WALNUT_BIG_UINT_H__

#include <algorithm>
#include <cassert>

#include "walnut/big_int_words.h"
#include "walnut/big_uint_word.h"

namespace walnut {

template <size_t max_words_template>
class BigUIntImpl {
  template <size_t other_words>
  friend class BigUIntImpl;

  using Storage = BigIntWords<max_words_template>;

 public:
  static constexpr size_t bits_per_word = Storage::bits_per_word;
  static constexpr size_t max_bits = Storage::max_bits;
  static constexpr size_t max_words = Storage::max_words;

  constexpr BigUIntImpl() : BigUIntImpl(static_cast<BigUIntHalfWord>(0)) {
  }

  explicit constexpr BigUIntImpl(BigUIntHalfWord value) : words_(1) {
    words_[0] = value;
  }

  explicit constexpr BigUIntImpl(BigUIntWord value) : words_(1) {
    words_[0] = value;
  }

  explicit constexpr BigUIntImpl(uint64_t value) : BigUIntImpl(BigUIntWord(value)) { }

  template <size_t other_max_words>
  constexpr BigUIntImpl(const BigUIntImpl<other_max_words>& other) :
      words_(other.words_) {
    Trim();
  }

  template <size_t other_max_words>
  constexpr BigUIntImpl(const BigIntWords<other_max_words>& words, size_t used) :
      words_(words, used) {
    Trim();
  }

  constexpr const Storage& words() const {
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

  template <size_t result_words=max_words>
  constexpr BigUIntImpl<result_words> operator << (size_t shift) const {
    if (IsHalfWord() && shift <= 32) {
      return BigUIntImpl<result_words>(words_[0].low_uint64() << shift);
    }

    BigUIntImpl<result_words> result;
    if (shift >= result.max_bits) return result;

    size_t in = 0;
    size_t out = shift / bits_per_word;
    const int word_left_shift = shift % bits_per_word;
    // The if statement is necessary to avoid shifting by bits_per_word.
    if (word_left_shift > 0) {
      size_t copy = std::min(used_words(), result_words - out);
      size_t allocate = std::min(copy + out + 1, result_words);
      result.words_.resize(allocate);
      BigUIntWord prev(0);
      const int prev_right_shift = bits_per_word - word_left_shift;
      for (; in < copy; in++, out++) {
        result.words_[out] = words_[in] << word_left_shift |
                               prev >> prev_right_shift;
        prev = words_[in];
      }
      if (out < result_words) {
        result.words_[out] = prev >> prev_right_shift;
        out++;
      }
    } else {
      size_t copy = std::min(used_words(), result_words - out);
      result.words_.resize(copy + out);
      for (; in < copy; in++, out++) {
        result.words_[out] = words_[in];
      }
    }
    assert(result.used_words() == out);
    result.Trim();
    return result;
  }

  template <size_t result_words = 0, size_t other_words,
            size_t rw = result_words == 0 ?
              std::max(max_words, other_words) : result_words>
  constexpr BigUIntImpl<rw> Subtract(const BigUIntImpl<other_words>& other) const {
    if (IsHalfWord() && other.IsHalfWord() &&
        words_[0].low_uint32() >= other.words_[0].low_uint32()) {
      return BigUIntImpl<rw>(words_[0].Subtract(other.words_[0]));
    }
    BigUIntImpl<rw> result;
    result.words_.resize(std::min(std::max(used_words(), other.used_words()),
                                  size_t(BigUIntImpl<rw>::max_words)));
    size_t i = 0;
    bool carry = false;
    size_t common_words = std::min(words_.size(), other.words_.size());
    for (; i < common_words && i < rw; i++) {
      result.words_[i] = words_[i].Subtract(other.words_[i], carry, &carry);
    }
    for (; i < std::min(used_words(), rw); i++) {
      result.words_[i] = words_[i].Subtract(carry, &carry);
    }
    for (; i < std::min(other.used_words(), rw); i++) {
      result.words_[i] = BigUIntWord(0).Subtract(other.words_[i], carry, &carry);
    }
    assert(!carry);
    assert(used_words() == i);
    result.Trim();
    return result;
  }

  template <size_t result_words = max_words + 1>
  constexpr BigUIntImpl<result_words> Multiply(BigUIntWord other) const {
    if (IsHalfWord() &&
        other <= std::numeric_limits<BigUIntHalfWord>::max()) {
      return BigUIntImpl<result_words>(words_[0].MultiplyAsHalfWord(other));
    }
    BigUIntImpl<result_words> result;
    result.words_.resize(std::min(used_words() + 1, result_words));
    size_t k = 0;
    BigUIntWord add;
    for (size_t i = 0; i < used_words(); ++i, ++k) {
      result.words_[k] = other.MultiplyAdd(words_[i], add, /*carry_in=*/false,
                                           &add);
    }
    if (k < result_words) {
      result.words_[k] = add;
      k++;
    }
    assert(result.used_words() == k);
    result.Trim();
    return result;
  }

  // Divide `this` by `other`. Return the quotient and store the remainder in `remainder_out`.
  template <size_t other_words>
  constexpr BigUIntImpl<max_words> DivideRemainder(const BigUIntImpl<other_words>& other,
      BigUIntImpl<std::min(max_words, other_words)>* remainder_out) const {
    if (used_words() == 1 && other.used_words() == 1) {
      *remainder_out = BigUIntImpl<std::min(max_words, other_words)>{words_[0] % other.words_[0]};
      return BigUIntImpl<max_words>{words_[0] / other.words_[0]};
    }
    return DivideRemainderSlow(other, remainder_out);
  }

  template <size_t other_max_words>
  constexpr bool operator >= (const BigUIntImpl<other_max_words>& other) const {
    if (used_words() > other.used_words()) return true;
    if (used_words() < other.used_words()) return false;

    for (size_t i = used_words() - 1; i > 0; i--) {
      if (words_[i] > other.words_[i]) return true;
      if (words_[i] < other.words_[i]) return false;
    }
    return words_[0] >= other.words_[0];
  }

  // Adds (add << shift) to this. The caller must ensure:
  //   shift/bits_per_word < max_words
  constexpr BigUIntImpl& AddLeftShifted(BigUIntWord add, unsigned shift) {
    bool carry = false;
    size_t pos = shift / bits_per_word;
    unsigned shift_mod = shift % bits_per_word;
    size_t old_used = used_words();
    words_.resize(std::max(used_words(),
                           (pos + 1 + (pos + 1 < max_words && shift_mod))));
    words_[pos] = words_[pos].Add(add << shift_mod, &carry);
    pos++;
    if (pos < max_words && shift_mod) {
      words_[pos] = words_[pos].Add(add >> (bits_per_word - shift_mod),
                                    carry, &carry);
      pos++;
    }
    for (; pos < max_words && carry; pos++) {
      words_.resize(std::max(used_words(), (pos + 1)));
      words_[pos] = words_[pos].Add(carry, &carry);
    }
    assert(used_words() == std::max(old_used, pos));
    Trim();
    return *this;
  }

  // Subtracts (other << shift) from this. The caller must ensure:
  //   shift/bits_per_word < max_words
  template <size_t other_words>
  constexpr BigUIntImpl& SubtractLeftShifted(const BigUIntImpl<other_words>& other, unsigned shift) {
    size_t in = 0;
    size_t out = shift / bits_per_word;
    const int word_left_shift = shift % bits_per_word;
    bool carry = false;
    // The if statement is necessary to avoid shifting by bits_per_word.
    if (word_left_shift > 0) {
      BigUIntWord prev(0);
      const size_t prev_right_shift = bits_per_word - word_left_shift;
      for (; in < other.used_words() && out < max_words; in++, out++) {
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
      for (; in < other.used_words() && out < max_words; in++, out++) {
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

  constexpr BigUIntImpl& ShiftRightWord() {
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

  // Divide `this` by `other`. Return the quotient and store the remainder in `remainder_out`.
  //
  // This function does not have any small word shortcuts.
  template <size_t other_words>
  constexpr BigUIntImpl<max_words> DivideRemainderSlow(const BigUIntImpl<other_words>& other,
      BigUIntImpl<std::min(max_words, other_words)>* remainder_out) const {
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

    BigUIntImpl<max_words> quotient;
    BigUIntImpl<max_words + 2> remainder = operator<< <max_words+2>(bits_per_word);
    // Number of bits to shift (*this) to the right, such that:
    // 0 < (*this >> this_shift_right_bits) < 2^bits_per_word
    int this_shift_right_bits = (used_words() - 1) * bits_per_word;
    for (;
          this_shift_right_bits >= other_shift_right;
          this_shift_right_bits -= bits_per_word/2 - 2) {
      BigUIntWord this_shifted = remainder.words_.GetAtBitOffset(this_shift_right_bits + bits_per_word);

      int shift_result_left = this_shift_right_bits - other_shift_right;
      BigUIntWord result = this_shifted / other_shifted;
      quotient.AddLeftShifted(result, shift_result_left);
      remainder.SubtractLeftShifted(other.Multiply(result), shift_result_left + bits_per_word);
      assert(remainder.words_.GetAtBitOffset(this_shift_right_bits + bits_per_word) <= BigUIntWord{1}<<34);
    }
    for (;
          this_shift_right_bits + int(bits_per_word)/2 >= other_shift_right;
          this_shift_right_bits -= bits_per_word/2 - 2) {
      BigUIntWord this_shifted = remainder.words_.GetAtBitOffset(this_shift_right_bits + bits_per_word);

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

  Storage words_;
};

}  // walnut

#endif // WALNUT_BIG_UINT_H__
