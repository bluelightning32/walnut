#ifndef WALNUT_BIG_INT_H__
#define WALNUT_BIG_INT_H__

// For std::min
#include <algorithm>
// For assert
#include <cassert>
// For ostream
#include <iostream>

#include "walnut/big_int_words.h"
#include "walnut/big_uint.h"

namespace walnut {

class BigInt {
 public:
  static constexpr size_t bits_per_word = BigIntWords::bits_per_word;
  static constexpr size_t bytes_per_word = BigIntWords::bytes_per_word;

  struct FlippableCompare {
    FlippableCompare(bool flip) : flip(flip) { }

    bool operator()(const BigInt& a, const BigInt& b) {
      return a.LessThan(flip, b);
    }

    bool flip;
  };

  constexpr BigInt() : BigInt(0) {
  }

  explicit constexpr BigInt(BigIntHalfWord value) {
    words_[0] = BigUIntWord{value};
  }

  explicit constexpr BigInt(BigIntWord value) {
    words_[0] = BigUIntWord{value};
  }

  BigInt(const BigInt&) = default;
  constexpr BigInt(BigInt&&) = default;

  constexpr BigInt(const BigUInt& other)
   : words_(/*used_words=*/other.used_words() +
            (BigIntWord{other.word(other.used_words() - 1)} < 0),
            /*copy_words=*/other.used_words(), /*from=*/other.words()) {
     Trim();
  }

  constexpr size_t used_words() const {
    return words_.size();
  }

  constexpr BigUIntWord word(size_t i) const {
    return words_[i];
  }

  constexpr uint32_t low_uint32() const {
    return word(0).low_uint32();
  }

  constexpr uint64_t low_uint64() const {
    return word(0).low_uint64();
  }

  constexpr int ToInt() const {
    assert(used_words() == 1);
    return word(0).ToInt();
  }

  constexpr bool IsHalfWord() const {
    return used_words() == 1 && CanTrimLastHalf(word(0));
  }

  constexpr BigInt& operator = (
      const BigInt& other) {
    words_.Assign(other.words_, other.used_words());
    this->Trim();
    return *this;
  }

  constexpr BigInt& operator = (BigIntWord value) {
    words_.resize(1);
    words_[0] = value;
    return *this;
  }

  constexpr BigInt& operator = (const BigUInt& other) {
    int copy_words = other.used_words();
    words_.Assign(other.words(), copy_words);
    if (BigIntWord{word(used_words() - 1)} < 0) {
      words_.push_back(BigUIntWord{0});
    }
    Trim();
    return *this;
  }

  static BigInt max_value(size_t set_bits);

  static BigInt min_value(size_t clear_bits);

  BigInt operator << (size_t shift) const {
    BigInt result;
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
      for (size_t i = 0; i < out; ++i) {
        result.words_[i] = 0;
      }
      for (; in < copy; in++, out++) {
        result.words_[out] = words_[in] << word_left_shift |
                               prev >> prev_right_shift;
        prev = words_[in];
      }
      result.words_[out] = BigIntWord(prev) >> prev_right_shift;
      out++;
      assert(result.used_words() == out);
    } else {
      size_t copy = used_words();
      result.words_.resize(copy + out);
      for (size_t i = 0; i < out; ++i) {
        result.words_[i] = 0;
      }
      for (; in < copy; in++, out++) {
        result.words_[out] = words_[in];
      }
    }
    result.Trim();
    return result;
  }

  // Returns -1 if *this < `other`,
  //          0 if *this ==  `other`, or
  //          1 if *this > `other`.
  constexpr int Compare(const BigInt& other) const {
    if (used_words() < other.used_words()) {
      // `this` is closer to 0 than `other`.
      // Since used_words() < other_used, other != 0.
      //
      // If other is positive, then *this < other, so return -1.
      // If other is negative, then *this > other, so return 1.
      //
      // So return -1 if the upper word of other >= 0, or return 1 if the upper
      // word of other < 0.
      int other_sign_extension =
        BigIntWord{other.words_[other.used_words() - 1].SignExtension()};
      return ~other_sign_extension | 1;
    }
    if (used_words() > other.used_words()) {
      // `other` is closer to 0 than `this`.
      // Since used_words() > other_used, *this != 0.
      //
      // If *this is positive, then *this > other, so return 1.
      // If *this is negative, then *this < other, so return -1.
      //
      // So return 1 if the upper word of *this >= 0, or return -1 if the upper
      // word of *this < 0.
      int sign_extension =
        BigIntWord{words_[used_words() - 1].SignExtension()};
      return sign_extension | 1;
    }
    size_t i = used_words() - 1;
    // Perform a signed comparison on the upper words (which contains the sign
    // bits) and an unsigned comparison on the following words.
    if (words_[i] != other.words_[i]) {
      return (BigIntWord{words_[i]} > BigIntWord{other.words_[i]}) ? 1 : -1;
    }
    for (i--; ssize_t(i) >= 0; i--) {
      if (words_[i] > other.words_[i]) return 1;
      if (words_[i] < other.words_[i]) return -1;
    }
    return 0;
  }

  BigInt Add(const BigInt& other) const {
    if (IsHalfWord() && other.IsHalfWord()) {
      return BigInt(BigIntWord{words_[0].Add(other.words_[0])});
    }
    BigInt result;
    result.words_.resize(std::max(used_words(), other.used_words()) + 1);
    size_t i = 0;
    bool carry = false;
    size_t common_words = std::min(words_.size(), other.words_.size());
    for (; i < common_words; i++) {
      result.words_[i] = words_[i].Add(other.words_[i], carry, &carry);
    }
    BigUIntWord this_extension(SignExtension());
    BigUIntWord other_extension(other.SignExtension());
    for (; i < used_words(); i++) {
      result.words_[i] = words_[i].Add(other_extension, carry, &carry);
    }
    for (; i < other.used_words(); i++) {
      result.words_[i] = this_extension.Add(other.words_[i], carry, &carry);
    }
    result.words_[i] = this_extension.Add(other_extension, carry, &carry);
    i++;
    assert(result.used_words() == i);
    result.Trim();
    return result;
  }

  BigInt operator+(const BigInt& other) const {
    return Add(other);
  }

  BigInt operator+(int other) const {
    return *this + BigInt(other);
  }

  constexpr BigInt& operator+=(const BigInt& other) {
    if (IsHalfWord() && other.IsHalfWord()) {
      words_[0] += other.words_[0];
      if ((BigIntWord)words_[0] < std::numeric_limits<BigIntHalfWord>::min() ||
          (BigIntWord)words_[0] > std::numeric_limits<BigIntHalfWord>::max()) {
        words_.resize(1);
      }
      return *this;
    }
    size_t i = 0;
    bool carry = false;
    size_t common_words = std::min(words_.size(), other.words_.size());
    BigUIntWord this_extension(SignExtension());
    BigUIntWord other_extension(other.SignExtension());
    size_t old_used_words = used_words();
    words_.resize(std::max(old_used_words, other.used_words()) + 1);
    for (; i < common_words; i++) {
      words_[i] = words_[i].Add(other.words_[i], carry, &carry);
    }
    for (; i < old_used_words; i++) {
      words_[i] = words_[i].Add(other_extension, carry, &carry);
    }
    for (; i < other.used_words(); i++) {
      words_[i] = this_extension.Add(other.words_[i], carry, &carry);
    }
    words_[i] = this_extension.Add(other_extension, carry, &carry);
    i++;
    assert(used_words() == i);
    Trim();
    return *this;
  }

  constexpr BigInt& operator+=(BigIntHalfWord other) {
    if (IsHalfWord()) {
      words_[0] += BigUIntWord{other};
      if ((BigIntWord)words_[0] < std::numeric_limits<BigIntHalfWord>::min() ||
          (BigIntWord)words_[0] > std::numeric_limits<BigIntHalfWord>::max()) {
        words_.resize(1);
      }
      return *this;
    } else {
      return *this += BigInt(other);
    }
  }

  constexpr BigInt& operator++() {
    if (IsHalfWord()) {
      ++words_[0];
      if ((BigIntWord)words_[0] > std::numeric_limits<BigIntHalfWord>::max()) {
        words_.resize(1);
      }
      return *this;
    }
    size_t i = 0;
    for (; i < used_words() - 1; i++) {
      ++words_[i];
      if (words_[i] != 0) {
        return *this;
      }
    }
    ++words_[i];
    if ((BigIntWord)words_[i] == std::numeric_limits<BigIntWord>::min()) {
      ++i;
      ++words_[i] = 0;
    } else {
      Trim();
    }
    return *this;
  }

  constexpr BigInt& operator-=(const BigInt& other) {
    if (IsHalfWord() && other.IsHalfWord()) {
      words_[0] -= other.words_[0];
      if ((BigIntWord)words_[0] < std::numeric_limits<BigIntHalfWord>::min() ||
          (BigIntWord)words_[0] > std::numeric_limits<BigIntHalfWord>::max()) {
        words_.resize(1);
      }
      return *this;
    }
    size_t i = 0;
    bool carry = false;
    size_t common_words = std::min(words_.size(), other.words_.size());
    BigUIntWord this_extension(SignExtension());
    BigUIntWord other_extension(other.SignExtension());
    size_t old_used_words = used_words();
    words_.resize(std::max(old_used_words, other.used_words()) + 1);
    for (; i < common_words; i++) {
      words_[i] = words_[i].Subtract(other.words_[i], carry, &carry);
    }
    for (; i < old_used_words; i++) {
      words_[i] = words_[i].Subtract(other_extension, carry, &carry);
    }
    for (; i < other.used_words(); i++) {
      words_[i] = this_extension.Subtract(other.words_[i], carry, &carry);
    }
    words_[i] = this_extension.Subtract(other_extension, carry, &carry);
    i++;
    assert(used_words() == i);
    Trim();
    return *this;
  }

  constexpr BigInt& operator-=(BigIntHalfWord other) {
    if (IsHalfWord()) {
      words_[0] -= BigUIntWord{other};
      if ((BigIntWord)words_[0] < std::numeric_limits<BigIntHalfWord>::min() ||
          (BigIntWord)words_[0] > std::numeric_limits<BigIntHalfWord>::max()) {
        words_.resize(1);
      }
      return *this;
    } else {
      return *this -= BigInt(other);
    }
  }


  constexpr BigInt& operator--() {
    if (IsHalfWord()) {
      --words_[0];
      if ((BigIntWord)words_[0] < std::numeric_limits<BigIntHalfWord>::min()) {
        words_.resize(1);
      }
      return *this;
    }
    size_t i = 0;
    for (; i < used_words() - 1; i++) {
      --words_[i];
      if ((BigIntWord)words_[i] != -1) {
        return *this;
      }
    }
    --words_[i];
    if ((BigIntWord)words_[i] == std::numeric_limits<BigIntWord>::max()) {
      words_.resize(used_words() + 1);
      ++i;
      ++words_[i] = (BigIntWord)-1;
    } else {
      Trim();
    }
    return *this;
  }

  BigInt Subtract(const BigInt& other) const {
    if (IsHalfWord() && other.IsHalfWord()) {
      return BigInt(BigIntWord{words_[0].Subtract(other.words_[0])});
    }
    BigInt result;
    result.words_.resize(std::max(used_words(), other.used_words()) + 1);
    size_t i = 0;
    bool carry = false;
    size_t common_words = std::min(words_.size(), other.words_.size());
    for (; i < common_words; i++) {
      result.words_[i] = words_[i].Subtract(other.words_[i], carry, &carry);
    }
    BigUIntWord this_extension(SignExtension());
    BigUIntWord other_extension(other.SignExtension());
    for (; i < used_words(); i++) {
      result.words_[i] = words_[i].Subtract(other_extension, carry, &carry);
    }
    for (; i < other.used_words(); i++) {
      result.words_[i] = this_extension.Subtract(other.words_[i], carry, &carry);
    }
    result.words_[i] = this_extension.Subtract(other_extension, carry, &carry);
    i++;
    assert(result.used_words() == i);
    result.Trim();
    return result;
  }

  BigInt operator-(const BigInt& other) const {
    return Subtract(other);
  }

  BigInt operator-(int other) const {
    return Subtract(BigInt(other));
  }

  BigInt Multiply(const BigInt& other) const {
    if (IsHalfWord() && other.IsHalfWord()) {
      return BigInt(BigIntWord{words_[0]} * BigIntWord{other.words_[0]});
    }
    if (used_words() == 1 && other.used_words() == 1) {
      BigInt result;
      result.words_.resize(2);
      result.words_[0] = words_[0].Multiply(other.words_[0], &result.words_[1]);
      result.words_[1] -= other.words_[0].SignExtension() & words_[0];
      result.words_[1] -= words_[0].SignExtension() & other.words_[0];
      result.Trim();
      return result;
    }
    BigInt result = MultiplySlow(other);
    result.SubtractLeftShiftedMasked(*this, other.used_words(), other.SignExtension());
    result.SubtractLeftShiftedMasked(other, used_words(), SignExtension());
    result.Trim();
    return result;
  }

  BigInt operator*(const BigInt& other) const {
    return Multiply(other);
  }

  BigInt operator*(const int other) const {
    return Multiply(BigInt(other));
  }

  BigInt& operator*=(const int other) {
    *this = *this * other;
    return *this;
  }

  constexpr bool operator < (const BigInt& other) const {
    if (used_words() < other.used_words()) {
      return 0 <= BigIntWord{other.words_[other.used_words() - 1]};
    }
    if (used_words() > other.used_words()) {
      return BigIntWord{words_[used_words() - 1]} < 0;
    }
    size_t i = used_words() - 1;
    if (words_[i] != other.words_[i]) {
      return BigIntWord{words_[i]} < BigIntWord{other.words_[i]};
    }
    for (i--; ssize_t(i) > 0; i--) {
      if (words_[i] < other.words_[i]) return true;
      if (words_[i] > other.words_[i]) return false;
    }
    return words_[0] < other.words_[0];
  }

  constexpr bool operator < (int other) const {
    static_assert(bytes_per_word >= sizeof(other), "Word size is smaller than int size");
    if (used_words() > 1) {
      return BigIntWord{words_[used_words() - 1]} < 0;
    }
    return BigIntWord{words_[0]} < other;
  }

  constexpr bool LessThan(bool flip, const BigInt& other) const {
    BigIntWord signed_flip = BigIntWord{0} - flip;
    if (used_words() < other.used_words()) {
      return 0 <=
        (BigIntWord{other.words_[other.used_words() - 1]} ^ signed_flip);
    }
    if (used_words() > other.used_words()) {
      return (BigIntWord{words_[used_words() - 1]} ^ signed_flip) < 0;
    }
    size_t i = used_words() - 1;
    if (words_[i] != other.words_[i]) {
      return (BigIntWord{words_[i]} ^ signed_flip) <
             (BigIntWord{other.words_[i]} ^ signed_flip);
    }
    BigUIntWord unsigned_flip(signed_flip);
    for (i--; ssize_t(i) > 0; i--) {
      if (words_[i] != other.words_[i]) {
        return (words_[i] ^ unsigned_flip) < (other.words_[i] ^ unsigned_flip);
      }
    }
    return (words_[0] ^ unsigned_flip) < (other.words_[0] ^ unsigned_flip);
  }

  constexpr bool operator <= (const BigInt& other) const {
    if (used_words() < other.used_words()) {
      return 0 <= BigIntWord{other.words_[other.used_words() - 1]};
    }
    if (used_words() > other.used_words()) {
      return BigIntWord{words_[used_words() - 1]} < 0;
    }
    size_t i = used_words() - 1;
    if (words_[i] != other.words_[i]) {
      return BigIntWord{words_[i]} < BigIntWord{other.words_[i]};
    }
    for (i--; ssize_t(i) > 0; i--) {
      if (words_[i] < other.words_[i]) return true;
      if (words_[i] > other.words_[i]) return false;
    }
    return words_[0] <= other.words_[0];
  }

  constexpr bool operator <= (int other) const {
    static_assert(bytes_per_word >= sizeof(other), "Word size is smaller than int size");
    if (used_words() > 1) {
      return BigIntWord{words_[used_words() - 1]} < 0;
    }
    return BigIntWord{words_[0]} <= other;
  }

  constexpr bool operator > (const BigInt& other) const {
    if (used_words() < other.used_words()) {
      return 0 > BigIntWord{other.words_[other.used_words() - 1]};
    }
    if (used_words() > other.used_words()) {
      return BigIntWord{words_[used_words() - 1]} >= 0;
    }
    size_t i = used_words() - 1;
    if (words_[i] != other.words_[i]) {
      return BigIntWord{words_[i]} > BigIntWord{other.words_[i]};
    }
    for (i--; ssize_t(i) > 0; i--) {
      if (words_[i] > other.words_[i]) return true;
      if (words_[i] < other.words_[i]) return false;
    }
    return words_[0] > other.words_[0];
  }

  constexpr bool operator > (int other) const {
    static_assert(bytes_per_word >= sizeof(other), "Word size is smaller than int size");
    if (used_words() > 1) {
      return BigIntWord{words_[used_words() - 1]} >= 0;
    }
    return BigIntWord{words_[0]} > other;
  }

  constexpr bool operator >= (const BigInt& other) const {
    if (used_words() < other.used_words()) {
      return 0 > BigIntWord{other.words_[other.used_words() - 1]};
    }
    if (used_words() > other.used_words()) {
      return BigIntWord{words_[used_words() - 1]} >= 0;
    }
    size_t i = used_words() - 1;
    if (words_[i] != other.words_[i]) {
      return BigIntWord{words_[i]} > BigIntWord{other.words_[i]};
    }
    for (i--; ssize_t(i) > 0; i--) {
      if (words_[i] > other.words_[i]) return true;
      if (words_[i] < other.words_[i]) return false;
    }
    return words_[0] >= other.words_[0];
  }

  constexpr bool operator >= (int other) const {
    static_assert(bytes_per_word >= sizeof(other), "Word size is smaller than int size");
    if (used_words() > 1) {
      return BigIntWord{words_[used_words() - 1]} >= 0;
    }
    return BigIntWord{words_[0]} >= other;
  }

  constexpr bool operator == (int other) const {
    assert(sizeof(int) <= bytes_per_word);
    return used_words() == 1 && words_[0] == BigUIntWord{other};
  }

  constexpr bool operator == (const BigInt& other) const {
    if (used_words() != other.used_words()) return false;

    for (size_t i = used_words() - 1; i > 0; i--) {
      if (words_[i] != other.words_[i]) return false;
    }
    return words_[0] == other.words_[0];
  }

  constexpr bool operator != (const BigInt& other) const {
    if (used_words() != other.used_words()) return true;

    for (size_t i = used_words() - 1; i > 0; i--) {
      if (words_[i] != other.words_[i]) return true;
    }
    return words_[0] != other.words_[0];
  }

  constexpr bool operator != (int other) const {
    return !(*this == other);
  }

  // Divide `this` by `other`. Return the quotient.
  BigInt operator/(const BigInt& other) const {
    if (used_words() == 1 && other.used_words() == 1) {
      return BigInt{BigIntWord{words_[0]} / BigIntWord{other.words_[0]}};
    }
    BigInt unused;
    return DivideRemainderSlow(other, &unused);
  }

  // Divide `this` by `other`. Return the quotient.
  BigInt operator/(int other) const {
    return *this / BigInt(other);
  }

  BigInt& operator/=(const BigInt& other) {
    *this = operator/(other);
    return *this;
  }

  // Divide `this` by `other`. Return the quotient and store the remainder in `remainder_out`.
  //
  // `remainder_out` may not equal `this`.
  //
  // The quotient is rounded towards 0.
  BigInt DivideRemainder(const BigInt& other,
                             BigInt* remainder_out) const {
    if (used_words() == 1 && other.used_words() == 1) {
      *remainder_out = BigInt(BigIntWord{words_[0]} %
                                  BigIntWord{other.words_[0]});
      return BigInt{BigIntWord{words_[0]} / BigIntWord{other.words_[0]}};
    }
    return DivideRemainderSlow(other, remainder_out);
  }

  // Divide `this` by `other`. Return the remainder.
  BigInt operator%(const BigInt& other) const {
    if (used_words() == 1 && other.used_words() == 1) {
      return BigInt(BigIntWord{words_[0]} %
                        BigIntWord{other.words_[0]});
    }
    BigInt remainder;
    DivideRemainderSlow(other, &remainder);
    return remainder;
  }

  // Negates *this and returns whether the result overflowed.
  //
  // A return value of false means it did not overflow.
  constexpr void Negate() {
    const BigUIntWord old_last_word(words_[used_words() - 1]);
    bool carry = true;
    size_t i = 0;
    for (i = 0; i < used_words(); ++i) {
      words_[i] = (~words_[i]).Add(carry, &carry);
    }
    // This will be 1 if the result switched signs, and 0 if the result kept
    // the same sign.
    const BigUIntWord sign_changed =
      (old_last_word ^ words_[i - 1]) >> (bits_per_word - 1);
    // carry is true if the result is 0.
    if (!(sign_changed.low_uint32() | carry)) {
      // The result kept the same sign, and the result isn't 0 (because carry
      // is false). This means that *this was equal to min_value, and it just
      // overflowed back to min_value. So allocate another word.
      ++i;
      words_.push_back(BigUIntWord{0});
    }
    assert(used_words() == i);
    Trim();
  }

  BigInt operator-() const {
    BigInt result = *this;
    result.Negate();
    return result;
  }

  BigInt abs() const {
    if (used_words() == 1) {
      return BigInt(BigIntWord{words_[0].SignedAbs()});
    }
    if (BigIntWord{words_[used_words() - 1]} >= 0) {
      return *this;
    } else {
      return -*this;
    }
  }

  BigInt GetAbs(bool& was_signed) const {
    BigInt result = *this;
    was_signed = GetSign() < 0;
    if (was_signed) {
      result.Negate();
    }
    return result;
  }

  BigUInt GetUIntAbs(bool* was_signed) const {
    *was_signed = BigIntWord{words_[used_words() - 1]} < 0;
    if (*was_signed) {
      BigInt pos = *this;
      // Ignore overflow
      pos.Negate();
      return BigUInt{pos.words_, pos.used_words()};
    } else {
      return BigUInt{words_, used_words()};
    }
  }

  // Returns 0 if this equals 0.
  // Returns >0 if this is greater than 0.
  // Returns <0 if this is less than 0.
  constexpr BigIntWord GetSign() const {
    size_t i = used_words() - 1;
    return BigIntWord{words_[i]} | i;
  }

  constexpr bool IsZero() const {
    return GetSign() == 0;
  }

  // Returns 1 if this is greater than or equal to 0.
  // Returns -1 if this is less than 0.
  constexpr int GetAbsMult() const {
    return static_cast<int>(
        BigIntWord(words_[used_words() - 1].SignExtension())) | 1;
  }

  // Returns 1 if this and other >= 0 or if both are negative.
  // Else, returns -1 if only one of this or other is negative.
  constexpr int GetAbsMult(const BigInt& other) const {
    return (SignExtension() ^ other.SignExtension()) | 1;
  }

  constexpr bool HasSameSign(const BigInt& other) const {
    return (GetSign() ^ other.GetSign()) >= 0;
  }

  constexpr bool HasDifferentSign(const BigInt& other) const {
    return (GetSign() ^ other.GetSign()) < 0;
  }

  // Returns 0 if this is greater than or equal to 0.
  // Returns -1 if this is less than 0.
  constexpr BigIntWord SignExtension() const {
    return BigIntWord{words_[used_words() - 1].SignExtension()};
  }

  explicit operator double() const;

  static BigInt Determinant(const BigInt& r1_c1,
                                const BigInt& r1_c2,
                                const BigInt& r2_c1,
                                const BigInt& r2_c2) {
    return r1_c1*r2_c2 - r2_c1*r1_c2;
  }

  BigInt GetGreatestCommonDivisor(const BigInt &other) const {
    if (other.IsZero()) return *this;

    BigInt mod = *this % other;
    return other.GetGreatestCommonDivisor(mod);
  }

 protected:
  static constexpr bool CanTrim(BigUIntWord low, BigUIntWord high) {
    // The high word must be 0 or -1.
    if (high.Add(BigUIntWord{1}) > BigUIntWord{1}) return false;
    // The high word must have the same sign as the lower word.
    if (BigIntWord(high ^ low) < 0) return false;
    return true;
  }

  static constexpr bool CanTrimLastHalf(BigUIntWord last) {
    return static_cast<BigIntHalfWord>(BigIntWord{last}) == BigIntWord{last};
  }

  constexpr void Trim() {
    int i = used_words() - 1;
    if (i > 0) {
      BigUIntWord check = word(i);
      BigUIntWord next;
      do {
        --i;
        next = word(i);

        if (!CanTrim(/*low=*/next, /*high=*/check)) break;

        check = next;
        words_.resize(i + 1);
      } while (i > 0);
    }
  }

  BigInt MultiplySlow(const BigInt& other) const;

  // Mask everyone of `other` words with `mask`, then subtract other_masked *
  // 2^(shift_left_words * bits_per_word) from this.
  //
  // Also note that the carry is not extended past used_words().
  constexpr void SubtractLeftShiftedMasked(const BigInt& other,
                                           int shift_left_words,
                                           BigIntWord mask) {
    bool carry = false;
    for (size_t i = 0, j = shift_left_words; i < other.used_words(); i++, j++) {
      assert(j < used_words());
      words_[j] = words_[j].Subtract(other.words_[i] & BigUIntWord{mask},
                                     carry, &carry);
    }
  }

  BigInt DivideRemainderSlow(const BigInt& other, BigInt* remainder_out) const;

  BigIntWords words_;
};

std::ostream& operator<<(std::ostream& out, const BigInt& bigint);

}  // walnut

#endif // WALNUT_BIG_INT_H__
