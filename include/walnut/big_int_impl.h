#ifndef WALNUT_BIG_INT_IMPL_H__
#define WALNUT_BIG_INT_IMPL_H__

#include <algorithm>
#include <cassert>
#include <iostream>

#include "walnut/big_int_base.h"
#include "walnut/big_uint.h"

namespace walnut {

template <int max_words>
class BigIntImplTrimMixin : public BigIntBase<max_words> {
  using Parent = BigIntBase<max_words>;
 public:
  using Parent::bytes_per_word;

 protected:
  using Parent::Parent;

  using Parent::used_;
  using Parent::words_;

  constexpr void Trim() {
    int i = used_ / bytes_per_word - 1;
    if (i > 0) {
      BigUIntWord check = words_[i];
      BigUIntWord next;
      do {
        --i;
        next = words_[i];

        if (check.Add(BigUIntWord{1}) > BigUIntWord{1} ||
            BigIntWord(check ^ next) < 0) {
          break;
        }

        check = next;
        used_-= bytes_per_word;
      } while (i > 0);
    }
    const constexpr BigUIntWord unsigned_int_min{BigUIntHalfWord(
      std::numeric_limits<BigIntHalfWord>::min())};
    const constexpr BigUIntWord limit = BigUIntWord{std::numeric_limits<BigIntHalfWord>::max()}.Add(unsigned_int_min);
    if (used_ == bytes_per_word &&
        words_[0].Add(unsigned_int_min) <= limit) {
      used_ = sizeof(BigUIntHalfWord);
    }
  }
};

template <int max_words>
class BigIntImpl : public BigIntBaseOperations<BigIntImplTrimMixin<max_words>>
{
  template <typename OtherMixin>
  friend class BigIntBaseOperations;

  template <int other_max_words>
  friend class BigIntImpl;

  using Parent = BigIntBaseOperations<BigIntImplTrimMixin<max_words>>;

 public:
  using Parent::bits_per_word;
  using Parent::bits_per_byte;
  using Parent::bytes_per_word;
  using Parent::max_bits;
  using Parent::max_bytes;

  constexpr BigIntImpl() : BigIntImpl(0) {
  }

  explicit constexpr BigIntImpl(BigIntHalfWord value) : Parent(sizeof(value)) {
    words_[0] = value;
  }

  explicit constexpr BigIntImpl(BigIntWord value) :
    Parent(static_cast<BigIntHalfWord>(value) == value ?
           sizeof(BigIntHalfWord) : bytes_per_word) {
    words_[0] = value;
  }

  template <int other_max_words>
  constexpr BigIntImpl(const BigIntImpl<other_max_words>& other)
   : Parent(other) { }

  template <int other_max_words>
  constexpr BigIntImpl(const BigUIntImpl<other_max_words>& other)
   : Parent(sizeof(BigIntHalfWord)) {
    int copy_bytes = max_words < other_max_words ?
                     std::min(other.used_bytes(), int(max_words * bytes_per_word)) :
                     other.used_bytes();
    int copy_words = (copy_bytes + bytes_per_word - 1) / bytes_per_word;
    int i;
    for (i = 0; i < copy_words; ++i) {
      words_[i] = other.words()[i];
    }
    if (BigIntWord{words_[i - 1]} < 0 && i < max_words) {
      ++i;
    }
    used_ = i * bytes_per_word;
    Trim();
  }

  template <int other_max_words>
  constexpr BigIntImpl<max_words>& operator = (
      const BigIntImpl<other_max_words>& other) {
    Parent::operator=(other);
    return *this;
  }

  template <int other_max_words>
  constexpr BigIntImpl<max_words>& operator = (
      const BigUIntImpl<other_max_words>& other) {
    int copy_bytes = max_words < other_max_words ?
                     std::min(other.used_bytes(), int(max_words * bytes_per_word)) :
                     other.used_bytes();
    int copy_words = (copy_bytes + bytes_per_word - 1) / bytes_per_word;
    int i;
    for (i = 0; i < copy_words; ++i) {
      words_[i] = other.words()[i];
    }
    if (BigIntWord{words_[i - 1]} < 0 && i < max_words) {
      ++i;
    }
    used_ = i * bytes_per_word;
    Trim();
    return *this;
  }

  static constexpr BigIntImpl max_value(int set_last_word_bits) {
    BigIntImpl result;
    for (int i = 0; i < max_words - 1; ++i) {
      result.words_[i] = BigUIntWord::max_value();
    }
    for (int i = 0; i < set_last_word_bits; ++i) {
      result.words_[max_words - 1] |= BigUIntWord{1} << i;
    }
    if (max_words > 1 ||
        result.words_[max_words - 1] >
          BigUIntWord{std::numeric_limits<BigIntHalfWord>::max()}) {
      result.used_ = max_bytes;
    } else {
      result.used_ = sizeof(BigUIntHalfWord);
    }
    return result;
  }

  static constexpr BigIntImpl max_value() {
    return max_value(/*set_last_word_bits=*/bits_per_word - 1);
  }

  static constexpr BigIntImpl min_value(int clear_last_word_bits) {
    BigIntImpl result;
    result.words_[max_words - 1] = BigUIntWord{-1};
    for (int i = 0; i < clear_last_word_bits; ++i) {
      result.words_[max_words - 1] &= ~(BigUIntWord{1} << i);
    }
    if (max_words > 1 ||
        result.words_[max_words - 1] <
          BigUIntWord{std::numeric_limits<BigIntHalfWord>::min()}) {
      result.used_ = max_bytes;
    } else {
      result.used_ = sizeof(BigUIntHalfWord);
    }
    return result;
  }

  static constexpr BigIntImpl min_value() {
    return max_value(/*clear_last_word_bits=*/bits_per_word - 1);
  }

  template <int result_words=max_words>
  constexpr BigIntImpl<result_words> operator << (int shift) const {
    BigIntImpl<result_words> result;
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
        result.words_[out] = BigIntWord(prev) >> prev_right_shift;
        out++;
      }
      result.used_ = out * bytes_per_word;
    } else {
      for (; in < used_words() && out < result_words; in++, out++) {
        result.words_[out] = words_[in];
      }
      result.used_ = out * bytes_per_word;
    }
    result.Trim();
    return result;
  }

  // Returns -1 if *this < `other`,
  //          0 if *this ==  `other`, or
  //          1 if *this > `other`.
  template <int other_max_words>
  constexpr int Compare(const BigIntImpl<other_max_words>& other) const {
    if (used_ < other.used_) {
      // `this` is closer to 0 than `other`.
      return (0 < BigIntWord{other.words_[other.used_words() - 1]}) ? -1 : 1;
    }
    if (used_ > other.used_) {
      // `other` is closer to 0 than `this`.
      return (0 < BigIntWord{words_[used_words() - 1]}) ? 1 : -1;
    }
    int i = used_words() - 1;
    if (words_[i] != other.words_[i]) {
      return (BigIntWord{words_[i]} > BigIntWord{other.words_[i]}) ? 1 : -1;
    }
    for (i--; i > 0; i--) {
      if (words_[i] > other.words_[i]) return 1;
      if (words_[i] < other.words_[i]) return -1;
    }
    return words_[0].CompareSigned(other.words_[0]);
  }

  template <int result_words = 0, int other_words,
            int rw = result_words == 0 ?
              std::max(max_words, other_words) : result_words>
  constexpr BigIntImpl<rw> Add(const BigIntImpl<other_words>& other) const {
    if (used_ == sizeof(BigIntHalfWord) && other.used_ == sizeof(BigIntHalfWord)) {
      return BigIntImpl<rw>(BigIntWord{words_[0].Add(other.words_[0])});
    }
    BigIntImpl<rw> result;
    int i = 0;
    bool carry = false;
    int common_words = GetCommonWordCount(other);
    for (; i < common_words && i < rw; i++) {
      result.words_[i] = words_[i].Add(other.words_[i], carry, &carry);
    }
    BigUIntWord this_extension = SignExtension();
    BigUIntWord other_extension = other.SignExtension();
    for (; i < std::min(used_ / bytes_per_word, rw); i++) {
      result.words_[i] = words_[i].Add(other_extension, carry, &carry);
    }
    for (; i < std::min(other.used_ / bytes_per_word, rw); i++) {
      result.words_[i] = this_extension.Add(other.words_[i], carry, &carry);
    }
    if (i < rw) {
      result.words_[i] = this_extension.Add(other_extension, carry, &carry);
      i++;
    }
    result.used_ = i * bytes_per_word;
    result.Trim();
    return result;
  }

  template <int other_words>
  constexpr BigIntImpl<std::max(max_words, other_words)> operator+(
      const BigIntImpl<other_words>& other) const {
    return Add(other);
  }

  template <int other_words>
  constexpr BigIntImpl& operator+=(const BigIntImpl<other_words>& other) {
    if (used_ == sizeof(BigIntHalfWord) && other.used_ == sizeof(BigIntHalfWord)) {
      words_[0] += other.words_[0];
      if ((BigIntWord)words_[0] < std::numeric_limits<BigIntHalfWord>::min() ||
          (BigIntWord)words_[0] > std::numeric_limits<BigIntHalfWord>::max()) {
        used_ = bytes_per_word;
      }
      return *this;
    }
    int i = 0;
    bool carry = false;
    int common_words = GetCommonWordCount(other);
    BigUIntWord this_extension = SignExtension();
    BigUIntWord other_extension = other.SignExtension();
    assert(("overflow", other.used_ <= max_bytes));
    for (; i < common_words && i < max_words; i++) {
      words_[i] = words_[i].Add(other.words_[i], carry, &carry);
    }
    for (; i < std::min(used_ / bytes_per_word, max_words); i++) {
      words_[i] = words_[i].Add(other_extension, carry, &carry);
    }
    for (; i < std::min(other.used_ / bytes_per_word, max_words); i++) {
      words_[i] = this_extension.Add(other.words_[i], carry, &carry);
    }
    if (i < max_words) {
      words_[i] = this_extension.Add(other_extension, carry, &carry);
      i++;
    }
    used_ = i * bytes_per_word;
    Trim();
    return *this;
  }

  template <int result_words = 0, int other_words,
            int rw = result_words == 0 ?
              std::max(max_words, other_words) : result_words>
  constexpr BigIntImpl<rw> Subtract(const BigIntImpl<other_words>& other) const {
    if (used_ == sizeof(BigIntHalfWord) && other.used_ == sizeof(BigIntHalfWord)) {
      return BigIntImpl<rw>(BigIntWord{words_[0].Subtract(other.words_[0])});
    }
    BigIntImpl<rw> result;
    int i = 0;
    bool carry = false;
    int common_words = GetCommonWordCount(other);
    for (; i < common_words && i < rw; i++) {
      result.words_[i] = words_[i].Subtract(other.words_[i], carry, &carry);
    }
    BigUIntWord this_extension = SignExtension();
    BigUIntWord other_extension = other.SignExtension();
    for (; i < std::min(used_ / bytes_per_word, rw); i++) {
      result.words_[i] = words_[i].Subtract(other_extension, carry, &carry);
    }
    for (; i < std::min(other.used_ / bytes_per_word, rw); i++) {
      result.words_[i] = this_extension.Subtract(other.words_[i], carry, &carry);
    }
    if (i < rw) {
      result.words_[i] = this_extension.Subtract(other_extension, carry, &carry);
      i++;
    }
    result.used_ = i * bytes_per_word;
    result.Trim();
    return result;
  }

  template <int other_words>
  constexpr BigIntImpl<std::max(max_words, other_words)> operator-(
      const BigIntImpl<other_words>& other) const {
    return Subtract(other);
  }

  template <int other_words>
  constexpr BigIntImpl<max_words + other_words>
  Multiply(const BigIntImpl<other_words>& other) const {
    constexpr int result_words = max_words + other_words;
    if (used_ == sizeof(BigIntHalfWord) && other.used_ == sizeof(BigIntHalfWord)) {
      return BigIntImpl<result_words>(BigIntWord{words_[0]} * BigIntWord{other.words_[0]});
    }
    if (used_ <= bytes_per_word && other.used_ <= bytes_per_word) {
      BigIntImpl<result_words> result;
      result.words_[0] = words_[0].Multiply(other.words_[0], &result.words_[1]);
      result.words_[1] -= other.words_[0].SignExtension() & words_[0];
      result.words_[1] -= words_[0].SignExtension() & other.words_[0];
      result.used_ = bytes_per_word * 2;
      result.Trim();
      return result;
    }
    BigIntImpl<result_words> result =
      Parent::template MultiplySlow<BigIntImpl<result_words> >(other);
    result.SubtractLeftShiftedMasked(*this, other.used_words(), other.SignExtension());
    result.SubtractLeftShiftedMasked(other, used_words(), SignExtension());
    result.Trim();
    return result;
  }

  template <int other_words>
  constexpr BigIntImpl<max_words + other_words>
  operator*(const BigIntImpl<other_words>& other) const {
    return Multiply(other);
  }

  constexpr BigIntImpl<max_words + 1>
  operator*(const int other) const {
    return Multiply(BigIntImpl<1>(other));
  }

  template <int other_max_words>
  constexpr bool operator < (const BigIntImpl<other_max_words>& other) const {
    if (used_ < other.used_) {
      return 0 <= BigIntWord{other.words_[other.used_words() - 1]};
    }
    if (used_ > other.used_) {
      return BigIntWord{words_[used_words() - 1]} < 0;
    }
    int i = used_words() - 1;
    if (words_[i] != other.words_[i]) {
      return BigIntWord{words_[i]} < BigIntWord{other.words_[i]};
    }
    for (i--; i > 0; i--) {
      if (words_[i] < other.words_[i]) return true;
      if (words_[i] > other.words_[i]) return false;
    }
    return words_[0] < other.words_[0];
  }

  constexpr bool operator < (int other) const {
    static_assert(bytes_per_word >= sizeof(other), "Word size is smaller than int size");
    if (used_ > sizeof(other)) {
      return BigIntWord{words_[used_words() - 1]} < 0;
    }
    return BigIntWord{words_[0]} < other;
  }

  template <int other_max_words>
  constexpr bool operator <= (const BigIntImpl<other_max_words>& other) const {
    if (used_ < other.used_) {
      return 0 <= BigIntWord{other.words_[other.used_words() - 1]};
    }
    if (used_ > other.used_) {
      return BigIntWord{words_[used_words() - 1]} < 0;
    }
    int i = used_words() - 1;
    if (words_[i] != other.words_[i]) {
      return BigIntWord{words_[i]} < BigIntWord{other.words_[i]};
    }
    for (i--; i > 0; i--) {
      if (words_[i] < other.words_[i]) return true;
      if (words_[i] > other.words_[i]) return false;
    }
    return words_[0] <= other.words_[0];
  }

  constexpr bool operator <= (int other) const {
    static_assert(bytes_per_word >= sizeof(other), "Word size is smaller than int size");
    if (used_ > sizeof(other)) {
      return BigIntWord{words_[used_words() - 1]} < 0;
    }
    return BigIntWord{words_[0]} <= other;
  }

  template <int other_max_words>
  constexpr bool operator > (const BigIntImpl<other_max_words>& other) const {
    if (used_ < other.used_) {
      return 0 > BigIntWord{other.words_[other.used_words() - 1]};
    }
    if (used_ > other.used_) {
      return BigIntWord{words_[used_words() - 1]} >= 0;
    }
    int i = used_words() - 1;
    if (words_[i] != other.words_[i]) {
      return BigIntWord{words_[i]} > BigIntWord{other.words_[i]};
    }
    for (i--; i > 0; i--) {
      if (words_[i] > other.words_[i]) return true;
      if (words_[i] < other.words_[i]) return false;
    }
    return words_[0] > other.words_[0];
  }

  constexpr bool operator > (int other) const {
    static_assert(bytes_per_word >= sizeof(other), "Word size is smaller than int size");
    if (used_ > sizeof(other)) {
      return BigIntWord{words_[used_words() - 1]} >= 0;
    }
    return BigIntWord{words_[0]} > other;
  }

  template <int other_max_words>
  constexpr bool operator >= (const BigIntImpl<other_max_words>& other) const {
    if (used_ < other.used_) {
      return 0 > BigIntWord{other.words_[other.used_words() - 1]};
    }
    if (used_ > other.used_) {
      return BigIntWord{words_[used_words() - 1]} >= 0;
    }
    int i = used_words() - 1;
    if (words_[i] != other.words_[i]) {
      return BigIntWord{words_[i]} > BigIntWord{other.words_[i]};
    }
    for (i--; i > 0; i--) {
      if (words_[i] > other.words_[i]) return true;
      if (words_[i] < other.words_[i]) return false;
    }
    return words_[0] >= other.words_[0];
  }

  constexpr bool operator >= (int other) const {
    static_assert(bytes_per_word >= sizeof(other), "Word size is smaller than int size");
    if (used_ > sizeof(other)) {
      return BigIntWord{words_[used_words() - 1]} >= 0;
    }
    return BigIntWord{words_[0]} >= other;
  }

  constexpr bool operator == (int other) const {
    if (sizeof(int) <= bytes_per_word) {
      return used_ <= sizeof(other) && words_[0] == BigUIntWord{other};
    } else {
      return *this ==
        BigIntImpl<(sizeof(int) + bytes_per_word - 1) / bytes_per_word>(other);
    }
  }

  template <int other_max_words>
  constexpr bool operator == (const BigIntImpl<other_max_words>& other) const {
    if (used_ != other.used_) return false;

    for (int i = used_ / bytes_per_word - 1; i > 0; i--) {
      if (words_[i] != other.words_[i]) return false;
    }
    return words_[0] == other.words_[0];
  }

  template <int other_max_words>
  constexpr bool operator != (const BigIntImpl<other_max_words>& other) const {
    if (used_ != other.used_) return true;

    for (int i = used_ / bytes_per_word - 1; i > 0; i--) {
      if (words_[i] != other.words_[i]) return true;
    }
    return words_[0] != other.words_[0];
  }

  constexpr bool operator != (int other) const {
    return !(*this == other);
  }

  // Divide `this` by `other`. Return the quotient.
  template <int other_words>
  constexpr BigIntImpl<max_words> operator/(const BigIntImpl<other_words>& other) const {
    if (used_ <= bytes_per_word && other.used_ <= bytes_per_word) {
      return BigIntImpl<max_words>{BigIntWord{words_[0]} / BigIntWord{other.words_[0]}};
    }
    BigIntImpl<std::min(max_words, other_words)> unused;
    return DivideRemainderSlow(other, &unused);
  }

  // Divide `this` by `other`. Return the quotient and store the remainder in `remainder_out`.
  //
  // `remainder_out` may not equal `this`.
  //
  // The quotient is rounded towards 0.
  template <int other_words>
  constexpr BigIntImpl<max_words> DivideRemainder(const BigIntImpl<other_words>& other,
      BigIntImpl<std::min(max_words, other_words)>* remainder_out) const {
    if (used_ <= bytes_per_word && other.used_ <= bytes_per_word) {
      *remainder_out = BigIntImpl<std::min(max_words, other_words)>{
        BigIntWord{words_[0]} % BigIntWord{other.words_[0]}};
      return BigIntImpl<max_words>{BigIntWord{words_[0]} / BigIntWord{other.words_[0]}};
    }
    return DivideRemainderSlow(other, remainder_out);
  }

  // Divide `this` by `other`. Return the remainder.
  template <int other_words>
  constexpr BigIntImpl<std::min(max_words, other_words)> operator%(
      const BigIntImpl<other_words>& other) const {
    if (used_ <= bytes_per_word && other.used_ <= bytes_per_word) {
      return BigIntImpl<std::min(max_words, other_words)>{
        BigIntWord{words_[0]} % BigIntWord{other.words_[0]}};
    }
    BigIntImpl<std::min(max_words, other_words)> remainder;
    DivideRemainderSlow(other, &remainder);
    return remainder;
  }

  constexpr void Negate() {
    bool carry = true;
    int i = 0;
    for (i = 0; i < used_words(); ++i) {
      words_[i] = (~words_[i]).Add(carry, &carry);
    }
    constexpr BigUIntWord min_word{std::numeric_limits<BigIntWord>::min()};
    if (words_[i - 1] == min_word && i < max_words) {
      // int_min was just negated and went back to int_min.
      ++i;
    }
    used_ = i * bytes_per_word;
    Trim();
  }

  constexpr BigIntImpl<max_words> operator-() const {
    BigIntImpl<max_words> result = *this;
    result.Negate();
    return result;
  }

  constexpr BigIntImpl<max_words> abs() const {
    if (used_ == sizeof(BigIntHalfWord)) {
      return BigIntImpl<max_words>(BigIntWord{words_[0].SignedAbs()});
    }
    if (BigIntWord{words_[used_words() - 1]} >= 0) {
      return *this;
    } else {
      return -*this;
    }
  }

  constexpr BigUIntImpl<max_words> GetUIntAbs(bool* was_signed) const {
    *was_signed = BigIntWord{words_[used_words() - 1]} < 0;
    if (*was_signed) {
      BigIntImpl<max_words> pos = -*this;
      return BigUIntImpl<max_words>{pos.words_, pos.used_};
    } else {
      return BigUIntImpl<max_words>{words_, used_};
    }
  }

  // Returns 0 if this equals 0.
  // Returns >0 if this is greater than 0.
  // Returns <0 if this is less than 0.
  constexpr BigIntWord GetSign() const {
    int i = used_words() - 1;
    return BigIntWord{words_[i]} | i;
  }

  template <int other_words>
  constexpr bool HasSameSign(const BigIntImpl<other_words>& other) const {
    return GetSign() ^ other.GetSign() >= 0;
  }

 protected:
  using Parent::Trim;
  using Parent::used_;
  using Parent::words_;

  using Parent::used_words;
  using Parent::GetCommonWordCount;

  constexpr BigUIntWord SignExtension() const {
    return words_[used_words() - 1].SignExtension();
  }

  // Mask everyone of `other` words with `mask`, then subtract other_masked *
  // 2^(shift_left_words * bits_per_word) from this.
  //
  // `this` must have space to do the subtraction. This is, the caller must ensure:
  //   other.used_words() + shift_left_words <= max_words
  // 
  // Also note that the carry is not extended past used_words().
  template <int other_words>
  constexpr void SubtractLeftShiftedMasked(const BigIntImpl<other_words>& other,
                                           int shift_left_words,
                                           BigUIntWord mask) {
    int i = 0, j = shift_left_words;
    bool carry = false;
    for (int i = 0, j = shift_left_words; i < other.used_words(); i++, j++) {
      assert(j < used_words());
      words_[j] = words_[j].Subtract(other.words_[i] & mask,
                                     carry, &carry);
    }
  }

  template <int other_words>
  constexpr BigIntImpl<max_words> DivideRemainderSlow(const BigIntImpl<other_words>& other,
      BigIntImpl<std::min(max_words, other_words)>* remainder_out) const {
    bool this_signed;
    BigUIntImpl<max_words> this_uint = GetUIntAbs(&this_signed);
    bool other_signed;
    BigUIntImpl<other_words> other_uint = other.GetUIntAbs(&other_signed);

    BigUIntImpl<std::min(max_words, other_words)> remainder;
    BigUIntImpl<max_words> quotient = this_uint.DivideRemainder(other_uint,
                                                                &remainder);
    *remainder_out = remainder;
    if (this_signed) {
      remainder_out->Negate();
    }
    BigIntImpl<max_words> result{quotient};
    if (this_signed ^ other_signed) {
      result.Negate();
    }
    return result;
  }
};

template <int max_words>
std::ostream& operator<<(std::ostream& out, const BigIntImpl<max_words>& bigint) {
  BigIntImpl<max_words> remaining = bigint;
  char digits[BigIntImpl<max_words>::max_bytes*3 + 2];
  char* digits_pos = &digits[BigIntImpl<max_words>::max_bytes*3 + 1];
  *digits_pos-- = '\0';

  bool printed_anything = false;
  BigIntImpl<1> digit;
  do {
    remaining = remaining.DivideRemainder(BigIntImpl<1>(10), &digit);
    *digits_pos-- = digit.words()[0].SignedAbs().low_uint32() + '0';
  } while (remaining != 0);

  if (digit.GetSign() < 0) {
    *digits_pos-- = '-';
  }
  out << digits_pos + 1;
  return out;
}

}  // walnut

#endif // WALNUT_BIG_INT_IMPL_H__
