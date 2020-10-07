#ifndef WALNUT_BIG_UINT_H__
#define WALNUT_BIG_UINT_H__

#include <algorithm>
#include <cassert>

#include "walnut/big_uint_word.h"

namespace walnut {

// Common base for BigIntImpl and BigUIntImpl
template <int max_words_template, typename ImplType>
class BigIntBase {
  template <int other_words, typename OtherImplType>
  friend class BigIntBase;

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
  // Get the most derived version of this class.
  constexpr const ImplType* impl() const {
    static_cast<const ImplType*>(this);
  }

  // Get the most derived version of this class.
  constexpr ImplType* impl() {
    return static_cast<ImplType*>(this);
  }

  constexpr BigIntBase(int used) : used_(used) { }

  constexpr BigIntBase(const BigUIntWord* words, int used) : used_(used) {
    assert(("overflow", used_ <= max_bytes));
    for (int i = 0; i < (used + bytes_per_word - 1) / bytes_per_word; ++i) {
      words_[i] = words[i];
    }
    impl()->Trim();
  }

  template <int other_max_words, typename OtherImpl>
  constexpr BigIntBase(const BigIntBase<other_max_words, OtherImpl>& other) {
    assert(("overflow", other.used_ <= max_bytes));
    int copy_bytes = max_words < other_max_words ?
                     std::min(other.used_, int(max_words * bytes_per_word)) :
                     other.used_;
    int copy_words = (copy_bytes + bytes_per_word - 1) / bytes_per_word;
    for (int i = 0; i < copy_words; i++) {
      words_[i] = other.words_[i];
    }
    used_ = copy_bytes;
    if (max_words < other_max_words) {
      impl()->Trim();
    }
  }

  template <int other_max_words, typename OtherImpl>
  constexpr ImplType& operator=(const BigIntBase<other_max_words, OtherImpl>& other) {
    assert(other.used_ <= max_bytes);
    AssignIgnoreOverflow(other);
    return *impl();
  }

  template <int other_max_words, typename OtherImpl>
  constexpr void AssignIgnoreOverflow(const BigIntBase<other_max_words, OtherImpl>& other) {
    int copy_bytes = max_words < other_max_words ?
                     std::min(other.used_, int(max_words * bytes_per_word)) :
                     other.used_;
    int copy_words = (copy_bytes + bytes_per_word - 1) / bytes_per_word;
    for (int i = 0; i < copy_words; i++) {
      words_[i] = other.words_[i];
    }
    used_ = copy_bytes;
    if (max_words < other_max_words) {
      impl()->Trim();
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

  template <typename Result, typename Other>
  constexpr Result MultiplySlow(const Other& other) const {
    if (used_ < other.used_) {
      return other.template MultiplySlow<Result>(*this);
    }
    Result result;
    int k = 0;
    {
      BigUIntWord add;
      for (int i = 0; i < used_words(); ++i, ++k) {
        result.words_[k] = other.words_[0].MultiplyAdd(words_[i], add, /*carry_in=*/false, &add);
      }
      result.words_[k] = add;
    }
    for (int j = 1; j < other.used_words(); j++) {
      k = j;
      BigUIntWord add;
      bool carry = false;
      for (int i = 0; i < used_words(); ++i, ++k) {
        add = add.Add(result.words_[k], carry, &carry);
        result.words_[k] = other.words_[j].MultiplyAdd(words_[i], add, /*carry_in=*/false, &add);
      }
      result.words_[k] = add.Add(carry, &carry);
    }
    k++;
    result.used_ = k * BigUIntWord::bytes_per_word;
    return result;
  }

  constexpr int used_words() const {
    return (used_ + bytes_per_word - 1) / bytes_per_word;
  }

  template <typename OtherImpl>
  constexpr int GetCommonWordCount(const OtherImpl& other) const {
    return (std::min(used_, other.used_) +
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

template <int max_words>
class BigUIntImpl : public BigIntBase<max_words, BigUIntImpl<max_words>>
{
  template <int other_words>
  friend class BigUIntImpl;

  template <int other_words, typename OtherImplType>
  friend class BigIntBase;

  using Parent = BigIntBase<max_words, BigUIntImpl<max_words>>;

 public:
  using Parent::bits_per_word;
  using Parent::bits_per_byte;
  using Parent::bytes_per_word;
  using Parent::max_bits;
  using Parent::max_bytes;

  constexpr BigUIntImpl() : BigUIntImpl(static_cast<BigUIntHalfWord>(0)) {
  }

  explicit constexpr BigUIntImpl(BigUIntHalfWord value) : Parent(sizeof(BigUIntHalfWord)) {
    words_[0] = value;
  }

  explicit constexpr BigUIntImpl(BigUIntWord value) :
    Parent(value <= std::numeric_limits<BigUIntHalfWord>::max() ?
          sizeof(BigUIntHalfWord) : BigUIntWord::bytes_per_word) {
    words_[0] = value;
  }

  explicit constexpr BigUIntImpl(uint64_t value) : BigUIntImpl(BigUIntWord(value)) { }

  explicit constexpr BigUIntImpl(int value) : Parent(
      value >= 0 ? sizeof(BigUIntHalfWord) :
                   max_words * BigUIntWord::bytes_per_word) {
    if (value >= 0) {
      words_[0] = value;
    } else {
      words_[0] = value;
      for (int i = 1; i < max_words; ++i) {
        words_[i] = -1;
      }
    }
  }

  template <int other_max_words>
  constexpr BigUIntImpl(const BigUIntImpl<other_max_words>& other)
   : Parent(other) { }

  constexpr BigUIntImpl(const BigUIntWord* words, int used) : Parent(words, used) { }

  template <int other_max_words>
  constexpr BigUIntImpl<max_words>& operator = (
      const BigUIntImpl<other_max_words>& other) {
    return Parent::operator=(other);
  }

  template <int other_max_words>
  constexpr void AssignIgnoreOverflow(
      const BigUIntImpl<other_max_words>& other) {
    Parent::AssignIgnoreOverflow(other);
  }

  static constexpr BigUIntImpl max_value() {
    BigUIntImpl result;
    for (int i = 0; i < max_words; ++i) {
      result.words_[i] = BigUIntWord::max_value();
    }
    result.used_ = max_bytes;
    return result;
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
  constexpr BigUIntImpl<rw> operator&(const BigUIntImpl<other_words>& other) {
    return Parent::template operator&<BigUIntImpl<rw>>(other);
  }

  template <int result_words = 0, int other_words,
            int rw = result_words == 0 ?
              std::max(max_words, other_words) : result_words>
  constexpr BigUIntImpl<rw> Add(const BigUIntImpl<other_words>& other) const {
    if (used_ == sizeof(BigUIntHalfWord) && other.used_ == sizeof(BigUIntHalfWord)) {
      return BigUIntImpl<rw>(words_[0].Add(other.words_[0]));
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

  template <int other_words>
  constexpr BigUIntImpl operator+=(const BigUIntImpl<other_words>& other) {
    if (used_ == sizeof(BigUIntHalfWord) && other.used_ == sizeof(BigUIntHalfWord)) {
      words_[0] += other.words_[0];
      if (words_[0] > BigUIntWord{std::numeric_limits<BigUIntHalfWord>::max()}) {
        used_ = bytes_per_word;
      }
      return *this;
    }
    int i = 0;
    bool carry = false;
    assert(("overflow", other.used_ <= max_bytes));
    int common_words = GetCommonWordCount(other);
    for (; i < common_words && i < max_words; i++) {
      words_[i] = words_[i].Add(other.words_[i], carry, &carry);
    }
    for (; i < std::min(used_ / BigUIntWord::bytes_per_word, max_words); i++) {
      words_[i] = words_[i].Add(carry, &carry);
    }
    for (; i < std::min(other.used_ / BigUIntWord::bytes_per_word, max_words); i++) {
      words_[i] = other.words_[i].Add(carry, &carry);
    }
    if (carry) {
      assert (("overflow", i < max_words));
      if (i < max_words) {
        words_[i] = 1;
        i++;
      }
    }
    used_ = i * BigUIntWord::bytes_per_word;
    Trim();
    return *this;
  }

  template <int result_words = 0, int other_words,
            int rw = result_words == 0 ?
              std::max(max_words, other_words) : result_words>
  constexpr BigUIntImpl<rw> Subtract(const BigUIntImpl<other_words>& other) const {
    if (used_ == sizeof(BigUIntHalfWord) && other.used_ == sizeof(BigUIntHalfWord) &&
        words_[0].low_uint32() >= other.words_[0].low_uint32()) {
      return BigUIntImpl<rw>(words_[0].Subtract(other.words_[0]));
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

  template <int other_words>
  constexpr BigUIntImpl<std::max(max_words, other_words)> operator-(
      const BigUIntImpl<other_words>& other) const {
    return Subtract(other);
  }

  template <int other_words>
  constexpr BigUIntImpl<max_words + other_words>
  Multiply(const BigUIntImpl<other_words>& other) const {
    assert(used_words() <= max_words);
    assert(other.used_words() <= other_words);
    constexpr int result_words = max_words + other_words;
    if (used_ == sizeof(BigUIntHalfWord) && other.used_ == sizeof(BigUIntHalfWord)) {
      return BigUIntImpl<result_words>(words_[0].MultiplyAsHalfWord(other.words_[0]));
    }
    if (used_ <= BigUIntWord::bytes_per_word && other.used_ <= BigUIntWord::bytes_per_word) {
      BigUIntImpl<result_words> result;
      result.words_[0] = words_[0].Multiply(other.words_[0], &result.words_[1]);
      result.used_ = BigUIntWord::bytes_per_word * 2;
      result.Trim();
      return result;
    }
    BigUIntImpl<result_words> result =
      Parent::template MultiplySlow<BigUIntImpl<result_words> >(other);
    result.Trim();
    return result;
  }

  template <int other_words>
  constexpr BigUIntImpl<max_words + other_words>
  operator*(const BigUIntImpl<other_words>& other) const {
    return Multiply(other);
  }

  template <int result_words = max_words + 1>
  constexpr BigUIntImpl<result_words> Multiply(BigUIntWord other) const {
    if (used_ == sizeof(BigUIntHalfWord) &&
        other <= std::numeric_limits<BigUIntHalfWord>::max()) {
      return BigUIntImpl<result_words>(words_[0].MultiplyAsHalfWord(other));
    }
    BigUIntImpl<result_words> result;
    int k = 0;
    BigUIntWord add;
    for (int i = 0; i < used_words(); ++i, ++k) {
      result.words_[k] = other.MultiplyAdd(words_[i], add, /*carry_in=*/false, &add);
    }
    if (k < result_words) {
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

  constexpr bool operator == (int other) const {
    if (sizeof(int) <= bytes_per_word) {
      return other >= 0 && used_ <= sizeof(other) && words_[0] == other;
    } else {
      return other >= 0 && *this ==
        BigUIntImpl<(sizeof(int) + bytes_per_word - 1) / bytes_per_word>(other);
    }
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
    used_ = std::max(used_, pos * bytes_per_word);
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

  template <int other_words>
  constexpr BigUIntImpl<std::max(max_words, other_words)>
  GetGreatestCommonDivisor(BigUIntImpl<other_words> &other) {
    if (other == BigUIntImpl<1>{0})
      return *this;

    BigUIntImpl<std::max(max_words, other_words)> mod = *this % other;
    return other.GetGreatestCommonDivisor(mod);
  }

 protected:
  using Parent::used_;
  using Parent::words_;

  using Parent::used_words;
  using Parent::GetCommonWordCount;

  constexpr void Trim() {
    while (used_ > BigUIntWord::bytes_per_word) {
      if (words_[used_ / BigUIntWord::bytes_per_word - 1] != 0) {
        break;
      }
      used_-= BigUIntWord::bytes_per_word;
    }
    if (used_ == BigUIntWord::bytes_per_word &&
        words_[0] <= std::numeric_limits<BigUIntHalfWord>::max()) {
      used_ = sizeof(BigUIntHalfWord);
    }
  }

  constexpr void SignExtend() const {
  }

  // Divide `this` by `other`. Return the quotient and store the remainder in `remainder_out`.
  //
  // This function does not have any small word shortcuts.
  template <int other_words>
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
    BigUIntImpl<max_words + 2> remainder = operator<< <max_words+2>(bits_per_word);
    // Number of bits to shift (*this) to the right, such that:
    // 0 < (*this >> this_shift_right_bits) < 2^bits_per_word
    int this_shift_right_bits = (used_ - bytes_per_word) * bits_per_byte;
    for (;
          this_shift_right_bits >= other_shift_right;
          this_shift_right_bits -= bits_per_word/2 - 2) {
      BigUIntWord this_shifted = remainder.GetWordAtBitOffset(this_shift_right_bits + bits_per_word);

      int shift_result_left = this_shift_right_bits - other_shift_right;
      BigUIntWord result = this_shifted / other_shifted;
      quotient.AddLeftShifted(result, shift_result_left);
      remainder.SubtractLeftShifted(other.Multiply(result), shift_result_left + bits_per_word);
      assert(remainder.GetWordAtBitOffset(this_shift_right_bits + bits_per_word) <= BigUIntWord{1}<<34);
    }
    for (;
          this_shift_right_bits + bits_per_word/2 >= other_shift_right;
          this_shift_right_bits -= bits_per_word/2 - 2) {
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
