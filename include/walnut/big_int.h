#ifndef WALNUT_BIG_INT_H__
#define WALNUT_BIG_INT_H__

#include "walnut/big_int_impl.h"

namespace walnut {

// `bits` is the minimum number of bits BigInt can hold. For example
// BigInt<128> can hold 128 bit signed integers.
template <size_t bits_template>
class BigInt {
  template <size_t other_bits>
  friend class BigInt;

  using BigIntRep = BigIntImpl<(bits_template +
                                BigUIntWord::bits_per_word - 1) /
                               BigUIntWord::bits_per_word>;
 public:
  static constexpr size_t bits = bits_template;
  static constexpr size_t bits_per_word = BigIntRep::bits_per_word;
  static constexpr size_t bytes_per_word = BigIntRep::bytes_per_word;
  static constexpr size_t word_count = (bits + bits_per_word - 1) /
                                       bits_per_word;

  constexpr BigInt() { }

  explicit constexpr BigInt(BigIntHalfWord value) : rep_(value) { }

  explicit constexpr BigInt(BigIntWord value) : rep_(value) { }

  template <size_t other_bits>
  constexpr BigInt(const BigInt<other_bits>& other) : rep_(other.rep_) { }

  template <size_t other_words>
  constexpr BigInt(const BigUIntImpl<other_words>& other) : rep_(other) { }

  template <size_t other_bits>
  constexpr BigInt(const BigUInt<other_bits>& other) : rep_(other) { }

  template <size_t other_bits>
  constexpr BigInt<bits>& operator = (const BigInt<other_bits>& other) {
    rep_ = other.rep_;
    return *this;
  }

  template <size_t other_bits>
  constexpr BigInt<bits>& operator = (const BigUInt<other_bits>& other) {
    rep_ = other;
    return *this;
  }

  constexpr BigInt<bits>& operator = (BigIntWord other) {
    rep_ = other;
    return *this;
  }

  static constexpr BigInt max_value() {
    return BigIntRep::max_value(
        /*set_last_word_bits=*/(bits - 1) % BigUIntWord::bits_per_word);
  }

  static constexpr BigInt min_value() {
    return BigIntRep::min_value(
        /*clear_last_word_bits=*/(bits - 1) % BigUIntWord::bits_per_word);
  }

  template <size_t result_bits=bits>
  constexpr BigInt<result_bits> operator << (int shift) const {
    return rep_ << shift;
  }

  template <size_t result_bits = 0, size_t other_bits,
            size_t rb = result_bits == 0 ?
              std::max(bits, other_bits) + 1 : result_bits>
  constexpr BigInt<rb> Add(const BigInt<other_bits>& other) const {
    return rep_.template Add<(rb + BigUIntWord::bits_per_word - 1) /
                             BigUIntWord::bits_per_word> (other.rep_);
  }

  template <size_t other_bits>
  constexpr BigInt<std::max(bits, other_bits) + 1> operator+(
      const BigInt<other_bits>& other) const {
    return Add<std::max(bits, other_bits) + 1>(other);
  }

  constexpr BigInt<bits + 1> operator+(int other) const {
    constexpr size_t rb = std::max(bits, sizeof(int) * 8) + 1;
    return rep_.template Add<(rb + BigUIntWord::bits_per_word - 1) /
                             BigUIntWord::bits_per_word>(BigIntImpl<1>(other));
  }

  template <size_t other_bits>
  constexpr BigInt& operator+=(const BigInt<other_bits>& other) {
    rep_ += other.rep_;
    return *this;
  }

  constexpr BigInt& operator+=(BigIntHalfWord other) {
    rep_ += other;
    return *this;
  }

  constexpr BigInt& operator++() {
    ++rep_;
    return *this;
  }

  template <size_t other_bits>
  constexpr BigInt& operator-=(const BigInt<other_bits>& other) {
    rep_ -= other.rep_;
    return *this;
  }

  constexpr BigInt& operator-=(BigIntHalfWord other) {
    rep_ -= other;
    return *this;
  }

  constexpr BigInt& operator--() {
    --rep_;
    return *this;
  }

  template <size_t result_bits = 0, size_t other_bits,
            size_t rb = result_bits == 0 ?
              std::max(bits, other_bits) + 1 : result_bits>
  constexpr BigInt<rb> Subtract(const BigInt<other_bits>& other) const {
    return rep_.template Subtract<(rb + BigUIntWord::bits_per_word - 1) /
                                  BigUIntWord::bits_per_word> (other.rep_);
  }

  template <size_t other_bits>
  constexpr BigInt<std::max(bits, other_bits) + 1> operator-(
      const BigInt<other_bits>& other) const {
    return Subtract<std::max(bits, other_bits) + 1>(other);
  }

  constexpr BigInt<bits + 1> operator-(int other) const {
    constexpr size_t rb = std::max(bits, sizeof(int) * 8) + 1;
    return rep_.template Subtract<(
                                   rb + BigUIntWord::bits_per_word - 1) /
                                   BigUIntWord::bits_per_word>(
      BigIntImpl<1>(other));
  }

  template <size_t other_bits>
  constexpr BigInt<bits + other_bits - 1>
  Multiply(const BigInt<other_bits>& other) const {
    return rep_.Multiply(other.rep_);
  }

  template <size_t other_bits>
  constexpr BigInt<bits + other_bits - 1>
  operator*(const BigInt<other_bits>& other) const {
    return rep_ * other.rep_;
  }

  constexpr BigInt<bits + sizeof(int)*8 - 1>
  operator*(const int other) const {
    return rep_ * other;
  }

  constexpr BigInt& operator*=(const int other) {
    rep_ *= other;
    return *this;
  }

  template <size_t other_bits>
  constexpr bool operator < (const BigInt<other_bits>& other) const {
    return rep_ < other.rep_;
  }

  constexpr bool operator < (int other) const {
    return rep_ < other;
  }

  // Returns -1 if *this < `other`,
  //          0 if *this ==  `other`, or
  //          1 if *this > `other`.
  template <size_t other_bits>
  constexpr int Compare(const BigInt<other_bits>& other) const {
    return rep_.Compare(other.rep_);
  }

  template <size_t other_bits>
  constexpr bool operator <= (const BigInt<other_bits>& other) const {
    return rep_ <= other.rep_;
  }

  constexpr bool operator <= (int other) const {
    return rep_ <= other;
  }

  template <size_t other_bits>
  constexpr bool operator > (const BigInt<other_bits>& other) const {
    return rep_ > other.rep_;
  }

  constexpr bool operator > (int other) const {
    return rep_ > other;
  }

  template <size_t other_bits>
  constexpr bool operator >= (const BigInt<other_bits>& other) const {
    return rep_ >= other.rep_;
  }

  constexpr bool operator >= (int other) const {
    return rep_ >= other;
  }

  template <size_t other_bits>
  constexpr bool operator == (const BigInt<other_bits>& other) const {
    return rep_ == other.rep_;
  }

  constexpr bool operator == (int other) const {
    return rep_ == other;
  }

  template <size_t other_bits>
  constexpr bool operator != (const BigInt<other_bits>& other) const {
    return rep_ != other.rep_;
  }

  constexpr bool operator != (int other) const {
    return rep_ != other;
  }

  // Divide `this` by `other`. Return the quotient.
  template <size_t other_bits>
  constexpr BigInt<bits> operator/(const BigInt<other_bits>& other) const {
    return rep_ / other.rep_;
  }

  // Divide `this` by `other`. Return the quotient.
  constexpr BigInt<bits> operator/(int other) const {
    return rep_ / other;
  }

  // Divide `this` by `other`. Return the quotient.
  template <size_t other_bits>
  constexpr BigInt<bits>& operator/=(const BigInt<other_bits>& other) {
    *this = rep_ / other.rep_;
    return *this;
  }

  // Divide `this` by `other`. Return the quotient and store the remainder in `remainder_out`.
  template <size_t other_bits>
  constexpr BigInt<bits> DivideRemainder(const BigInt<other_bits>& other,
      BigInt<std::min(bits, other_bits)>* remainder_out) const {
    return rep_.DivideRemainder(other.rep_, &remainder_out->rep_);
  }

  // Divide `this` by `other`. Return the remainder.
  //
  // If       *this >= 0, then remainder >= 0,
  // else if  *this <  0, then remainder <= 0.
  template <size_t other_bits>
  constexpr BigInt<std::min(bits, other_bits)> operator%(
      const BigInt<other_bits>& other) const {
    return rep_ % other.rep_;
  }

  // Negates *this and returns whether the result overflowed.
  //
  // A return value of false means it did not overflow.
  constexpr bool Negate() {
    return rep_.Negate();
  }

  constexpr BigInt<bits> operator-() const {
    return -rep_;
  }

  constexpr BigInt<bits> abs() const {
    return rep_.abs();
  }

  constexpr BigUInt<bits> GetUIntAbs(bool* was_signed) const {
    return rep_.GetUIntAbs(was_signed);
  }

  constexpr BigInt<bits+1> GetAbs(bool& was_signed) const {
    return rep_.GetAbs(was_signed);
  }

  // Returns a negative integer if `*this` is negative, 0 if *`this` is 0, or a
  // positive integer if `*this` is positive.
  constexpr BigIntWord GetSign() const {
    return rep_.GetSign();
  }

  // Returns 1 if this is greater than or equal to 0.
  // Returns -1 if this is less than 0.
  constexpr int GetAbsMult() const {
    return rep_.GetAbsMult();
  }

  // Returns 1 if this and other >= 0 or if both are negative.
  // Else, returns -1 if only one of this or other is negative.
  template <size_t other_bits>
  constexpr int GetAbsMult(const BigInt<other_bits>& other) const {
    return (SignExtension() ^ other.SignExtension()) | 1;
  }

  // Returns 0 if this is greater than or equal to 0.
  // Returns -1 if this is less than 0.
  constexpr BigIntWord SignExtension() const {
    return BigIntWord{rep_.SignExtension()};
  }

  constexpr bool IsZero() const {
    return GetSign() == 0;
  }

  template <size_t other_bits>
  constexpr bool HasSameSign(const BigInt<other_bits>& other) const {
    return rep_.HasSameSign(other.rep_);
  }

  constexpr uint32_t low_uint32() const {
    return rep_.low_uint32();
  }

  constexpr uint64_t low_uint64() const {
    return rep_.low_uint64();
  }

  constexpr int ToInt() const {
    return rep_.ToInt();
  }

  constexpr int used_bytes() const {
    return rep_.used_bytes();
  }

  constexpr const BigUIntWord* words() const {
    return rep_.words();
  }

  // Verifies the value is in the supported range.
  //
  // BigInts can sometimes internally support a larger range than
  // [min_value(), max_value()].
  // This function returns true if the value is within the [min_value(),
  // max_value()]. For invalid values, it is possible for this function to
  // return false, because some BigInts can internally a larger range.
  bool IsValidState() const {
    return min_value() <= *this && *this <= max_value();
  }

  template <size_t r1_c1_bits, size_t r1_c2_bits, size_t r2_c1_bits,
            size_t r2_c2_bits>
  static BigInt Determinant(const BigInt<r1_c1_bits>& r1_c1,
                            const BigInt<r1_c2_bits>& r1_c2,
                            const BigInt<r2_c1_bits>& r2_c1,
                            const BigInt<r2_c2_bits>& r2_c2) {
    return r1_c1*r2_c2 - r2_c1*r1_c2;
  }

  template <size_t other_bits>
  constexpr BigInt<std::max(bits, other_bits)>
  GetGreatestCommonDivisor(const BigInt<other_bits> &other) const {
    if (other.IsZero()) return *this;

    BigInt<std::max(bits, other_bits)> mod = *this % other;
    return other.GetGreatestCommonDivisor(mod);
  }

  explicit operator double() const {
    return (double)rep_;
  }

  template <size_t print_bits>
  friend std::ostream& operator<<(std::ostream& out, const BigInt<print_bits>& bigint);

 private:
  template <size_t other_words>
  constexpr BigInt(const BigIntImpl<other_words>& other) : rep_(other) { }

  BigIntRep rep_;
};

template <size_t print_bits>
std::ostream& operator<<(std::ostream& out, const BigInt<print_bits>& bigint) {
  return out << bigint.rep_;
}

}  // walnut

#endif // WALNUT_BIG_INT_H__
