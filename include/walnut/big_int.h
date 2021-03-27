#ifndef WALNUT_BIG_INT_H__
#define WALNUT_BIG_INT_H__

#include "walnut/big_int_impl.h"

namespace walnut {

// `bits` is the minimum number of bits BigInt can hold. For example
// BigInt<128> can hold 128 bit signed integers.
template <size_t bits_template>
class BigInt : public BigIntImpl {
  template <size_t other_bits>
  friend class BigInt;
 public:

  static constexpr size_t bits = bits_template;
  using BigIntImpl::bits_per_word;
  using BigIntImpl::bytes_per_word;
  static constexpr size_t word_count = (bits + bits_per_word - 1) /
                                       bits_per_word;

  using BigIntImpl::BigIntImpl;

  template <size_t other_bits>
  constexpr BigInt(const BigInt<other_bits>& other) : BigIntImpl(other) { }

  constexpr BigInt(const BigIntImpl& other) : BigIntImpl(other) { }

  template <size_t other_bits>
  constexpr BigInt<bits>& operator = (const BigInt<other_bits>& other) {
    BigIntImpl::operator=(other);
    return *this;
  }

  constexpr BigInt<bits>& operator = (BigIntWord other) {
    BigIntImpl::operator=(other);
    return *this;
  }

  static constexpr BigInt max_value() {
    return BigIntImpl::max_value(/*set_bits=*/bits - 1);
  }

  static constexpr BigInt min_value() {
    return BigIntImpl::min_value(/*clear_bits=*/bits - 1);
  }

  template <size_t result_bits=bits>
  constexpr BigInt<result_bits> operator << (int shift) const {
    return BigIntImpl::operator<<(shift);
  }

  template <size_t result_bits = 0, size_t other_bits,
            size_t rb = result_bits == 0 ?
              std::max(bits, other_bits) + 1 : result_bits>
  constexpr BigInt<rb> Add(const BigInt<other_bits>& other) const {
    return BigIntImpl::Add(other);
  }

  template <size_t other_bits>
  constexpr BigInt<std::max(bits, other_bits) + 1> operator+(
      const BigInt<other_bits>& other) const {
    return Add<std::max(bits, other_bits) + 1>(other);
  }

  constexpr BigInt<bits + 1> operator+(int other) const {
    return BigIntImpl::Add(BigIntImpl(other));
  }

  template <size_t other_bits>
  constexpr BigInt& operator+=(const BigInt<other_bits>& other) {
    BigIntImpl::operator+=(other);
    return *this;
  }

  constexpr BigInt& operator+=(BigIntHalfWord other) {
    BigIntImpl::operator+=(other);
    return *this;
  }

  constexpr BigInt& operator++() {
    BigIntImpl::operator++();
    return *this;
  }

  template <size_t other_bits>
  constexpr BigInt& operator-=(const BigInt<other_bits>& other) {
    BigIntImpl::operator-=(other);
    return *this;
  }

  constexpr BigInt& operator-=(BigIntHalfWord other) {
    BigIntImpl::operator-=(other);
    return *this;
  }

  constexpr BigInt& operator--() {
    BigIntImpl::operator--();
    return *this;
  }

  template <size_t result_bits = 0, size_t other_bits,
            size_t rb = result_bits == 0 ?
              std::max(bits, other_bits) + 1 : result_bits>
  constexpr BigInt<rb> Subtract(const BigInt<other_bits>& other) const {
    return BigIntImpl::Subtract(other);
  }

  template <size_t other_bits>
  constexpr BigInt<std::max(bits, other_bits) + 1> operator-(
      const BigInt<other_bits>& other) const {
    return Subtract<std::max(bits, other_bits) + 1>(other);
  }

  constexpr BigInt<bits + 1> operator-(int other) const {
    return BigIntImpl::operator-(other);
  }

  template <size_t other_bits>
  constexpr BigInt<bits + other_bits - 1>
  Multiply(const BigInt<other_bits>& other) const {
    return BigIntImpl::Multiply(other);
  }

  template <size_t other_bits>
  constexpr BigInt<bits + other_bits - 1>
  operator*(const BigInt<other_bits>& other) const {
    return BigIntImpl::operator*(other);
  }

  constexpr BigInt<bits + sizeof(int)*8 - 1>
  operator*(const int other) const {
    return BigIntImpl::operator*(other);
  }

  constexpr BigInt& operator*=(const int other) {
    BigIntImpl::operator*=(other);
    return *this;
  }

  // Divide `this` by `other`. Return the quotient.
  template <size_t other_bits>
  constexpr BigInt<bits> operator/(const BigInt<other_bits>& other) const {
    return BigIntImpl::operator/(other);
  }

  // Divide `this` by `other`. Return the quotient.
  constexpr BigInt<bits> operator/(int other) const {
    return BigIntImpl::operator/(other);
  }

  // Divide `this` by `other`. Return the quotient.
  template <size_t other_bits>
  constexpr BigInt<bits>& operator/=(const BigInt<other_bits>& other) {
    *this = BigIntImpl::operator/(other);
    return *this;
  }

  // Divide `this` by `other`. Return the quotient and store the remainder in `remainder_out`.
  template <size_t other_bits>
  constexpr BigInt<bits> DivideRemainder(const BigInt<other_bits>& other,
      BigInt<std::min(bits, other_bits)>* remainder_out) const {
    return BigIntImpl::DivideRemainder(other, remainder_out);
  }

  // Divide `this` by `other`. Return the remainder.
  //
  // If       *this >= 0, then remainder >= 0,
  // else if  *this <  0, then remainder <= 0.
  template <size_t other_bits>
  constexpr BigInt<std::min(bits, other_bits)> operator%(
      const BigInt<other_bits>& other) const {
    return BigIntImpl::operator%(other);
  }

  constexpr BigInt<bits> operator-() const {
    return BigIntImpl::operator-();
  }

  constexpr BigInt<bits> abs() const {
    return BigIntImpl::abs();
  }

  constexpr BigInt<bits+1> GetAbs(bool& was_signed) const {
    return BigIntImpl::GetAbs(was_signed);
  }

  // Verifies the value is in the supported range.
  //
  // BigInts can sometimes internally support a larger range than
  // [min_value(), max_value()].
  // This function returns true if the value is within the [min_value(),
  // max_value()]. For invalid values, it is possible for this function to
  // return false, because some BigInts can internally a larger range.
  bool IsValidState() const {
    BigInt minv = min_value();
    BigInt maxv = max_value();
    return minv <= *this && *this <= maxv;
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
};

template <size_t print_bits>
std::ostream& operator<<(std::ostream& out, const BigInt<print_bits>& bigint) {
  return out << static_cast<const BigIntImpl&>(bigint);
}

}  // walnut

#endif // WALNUT_BIG_INT_H__
