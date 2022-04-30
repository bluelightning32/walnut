#ifndef WALNUT_BIG_UINT_WORD_H__
#define WALNUT_BIG_UINT_WORD_H__

#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace walnut {

using BigIntHalfWord = int32_t;
using BigUIntHalfWord = uint32_t;

using BigIntWord = int64_t;

#ifndef __has_builtin
  // Compatibility with non-clang compilers.
#  define __has_builtin(x) 0
#endif

#ifndef HAS_GNUC_VERSION
#  ifdef __GNUC__
#    define HAS_GNUC_VERSION(major, minor, patch) \
       ((__GNUC__ > (major)) || (__GNUC__ == (major) && \
           (__GNUC_MINOR__ > (minor) || __GNUC_MINOR__ == (minor) && \
              __GNUC_PATCHLEVEL__ >= (patch))))
#  else
#    define HAS_GNUC_VERSION(major, minor, patch) false
#  endif
#endif

#if __has_builtin(__builtin_add_overflow)
#  define HAS_BUILTIN_ADD_OVERFLOW
#elif HAS_GNUC_VERSION(5, 0, 0)
#  define HAS_BUILTIN_ADD_OVERFLOW
#endif

#if __has_builtin(__builtin_sub_overflow)
#  define HAS_BUILTIN_SUB_OVERFLOW
#elif HAS_GNUC_VERSION(5, 0, 0)
#  define HAS_BUILTIN_SUB_OVERFLOW
#endif

#if __has_builtin(__builtin_clzl)
#  define HAS_BUILTIN_CLZL
#elif HAS_GNUC_VERSION(3, 4, 6)
#  define HAS_BUILTIN_CLZL
#endif

#if __has_builtin(__builtin_ctzl)
#  define HAS_BUILTIN_CTZL
#elif HAS_GNUC_VERSION(3, 4, 6)
#  define HAS_BUILTIN_CTZL
#endif

// A structure that holds an integer of the system's largest native format.
class BigUIntWord {
 public:
  static constexpr bool has_overloads = true;
  static constexpr unsigned bytes_per_word = sizeof(uint64_t);
  static constexpr unsigned bits_per_word = 8 * bytes_per_word;

  constexpr BigUIntWord() : i_(0) { }

  explicit constexpr BigUIntWord(int i) : i_(i) { }

  explicit constexpr BigUIntWord(uint64_t i) : i_(i) { }

  explicit constexpr BigUIntWord(BigIntWord i) : i_(i) { }

  // This constructor is only used on 32 bit platforms.
  template <typename Long = long,
            std::enable_if_t<!std::is_same<BigIntWord, Long>::value,
                             bool> = true>
  explicit constexpr BigUIntWord(Long i) : i_(i) { }

  explicit constexpr BigUIntWord(uint32_t i) : i_(i) { }

  constexpr BigUIntWord& operator = (int v) {
    i_ = v;
    return *this;
  }

  constexpr BigUIntWord& operator = (uint32_t v) {
    i_ = v;
    return *this;
  }

  constexpr BigUIntWord& operator = (uint64_t v) {
    i_ = v;
    return *this;
  }

  constexpr BigUIntWord& operator = (BigIntWord v) {
    i_ = v;
    return *this;
  }

  constexpr BigUIntWord& operator = (const BigUIntWord& other) = default;

  explicit constexpr operator BigIntWord() const {
    return i_;
  }

  static constexpr BigUIntWord max_value() {
    return BigUIntWord(std::numeric_limits<uint64_t>::max());
  }

  constexpr uint32_t low_uint32() const {
    return static_cast<uint32_t>(i_);
  }

  constexpr int ToInt() const {
    assert(BigIntWord(std::numeric_limits<int>::min()) <= BigIntWord(i_));
    assert(BigIntWord(i_) <= BigIntWord(std::numeric_limits<int>::max()));
    return int(i_);
  }

  constexpr BigUIntHalfWord low_half_word() const {
    return low_uint32();
  }

  constexpr BigUIntHalfWord high_half_word() const {
    return i_ >> 32;
  }

  constexpr uint64_t low_uint64() const {
    return i_;
  }

  constexpr bool operator == (const BigUIntWord& other) const {
    return i_ == other.i_;
  }

  constexpr bool operator != (const BigUIntWord& other) const {
    return i_ != other.i_;
  }

  constexpr bool operator < (const BigUIntWord& other) const {
    return i_ < other.i_;
  }

  constexpr bool operator <= (const BigUIntWord& other) const {
    return i_ <= other.i_;
  }

  constexpr bool operator > (const BigUIntWord& other) const {
    return i_ > other.i_;
  }

  constexpr bool operator >= (const BigUIntWord& other) const {
    return i_ >= other.i_;
  }

  // Returns -1 if *this < `other`,
  //          0 if *this ==  `other`, or
  //          1 if *this > `other`.
  constexpr int Compare(const BigUIntWord& other) const {
    if (i_ == other.i_) return 0;
    return (i_ < other.i_) ? -1 : 1;
  }

  constexpr bool operator == (uint32_t other) const {
    return i_ == other;
  }

  constexpr bool operator != (uint32_t other) const {
    return i_ != other;
  }

  constexpr bool operator <= (uint32_t other) const {
    return i_ <= other;
  }

  constexpr bool operator < (uint32_t other) const {
    return i_ < other;
  }

  constexpr BigUIntWord operator^(const BigUIntWord& other) const {
    return BigUIntWord(i_ ^ other.i_);
  }

  constexpr BigUIntWord operator&(const BigUIntWord& other) const {
    return BigUIntWord(i_ & other.i_);
  }

  constexpr BigUIntWord& operator-=(const BigUIntWord& other) {
    i_ -= other.i_;
    return *this;
  }

  constexpr BigUIntWord Add(const BigUIntWord& other) const {
    return BigUIntWord(i_ + other.i_);
  }

#ifdef HAS_BUILTIN_ADD_OVERFLOW
  constexpr BigUIntWord Add(const BigUIntWord& other, bool* carry_out) const {
    uint64_t result = 0;
    *carry_out = __builtin_add_overflow(i_, other.i_, &result);
    return BigUIntWord(result);
  }

  constexpr BigUIntWord Add(const BigUIntWord& other, bool carry_in,
                            bool* carry_out) const {
    uint64_t result = 0;
    *carry_out =
      static_cast<int>(__builtin_add_overflow(i_, other.i_, &result)) |
      static_cast<int>(__builtin_add_overflow(result, carry_in, &result));
    return BigUIntWord(result);
  }

  constexpr BigUIntWord Add(bool carry_in, bool* carry_out) const {
    uint64_t result = 0;
    *carry_out = __builtin_add_overflow(i_, carry_in, &result);
    return BigUIntWord(result);
  }

#else

  constexpr BigUIntWord Add(const BigUIntWord& other, bool* carry_out) const {
    uint64_t result = i_ + other.i_;
    *carry_out = result < i_;
    return BigUIntWord(result);
  }

  constexpr BigUIntWord Add(const BigUIntWord& other, bool carry_in,
                            bool* carry_out) const {
    uint64_t result1 = i_ + other.i_;
    uint64_t result2 = result1 + carry_in;
    *carry_out = static_cast<int>(result1 < i_) |
                 static_cast<int>(result2 < result1);
    return BigUIntWord(result2);
  }

  constexpr BigUIntWord Add(bool carry_in, bool* carry_out) const {
    uint64_t result = i_ + carry_in;
    *carry_out = result < i_;
    return BigUIntWord(result);
  }
#endif // HAS_BUILTIN_ADD_OVERFLOW

  constexpr BigUIntWord Subtract(const BigUIntWord& other) const {
    return BigUIntWord(i_ - other.i_);
  }

#ifdef HAS_BUILTIN_SUB_OVERFLOW
  constexpr BigUIntWord Subtract(const BigUIntWord& other, bool carry_in,
                                 bool* carry_out) const {
    uint64_t result = 0;
    *carry_out =
      static_cast<int>(__builtin_sub_overflow(i_, other.i_, &result)) |
      static_cast<int>(__builtin_sub_overflow(result, carry_in, &result));
    return BigUIntWord(result);
  }

  constexpr BigUIntWord Subtract(const BigUIntWord& other,
                                 bool* carry_out) const {
    uint64_t result = 0;
    *carry_out = __builtin_sub_overflow(i_, other.i_, &result);
    return BigUIntWord(result);
  }

  constexpr BigUIntWord Subtract(bool carry_in, bool* carry_out) const {
    uint64_t result = 0;
    *carry_out = __builtin_sub_overflow(i_, carry_in, &result);
    return BigUIntWord(result);
  }

#else

  constexpr BigUIntWord Subtract(const BigUIntWord& other, bool carry_in,
                                 bool* carry_out) const {
    uint64_t result1 = i_ - other.i_;
    *carry_out = static_cast<int>(result1 > i_) |
                 static_cast<int>(result1 < static_cast<uint64_t>(carry_in));
    uint64_t result2 = result1 - carry_in;
    return BigUIntWord(result2);
  }

  constexpr BigUIntWord Subtract(const BigUIntWord& other,
                                 bool* carry_out) const {
    uint64_t result = i_ - other.i_;
    *carry_out = result > i_;
    return BigUIntWord(result);
  }

  constexpr BigUIntWord Subtract(bool carry_in, bool* carry_out) const {
    uint64_t result = i_ - carry_in;
    *carry_out = result > i_;
    return BigUIntWord(result);
  }
#endif // HAS_BUILTIN_SUB_OVERFLOW

  constexpr BigUIntWord MultiplyAsHalfWord(const BigUIntWord& other) const {
    return BigUIntWord(i_ * other.i_);
  }

  constexpr BigUIntWord MultiplyAsHalfWord(BigUIntHalfWord other) const {
    return BigUIntWord(i_ * other);
  }

#ifdef __SIZEOF_INT128__
  constexpr BigUIntWord Multiply(const BigUIntWord& other,
                                 BigUIntWord* high_out) const {
    unsigned __int128 full = static_cast<unsigned __int128>(this->i_) *
      static_cast<unsigned __int128>(other.i_);
    *high_out = BigUIntWord(static_cast<uint64_t>(full >> 64));
    return BigUIntWord(static_cast<uint64_t>(full));
  }

  constexpr BigUIntWord MultiplySigned(const BigUIntWord& other,
                                       BigUIntWord* high_out) const {
    __int128 full = static_cast<__int128>(static_cast<BigIntWord>(this->i_)) *
      static_cast<__int128>(static_cast<BigIntWord>(other.i_));
    *high_out = BigUIntWord(static_cast<uint64_t>(full >> 64));
    return BigUIntWord(static_cast<uint64_t>(full));
  }

  constexpr BigUIntWord MultiplyAdd(const BigUIntWord& other,
                                    const BigUIntWord& add, bool carry_in,
                                    BigUIntWord* high_out) const {
    unsigned __int128 full = static_cast<unsigned __int128>(this->i_) *
      static_cast<unsigned __int128>(other.i_) + add.i_ + carry_in;
    *high_out = BigUIntWord(static_cast<uint64_t>(full >> 64));
    return BigUIntWord(static_cast<uint64_t>(full));
  }

  constexpr BigUIntWord ShiftRight(const BigUIntWord& high,
                                   unsigned shift) const {
    unsigned __int128 full = (static_cast<unsigned __int128>(high.i_) << 64) |
                             this->i_;
    return BigUIntWord(static_cast<uint64_t>(full >> shift));
  }

#else

  constexpr BigUIntWord Multiply(const BigUIntWord& other,
                                 BigUIntWord* high_out) const {
    BigUIntWord low(static_cast<uint64_t>(low_uint32()) * other.low_uint32());

    bool carry = false;
    BigUIntWord middle =
      BigUIntWord(static_cast<uint64_t>(hi_uint32()) * other.low_uint32() +
                  low.hi_uint32())
        .Add(BigUIntWord{static_cast<uint64_t>(low_uint32()) *
                                               other.hi_uint32()},
             &carry);

    high_out->i_ = static_cast<uint64_t>(hi_uint32()) * other.hi_uint32() +
      middle.hi_uint32() + (static_cast<uint64_t>(carry) << 32);

    return BigUIntWord(low.low_uint32(), middle.low_uint32());
  }

  constexpr BigUIntWord MultiplySigned(const BigUIntWord& other,
                                    BigUIntWord* high_out) const {
    BigUIntWord low(static_cast<uint64_t>(low_uint32()) * other.low_uint32());

    bool carry = false;
    BigUIntWord middle =
      BigUIntWord(static_cast<uint64_t>(hi_uint32()) * other.low_uint32() +
                  low.hi_uint32())
        .Add(BigUIntWord{static_cast<uint64_t>(low_uint32()) *
                         other.hi_uint32()},
             &carry);

    high_out->i_ = static_cast<uint64_t>(hi_uint32()) * other.hi_uint32() +
      middle.hi_uint32() + (static_cast<uint64_t>(carry) << 32) -
      (other.SignExtension().i_ & i_) -
      (SignExtension().i_ & other.i_);

    return BigUIntWord(low.low_uint32(), middle.low_uint32());
  }

  constexpr BigUIntWord MultiplyAdd(const BigUIntWord& other,
                                    const BigUIntWord& add, bool carry_in,
                                    BigUIntWord* high_out) const {
    bool low_carry_out = false;
    BigUIntWord low = BigUIntWord{
      static_cast<uint64_t>(low_uint32()) * other.low_uint32()}
      .Add(add, carry_in, &low_carry_out);

    bool middle_carry_out = false;
    BigUIntWord middle =
      BigUIntWord(static_cast<uint64_t>(hi_uint32()) * other.low_uint32() +
                  low.hi_uint32())
        .Add(BigUIntWord{static_cast<uint64_t>(low_uint32()) *
                         other.hi_uint32()},
             &middle_carry_out);

    high_out->i_ = static_cast<uint64_t>(hi_uint32()) * other.hi_uint32() +
      middle.hi_uint32() + (static_cast<uint64_t>(middle_carry_out) << 32) +
      low_carry_out;

    return BigUIntWord(low.low_uint32(), middle.low_uint32());
  }

  // Undefined if shift >= bits_per_word
  constexpr BigUIntWord ShiftRight(const BigUIntWord& high,
                                   unsigned shift) const {
    // The if statement is necessary because shifting more than bit_per_word is
    // undefined.
    if (shift) {
      return high << (bits_per_word - shift) | (*this) >> shift;
    } else {
      return *this;
    }
  }
#endif // __SIZEOF_INT128__

  // Undefined if shift >= bits_per_word
  constexpr BigUIntWord operator<<(unsigned shift) const {
    return BigUIntWord(i_ << shift);
  }

  // Undefined if shift >= bits_per_word
  constexpr BigUIntWord operator>>(unsigned shift) const {
    return BigUIntWord(i_ >> shift);
  }

  constexpr BigUIntWord operator|(const BigUIntWord& other) const {
    return BigUIntWord(i_ | other.i_);
  }

  constexpr BigUIntWord operator~() const {
    return BigUIntWord(~i_);
  }

  // Return the 1 based index of the highest set bit, or 0 if no bits are set.
  constexpr unsigned GetHighestSetBitUnaccelerated() const {
    const uint64_t test[] = {
      static_cast<uint64_t>(0xFFFFFFFF00000000),
      static_cast<uint64_t>(0xFFFF0000),
      static_cast<uint64_t>(0xFF00),
      static_cast<uint64_t>(0xF0),
      static_cast<uint64_t>(0xC),
      static_cast<uint64_t>(0x2),
    };
    const unsigned int low_bit_index[] = {32, 16, 8, 4, 2, 1};

    unsigned result = 0;
    uint64_t v = i_;
    for (unsigned j = 0; j < sizeof(test)/sizeof(test[0]); j++) {
      if (v & test[j])
      {
        v >>= low_bit_index[j];
        result |= low_bit_index[j];
      } 
    }
    return result + static_cast<unsigned>(v);
  }

#ifdef HAS_BUILTIN_CLZL
  // Return the 1 based index of the highest set bit, or 0 if no bits are set.
  template <typename X=unsigned>
  constexpr
  typename std::enable_if_t<bytes_per_word == sizeof(unsigned long), X>
  GetHighestSetBit() const {
    return this->i_ == 0 ? 0 : bits_per_word - __builtin_clzl(this->i_);
  }

  // Return the 1 based index of the highest set bit, or 0 if no bits are set.
  template <typename X=unsigned>
  constexpr
  typename std::enable_if_t<bytes_per_word != sizeof(unsigned long), X>
  GetHighestSetBit() const {
    return GetHighestSetBitUnaccelerated();
  }

#else

  // Return the 1 based index of the highest set bit, or 0 if no bits are set.
  constexpr unsigned GetHighestSetBit() const {
    return GetHighestSetBitUnaccelerated();
  }

#endif // HAS_BUILTIN_CLZL

  // Return the number of least significant zeros, or 0 if no bits are set.
  constexpr unsigned GetTrailingZerosUnaccelerated() const {
    if (i_ == 0) return 0;

    unsigned trailing_zeros = 0;
    while (!(i_ & (static_cast<uint64_t>(1) << trailing_zeros))) {
      ++trailing_zeros;
    }
    return trailing_zeros;
  }

#ifdef HAS_BUILTIN_CTZL
  // Return the number of least significant zeros, or 0 if no bits are set.
  template <typename X=unsigned>
  constexpr
  typename std::enable_if_t<bytes_per_word == sizeof(unsigned long), X>
  GetTrailingZeros() const {
    return this->i_ == 0 ? 0 : __builtin_ctzl(this->i_);
  }

  // Return the number of least significant zeros, or 0 if no bits are set.
  template <typename X=unsigned>
  constexpr
  typename std::enable_if_t<bytes_per_word != sizeof(unsigned long), X>
  GetTrailingZeros() const {
    return GetTrailingZerosUnaccelerated();
  }
#else
  // Return the number of least significant zeros, or 0 if no bits are set.
  constexpr unsigned GetTrailingZeros() const {
    return GetTrailingZerosUnaccelerated();
  }
#endif // HAS_BUILTIN_CTZL

  // Prefix overload
  constexpr BigUIntWord& operator++() {
    ++i_;
    return *this;
  }

  constexpr BigUIntWord& operator--() {
    --i_;
    return *this;
  }

  constexpr BigUIntWord& operator+=(const BigUIntWord& other) {
    i_ += other.i_;
    return *this;
  }

  constexpr BigUIntWord operator/(const BigUIntWord& other) const {
    return BigUIntWord{i_ / other.i_};
  }

  constexpr BigUIntWord operator%(const BigUIntWord& other) const {
    return BigUIntWord{i_ % other.i_};
  }

  constexpr BigUIntWord& operator|=(const BigUIntWord& other) {
    i_ |= other.i_;
    return *this;
  }

  constexpr BigUIntWord& operator&=(const BigUIntWord& other) {
    i_ &= other.i_;
    return *this;
  }

  constexpr BigUIntWord& operator^=(const BigUIntWord& other) {
    i_ ^= other.i_;
    return *this;
  }

  constexpr BigUIntWord SignExtension() const {
    return BigUIntWord{BigIntWord(i_) >> 63};
  }

  template <typename X = BigUIntWord>
  constexpr
  typename std::enable_if_t<sizeof(long long int) >= sizeof(BigIntWord), X>
  SignedAbs() const {
    return BigUIntWord{uint64_t(llabs(BigIntWord(i_)))};
  }

  template <typename X = BigUIntWord>
  constexpr
  typename std::enable_if_t<sizeof(long long int) < sizeof(BigIntWord), X>
  SignedAbs() const {
    return BigUIntWord{BigIntWord(i_) >= 0 ? BigIntWord(i_) : -BigIntWord(i_)};
  }

  double ToDoubleWithShift(int shift) const {
    return std::ldexp(i_, shift);
  }

  long double ToLongDoubleWithShift(int shift) const {
    return std::ldexp((long double)i_, shift);
  }

 private:
  uint64_t i_;

  constexpr BigUIntWord(uint32_t lo, uint32_t hi)
    : i_(static_cast<uint64_t>(hi) << 32 | lo) { }

  constexpr uint32_t hi_uint32() const {
    return i_ >> 32;
  }
};

}  // walnut

#endif // WALNUT_BIG_UINT_WORD_H__
