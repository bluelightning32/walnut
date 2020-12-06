#ifndef WALNUT_BIG_UINT_WORD_H__
#define WALNUT_BIG_UINT_WORD_H__

#include <cmath>
#include <cstdint>
#include <limits>
#include <cmath>
#include <type_traits>

namespace walnut {

using BigIntHalfWord = int32_t;
using BigUIntHalfWord = uint32_t;

using BigIntWord = int64_t;

template <typename Descendent, typename Default>
using GetImplType =
  typename std::conditional<std::is_same<Descendent, void>::value,
                            Default, Descendent>::type;

// A structure that holds an integer of the system's largest native format.
// Subclasses provide progressively more acceleration (when the necessary
// compiler or build target is available). The acceleration is moved into
// subclasses so that the same unit tests can be run against all versions that
// can run on the target.
template <typename Descendent = void>
class BigUIntWordBase {
 protected:
  using ImplType = GetImplType<Descendent, BigUIntWordBase>;
  // Get the most derived version of this class.
  constexpr const ImplType* impl() const {
    static_cast<const ImplType*>(this);
  }

  // Get the most derived version of this class.
  constexpr ImplType* impl() {
    return static_cast<ImplType*>(this);
  }

 public:
  static constexpr bool has_overloads = true;
  static constexpr int bytes_per_word = sizeof(uint64_t);
  static constexpr int bits_per_word = 8 * bytes_per_word;

  constexpr BigUIntWordBase() : i_(0) { }

  explicit constexpr BigUIntWordBase(int i) : i_(i) { }

  explicit constexpr BigUIntWordBase(uint64_t i) : i_(i) { }

  explicit constexpr BigUIntWordBase(BigIntWord i) : i_(i) { }

  explicit constexpr BigUIntWordBase(uint32_t i) : i_(i) { }

  constexpr ImplType& operator = (int v) {
    i_ = v;
    return *impl();
  }

  constexpr ImplType& operator = (uint32_t v) {
    i_ = v;
    return *impl();
  }

  constexpr ImplType& operator = (uint64_t v) {
    i_ = v;
    return *impl();
  }

  constexpr ImplType& operator = (BigIntWord v) {
    i_ = v;
    return *impl();
  }

  constexpr BigUIntWordBase& operator = (const BigUIntWordBase& other) = default;

  explicit constexpr operator BigIntWord() const {
    return i_;
  }

  static constexpr ImplType max_value() {
    return ImplType(std::numeric_limits<uint64_t>::max());
  }

  constexpr uint32_t low_uint32() const {
    return i_;
  }

  constexpr BigUIntHalfWord low_half_word() const {
    return low_uint32();
  }

  constexpr uint64_t low_uint64() const {
    return i_;
  }

  constexpr bool operator == (const ImplType& other) const {
    return i_ == other.i_;
  }

  constexpr bool operator != (const ImplType& other) const {
    return i_ != other.i_;
  }

  constexpr bool operator < (const ImplType& other) const {
    return i_ < other.i_;
  }

  constexpr bool operator <= (const ImplType& other) const {
    return i_ <= other.i_;
  }

  constexpr bool operator > (const ImplType& other) const {
    return i_ > other.i_;
  }

  constexpr bool operator >= (const ImplType& other) const {
    return i_ >= other.i_;
  }

  // Returns -1 if *this < `other`,
  //          0 if *this ==  `other`, or
  //          1 if *this > `other`.
  constexpr int CompareSigned(const ImplType& other) const {
    if (i_ == other.i_) return 0;
    return (i_ < other.i_) ? -1 : 1;
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

  constexpr ImplType operator^(const ImplType& other) const {
    return ImplType(i_ ^ other.i_);
  }

  constexpr ImplType operator&(const ImplType& other) const {
    return ImplType(i_ & other.i_);
  }

  constexpr ImplType& operator-=(const ImplType& other) {
    i_ -= other.i_;
    return *impl();
  }

  constexpr ImplType Add(const ImplType& other, bool* carry_out) const {
    uint64_t result = i_ + other.i_;
    *carry_out = result < i_;
    return ImplType(result);
  }

  constexpr ImplType Add(const ImplType& other, bool carry_in, bool* carry_out) const {
    uint64_t result1 = i_ + other.i_;
    uint64_t result2 = result1 + carry_in;
    *carry_out = result1 < i_ || result2 < result1;
    return ImplType(result2);
  }

  constexpr ImplType Add(bool carry_in, bool* carry_out) const {
    uint64_t result = i_ + carry_in;
    *carry_out = result < i_;
    return ImplType(result);
  }

  constexpr ImplType Add(const ImplType& other) const {
    return ImplType(i_ + other.i_);
  }

  constexpr ImplType Subtract(const ImplType& other) const {
    return ImplType(i_ - other.i_);
  }

  constexpr ImplType Subtract(const ImplType& other, bool carry_in, bool* carry_out) const {
    uint64_t result1 = i_ - other.i_;
    *carry_out = result1 > i_ || result1 < carry_in;
    uint64_t result2 = result1 - carry_in;
    return ImplType(result2);
  }

  constexpr ImplType Subtract(const ImplType& other, bool* carry_out) const {
    uint64_t result = i_ - other.i_;
    *carry_out = result > i_;
    return ImplType(result);
  }

  constexpr ImplType Subtract(bool carry_in, bool* carry_out) const {
    uint64_t result = i_ - carry_in;
    *carry_out = result > i_;
    return ImplType(result);
  }

  constexpr ImplType MultiplyAsHalfWord(const ImplType& other) const {
    return ImplType(i_ * other.i_);
  }

  constexpr ImplType MultiplyAsHalfWord(BigUIntHalfWord other) const {
    return ImplType(i_ * other);
  }

  constexpr ImplType Multiply(const ImplType& other, ImplType* high_out) const {
    ImplType low(static_cast<uint64_t>(low_uint32()) * other.low_uint32());

    bool carry = false;
    ImplType middle =
      ImplType(static_cast<uint64_t>(hi_uint32()) * other.low_uint32() + low.hi_uint32())
        .Add(ImplType{static_cast<uint64_t>(low_uint32()) * other.hi_uint32()}, &carry);

    high_out->i_ = static_cast<uint64_t>(hi_uint32()) * other.hi_uint32() +
      middle.hi_uint32() + (static_cast<uint64_t>(carry) << 32);

    return ImplType(low.low_uint32(), middle.low_uint32());
  }

  constexpr ImplType MultiplyAdd(const ImplType& other,
                                              const ImplType& add,
                                              bool carry_in,
                                              ImplType* high_out) const {
    bool low_carry_out = false;
    ImplType low = ImplType{
      static_cast<uint64_t>(low_uint32()) * other.low_uint32()}
      .Add(add, carry_in, &low_carry_out);

    bool middle_carry_out = false;
    ImplType middle =
      ImplType(static_cast<uint64_t>(hi_uint32()) * other.low_uint32() + low.hi_uint32())
        .Add(ImplType{static_cast<uint64_t>(low_uint32()) * other.hi_uint32()}, &middle_carry_out);

    high_out->i_ = static_cast<uint64_t>(hi_uint32()) * other.hi_uint32() +
      middle.hi_uint32() + (static_cast<uint64_t>(middle_carry_out) << 32) + low_carry_out;

    return ImplType(low.low_uint32(), middle.low_uint32());
  }

  // Undefined if shift >= bits_per_word
  constexpr ImplType operator<<(unsigned shift) const {
    return ImplType(i_ << shift);
  }

  // Undefined if shift >= bits_per_word
  constexpr ImplType operator>>(unsigned shift) const {
    return ImplType(i_ >> shift);
  }

  // Undefined if shift >= bits_per_word
  constexpr ImplType ShiftRight(const ImplType& high, unsigned shift) const {
    // The if statement is necessary because shifting more than bit_per_word is
    // undefined.
    if (shift) {
      return high << (bits_per_word - shift) | (*this) >> shift;
    } else {
      return *this;
    }
  }

  constexpr ImplType operator|(const ImplType& other) const {
    return ImplType(i_ | other.i_);
  }

  constexpr ImplType operator~() const {
    return ImplType(~i_);
  }

  // Return the 1 based index of the highest set bit, or 0 if no bits are set.
  constexpr unsigned GetHighestSetBit() const {
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

  // Prefix overload
  constexpr ImplType& operator++() {
    ++i_;
    return *impl();
  }

  constexpr ImplType& operator+=(const ImplType& other) {
    i_ += other.i_;
    return *impl();
  }

  constexpr ImplType operator/(const ImplType& other) const {
    return ImplType{i_ / other.i_};
  }

  constexpr ImplType operator%(const ImplType& other) const {
    return ImplType{i_ % other.i_};
  }

  constexpr ImplType& operator|=(const ImplType& other) {
    i_ |= other.i_;
    return *impl();
  }

  constexpr ImplType& operator&=(const ImplType& other) {
    i_ &= other.i_;
    return *impl();
  }

  constexpr ImplType SignExtension() const {
    return ImplType{BigIntWord(i_) >> 63};
  }

  constexpr ImplType SignedAbs() const {
    if (sizeof(long long int) >= sizeof(BigIntWord)) {
      return ImplType{uint64_t(llabs(BigIntWord(i_)))};
    } else {
      return ImplType{BigIntWord(i_) >= 0 ? BigIntWord(i_) : -BigIntWord(i_)};
    }
  }

  double ToDoubleWithShift(int shift) const {
    return std::ldexp(i_, shift);
  }

 protected:
  uint64_t i_;

  constexpr BigUIntWordBase(uint32_t lo, uint32_t hi) : i_(static_cast<uint64_t>(hi) << 32 | lo) { }

  constexpr uint32_t hi_uint32() const {
    return i_ >> 32;
  }

  // Allow BigUIntWordGCC to not only access i_ in its instance (as allowed
  // by protected) but in other instances too.
  template <typename Descendent2>
  friend class BigUIntWordGCC;
};


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
#    define HAS_GNUC_VERSION false
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

// Overload some functions in BigUIntWordBase to use the builtin overflow functions provided by GCC and clang.
template <typename Descendent = void>
class BigUIntWordGCC
  : public BigUIntWordBase<GetImplType<Descendent, BigUIntWordGCC<> >> {
 protected:
  using Parent = BigUIntWordBase<GetImplType<Descendent, BigUIntWordGCC<> >>;
  using typename Parent::ImplType;

 public:
  // This is set to true if the class provides any acceleration. The unit tests
  // use it to determine whether the class should be tested.
#if defined(HAS_BUILTIN_ADD_OVERFLOW) || defined(HAS_BUILTIN_SUB_OVERFLOW) || \
    defined(__SIZEOF_INT128__) || defined(HAS_BUILTIN_CLZL)
  static constexpr bool has_overloads = true;
#else
  static constexpr bool has_overloads = false;
#endif

  // Allow a Parent to be implicitly converted to a
  // BigUIntWordGCC.
  constexpr BigUIntWordGCC(const Parent& other)
    : Parent(other) {
  }
  // Also allow all of Parent's constructors.
  using Parent::Parent;

  // Allow all of Parent's assignment operators.
  using Parent::operator=;

#ifdef HAS_BUILTIN_ADD_OVERFLOW
  constexpr ImplType Add(const Parent& other, bool carry_in, bool* carry_out) const {
    uint64_t result = 0;
    *carry_out = __builtin_add_overflow(Parent::i_, other.i_, &result) |
      __builtin_add_overflow(result, carry_in, &result);
    return ImplType(result);
  }

  constexpr ImplType Add(const Parent& other, bool* carry_out) const {
    uint64_t result = 0;
    *carry_out = __builtin_add_overflow(Parent::i_, other.i_, &result);
    return ImplType(result);
  }

  constexpr ImplType Add(bool carry_in, bool* carry_out) const {
    uint64_t result = 0;
    *carry_out = __builtin_add_overflow(Parent::i_, carry_in, &result);
    return ImplType(result);
  }
#endif
  // Allow Parent's other overloads of Add.
  using Parent::Add;

#ifdef HAS_BUILTIN_SUB_OVERFLOW
  constexpr ImplType Subtract(const Parent& other, bool carry_in, bool* carry_out) const {
    uint64_t result = 0;
    *carry_out = __builtin_sub_overflow(Parent::i_, other.i_, &result) |
      __builtin_sub_overflow(result, carry_in, &result);
    return ImplType(result);
  }

  constexpr ImplType Subtract(const Parent& other, bool* carry_out) const {
    uint64_t result = 0;
    *carry_out = __builtin_sub_overflow(Parent::i_, other.i_, &result);
    return ImplType(result);
  }

  constexpr ImplType Subtract(bool carry_in, bool* carry_out) const {
    uint64_t result = 0;
    *carry_out = __builtin_sub_overflow(Parent::i_, carry_in, &result);
    return ImplType(result);
  }
#endif
  // Allow Parent's other overloads of Subtract.
  using Parent::Subtract;

#ifdef __SIZEOF_INT128__
  constexpr ImplType Multiply(const ImplType& other, ImplType* high_out) const {
    unsigned __int128 full = static_cast<unsigned __int128>(this->i_) *
      static_cast<unsigned __int128>(other.i_);
    *high_out = ImplType(static_cast<uint64_t>(full >> 64));
    return ImplType(static_cast<uint64_t>(full));
  }

  constexpr ImplType MultiplyAdd(const ImplType& other,
                                              const ImplType& add,
                                              bool carry_in,
                                              ImplType* high_out) const {
    unsigned __int128 full = static_cast<unsigned __int128>(this->i_) *
      static_cast<unsigned __int128>(other.i_) + add.i_ + carry_in;
    *high_out = ImplType(static_cast<uint64_t>(full >> 64));
    return ImplType(static_cast<uint64_t>(full));
  }

  constexpr ImplType ShiftRight(const ImplType& high, unsigned shift) const {
    unsigned __int128 full = (static_cast<unsigned __int128>(high.i_) << 64) | this->i_;
    return ImplType(static_cast<uint64_t>(full >> shift));
  }
#endif

#if defined(HAS_BUILTIN_CLZL)
  template <typename X=unsigned>
  constexpr
  typename std::enable_if<sizeof(Parent::i_) == sizeof(unsigned long), X>::type
  GetHighestSetBit() const {
    return this->i_ == 0 ? 0 : Parent::bits_per_word - __builtin_clz(this->i_);
  }
  using Parent::GetHighestSetBit;
#endif
};

using BigUIntWord = BigUIntWordGCC<>;

}  // walnut

#endif // WALNUT_BIG_UINT_WORD_H__
