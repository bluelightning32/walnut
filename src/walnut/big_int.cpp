#include "walnut/big_int.h"

#include <cmath>
#include <limits>
#include <vector>

namespace walnut {

BigInt BigInt::max_value(size_t set_bits) {
  BigInt result;
  result.words_.resize((set_bits + bits_per_word) / bits_per_word);
  for (size_t i = 0; i < result.words_.size() - 1; ++i) {
    result.words_[i] = BigUIntWord::max_value();
  }
  BigUIntWord last_word;
  const unsigned set_last_word_bits = static_cast<unsigned>(
          set_bits - (result.words_.size() - 1) * bits_per_word);
  for (unsigned i = 0; i < set_last_word_bits; ++i) {
    last_word |= BigUIntWord{1} << i;
  }
  result.words_[result.words_.size() - 1] = last_word;
  result.Trim();
  return result;
}

BigInt BigInt::min_value(size_t clear_bits) {
  BigInt result;
  result.words_.resize((clear_bits + bits_per_word) / bits_per_word);
  for (size_t i = 0; i < result.words_.size() - 1; ++i) {
    result.words_[i] = BigUIntWord{0};
  }
  const unsigned clear_last_word_bits = static_cast<unsigned>(
    clear_bits - (result.words_.size() - 1) * bits_per_word);
  BigUIntWord last_word = BigUIntWord{-1};
  for (unsigned i = 0; i < clear_last_word_bits; ++i) {
    last_word &= ~(BigUIntWord{1} << i);
  }
  result.words_[result.words_.size() - 1] = last_word;
  result.Trim();
  return result;
}

BigInt::operator double() const {
  static_assert(sizeof(BigIntWord) >= sizeof(double),
                "The cast function assumes that at most 2 words need to be "
                "inspected to convert to a double.");
  if (used_words() == 1) {
    return (double)BigIntWord{words_[0]};
  } else {
    size_t used = used_words();
    size_t first_shift = bits_per_word * (used - 1);
    if (first_shift > std::numeric_limits<int>::max()) {
      return std::numeric_limits<double>::infinity() *
             BigIntWord{words_[used - 1]};
    }
    return std::ldexp(BigIntWord{words_[used - 1]},
                      static_cast<int>(first_shift)) +
           words_[used - 2].ToDoubleWithShift(
               static_cast<int>(bits_per_word * (used - 2)));
  }
}

BigInt::operator long double() const {
  static_assert(2*sizeof(BigIntWord) >= sizeof(long double),
                "The cast function assumes that at most 3 words need to be "
                "inspected to convert to a double.");
  if (used_words() == 1) {
    return (long double)BigIntWord{words_[0]};
  } else if (used_words() == 2) {
    return std::ldexp((long double)BigIntWord{words_[1]}, bits_per_word) +
           words_[0].ToLongDoubleWithShift(0);
  } else {
    size_t used = used_words();
    size_t first_shift = bits_per_word * (used - 1);
    if (first_shift > std::numeric_limits<int>::max()) {
      return std::numeric_limits<long double>::infinity() *
             BigIntWord{words_[used - 1]};
    }
    return std::ldexp((long double)BigIntWord{words_[used - 1]},
                      static_cast<int>(bits_per_word * (used - 1))) +
           words_[used - 2].ToLongDoubleWithShift(
               static_cast<int>(bits_per_word * (used - 2))) +
           words_[used - 3].ToLongDoubleWithShift(
               static_cast<int>(bits_per_word * (used - 3)));
  }
}

BigInt BigInt::MultiplySlow(const BigInt& other) const {
  if (used_words() < other.used_words()) {
    return other.MultiplySlow(*this);
  }
  BigInt result;
  result.words_.resize(used_words() + other.used_words());
  size_t k = 0;
  {
    BigUIntWord add;
    for (size_t i = 0; i < this->used_words(); ++i, ++k) {
      result.words_[k] = other.words_[0].MultiplyAdd(words_[i], add, /*carry_in=*/false, &add);
    }
    result.words_[k] = add;
  }
  for (size_t j = 1; j < other.used_words(); j++) {
    k = j;
    BigUIntWord add;
    bool carry = false;
    for (size_t i = 0; i < this->used_words(); ++i, ++k) {
      add = add.Add(result.words_[k], carry, &carry);
      result.words_[k] = other.words_[j].MultiplyAdd(words_[i], add, /*carry_in=*/false, &add);
    }
    result.words_[k] = add.Add(carry, &carry);
  }
  k++;
  assert(result.used_words() == k);
  result.SubtractLeftShiftedMasked(*this, other.used_words(),
                                   other.SignExtension());
  result.SubtractLeftShiftedMasked(other, used_words(), SignExtension());
  result.Trim();
  return result;
}

std::ostream& operator<<(std::ostream& out, const BigInt& bigint) {
  BigInt remaining = bigint;
  std::vector<char> digits(
      bigint.used_words() * BigInt::bytes_per_word*3 + 2);
  char* digits_pos = &digits.back();
  *digits_pos-- = '\0';

  BigInt digit;
  do {
    remaining = remaining.DivideRemainder(BigInt(10), &digit);
    *digits_pos-- = static_cast<char>(digit.word(0).SignedAbs().low_uint32())
                    + '0';
  } while (remaining != 0);

  if (digit.GetSign() < 0) {
    *digits_pos-- = '-';
  }
  out << digits_pos + 1;
  return out;
}

BigInt BigInt::DivideRemainderSlow(const BigInt& other,
                                   BigInt* remainder_out) const {
  bool this_signed = false;
  BigUInt this_uint = GetUIntAbs(&this_signed);
  bool other_signed = false;
  BigUInt other_uint = other.GetUIntAbs(&other_signed);

  BigUInt remainder;
  BigUInt quotient = this_uint.DivideRemainder(other_uint, &remainder);
  *remainder_out = remainder;
  if (this_signed) {
    remainder_out->Negate();
  }
  BigInt result{quotient};
  if (this_signed ^ other_signed) {
    result.Negate();
  }
  return result;
}

}  // walnut
