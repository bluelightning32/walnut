#include "walnut/big_int.h"

#include <cmath>
#include <vector>

namespace walnut {

BigInt BigInt::max_value(size_t set_bits) {
  BigInt result;
  result.words_.resize((set_bits + bits_per_word) / bits_per_word);
  for (size_t i = 0; i < result.words_.size() - 1; ++i) {
    result.words_[i] = BigUIntWord::max_value();
  }
  BigUIntWord last_word;
  const size_t set_last_word_bits =
    set_bits - (result.words_.size() - 1) * bits_per_word;
  for (size_t i = 0; i < set_last_word_bits; ++i) {
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
  const size_t clear_last_word_bits =
    clear_bits - (result.words_.size() - 1) * bits_per_word;
  BigUIntWord last_word = BigUIntWord{-1};
  for (size_t i = 0; i < clear_last_word_bits; ++i) {
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
    return std::ldexp(BigIntWord{words_[used - 1]},
                      bits_per_word * (used - 1)) +
           words_[used - 2].ToDoubleWithShift(bits_per_word * (used - 2));
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
    *digits_pos-- = digit.word(0).SignedAbs().low_uint32() + '0';
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
