#ifndef WALNUT_BIG_INT_WORDS_H__
#define WALNUT_BIG_INT_WORDS_H__

// For std::min
#include <algorithm>
#include <cassert>
#include <cstdlib>

#include "walnut/big_uint_word.h"

namespace walnut {

// Variable length container of BigUIntWords
//
// This container always has at least size 1. The first word is stored inline.
// The remaining words are heap allocated.
class BigIntWords {
 public:
  static constexpr size_t bits_per_word = BigUIntWord::bits_per_word;
  static constexpr size_t bytes_per_word = BigUIntWord::bytes_per_word;

  constexpr BigIntWords() = default;

  constexpr BigIntWords(int size) : extra_size_(size - 1),
                                    extra_(AllocateExtra(extra_size_)) { }

  constexpr BigIntWords(BigIntWords&& other) :
      first_(other.first_),
      extra_size_(other.extra_size_) {
    std::swap(extra_, other.extra_);
  }

  constexpr BigIntWords(const BigIntWords& other) :
      first_(other.first_),
      extra_size_(other.extra_size_),
      extra_(AllocateExtra(extra_size_)) {
    assert(other.extra_size_ == 0 || other.extra_ != nullptr);
    for (size_t i = 0; i < other.extra_size_; ++i) {
      extra_[i] = other.extra_[i];
    }
  }

  constexpr BigIntWords(const BigIntWords& other, size_t used_words) :
      first_(other.first_),
      extra_size_(used_words - 1),
      extra_(AllocateExtra(extra_size_)) {
    for (size_t i = 0; i < extra_size_; ++i) {
      extra_[i] = other.extra_[i];
    }
  }

  constexpr BigIntWords(size_t used_words, size_t copy_words,
                        const BigIntWords& from) :
      first_(from.first_),
      extra_size_(used_words - 1),
      extra_(AllocateExtra(extra_size_)) {
    size_t i = 0;
    for (; i < copy_words - 1; ++i) {
      extra_[i] = from.extra_[i];
    }
    for (; i < extra_size_; ++i) {
      extra_[i] = 0;
    }
  }

  ~BigIntWords() {
    FreeExtra(extra_);
  }

  constexpr size_t size() const {
    return extra_size_ + 1;
  }

  constexpr size_t extra_size() const {
    return extra_size_;
  }

  constexpr const BigUIntWord& first() const {
    return first_;
  }

  constexpr BigUIntWord& first() {
    return first_;
  }

  constexpr const BigUIntWord& operator[](size_t i) const {
    assert(i < size());
    if (i == 0) {
      return first_;
    } else {
      return extra_[i - 1];
    }
  }

  constexpr BigUIntWord& operator[](size_t i) {
    assert(i < size());
    if (i == 0) {
      return first_;
    } else {
      return extra_[i - 1];
    }
  }

  constexpr void Assign(
      const BigIntWords& other, size_t used) {
    resize_without_copy(used);
    first_ = other.first_;
    for (size_t i = 0; i < used - 1; ++i) {
      extra_[i] = other.extra_[i];
    }
  }

  constexpr void push_back(BigUIntWord word) {
    const size_t used = size();
    resize(size() + 1);
    extra_[used - 1] = word;
  }

  constexpr size_t extra_capacity() const {
    if (extra_ != nullptr) {
      return GetExtraFromFirst(extra_)->capacity;
    } else {
      return 0;
    }
  }

  // Sets the container size and leaves the contents undefined.
  constexpr void resize_without_copy(size_t used_words) {
    assert(used_words >= 1);
    const size_t extra_size = used_words - 1;
    if (extra_size > extra_capacity()) {
      FreeExtra(extra_);
      extra_ = AllocateExtra(extra_size);
    }
    extra_size_ = extra_size;
  }

  // If the resize causes the container to grow, the additional entries are
  // undefined.
  constexpr void resize(size_t used_words) {
    assert(used_words >= 1);
    const size_t extra_size = used_words - 1;
    if (extra_size > extra_capacity()) {
      BigUIntWord* new_extra = AllocateExtra(extra_size);
      for (size_t i = 0; i < extra_size_; ++i) {
        new_extra[i] = extra_[i];
      }
      FreeExtra(extra_);
      extra_ = new_extra;
    }
    extra_size_ = extra_size;
  }

  // If the resize causes the container to grow, the additional entries are
  // set to `init`.
  constexpr void resize(size_t used_words, BigUIntWord init) {
    assert(used_words >= 1);
    const size_t extra_size = used_words - 1;
    if (extra_size > extra_capacity()) {
      BigUIntWord* new_extra = AllocateExtra(extra_size);
      for (size_t i = 0; i < extra_size_; ++i) {
        new_extra[i] = extra_[i];
      }
      FreeExtra(extra_);
      extra_ = new_extra;
    }
    while (extra_size_ < extra_size) {
      extra_[extra_size_] = init;
      ++extra_size_;
    }
  }

  constexpr BigIntWords& operator=(const BigIntWords& other) {
    Assign(other, other.size());
    return *this;
  }

  constexpr BigIntWords& operator=(BigIntWords&& other) {
    first_ = other.first_;
    extra_size_ = other.extra_size_;
    BigUIntWord* const extra = extra_;
    extra_ = other.extra_;
    other.extra_ = extra;
    return *this;
  }

 private:
  // This is used when the inline storage is exceeded.
  struct ExtraStorage {
    // Number of items in `words_`.
    size_t capacity;
    // `capacity` length array of words.
    BigUIntWord words_[1];
  };

  constexpr static BigUIntWord* AllocateExtra(size_t capacity) {
    if (capacity == 0) {
      return nullptr;
    }
    const size_t alloc_size = sizeof(ExtraStorage) +
                              (capacity - 1) * sizeof(BigUIntWord);
    ExtraStorage* storage = reinterpret_cast<ExtraStorage*>(std::malloc(
          alloc_size));
    storage->capacity = capacity;
    return &storage->words_[0];
  }

  static ExtraStorage* GetExtraFromFirst(BigUIntWord* extra) {
    return reinterpret_cast<ExtraStorage*>(
        reinterpret_cast<char *>(extra) - offsetof(ExtraStorage, words_[0]));
  }

  constexpr static void FreeExtra(BigUIntWord* extra) {
    if (extra == nullptr) {
      return;
    }
    std::free(GetExtraFromFirst(extra));
  }

  BigUIntWord first_{0};
  // The number of words used in `extra_`.
  size_t extra_size_ = 0;
  // If any extra storage is allocated, this points to ExtraStorage::words_[0].
  // If no extra storage is allocated, then this is nullptr.
  BigUIntWord* extra_ = nullptr;
};

}  // walnut

#endif // WALNUT_BIG_INT_WORDS_H__
