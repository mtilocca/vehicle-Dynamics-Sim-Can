// zephyr/src/utils/result.hpp
// Lightweight Result<T> for Zephyr: no heap, no exceptions, C++17.
//
// Usage:
//   Result<int> r = some_fn();
//   if (!r) { handle(r.error()); return; }
//   int val = r.value();
//
//   Status s = void_fn();          // Result<void>
//   if (!s) { ... }
#pragma once

#include <stdint.h>

namespace hdv {

enum class Error : uint8_t {
    Ok               = 0,
    Timeout          = 1,
    InvalidArg       = 2,
    NoDevice         = 3,
    PermissionDenied = 4,
    BufferFull       = 5,
    NotConnected     = 6,
    ParseError       = 7,
    AlreadyExists    = 8,
    Unknown          = 0xFF,
};

template<typename T>
class Result {
public:
    static Result ok(T v)      { Result r; r.ok_ = true;  r.val_ = v; return r; }
    static Result err(Error e) { Result r; r.ok_ = false; r.err_ = e; return r; }

    explicit operator bool() const { return ok_; }
    T     value() const { return val_; }
    Error error() const { return err_; }

private:
    T     val_{};
    Error err_ = Error::Unknown;
    bool  ok_  = false;
};

template<>
class Result<void> {
public:
    static Result ok()         { Result r; r.ok_ = true;  return r; }
    static Result err(Error e) { Result r; r.ok_ = false; r.err_ = e; return r; }

    explicit operator bool() const { return ok_; }
    Error error() const { return err_; }

private:
    Error err_ = Error::Unknown;
    bool  ok_  = false;
};

using Status = Result<void>;

} // namespace hdv
