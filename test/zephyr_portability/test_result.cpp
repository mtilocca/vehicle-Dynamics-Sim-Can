// test_result.cpp — tests for hdv::Result<T> and hdv::Status (Result<void>).
#include "zephyr/src/utils/result.hpp"
#include <gtest/gtest.h>

using namespace hdv;

// ── Result<int> ───────────────────────────────────────────────────────────────

TEST(Result, OkHoldsValue)
{
    auto r = Result<int>::ok(42);
    EXPECT_TRUE(static_cast<bool>(r));
    EXPECT_EQ(r.value(), 42);
}

TEST(Result, ErrHoldsError)
{
    auto r = Result<int>::err(Error::Timeout);
    EXPECT_FALSE(static_cast<bool>(r));
    EXPECT_EQ(r.error(), Error::Timeout);
}

TEST(Result, DefaultErrIsUnknown)
{
    auto r = Result<int>::err(Error::Unknown);
    EXPECT_EQ(r.error(), Error::Unknown);
}

TEST(Result, AllErrorValues)
{
    EXPECT_EQ(static_cast<uint8_t>(Error::Ok),               0u);
    EXPECT_EQ(static_cast<uint8_t>(Error::Timeout),          1u);
    EXPECT_EQ(static_cast<uint8_t>(Error::InvalidArg),       2u);
    EXPECT_EQ(static_cast<uint8_t>(Error::NoDevice),         3u);
    EXPECT_EQ(static_cast<uint8_t>(Error::PermissionDenied), 4u);
    EXPECT_EQ(static_cast<uint8_t>(Error::BufferFull),       5u);
    EXPECT_EQ(static_cast<uint8_t>(Error::NotConnected),     6u);
    EXPECT_EQ(static_cast<uint8_t>(Error::ParseError),       7u);
    EXPECT_EQ(static_cast<uint8_t>(Error::AlreadyExists),    8u);
    EXPECT_EQ(static_cast<uint8_t>(Error::Unknown),       0xFFu);
}

TEST(Result, BoolOperatorDistinguishesOkErr)
{
    EXPECT_TRUE( static_cast<bool>(Result<int>::ok(0)));
    EXPECT_FALSE(static_cast<bool>(Result<int>::err(Error::ParseError)));
}

TEST(Result, ZeroValueOk)
{
    auto r = Result<int>::ok(0);
    EXPECT_TRUE(static_cast<bool>(r));
    EXPECT_EQ(r.value(), 0);
}

TEST(Result, NegativeValueOk)
{
    auto r = Result<int>::ok(-99);
    EXPECT_TRUE(static_cast<bool>(r));
    EXPECT_EQ(r.value(), -99);
}

// ── Result<void> / Status ─────────────────────────────────────────────────────

TEST(Status, OkIsTrue)
{
    Status s = Status::ok();
    EXPECT_TRUE(static_cast<bool>(s));
}

TEST(Status, ErrIsFalse)
{
    Status s = Status::err(Error::NotConnected);
    EXPECT_FALSE(static_cast<bool>(s));
    EXPECT_EQ(s.error(), Error::NotConnected);
}

TEST(Status, AliasMatchesResultVoid)
{
    static_assert(std::is_same_v<Status, Result<void>>,
                  "Status must be an alias for Result<void>");
}
