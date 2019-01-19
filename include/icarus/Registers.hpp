#pragma once

#include <cstdint>
#include <cstddef>
#include <type_traits>

namespace icarus::registers
{
    enum class Endian
    {
        little,
        big
    };

    template<typename T, Endian ENDIAN>
    struct Scalar;

    template<typename T>
    struct Scalar<T, Endian::little>
    {
        T value() const
        {
            return mData;
        }
    private:
        T mData = 0;
    };

    template<typename T, size_t I>
    struct SwapBytes
    {
        constexpr T operator()(uint8_t const* data)
        {
            return static_cast<T>(data[I - 1]) | SwapBytes<T, I - 1>()(data) << 8;
        }
    };

    template<typename T>
    struct SwapBytes<T, 0>
    {
        constexpr T operator()(uint8_t const*)
        {
            return 0;
        }
    };

    template<typename T>
    struct Scalar<T, Endian::big>
    {
        T value() const
        {
            auto ret = partial<sizeof(T)>();
            return reinterpret_cast<T&>(ret);
        }
    private:
        template<size_t i>
        constexpr std::make_unsigned_t<T> partial() const
        {
            return SwapBytes<std::make_unsigned_t<T>, sizeof(T)>()(mData);
        }
        uint8_t mData[sizeof(T)] = {};
    };

    template<typename T, size_t SIZE, Endian ENDIAN = Endian::little>
    struct Vector
    {
        T x() const
        {
            return mComponents[0].value();
        }

        T y() const
        {
            static_assert(SIZE >= 2, "Vector has to have SIZE >= 2");
            return mComponents[1].value();
        }

        T z() const
        {
            static_assert(SIZE >= 3, "Vector has to have SIZE >= 3");
            return mComponents[2].value();
        }
    private:
        using Component = Scalar<T, ENDIAN>;
        Component mComponents[SIZE];
    };
}
