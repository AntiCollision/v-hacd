#pragma once
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <limits.h>

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <future>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace VHACD {

    class Googol
    {
#define VHACD_GOOGOL_SIZE 4
    public:
        Googol() = default;
        Googol(double value);

        operator double() const;
        Googol operator+(const Googol& A) const;
        Googol operator-(const Googol& A) const;
        Googol operator*(const Googol& A) const;
        Googol operator/ (const Googol& A) const;

        Googol& operator+= (const Googol& A);
        Googol& operator-= (const Googol& A);

        bool operator>(const Googol& A) const;
        bool operator>=(const Googol& A) const;
        bool operator<(const Googol& A) const;
        bool operator<=(const Googol& A) const;
        bool operator==(const Googol& A) const;
        bool operator!=(const Googol& A) const;

        Googol Abs() const;
        Googol Floor() const;
        Googol InvSqrt() const;
        Googol Sqrt() const;

        void ToString(char* const string) const;

    private:
        void NegateMantissa(std::array<uint64_t, VHACD_GOOGOL_SIZE>& mantissa) const;
        void CopySignedMantissa(std::array<uint64_t, VHACD_GOOGOL_SIZE>& mantissa) const;
        int NormalizeMantissa(std::array<uint64_t, VHACD_GOOGOL_SIZE>& mantissa) const;
        void ShiftRightMantissa(std::array<uint64_t, VHACD_GOOGOL_SIZE>& mantissa,
            int bits) const;
        uint64_t CheckCarrier(uint64_t a, uint64_t b) const;

        int LeadingZeros(uint64_t a) const;
        void ExtendedMultiply(uint64_t a,
            uint64_t b,
            uint64_t& high,
            uint64_t& low) const;
        void ScaleMantissa(uint64_t* out,
            uint64_t scale) const;

        int m_sign{ 0 };
        int m_exponent{ 0 };
        std::array<uint64_t, VHACD_GOOGOL_SIZE> m_mantissa{ 0 };

    public:
        static Googol m_zero;
        static Googol m_one;
        static Googol m_two;
        static Googol m_three;
        static Googol m_half;
    };

    Googol Googol::m_zero(double(0.0));
    Googol Googol::m_one(double(1.0));
    Googol Googol::m_two(double(2.0));
    Googol Googol::m_three(double(3.0));
    Googol Googol::m_half(double(0.5));
    
    Googol::Googol(double value)
    {
        int exp;
        double mantissa = fabs(frexp(value, &exp));

        m_exponent = exp;
        m_sign = (value >= 0) ? 0 : 1;

        m_mantissa[0] = uint64_t(double(uint64_t(1) << 62) * mantissa);
    }

    Googol::operator double() const
    {
        double mantissa = (double(1.0) / double(uint64_t(1) << 62)) * double(m_mantissa[0]);
        mantissa = ldexp(mantissa, m_exponent) * (m_sign ? double(-1.0) : double(1.0));
        return mantissa;
    }

    Googol Googol::operator+(const Googol& A) const
    {
        Googol tmp;
        if (m_mantissa[0] && A.m_mantissa[0])
        {
            std::array<uint64_t, VHACD_GOOGOL_SIZE> mantissa0;
            std::array<uint64_t, VHACD_GOOGOL_SIZE> mantissa1;
            std::array<uint64_t, VHACD_GOOGOL_SIZE> mantissa;

            CopySignedMantissa(mantissa0);
            A.CopySignedMantissa(mantissa1);

            int exponentDiff = m_exponent - A.m_exponent;
            int exponent = m_exponent;
            if (exponentDiff > 0)
            {
                ShiftRightMantissa(mantissa1,
                    exponentDiff);
            }
            else if (exponentDiff < 0)
            {
                exponent = A.m_exponent;
                ShiftRightMantissa(mantissa0,
                    -exponentDiff);
            }

            uint64_t carrier = 0;
            for (int i = VHACD_GOOGOL_SIZE - 1; i >= 0; i--)
            {
                uint64_t m0 = mantissa0[i];
                uint64_t m1 = mantissa1[i];
                mantissa[i] = m0 + m1 + carrier;
                carrier = CheckCarrier(m0, m1) | CheckCarrier(m0 + m1, carrier);
            }

            int sign = 0;
            if (int64_t(mantissa[0]) < 0)
            {
                sign = 1;
                NegateMantissa(mantissa);
            }

            int bits = NormalizeMantissa(mantissa);
            if (bits <= (-64 * VHACD_GOOGOL_SIZE))
            {
                tmp.m_sign = 0;
                tmp.m_exponent = 0;
            }
            else
            {
                tmp.m_sign = sign;
                tmp.m_exponent = int(exponent + bits);
            }

            tmp.m_mantissa = mantissa;
        }
        else if (A.m_mantissa[0])
        {
            tmp = A;
        }
        else
        {
            tmp = *this;
        }

        return tmp;
    }

    Googol Googol::operator-(const Googol& A) const
    {
        Googol tmp(A);
        tmp.m_sign = !tmp.m_sign;
        return *this + tmp;
    }

    Googol Googol::operator*(const Googol& A) const
    {
        if (m_mantissa[0] && A.m_mantissa[0])
        {
            std::array<uint64_t, VHACD_GOOGOL_SIZE * 2> mantissaAcc{ 0 };
            for (int i = VHACD_GOOGOL_SIZE - 1; i >= 0; i--)
            {
                uint64_t a = m_mantissa[i];
                if (a)
                {
                    uint64_t mantissaScale[2 * VHACD_GOOGOL_SIZE] = { 0 };
                    A.ScaleMantissa(&mantissaScale[i], a);

                    uint64_t carrier = 0;
                    for (int j = 0; j < 2 * VHACD_GOOGOL_SIZE; j++)
                    {
                        const int k = 2 * VHACD_GOOGOL_SIZE - 1 - j;
                        uint64_t m0 = mantissaAcc[k];
                        uint64_t m1 = mantissaScale[k];
                        mantissaAcc[k] = m0 + m1 + carrier;
                        carrier = CheckCarrier(m0, m1) | CheckCarrier(m0 + m1, carrier);
                    }
                }
            }

            uint64_t carrier = 0;
            int bits = LeadingZeros(mantissaAcc[0]) - 2;
            for (int i = 0; i < 2 * VHACD_GOOGOL_SIZE; i++)
            {
                const int k = 2 * VHACD_GOOGOL_SIZE - 1 - i;
                uint64_t a = mantissaAcc[k];
                mantissaAcc[k] = (a << uint64_t(bits)) | carrier;
                carrier = a >> uint64_t(64 - bits);
            }

            int exp = m_exponent + A.m_exponent - (bits - 2);

            Googol tmp;
            tmp.m_sign = m_sign ^ A.m_sign;
            tmp.m_exponent = exp;
            for (std::size_t i = 0; i < tmp.m_mantissa.size(); ++i)
            {
                tmp.m_mantissa[i] = mantissaAcc[i];
            }

            return tmp;
        }
        return Googol(double(0.0));
    }

    Googol Googol::operator/(const Googol& A) const
    {
        Googol tmp(double(1.0) / A);
        tmp = tmp * (m_two - A * tmp);
        tmp = tmp * (m_two - A * tmp);
        bool test = false;
        int passes = 0;
        do
        {
            passes++;
            Googol tmp0(tmp);
            tmp = tmp * (m_two - A * tmp);
            test = tmp0 == tmp;
        } while (test && (passes < (2 * VHACD_GOOGOL_SIZE)));
        return (*this) * tmp;
    }

    Googol& Googol::operator+=(const Googol& A)
    {
        *this = *this + A;
        return *this;
    }

    Googol& Googol::operator-=(const Googol& A)
    {
        *this = *this - A;
        return *this;
    }

    bool Googol::operator>(const Googol& A) const
    {
        Googol tmp(*this - A);
        return double(tmp) > double(0.0);
    }

    bool Googol::operator>=(const Googol& A) const
    {
        Googol tmp(*this - A);
        return double(tmp) >= double(0.0);
    }

    bool Googol::operator<(const Googol& A) const
    {
        Googol tmp(*this - A);
        return double(tmp) < double(0.0);
    }

    bool Googol::operator<=(const Googol& A) const
    {
        Googol tmp(*this - A);
        return double(tmp) <= double(0.0);
    }

    bool Googol::operator==(const Googol& A) const
    {
        return    m_sign == A.m_sign
            && m_exponent == A.m_exponent
            && m_mantissa == A.m_mantissa;
    }

    bool Googol::operator!=(const Googol& A) const
    {
        return !(*this == A);
    }

    Googol Googol::Abs() const
    {
        Googol tmp(*this);
        tmp.m_sign = 0;
        return tmp;
    }

    Googol Googol::Floor() const
    {
        if (m_exponent < 1)
        {
            return Googol(double(0.0));
        }
        int bits = m_exponent + 2;
        int start = 0;
        while (bits >= 64)
        {
            bits -= 64;
            start++;
        }

        Googol tmp(*this);
        for (int i = VHACD_GOOGOL_SIZE - 1; i > start; i--)
        {
            tmp.m_mantissa[i] = 0;
        }
        // some compilers do no like this and I do not know why is that
        //uint64_t mask = (-1LL) << (64 - bits);
        uint64_t mask(~0ULL);
        mask <<= (64 - bits);
        tmp.m_mantissa[start] &= mask;
        return tmp;
    }

    Googol Googol::InvSqrt() const
    {
        const Googol& me = *this;
        Googol x(double(1.0) / sqrt(me));

        int test = 0;
        int passes = 0;
        do
        {
            passes++;
            Googol tmp(x);
            x = m_half * x * (m_three - me * x * x);
            test = (x != tmp);
        } while (test && (passes < (2 * VHACD_GOOGOL_SIZE)));
        return x;
    }

    Googol Googol::Sqrt() const
    {
        return *this * InvSqrt();
    }

    void Googol::ToString(char* const string) const
    {
        Googol tmp(*this);
        Googol base(double(10.0));
        while (double(tmp) > double(1.0))
        {
            tmp = tmp / base;
        }

        int index = 0;
        while (tmp.m_mantissa[0])
        {
            tmp = tmp * base;
            Googol digit(tmp.Floor());
            tmp -= digit;
            double val = digit;
            string[index] = char(val) + '0';
            index++;
        }
        string[index] = 0;
    }

    void Googol::NegateMantissa(std::array<uint64_t, VHACD_GOOGOL_SIZE>& mantissa) const
    {
        uint64_t carrier = 1;
        for (size_t i = mantissa.size() - 1; i < mantissa.size(); i--)
        {
            uint64_t a = ~mantissa[i] + carrier;
            if (a)
            {
                carrier = 0;
            }
            mantissa[i] = a;
        }
    }

    void Googol::CopySignedMantissa(std::array<uint64_t, VHACD_GOOGOL_SIZE>& mantissa) const
    {
        mantissa = m_mantissa;
        if (m_sign)
        {
            NegateMantissa(mantissa);
        }
    }

    int Googol::NormalizeMantissa(std::array<uint64_t, VHACD_GOOGOL_SIZE>& mantissa) const
    {
        int bits = 0;
        if (int64_t(mantissa[0] * 2) < 0)
        {
            bits = 1;
            ShiftRightMantissa(mantissa, 1);
        }
        else
        {
            while (!mantissa[0] && bits > (-64 * VHACD_GOOGOL_SIZE))
            {
                bits -= 64;
                for (int i = 1; i < VHACD_GOOGOL_SIZE; i++) {
                    mantissa[i - 1] = mantissa[i];
                }
                mantissa[VHACD_GOOGOL_SIZE - 1] = 0;
            }

            if (bits > (-64 * VHACD_GOOGOL_SIZE))
            {
                int n = LeadingZeros(mantissa[0]) - 2;
                if (n > 0)
                {
                    uint64_t carrier = 0;
                    for (int i = VHACD_GOOGOL_SIZE - 1; i >= 0; i--)
                    {
                        uint64_t a = mantissa[i];
                        mantissa[i] = (a << n) | carrier;
                        carrier = a >> (64 - n);
                    }
                    bits -= n;
                }
                else if (n < 0)
                {
                    // this is very rare but it does happens, whee the leading zeros of the mantissa is an exact multiple of 64
                    uint64_t carrier = 0;
                    int shift = -n;
                    for (int i = 0; i < VHACD_GOOGOL_SIZE; i++)
                    {
                        uint64_t a = mantissa[i];
                        mantissa[i] = (a >> shift) | carrier;
                        carrier = a << (64 - shift);
                    }
                    bits -= n;
                }
            }
        }
        return bits;
    }

    void Googol::ShiftRightMantissa(std::array<uint64_t, VHACD_GOOGOL_SIZE>& mantissa,
        int bits) const
    {
        uint64_t carrier = 0;
        if (int64_t(mantissa[0]) < int64_t(0))
        {
            carrier = uint64_t(-1);
        }

        while (bits >= 64)
        {
            for (int i = VHACD_GOOGOL_SIZE - 2; i >= 0; i--)
            {
                mantissa[i + 1] = mantissa[i];
            }
            mantissa[0] = carrier;
            bits -= 64;
        }

        if (bits > 0)
        {
            carrier <<= (64 - bits);
            for (int i = 0; i < VHACD_GOOGOL_SIZE; i++)
            {
                uint64_t a = mantissa[i];
                mantissa[i] = (a >> bits) | carrier;
                carrier = a << (64 - bits);
            }
        }
    }

    uint64_t Googol::CheckCarrier(uint64_t a, uint64_t b) const
    {
        return ((uint64_t(-1) - b) < a) ? uint64_t(1) : 0;
    }

    int Googol::LeadingZeros(uint64_t a) const
    {
#define VHACD_COUNTBIT(mask, add)	\
    do {								\
        uint64_t test = a & mask;		\
        n += test ? 0 : add;			\
        a = test ? test : (a & ~mask);	\
    } while (false)

        int n = 0;
        VHACD_COUNTBIT(0xffffffff00000000LL, 32);
        VHACD_COUNTBIT(0xffff0000ffff0000LL, 16);
        VHACD_COUNTBIT(0xff00ff00ff00ff00LL, 8);
        VHACD_COUNTBIT(0xf0f0f0f0f0f0f0f0LL, 4);
        VHACD_COUNTBIT(0xccccccccccccccccLL, 2);
        VHACD_COUNTBIT(0xaaaaaaaaaaaaaaaaLL, 1);

        return n;
    }

    void Googol::ExtendedMultiply(uint64_t a,
        uint64_t b,
        uint64_t& high,
        uint64_t& low) const
    {
        uint64_t bLow = b & 0xffffffff;
        uint64_t bHigh = b >> 32;
        uint64_t aLow = a & 0xffffffff;
        uint64_t aHigh = a >> 32;

        uint64_t l = bLow * aLow;

        uint64_t c1 = bHigh * aLow;
        uint64_t c2 = bLow * aHigh;
        uint64_t m = c1 + c2;
        uint64_t carrier = CheckCarrier(c1, c2) << 32;

        uint64_t h = bHigh * aHigh + carrier;

        uint64_t ml = m << 32;
        uint64_t ll = l + ml;
        uint64_t mh = (m >> 32) + CheckCarrier(l, ml);
        uint64_t hh = h + mh;

        low = ll;
        high = hh;
    }

    void Googol::ScaleMantissa(uint64_t* dst,
        uint64_t scale) const
    {
        uint64_t carrier = 0;
        for (int i = VHACD_GOOGOL_SIZE - 1; i >= 0; i--)
        {
            if (m_mantissa[i])
            {
                uint64_t low;
                uint64_t high;
                ExtendedMultiply(scale,
                    m_mantissa[i],
                    high,
                    low);
                uint64_t acc = low + carrier;
                carrier = CheckCarrier(low,
                    carrier);
                carrier += high;
                dst[i + 1] = acc;
            }
            else
            {
                dst[i + 1] = carrier;
                carrier = 0;
            }

        }
        dst[0] = carrier;
    }
}

