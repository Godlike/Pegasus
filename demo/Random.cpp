/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#include "demo/Random.hpp"
#include <ctime>

using namespace pegasus;

Random::Random()
{
    Seed(0);
}

Random::Random(unsigned seed)
{
    Random::Seed(seed);
}

void Random::Seed(unsigned s)
{
    if (s == 0)
    {
        s = static_cast<unsigned>(clock());
    }

    // Fill the buffer with some basic random numbers
    for (unsigned i = 0; i < 17; i++)
    {
        // Simple linear congruential generator
        s = s * 2891336453 + 1;
        m_buffer[i] = s;
    }

    // Initialize pointers into the buffer
    m_p1 = 0;
    m_p2 = 10;
}

unsigned Random::Rotl(unsigned n, unsigned r)
{
    return (n << r) | (n >> (32 - r));
}

unsigned Random::Rotr(unsigned n, unsigned r)
{
    return (n >> r) | (n << (32 - r));
}

unsigned Random::RandomBits()
{
    // Rotate the buffer and store it back to itself
    unsigned result = m_buffer[m_p1] = Rotl(m_buffer[m_p2], 13) + Rotl(m_buffer[m_p1], 9);

    // Rotate pointers
    if (--m_p1 < 0)
        m_p1 = 16;
    if (--m_p2 < 0)
        m_p2 = 16;

    // Return result
    return result;
}

#ifdef SINGLE_PRECISION
double Random::randomDouble()
{
// Get the random number
    unsigned bits = randomBits();

// Set up a reinterpret structure for manipulation
    union {
        double value;
        unsigned word;
    } convert;

// Now assign the bits to the word. This works by fixing the ieee
// sign and exponent bits (so that the size of the result is 1-2)
// and using the bits to create the fraction part of the float.
    convert.word = (bits >> 9) | 0x3f800000;

// And return the value
    return convert.value - 1.0f;
}
#else
double Random::RandomDouble()
{
    // Get the random number
    auto bits = RandomBits();

    // Set up a reinterpret structure for manipulation
    union
    {
        double value;
        unsigned words[2];
    } convert = {};

    // Now assign the bits to the words. This works by fixing the ieee
    // sign and exponent bits (so that the size of the result is 1-2)
    // and using the bits to create the fraction part of the float. Note
    // that bits are used more than once in this process.
    convert.words[0] = bits << 20; // Fill in the top 16 bits
    convert.words[1] = (bits >> 12) | 0x3FF00000; // And the bottom 20

    // And return the value
    return convert.value - static_cast<double>(1);
}
#endif

double Random::RandomDouble(double min, double max)
{
    return RandomDouble() * (max - min) + min;
}

double Random::RandomDouble(double scale)
{
    return RandomDouble() * scale;
}

unsigned Random::RandomInt(unsigned max)
{
    return RandomBits() % max;
}

double Random::RandomBinomial(double scale)
{
    return (RandomDouble() - RandomDouble()) * scale;
}

Vector3 Random::RandomVector(double scale)
{
    return Vector3(RandomBinomial(scale), RandomBinomial(scale),
                   RandomBinomial(scale));
}

Vector3 Random::RandomXzVector(double scale)
{
    return Vector3(RandomBinomial(scale), 0, RandomBinomial(scale));
}

Vector3 Random::RandomVector(const Vector3& scale)
{
    return Vector3(RandomBinomial(scale.x), RandomBinomial(scale.y),
                   RandomBinomial(scale.z));
}

Vector3 Random::RandomVector(const Vector3& min, const Vector3& max)
{
    return Vector3(RandomDouble(min.x, max.x), RandomDouble(min.y, max.y),
                   RandomDouble(min.z, max.z));
}
