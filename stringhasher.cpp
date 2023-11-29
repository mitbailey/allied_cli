#include "stringhasher.hpp"

StringHasher::StringHasher()
{
    for (ssize_t i = 0; i < 0x100; i++)
        state[i] = rand() % 0x100;
}

uint32_t StringHasher::get_hash(std::string str)
{
    uint32_t h = 0x1F351F35;
    for (auto &ch : str)
    {
        h = ((h >> 11) | (h << (32 - 11))) + state[(uint8_t)(ch ^ h)];
    }
    h ^= h >> 16;
    return h ^ (h >> 8);
}
