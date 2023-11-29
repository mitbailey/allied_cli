#pragma once
#include <stdlib.h>
#include <stdint.h>
#include <string>

class StringHasher
{
private:
    uint8_t state[0x100];

public:
    StringHasher();

    uint32_t get_hash(std::string str);
};