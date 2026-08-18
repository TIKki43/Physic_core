#pragma once

#include <stdexcept>

namespace Utils
{
    [[noreturn]] void panic(const std::string& message) {
        std::cerr << message << std::endl;
        std::abort();
    };
};
