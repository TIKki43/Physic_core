#pragma once
#include <iostream>
#include <stdexcept>

namespace Utils
{
[[noreturn]] void inline panic (const std::string &message)
{
  std::cerr << message << std::endl;
  std::abort ();
};
};
