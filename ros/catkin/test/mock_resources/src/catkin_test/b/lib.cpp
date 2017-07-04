#include <iostream>

#include <a/foo.hpp>

namespace b {
  void foo() { 
    a::foo();
    std::cout << __PRETTY_FUNCTION__ << "\n";
  }
}
