#include <iostream>

#include <b/foo.hpp>
#include <c/foo.hpp>

namespace d {
  void foo() { 
    b::foo();
    c::foo();
    std::cout << __PRETTY_FUNCTION__ << "\n";
  }
}
