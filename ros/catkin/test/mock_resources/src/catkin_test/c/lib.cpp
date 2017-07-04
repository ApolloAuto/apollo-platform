#include <iostream>

#include <a/foo.hpp>
namespace c {
  void foo() { 
    a::foo();
    std::cout << __PRETTY_FUNCTION__ << "\n";
  }
}
