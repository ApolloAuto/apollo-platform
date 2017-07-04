#ifndef PLUGINLIB_TEST_PLUGINS_H_
#define PLUGINLIB_TEST_PLUGINS_H_
#include "test_base.h"
#include <cmath>

namespace test_plugins
{
class Bar : public test_base::Fubar
{
public:
  Bar(){}

  void initialize(double foo)
  {
    foo_ = foo;
  }

  double result()
  {
    return 0.5 * foo_ * getBar();
  }

  double getBar()
  {
    return sqrt((foo_ * foo_) - ((foo_ / 2) * (foo_ / 2)));
  }

private:
  double foo_;
};

class Foo : public test_base::Fubar
{
public:
  Foo(){}

  void initialize(double foo)
  {
    foo_ = foo;
  }

  double result()
  {
    return foo_ * foo_;
  }

private:
  double foo_;

};
};
#endif
