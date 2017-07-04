#ifndef PLUGINLIB_TEST_BASE_H_
#define PLUGINLIB_TEST_BASE_H_

namespace test_base
{
  class Fubar
  {
  public:
    virtual void initialize(double foo) = 0;
    virtual double result() = 0;
    virtual ~Fubar(){}

  protected:
    Fubar(){}
  };
};
#endif
