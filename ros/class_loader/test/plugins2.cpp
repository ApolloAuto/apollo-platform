#include "base.h"
#include <iostream>
#include <class_loader/class_loader.h>

class Robot : public Base
{
  public:
    virtual void saySomething(){std::cout << "Beep boop" << std::endl;}
};

class Alien : public Base
{
  public:
    virtual void saySomething(){std::cout << "Znornoff!!!" << std::endl;}
};

class Monster : public Base
{
  public:
    virtual void saySomething(){std::cout << "BEAAAHHHH" << std::endl;}
};

class Zombie : public Base
{
  public:
    virtual void saySomething(){std::cout << "Brains!!!" << std::endl;}
};


CLASS_LOADER_REGISTER_CLASS(Robot, Base);
CLASS_LOADER_REGISTER_CLASS(Alien, Base);
CLASS_LOADER_REGISTER_CLASS(Monster, Base);
CLASS_LOADER_REGISTER_CLASS(Zombie, Base);

