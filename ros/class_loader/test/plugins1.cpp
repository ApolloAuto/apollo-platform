#include "base.h"
#include <iostream>
#include <class_loader/class_loader.h>

class Dog : public Base
{
  public:
    virtual void saySomething(){std::cout << "Bark" << std::endl;}
};

class Cat : public Base
{
  public:
    virtual void saySomething(){std::cout << "Meow" << std::endl;}
};

class Duck : public Base
{
  public:
    virtual void saySomething(){std::cout << "Quack" << std::endl;}
};

class Cow : public Base
{
  public:
    virtual void saySomething(){std::cout << "Moooo" << std::endl;}
};

class Sheep : public Base
{
  public:
    virtual void saySomething(){std::cout << "Baaah" << std::endl;}
};

CLASS_LOADER_REGISTER_CLASS(Dog, Base);
CLASS_LOADER_REGISTER_CLASS(Cat, Base);
CLASS_LOADER_REGISTER_CLASS(Duck, Base);
CLASS_LOADER_REGISTER_CLASS(Cow, Base);
CLASS_LOADER_REGISTER_CLASS(Sheep, Base);
