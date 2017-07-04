#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <class_loader/class_loader.h>
#include <class_loader/multi_library_class_loader.h>
#include "base.h"
#include <gtest/gtest.h>

const std::string LIBRARY_1 = "libclass_loader_TestPlugins1.so";
const std::string LIBRARY_2 = "libclass_loader_TestPlugins2.so";

/*****************************************************************************/
TEST(ClassLoaderTest, basicLoad)
{
  try
  {
    class_loader::ClassLoader loader1(LIBRARY_1, false);
    loader1.createInstance<Base>("Cat")->saySomething(); //See if lazy load works
  }
  catch(class_loader::ClassLoaderException& e)
  {
    FAIL() << "ClassLoaderException: " << e.what() << "\n";
  }

  SUCCEED();
}

/*****************************************************************************/
TEST(ClassLoaderTest, correctNonLazyLoadUnload)
{
  try
  {
  ASSERT_FALSE(class_loader::class_loader_private::isLibraryLoadedByAnybody(LIBRARY_1));
    class_loader::ClassLoader loader1(LIBRARY_1, false);
    ASSERT_TRUE(class_loader::class_loader_private::isLibraryLoadedByAnybody(LIBRARY_1));
    ASSERT_TRUE(loader1.isLibraryLoaded());
    loader1.unloadLibrary();
    ASSERT_FALSE(class_loader::class_loader_private::isLibraryLoadedByAnybody(LIBRARY_1));
    ASSERT_FALSE(loader1.isLibraryLoaded());
    return;
  }
  catch(class_loader::ClassLoaderException& e)
  {
    FAIL() << "ClassLoaderException: " << e.what() << "\n";
  }
  catch(...)
  {
    FAIL() << "Unhandled exception";
  }
}

/*****************************************************************************/
TEST(ClassLoaderTest, correctLazyLoadUnload)
{
  try
  {
    ASSERT_FALSE(class_loader::class_loader_private::isLibraryLoadedByAnybody(LIBRARY_1));
    class_loader::ClassLoader loader1(LIBRARY_1, true);
    ASSERT_FALSE(class_loader::class_loader_private::isLibraryLoadedByAnybody(LIBRARY_1));
    ASSERT_FALSE(loader1.isLibraryLoaded());

    {
      boost::shared_ptr<Base> obj = loader1.createInstance<Base>("Cat");
      ASSERT_TRUE(class_loader::class_loader_private::isLibraryLoadedByAnybody(LIBRARY_1));
      ASSERT_TRUE(loader1.isLibraryLoaded());
    }

    //The library will unload automatically when the only plugin object left is destroyed
    ASSERT_FALSE(class_loader::class_loader_private::isLibraryLoadedByAnybody(LIBRARY_1));
    return;
  }
  catch(class_loader::ClassLoaderException& e)
  {
    FAIL() << "ClassLoaderException: " << e.what() << "\n";
  }
  catch(...)
  {
    FAIL() << "Unhandled exception";
  }
}

/*****************************************************************************/

TEST(ClassLoaderTest, nonExistentPlugin)
{
  class_loader::ClassLoader loader1(LIBRARY_1, false);

  try
  {
    boost::shared_ptr<Base> obj = loader1.createInstance<Base>("Bear");
    if(obj == NULL)
      FAIL() << "Null object being returned instead of exception thrown.";

    obj->saySomething();
  }
  catch(const class_loader::CreateClassException& e)
  {
    SUCCEED();
    return;
  }
  catch(...)
  {
    FAIL() << "Unknown exception caught.\n";
  }

  FAIL() << "Did not throw exception as expected.\n";
}

/*****************************************************************************/
TEST(ClassLoaderTest, nonExistentLibrary)
{
  try
  {
    class_loader::ClassLoader loader1("libDoesNotExist.so", false);
  }
  catch(const class_loader::LibraryLoadException& e)
  {
    SUCCEED();
    return;
  }
  catch(...)
  {
    FAIL() << "Unknown exception caught.\n";
  }

  FAIL() << "Did not throw exception as expected.\n";
}

/*****************************************************************************/

class InvalidBase
{
};

TEST(ClassLoaderTest, invalidBase)
{
  try
  {
    class_loader::ClassLoader loader1(LIBRARY_1, false);
    if(loader1.isClassAvailable<InvalidBase>("Cat"))
    {
      FAIL() << "Cat should not be available for InvalidBase";
    }
    else if(loader1.isClassAvailable<Base>("Cat"))
    {
      SUCCEED();
      return;
    }
    else
      FAIL() << "Class not available for correct base class.";
  }
  catch(const class_loader::LibraryLoadException& e)
  {
    FAIL() << "Unexpected exception";
  }
  catch(...)
  {
    FAIL() << "Unexpected and unknown exception caught.\n";
  }
}

/*****************************************************************************/

void wait(int seconds)
{
  boost::this_thread::sleep(boost::posix_time::seconds(seconds));
}

void run(class_loader::ClassLoader* loader)
{
  std::vector<std::string> classes = loader->getAvailableClasses<Base>();
  for(unsigned int c = 0; c < classes.size(); c++)
  {
    loader->createInstance<Base>(classes.at(c))->saySomething();
  }
}

TEST(ClassLoaderTest, threadSafety)
{
  class_loader::ClassLoader loader1(LIBRARY_1);
  ASSERT_TRUE(loader1.isLibraryLoaded());

  //Note: Hard to test thread safety to make sure memory isn't corrupted. The hope is this test is hard enough that once in a while it'll segfault or something if there's some implementation error.
  try
  {
    std::vector<boost::thread*> client_threads;

    for(unsigned int c = 0; c < 1000; c++)
      client_threads.push_back(new boost::thread(boost::bind(&run, &loader1)));

    for(unsigned int c = 0; c < client_threads.size(); c++)
      client_threads.at(c)->join();

    for(unsigned int c = 0; c < client_threads.size(); c++)
      delete(client_threads.at(c));

    loader1.unloadLibrary();
    ASSERT_FALSE(loader1.isLibraryLoaded());

  }
  catch(const class_loader::ClassLoaderException& ex)
  {
    FAIL() << "Unexpected ClassLoaderException.";
  }
  catch(...)
  {
    FAIL() << "Unknown exception.";
  }
}

/*****************************************************************************/

TEST(ClassLoaderTest, loadRefCountingNonLazy)
{
  try
  {
    class_loader::ClassLoader loader1(LIBRARY_1, false);
    ASSERT_TRUE(loader1.isLibraryLoaded());

    loader1.loadLibrary();
    loader1.loadLibrary();
    ASSERT_TRUE(loader1.isLibraryLoaded());

    loader1.unloadLibrary();
    ASSERT_TRUE(loader1.isLibraryLoaded());
    
    loader1.unloadLibrary();
    ASSERT_TRUE(loader1.isLibraryLoaded());

    loader1.unloadLibrary();
    ASSERT_FALSE(loader1.isLibraryLoaded());

    loader1.unloadLibrary();
    ASSERT_FALSE(loader1.isLibraryLoaded());

    loader1.loadLibrary();
    ASSERT_TRUE(loader1.isLibraryLoaded());

    return;
  }
  catch(const class_loader::ClassLoaderException& e)
  {
    FAIL() << "Unexpected exception.\n";
  }
  catch(...)
  {
    FAIL() << "Unknown exception caught.\n";
  }

  FAIL() << "Did not throw exception as expected.\n";
}

/*****************************************************************************/

TEST(ClassLoaderTest, loadRefCountingLazy)
{
  try
  {
    class_loader::ClassLoader loader1(LIBRARY_1, true);
    ASSERT_FALSE(loader1.isLibraryLoaded());

    {
      boost::shared_ptr<Base> obj = loader1.createInstance<Base>("Dog");
      ASSERT_TRUE(loader1.isLibraryLoaded());
    }
    
    ASSERT_FALSE(loader1.isLibraryLoaded());
    
    loader1.loadLibrary();
    ASSERT_TRUE(loader1.isLibraryLoaded());
    
    loader1.loadLibrary();
    ASSERT_TRUE(loader1.isLibraryLoaded());

    loader1.unloadLibrary();
    ASSERT_TRUE(loader1.isLibraryLoaded());
    
    loader1.unloadLibrary();
    ASSERT_FALSE(loader1.isLibraryLoaded());

    loader1.unloadLibrary();
    ASSERT_FALSE(loader1.isLibraryLoaded());

    loader1.loadLibrary();
    ASSERT_TRUE(loader1.isLibraryLoaded());

    return;
  }
  catch(const class_loader::ClassLoaderException& e)
  {
    FAIL() << "Unexpected exception.\n";
  }
  catch(...)
  {
    FAIL() << "Unknown exception caught.\n";
  }

  FAIL() << "Did not throw exception as expected.\n";
}


/*****************************************************************************/

void testMultiClassLoader(bool lazy)
{
  try
  {
    class_loader::MultiLibraryClassLoader loader(lazy);
    loader.loadLibrary(LIBRARY_1);
    loader.loadLibrary(LIBRARY_2);
    for (int i=0; i < 2; ++i) {
      loader.createInstance<Base>("Cat")->saySomething();
      loader.createInstance<Base>("Dog")->saySomething();
      loader.createInstance<Base>("Robot")->saySomething();
    }
  }
  catch(class_loader::ClassLoaderException& e)
  {
    FAIL() << "ClassLoaderException: " << e.what() << "\n";
  }

  SUCCEED();
}

TEST(MultiClassLoaderTest, lazyLoad)
{
  testMultiClassLoader(true);
}
TEST(MultiClassLoaderTest, lazyLoadSecondTime)
{
  testMultiClassLoader(true);
}
TEST(MultiClassLoaderTest, nonLazyLoad)
{
  testMultiClassLoader(false);
}
TEST(MultiClassLoaderTest, noWarningOnLazyLoad)
{
  try
  {
    boost::shared_ptr<Base> cat, dog, rob;
    {
      class_loader::MultiLibraryClassLoader loader(true);
      loader.loadLibrary(LIBRARY_1);
      loader.loadLibrary(LIBRARY_2);

      cat = loader.createInstance<Base>("Cat");
      dog = loader.createInstance<Base>("Dog");
      rob = loader.createInstance<Base>("Robot");
    }
    cat->saySomething();
    dog->saySomething();
    rob->saySomething();
  }
  catch(class_loader::ClassLoaderException& e)
  {
    FAIL() << "ClassLoaderException: " << e.what() << "\n";
  }

  SUCCEED();
}

/*****************************************************************************/

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


