#include "base.h"
#include <class_loader/class_loader.h>
#include <class_loader/multi_library_class_loader.h>

#include <gtest/gtest.h>
#include <boost/thread.hpp>

#include <functional>
#include <iostream>

const std::string LIBRARY_1 = "libclass_loader_TestPlugins1.so";
const std::string LIBRARY_2 = "libclass_loader_TestPlugins2.so";

using class_loader::ClassLoader;

/*****************************************************************************/
TEST(ClassLoaderUniquePtrTest, basicLoad)
{
  try
  {
    ClassLoader loader1(LIBRARY_1, false);
    loader1.createUniqueInstance<Base>("Cat")->saySomething(); //See if lazy load works
    SUCCEED();
  }
  catch(class_loader::ClassLoaderException& e)
  {
    FAIL() << "ClassLoaderException: " << e.what() << "\n";
  }
}

/*****************************************************************************/
TEST(ClassLoaderUniquePtrTest, correctLazyLoadUnload)
{
  try
  {
    ASSERT_FALSE(class_loader::class_loader_private::isLibraryLoadedByAnybody(LIBRARY_1));
    ClassLoader loader1(LIBRARY_1, true);
    ASSERT_FALSE(class_loader::class_loader_private::isLibraryLoadedByAnybody(LIBRARY_1));
    ASSERT_FALSE(loader1.isLibraryLoaded());

    {
      ClassLoader::UniquePtr<Base> obj = loader1.createUniqueInstance<Base>("Cat");
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

TEST(ClassLoaderUniquePtrTest, nonExistentPlugin)
{
  ClassLoader loader1(LIBRARY_1, false);

  try
  {
    ClassLoader::UniquePtr<Base> obj = loader1.createUniqueInstance<Base>("Bear");
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

void wait(int seconds)
{
  boost::this_thread::sleep(boost::posix_time::seconds(seconds));
}

void run(ClassLoader* loader)
{
  std::vector<std::string> classes = loader->getAvailableClasses<Base>();
  for(unsigned int c = 0; c < classes.size(); c++)
  {
    loader->createUniqueInstance<Base>(classes.at(c))->saySomething();
  }
}

TEST(ClassLoaderUniquePtrTest, threadSafety)
{
  ClassLoader loader1(LIBRARY_1);
  ASSERT_TRUE(loader1.isLibraryLoaded());

  //Note: Hard to test thread safety to make sure memory isn't corrupted.
  //The hope is this test is hard enough that once in a while it'll segfault
  //or something if there's some implementation error.
  try
  {
    std::vector<boost::thread> client_threads;

    for(unsigned int c = 0; c < 1000; c++)
      client_threads.emplace_back(std::bind(&run, &loader1));

    for(unsigned int c = 0; c < client_threads.size(); c++)
      client_threads.at(c).join();

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

TEST(ClassLoaderUniquePtrTest, loadRefCountingLazy)
{
  try
  {
    ClassLoader loader1(LIBRARY_1, true);
    ASSERT_FALSE(loader1.isLibraryLoaded());

    {
      ClassLoader::UniquePtr<Base> obj = loader1.createUniqueInstance<Base>("Dog");
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
      loader.createUniqueInstance<Base>("Cat")->saySomething();
      loader.createUniqueInstance<Base>("Dog")->saySomething();
      loader.createUniqueInstance<Base>("Robot")->saySomething();
    }
  }
  catch(class_loader::ClassLoaderException& e)
  {
    FAIL() << "ClassLoaderException: " << e.what() << "\n";
  }

  SUCCEED();
}

TEST(MultiClassLoaderUniquePtrTest, lazyLoad)
{
  testMultiClassLoader(true);
}
TEST(MultiClassLoaderUniquePtrTest, lazyLoadSecondTime)
{
  testMultiClassLoader(true);
}
TEST(MultiClassLoaderUniquePtrTest, nonLazyLoad)
{
  testMultiClassLoader(false);
}
TEST(MultiClassLoaderUniquePtrTest, noWarningOnLazyLoad)
{
  try
  {
    ClassLoader::UniquePtr<Base> cat = nullptr, dog = nullptr, rob = nullptr;
    {
      class_loader::MultiLibraryClassLoader loader(true);
      loader.loadLibrary(LIBRARY_1);
      loader.loadLibrary(LIBRARY_2);

      cat = loader.createUniqueInstance<Base>("Cat");
      dog = loader.createUniqueInstance<Base>("Dog");
      rob = loader.createUniqueInstance<Base>("Robot");
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


