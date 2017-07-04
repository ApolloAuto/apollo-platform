#include <pluginlib/class_loader.h>
#include "test_base.h"
#include <gtest/gtest.h>

TEST(PluginlibUniquePtrTest, unknownPlugin)
{
  pluginlib::ClassLoader<test_base::Fubar> test_loader("pluginlib", "test_base::Fubar");
  ASSERT_THROW(test_loader.createUniqueInstance("pluginlib/foobar"), pluginlib::LibraryLoadException);
}


TEST(PluginlibUniquePtrTest, misspelledPlugin)
{
  pluginlib::ClassLoader<test_base::Fubar> bad_test_loader("pluginlib", "test_base::Fuba");
  ASSERT_THROW(bad_test_loader.createUniqueInstance("pluginlib/foo"), pluginlib::LibraryLoadException);
}

TEST(PluginlibTest, brokenPlugin)
{
  pluginlib::ClassLoader<test_base::Fubar> test_loader("pluginlib", "test_base::Fubar");
  ASSERT_THROW(test_loader.createUniqueInstance("pluginlib/none"), pluginlib::PluginlibException);
}

TEST(PluginlibUniquePtrTest, workingPlugin)
{
  pluginlib::ClassLoader<test_base::Fubar> test_loader("pluginlib", "test_base::Fubar");

  try
  {
    pluginlib::UniquePtr<test_base::Fubar> foo = test_loader.createUniqueInstance("pluginlib/foo");
    foo->initialize(10.0);
    EXPECT_EQ(foo->result(),100.0);
  }
  catch(pluginlib::PluginlibException& ex)
  {
    FAIL() << "Throwing exception: " << ex.what();
    return;
  }
  catch(...)
  {
    FAIL() << "Uncaught exception";
  }
}

TEST(PluginlibUniquePtrTest, createUniqueInstanceAndUnloadLibrary)
{
  ROS_INFO( "Making the ClassLoader..." );
  pluginlib::ClassLoader<test_base::Fubar> pl("pluginlib", "test_base::Fubar");

  ROS_INFO( "Instantiating plugin..." );
  {
   pluginlib::UniquePtr<test_base::Fubar> inst = pl.createUniqueInstance("pluginlib/foo");
  }

  ROS_INFO( "Checking if plugin is loaded with isClassLoaded..." );
  if( pl.isClassLoaded( "pluginlib/foo" ) )
    ROS_INFO( "Class is loaded" );
  else
  {
    FAIL() <<  "Library containing class should be loaded but isn't.";
  }

  ROS_INFO( "Trying to unload class with unloadLibraryForClass..." );
  try
  {
    pl.unloadLibraryForClass("pluginlib/foo");
  }
  catch(pluginlib::PluginlibException& e)
  {
    FAIL() << "Could not unload library when I should be able to.";
  }
  ROS_INFO( "Done." );
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


