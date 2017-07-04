#include <pluginlib/class_list_macros.h>
#include "test_base.h"
#include "test_plugins.h"

PLUGINLIB_DECLARE_CLASS(pluginlib, foo, test_plugins::Foo, test_base::Fubar)
PLUGINLIB_DECLARE_CLASS(pluginlib, bar, test_plugins::Bar, test_base::Fubar)
