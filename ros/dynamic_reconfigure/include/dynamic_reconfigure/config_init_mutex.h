#ifndef __DYNAMIC_RECONFIGURE__CONFIG_INIT_MUTEX_H__
#define __DYNAMIC_RECONFIGURE__CONFIG_INIT_MUTEX_H__

#include <boost/thread/mutex.hpp>

namespace dynamic_reconfigure
{
  extern boost::mutex __init_mutex__;
}

#endif
