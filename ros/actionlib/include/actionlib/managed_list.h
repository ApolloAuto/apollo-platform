/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef ACTIONLIB_MANAGED_LIST_H_
#define ACTIONLIB_MANAGED_LIST_H_

#include "ros/console.h"
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <list>
#include <actionlib/destruction_guard.h>

namespace actionlib
{

/**
 * \brief wrapper around an STL list to help with reference counting
 * Provides handles elements in an STL list. When all the handles go out of scope,
 * the element in the list is destroyed.
 */
template <class T>
class ManagedList
{
private:
  struct TrackedElem
  {
    T elem;
    boost::weak_ptr<void> handle_tracker_;
  };

public:
  class Handle;

  class iterator
  {
    public:
      iterator() { }
      T& operator*()  { return it_->elem; }
      T& operator->() { return it_->elem; }
      bool operator==(const iterator& rhs) const { return it_ == rhs.it_; }
      bool operator!=(const iterator& rhs) const { return !(*this == rhs); }
      void operator++() { it_++; }
      Handle createHandle();                   //!< \brief Creates a refcounted Handle from an iterator
      friend class ManagedList;
    private:
      iterator(typename std::list<TrackedElem>::iterator it) : it_(it) { }
      typename std::list<TrackedElem>::iterator it_;
  };

  typedef typename boost::function< void(iterator)> CustomDeleter;

private:
  class ElemDeleter
  {
    public:
      ElemDeleter(iterator it, CustomDeleter deleter, const boost::shared_ptr<DestructionGuard>& guard) :
        it_(it), deleter_(deleter), guard_(guard)
      { }

      void operator() (void* ptr)
      {
        DestructionGuard::ScopedProtector protector(*guard_);
        if (!protector.isProtected())
        {
          ROS_ERROR_NAMED("actionlib", "ManagedList: The DestructionGuard associated with this list has already been destructed. You must delete all list handles before deleting the ManagedList");
          return;
        }

        ROS_DEBUG_NAMED("actionlib", "IN DELETER");
        if (deleter_)
          deleter_(it_);
      }

    private:
      iterator it_;
      CustomDeleter deleter_;
      boost::shared_ptr<DestructionGuard> guard_;
  };

public:

  class Handle
  {
    public:
      /**
       * \brief Construct an empty handle
       */
      Handle() : it_(iterator()), handle_tracker_(boost::shared_ptr<void>()), valid_(false) { }

      const Handle& operator=(const Handle& rhs)
      {
    	if ( rhs.valid_ ) {
          it_ = rhs.it_;
    	}
        handle_tracker_ = rhs.handle_tracker_;
        valid_ = rhs.valid_;
        return rhs;
      }

      /**
       * \brief stop tracking the list element with this handle, even though the
       * Handle hasn't gone out of scope
       */
      void reset()
      {
        valid_ = false;
#ifndef _MSC_VER
        // this prevents a crash on MSVC, but I bet the problem is elsewhere.
        // it puts the lotion in the basket.
        it_ = iterator();
#endif
        handle_tracker_.reset();
      }

      /**
       * \brief get the list element that this handle points to
       * fails/asserts if this is an empty handle
       * \return Reference to the element this handle points to
       */
      T& getElem()
      {
        assert(valid_);
        return *it_;
      }

      /**
       * \brief Checks if two handles point to the same list elem
       */
      bool operator==(const Handle& rhs) const
      {
          assert(valid_);
          assert(rhs.valid_);
        return (it_ == rhs.it_);
      }

      friend class ManagedList;
      // Need this friend declaration so that iterator::createHandle() can
      // call the private Handle::Handle() declared below.
      friend class iterator;
    private:
      Handle( const boost::shared_ptr<void>& handle_tracker, iterator it) :
        it_(it), handle_tracker_(handle_tracker), valid_(true)
      { }

      iterator it_;
      boost::shared_ptr<void> handle_tracker_;
      bool valid_;
  };

  ManagedList() { }

  /**
   * \brief Add an element to the back of the ManagedList
   */
  Handle add(const T& elem)
  {
    return add(elem, boost::bind(&ManagedList<T>::defaultDeleter, this, _1) );
  }

  /**
   * \brief Add an element to the back of the ManagedList, along with a Custom deleter
   * \param elem The element we want to add
   * \param deleter Object on which operator() is called when refcount goes to 0
   */
  Handle add(const T& elem, CustomDeleter custom_deleter, const boost::shared_ptr<DestructionGuard>& guard)
  {
    TrackedElem tracked_t;
    tracked_t.elem = elem;

    typename std::list<TrackedElem>::iterator list_it = list_.insert(list_.end(), tracked_t);
    iterator managed_it = iterator(list_it);

    ElemDeleter deleter(managed_it, custom_deleter, guard);
    boost::shared_ptr<void> tracker( (void*) NULL, deleter);

    list_it->handle_tracker_ = tracker;

    return Handle(tracker, managed_it);
  }

  /**
   * \brief Removes an element from the ManagedList
   */
  void erase(iterator it)
  {
    list_.erase(it.it_);
  }

  iterator end()    { return iterator(list_.end()); }
  iterator begin()  { return iterator(list_.begin()); }

private:
  void defaultDeleter(iterator it)
  {
    erase(it);
  }
  std::list<TrackedElem> list_;
};


template<class T>
typename ManagedList<T>::Handle ManagedList<T>::iterator::createHandle()
{
  if (it_->handle_tracker_.expired())
    ROS_ERROR_NAMED("actionlib", "Tried to create a handle to a list elem with refcount 0");

  boost::shared_ptr<void> tracker = it_->handle_tracker_.lock();

  return Handle(tracker, *this);
}

}

#endif
