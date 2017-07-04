/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "class_loader/meta_object.h"
#include "class_loader/class_loader.h"

namespace class_loader
{
namespace class_loader_private
{

AbstractMetaObjectBase::AbstractMetaObjectBase(const std::string& class_name, const std::string& base_class_name) :  
associated_library_path_("Unknown"),
base_class_name_(base_class_name),
class_name_(class_name),
typeid_base_class_name_("UNSET")
/*****************************************************************************/
{
    logDebug("class_loader.class_loader_private.AbstractMetaObjectBase: Creating MetaObject %p (base = %s, derived = %s, library path = %s)", this, baseClassName().c_str(), className().c_str(), getAssociatedLibraryPath().c_str());
}

AbstractMetaObjectBase::~AbstractMetaObjectBase()
/*****************************************************************************/
{
    logDebug("class_loader.class_loader_private.AbstractMetaObjectBase: Destroying MetaObject %p (base = %s, derived = %s, library path = %s)", this, baseClassName().c_str(), className().c_str(), getAssociatedLibraryPath().c_str());
} 

std::string AbstractMetaObjectBase::className() const
/*****************************************************************************/
{
  return class_name_;
}

std::string AbstractMetaObjectBase::baseClassName() const
/*****************************************************************************/
{
  return base_class_name_;
}

std::string AbstractMetaObjectBase::typeidBaseClassName() const 
/*****************************************************************************/
{
  return typeid_base_class_name_;
}

std::string AbstractMetaObjectBase::getAssociatedLibraryPath()
/*****************************************************************************/
{
  return(associated_library_path_);
}

void AbstractMetaObjectBase::setAssociatedLibraryPath(std::string library_path)
/*****************************************************************************/
{
  associated_library_path_ = library_path;
}

void AbstractMetaObjectBase::addOwningClassLoader(ClassLoader* loader)
/*****************************************************************************/
{
  ClassLoaderVector& v = associated_class_loaders_;
  if(std::find(v.begin(), v.end(), loader) == v.end())
    v.push_back(loader);
}

void AbstractMetaObjectBase::removeOwningClassLoader(const ClassLoader* loader)
/*****************************************************************************/
{
  ClassLoaderVector& v = associated_class_loaders_;
  ClassLoaderVector::iterator itr = std::find(v.begin(), v.end(), loader);
  if(itr != v.end())
    v.erase(itr);
}

bool AbstractMetaObjectBase::isOwnedBy(const ClassLoader* loader)
/*****************************************************************************/
{
  ClassLoaderVector& v = associated_class_loaders_;
  ClassLoaderVector::iterator itr = std::find(v.begin(), v.end(), loader);
  return(itr != v.end());
}

bool AbstractMetaObjectBase::isOwnedByAnybody()
/*****************************************************************************/
{
  return(associated_class_loaders_.size() > 0);
}

ClassLoaderVector AbstractMetaObjectBase::getAssociatedClassLoaders()
/*****************************************************************************/
{
  return(associated_class_loaders_);
}

}


}
