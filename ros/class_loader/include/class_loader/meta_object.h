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

//Note: This header defines a simplication of Poco::MetaObject that allows us to tag MetaObjects with an associated library name.

#ifndef PLUGINS_PRIVATE_META_OBJECT_DEFINED
#define PLUGINS_PRIVATE_META_OBJECT_DEFINED

#include <console_bridge/console.h>
#include <vector>
#include <typeinfo>

namespace class_loader
{

class ClassLoader; //Forward declaration

namespace class_loader_private 
{

typedef std::vector<class_loader::ClassLoader*> ClassLoaderVector;

/**
 * @class AbstractMetaObjectBase
 * @brief A base class for MetaObjects that excludes a polymorphic type parameter. Subclasses are class templates though.
 */
class AbstractMetaObjectBase
{  
  public:

    /**
     * @brief Constructor for the class
     */
    AbstractMetaObjectBase(const std::string& class_name, const std::string& base_class_name);
    /**
     * @brief Destructor for the class. THIS MUST NOT BE VIRTUAL AND OVERRIDDEN BY
     * TEMPLATE SUBCLASSES, OTHERWISE THEY WILL PULL IN A REDUNDANT METAOBJECT
     * DESTRUCTOR OUTSIDE OF libclass_loader WITHIN THE PLUGIN LIBRARY! T
     */
    ~AbstractMetaObjectBase();

    /**
     * @brief Gets the literal name of the class.
     * @return The literal name of the class as a C-string.
     */
    std::string className() const;

    /**
     * @brief gets the base class for the class this factory represents
     */
    std::string baseClassName() const;
    /**
     * @brief Gets the name of the class as typeid(BASE_CLASS).name() would return it
     */
    std::string typeidBaseClassName() const;

    /**
     * @brief Gets the path to the library associated with this factory
     * @return Library path as a std::string
     */
    std::string getAssociatedLibraryPath();

    /**
     * @brief Sets the path to the library associated with this factory
     */
    void setAssociatedLibraryPath(std::string library_path);

    /**
     * @brief Associates a ClassLoader owner with this factory,
     * @param loader Handle to the owning ClassLoader.
     */
    void addOwningClassLoader(ClassLoader* loader);

    /**
     * @brief Removes a ClassLoader that is an owner of this factory
     * @param loader Handle to the owning ClassLoader. 
     */
    void removeOwningClassLoader(const ClassLoader* loader);

    /**
     * @brief Indicates if the factory is within the usable scope of a ClassLoader
     * @param loader Handle to the owning ClassLoader. 
     */
    bool isOwnedBy(const ClassLoader* loader);

    /**
     * @brief Indicates if the factory is within the usable scope of any ClassLoader
     */
    bool isOwnedByAnybody();

    /**
     * A vector of class loaders that own this metaobject
     */
    ClassLoaderVector getAssociatedClassLoaders();

  protected:
    /**
     * This is needed to make base class polymorphic (i.e. have a vtable)
     */
    virtual void dummyMethod(){}

  protected:
    ClassLoaderVector associated_class_loaders_;    
    std::string associated_library_path_;
    std::string base_class_name_;
    std::string class_name_;
    std::string typeid_base_class_name_;
};

/**
 * @class AbstractMetaObject
 * @brief Abstract base class for factories where polymorphic type variable indicates base class for plugin interface.
 * @parm B The base class interface for the plugin
 */
template <class B>
class AbstractMetaObject : public AbstractMetaObjectBase
{
  public:
    /**
     * @brief A constructor for this class
     * @param name The literal name of the class.
     */
    AbstractMetaObject(const std::string& class_name, const std::string& base_class_name) : 
    AbstractMetaObjectBase(class_name, base_class_name)
    {
        AbstractMetaObjectBase::typeid_base_class_name_ = std::string(typeid(B).name());
    }

    /**
     * @brief Defines the factory interface that the MetaObject must implement.
     * @return A pointer of parametric type B to a newly created object.
     */
    virtual B* create() const = 0;
	    /// Create a new instance of a class.
	    /// Cannot be used for singletons.

  private:
    AbstractMetaObject();
    AbstractMetaObject(const AbstractMetaObject&);
    AbstractMetaObject& operator = (const AbstractMetaObject&);
};

/**
 * @class MetaObject
 * @brief The actual factory. 
 * @parm C The derived class (the actual plugin)
 * @parm B The base class interface for the plugin
 */
template <class C, class B>
class MetaObject: public AbstractMetaObject<B>
{
  public:
    /**
     * @brief Constructor for the class
     */
    MetaObject(const std::string& class_name, const std::string& base_class_name) :
    AbstractMetaObject<B>(class_name, base_class_name)
    {
    }

    /**
     * @brief The factory interface to generate an object. The object has type C in reality, though a pointer of the base class type is returned.
     * @return A pointer to a newly created plugin with the base class type (type parameter B)
     */
    B* create() const
    {
	    return new C;
    }
};

} // End namespace class_loader_private
} // End namespace class_loader

#endif 
