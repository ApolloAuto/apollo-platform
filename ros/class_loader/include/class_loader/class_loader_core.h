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

#ifndef class_loader_private_H_DEFINED
#define class_loader_private_H_DEFINED

#include <Poco/SharedLibrary.h>
#include <boost/thread/recursive_mutex.hpp>
//#include <vector>
#include <map>
#include <typeinfo>
#include <string>
#include "class_loader/meta_object.h"
#include "class_loader/class_loader_exceptions.h"
#include <cstdio>
 
/**
 * @note This header file is the internal implementation of the plugin system which is exposed via the ClassLoader class
 */

namespace class_loader
{

class ClassLoader; //Forward declaration

namespace class_loader_private
{

//Typedefs
/*****************************************************************************/
typedef std::string LibraryPath;
typedef std::string ClassName;
typedef std::string BaseClassName;
typedef std::map<ClassName, class_loader_private::AbstractMetaObjectBase*> FactoryMap;
typedef std::map<BaseClassName, FactoryMap> BaseToFactoryMapMap;
typedef std::pair<LibraryPath, Poco::SharedLibrary*> LibraryPair;
typedef std::vector<LibraryPair> LibraryVector;
typedef std::vector<AbstractMetaObjectBase*> MetaObjectVector;

//Debug 
/*****************************************************************************/
void printDebugInfoToScreen();

//Global storage
/*****************************************************************************/

/**
 * @brief Gets a handle to a global data structure that holds a map of base class names (Base class describes plugin interface) to a FactoryMap which holds the factories for the various different concrete classes that can be instantiated. Note that the Base class is NOT THE LITERAL CLASSNAME, but rather the result of typeid(Base).name() which sometimes is the literal class name (as on Windows) but is often in mangled form (as on Linux).
 * @return A reference to the global base to factory map
 */
BaseToFactoryMapMap& getGlobalPluginBaseToFactoryMapMap();

/**
 * @brief Gets a handle to a list of open libraries in the form of LibraryPairs which encode the library path+name and the handle to the underlying Poco::SharedLibrary
 * @return A reference to the global vector that tracks loaded libraries
 */
LibraryVector& getLoadedLibraryVector();

/**
 * @brief When a library is being loaded, in order for factories to know which library they are being associated with, they use this function to query which library is being loaded.
 * @return The currently set loading library name as a string
 */
std::string getCurrentlyLoadingLibraryName();

/**
 * @brief When a library is being loaded, in order for factories to know which library they are being associated with, this function is called to set the name of the library currently being loaded.
 * @param library_name - The name of library that is being loaded currently
 */
void setCurrentlyLoadingLibraryName(const std::string& library_name);


/**
 * @brief Gets the ClassLoader currently in scope which used when a library is being loaded.
 * @return A pointer to the currently active ClassLoader.
 */
ClassLoader* getCurrentlyActiveClassLoader();

/**
 * @brief Sets the ClassLoader currently in scope which used when a library is being loaded. 
 * @param loader - pointer to the currently active ClassLoader.
 */
void setCurrentlyActiveClassLoader(ClassLoader* loader);


/**
 * @brief This function extracts a reference to the FactoryMap for appropriate base class out of the global plugin base to factory map. This function should be used by functions in this namespace that need to access the various factories so as to make sure the right key is generated to index into the global map.
 * @return A reference to the FactoryMap contained within the global Base-to-FactoryMap map.
 */
FactoryMap& getFactoryMapForBaseClass(const std::string& typeid_base_class_name);

/**
 * @brief Same as above but uses a type parameter instead of string for more safety if info is available.
 * @return A reference to the FactoryMap contained within the global Base-to-FactoryMap map.
 */
template <typename Base>
FactoryMap& getFactoryMapForBaseClass()
{
    return(getFactoryMapForBaseClass(typeid(Base).name()));
}

/**
 * @brief To provide thread safety, all exposed plugin functions can only be run serially by multiple threads. This is implemented by using critical sections enforced by a single mutex which is locked and released with the following two functions
 * @return A reference to the global mutex
 */
boost::recursive_mutex& getLoadedLibraryVectorMutex();
boost::recursive_mutex& getPluginBaseToFactoryMapMapMutex();

/**
 * @brief Indicates if a library containing more than just plugins has been opened by the running process
 * @return True if a non-pure plugin library has been opened, otherwise false
 */
bool hasANonPurePluginLibraryBeenOpened();

/**
 * @brief Sets a flag indicating if a library containing more than just plugins has been opened by the running process
 * @param hasIt - The flag
 */
void hasANonPurePluginLibraryBeenOpened(bool hasIt);

//Plugin Functions
/*****************************************************************************/

/**
 * @brief This function is called by the CLASS_LOADER_REGISTER_CLASS macro in plugin_register_macro.h to register factories.
 * Classes that use that macro will cause this function to be invoked when the library is loaded. The function will create a MetaObject (i.e. factory) for the corresponding Derived class and insert it into the appropriate FactoryMap in the global Base-to-FactoryMap map. Note that the passed class_name is the literal class name and not the mangled version.
 * @param Derived - parameteric type indicating concrete type of plugin
 * @param Base - parameteric type indicating base type of plugin
 * @param class_name - the literal name of the class being registered (NOT MANGLED)
 */
template <typename Derived, typename Base> 
void registerPlugin(const std::string& class_name, const std::string& base_class_name)
{
  //Note: This function will be automatically invoked when a dlopen() call
  //opens a library. Normally it will happen within the scope of loadLibrary(),
  //but that may not be guaranteed.
  CONSOLE_BRIDGE_logDebug("class_loader.class_loader_private: Registering plugin factory for class = %s, ClassLoader* = %p and library name %s.", class_name.c_str(), getCurrentlyActiveClassLoader(), getCurrentlyLoadingLibraryName().c_str());

  if(getCurrentlyActiveClassLoader() == NULL)
  {
    CONSOLE_BRIDGE_logDebug("class_loader.class_loader_private: ALERT!!! A library containing plugins has been opened through a means other than through the class_loader or pluginlib package. This can happen if you build plugin libraries that contain more than just plugins (i.e. normal code your app links against). This inherently will trigger a dlopen() prior to main() and cause problems as class_loader is not aware of plugin factories that autoregister under the hood. The class_loader package can compensate, but you may run into namespace collision problems (e.g. if you have the same plugin class in two different libraries and you load them both at the same time). The biggest problem is that library can now no longer be safely unloaded as the ClassLoader does not know when non-plugin code is still in use. In fact, no ClassLoader instance in your application will be unable to unload any library once a non-pure one has been opened. Please refactor your code to isolate plugins into their own libraries.");
    hasANonPurePluginLibraryBeenOpened(true);    
  }

  //Create factory
  class_loader_private::AbstractMetaObject<Base>* new_factory = new class_loader_private::MetaObject<Derived, Base>(class_name, base_class_name);
  new_factory->addOwningClassLoader(getCurrentlyActiveClassLoader());
  new_factory->setAssociatedLibraryPath(getCurrentlyLoadingLibraryName());


  //Add it to global factory map map
  getPluginBaseToFactoryMapMapMutex().lock();
  FactoryMap& factoryMap = getFactoryMapForBaseClass<Base>();
  if(factoryMap.find(class_name) != factoryMap.end())
    CONSOLE_BRIDGE_logWarn("class_loader.class_loader_private: SEVERE WARNING!!! A namespace collision has occured with plugin factory for class %s. New factory will OVERWRITE existing one. This situation occurs when libraries containing plugins are directly linked against an executable (the one running right now generating this message). Please separate plugins out into their own library or just don't link against the library and use either class_loader::ClassLoader/MultiLibraryClassLoader to open.", class_name.c_str());
  factoryMap[class_name] = new_factory;
  getPluginBaseToFactoryMapMapMutex().unlock();

  CONSOLE_BRIDGE_logDebug("class_loader.class_loader_private: Registration of %s complete (Metaobject Address = %p)", class_name.c_str(), new_factory);
}

/**
 * @brief This function creates an instance of a plugin class given the derived name of the class and returns a pointer of the Base class type.
 * @param derived_class_name - The name of the derived class (unmangled)
 * @param loader - The ClassLoader whose scope we are within
 * @return A pointer to newly created plugin, note caller is responsible for object destruction
 */
template <typename Base> 
Base* createInstance(const std::string& derived_class_name, ClassLoader* loader)
{
  AbstractMetaObject<Base>* factory = NULL;
  
  getPluginBaseToFactoryMapMapMutex().lock();
  FactoryMap& factoryMap = getFactoryMapForBaseClass<Base>();
  if(factoryMap.find(derived_class_name) != factoryMap.end())
    factory = dynamic_cast<class_loader_private::AbstractMetaObject<Base>*>(factoryMap[derived_class_name]);
  else
  {
    CONSOLE_BRIDGE_logError("class_loader.class_loader_private: No metaobject exists for class type %s.", derived_class_name.c_str());
  }
  getPluginBaseToFactoryMapMapMutex().unlock();

  Base* obj = NULL;
  if(factory != NULL && factory->isOwnedBy(loader))
    obj = factory->create();

  if(obj == NULL) //Was never created
  {
    if(factory && factory->isOwnedBy(NULL))
    {
      CONSOLE_BRIDGE_logDebug("class_loader.class_loader_private: ALERT!!! A metaobject (i.e. factory) exists for desired class, but has no owner. This implies that the library containing the class was dlopen()ed by means other than through the class_loader interface. This can happen if you build plugin libraries that contain more than just plugins (i.e. normal code your app links against) -- that intrinsically will trigger a dlopen() prior to main(). You should isolate your plugins into their own library, otherwise it will not be possible to shutdown the library!");

      obj = factory->create();
    }
    else
      throw(class_loader::CreateClassException("Could not create instance of type " + derived_class_name));
  }

  CONSOLE_BRIDGE_logDebug("class_loader.class_loader_private: Created instance of type %s and object pointer = %p", (typeid(obj).name()), obj);

  return(obj);
}

/**
 * @brief This function returns all the available class_loader in the plugin system that are derived from Base and within scope of the passed ClassLoader.
 * @param loader - The pointer to the ClassLoader whose scope we are within,
 * @return A vector of strings where each string is a plugin we can create
 */
template <typename Base> 
std::vector<std::string> getAvailableClasses(ClassLoader* loader)
{
  boost::recursive_mutex::scoped_lock lock(getPluginBaseToFactoryMapMapMutex());

  FactoryMap& factory_map = getFactoryMapForBaseClass<Base>();
  std::vector<std::string> classes;
  std::vector<std::string> classes_with_no_owner;

  for(FactoryMap::const_iterator itr = factory_map.begin(); itr != factory_map.end(); ++itr)
  {
    AbstractMetaObjectBase* factory = itr->second;
    if(factory->isOwnedBy(loader))
      classes.push_back(itr->first);
    else if(factory->isOwnedBy(NULL))
      classes_with_no_owner.push_back(itr->first);
  }

  //Added classes not associated with a class loader (Which can happen through
  //an unexpected dlopen() to the library)
  classes.insert(classes.end(), classes_with_no_owner.begin(),  classes_with_no_owner.end());
  return(classes);
}

/**
 * @brief This function returns the names of all libraries in use by a given class loader.
 * @param loader - The ClassLoader whose scope we are within
 * @return A vector of strings where each string is the path+name of each library that are within a ClassLoader's visible scope
 */
std::vector<std::string> getAllLibrariesUsedByClassLoader(const ClassLoader* loader);

/**
 * @brief Indicates if passed library loaded within scope of a ClassLoader. The library maybe loaded in memory, but to the class loader it may not be.
 * @param library_path - The name of the library we wish to check is open
 * @param loader - The pointer to the ClassLoader whose scope we are within
 * @return true if the library is loaded within loader's scope, else false
 */
bool isLibraryLoaded(const std::string& library_path, ClassLoader* loader);

/**
 * @brief Indicates if passed library has been loaded by ANY ClassLoader
 * @param library_path - The name of the library we wish to check is open
 * @return true if the library is loaded in memory, otherwise false
 */
bool isLibraryLoadedByAnybody(const std::string& library_path);

/**
 * @brief Loads a library into memory if it has not already been done so. Attempting to load an already loaded library has no effect.
 * @param library_path - The name of the library to open
 * @param loader - The pointer to the ClassLoader whose scope we are within
 */
void loadLibrary(const std::string& library_path, ClassLoader* loader);

/**
 * @brief Unloads a library if it loaded in memory and cleans up its corresponding class factories. If it is not loaded, the function has no effect
 * @param library_path - The name of the library to open
 * @param loader - The pointer to the ClassLoader whose scope we are within
 */
void unloadLibrary(const std::string& library_path, ClassLoader* loader);


} //End namespace class_loader_private
} //End namespace class_loader

#endif
