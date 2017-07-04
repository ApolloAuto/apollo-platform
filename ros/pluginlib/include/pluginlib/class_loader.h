/*
 * Copyright (c) 2009, Willow Garage, Inc.
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
#ifndef PLUGINLIB_CLASS_LOADER_H
#define PLUGINLIB_CLASS_LOADER_H

#include "boost/algorithm/string.hpp"
#include "class_loader/multi_library_class_loader.h"
#include <map>
#include "pluginlib/class_desc.h"
#include "pluginlib/class_loader_base.h"
#include "pluginlib/pluginlib_exceptions.h"
#include "ros/console.h"
#include "ros/package.h"
#include "tinyxml.h"

//Note: pluginlib has traditionally utilized a "lookup name" for classes that does not match its real C++ name. This was
//done due to limitations of how pluginlib was implemented. As of version 1.9, a lookup name is no longer necessary and
//an attempt to the merge two concepts is underway

namespace pluginlib
{

#if __cplusplus >= 201103L
  template<typename T>
  using UniquePtr = class_loader::ClassLoader::UniquePtr<T>;
#endif
  /**
   * @class ClassLoader
   * @brief A class to help manage and load classes
   */
  template <class T>
    class ClassLoader: public ClassLoaderBase
    {
      public:
        typedef typename std::map<std::string, ClassDesc>::iterator ClassMapIterator;

      public:
        /**
         * @brief  Constructor for a ClassLoader
         * @param package The package containing the base class
         * @param base_class The type of the base class for classes to be loaded
         * @param attrib_name The attribute to search for in manifext.xml files, defaults to "plugin"
         * @param plugin_xml_paths The list of paths of plugin.xml files, defaults to be crawled via ros::package::getPlugins()
         * @exception pluginlib::ClassLoaderException Thrown if package manifest cannot be found
         */
        ClassLoader(std::string package, std::string base_class, std::string attrib_name = std::string("plugin"), std::vector<std::string> plugin_xml_paths = std::vector<std::string>());

        /**
         * @brief  Destructor for ClassLoader
         */
        ~ClassLoader();

        /**
         * @brief  Creates an instance of a desired class, optionally loading the associated library automatically if necessary
         * @param  lookup_name The name of the class to load
         * @param  auto_load Specifies whether or not to automatically load the library containing the class, set to true by default
         * @exception pluginlib::LibraryLoadException Thrown when the library associated with the class cannot be loaded
         * @exception pluginlib::CreateClassException Thrown when the class cannot be instantiated
         * @return An instance of the class
         * @deprecated use either createInstance() or createUnmanagedInstance().
         */
        __attribute__((deprecated)) T* createClassInstance(const std::string& lookup_name, bool auto_load = true);

        /**
         * @brief  Creates an instance of a desired class (which implicitly calls loadLibraryForClass() to increment the library counter).
         * Deleting the instance and calling unloadLibraryForClass() is automatically handled by the shared pointer.
         * @param  lookup_name The name of the class to load
         * @exception pluginlib::LibraryLoadException Thrown when the library associated with the class cannot be loaded
         * @exception pluginlib::CreateClassException Thrown when the class cannot be instantiated
         * @return An instance of the class
         */
        boost::shared_ptr<T> createInstance(const std::string& lookup_name);

#if __cplusplus >= 201103L
        /**
         * @brief  Creates an instance of a desired class (which implicitly calls loadLibraryForClass() to increment the library counter).
         * Deleting the instance and calling unloadLibraryForClass() is automatically handled by the unique pointer.
         *
         * If you release the wrapped pointer you must manually call the original deleter when you want to destroy the released pointer.
         *
         * @param  lookup_name The name of the class to load
         * @exception pluginlib::LibraryLoadException Thrown when the library associated with the class cannot be loaded
         * @exception pluginlib::CreateClassException Thrown when the class cannot be instantiated
         * @return An instance of the class
         */
        UniquePtr<T> createUniqueInstance(const std::string& lookup_name);
#endif

        /**
         * @brief  Creates an instance of a desired class (which implicitly calls loadLibraryForClass() to increment the library counter).
         * @attention The ownership is transfered to the caller, which is responsible for deleting the instance and calling unloadLibraryForClass()
         * (in order to decrement the associated library counter and unloading it if it is no more used).
         * @param  lookup_name The name of the class to load
         * @exception pluginlib::LibraryLoadException Thrown when the library associated with the class cannot be loaded
         * @exception pluginlib::CreateClassException Thrown when the class cannot be instantiated
         * @return An instance of the class
         */
        T* createUnmanagedInstance(const std::string& lookup_name);

        /**
         * @brief  Returns a list of all available plugin manifest paths for this ClassLoader's base class type
         * @return A vector of strings corresponding to the paths of all available plugin manifests
         */
        std::vector<std::string> getPluginXmlPaths();

        /**
         * @brief  Returns a list of all available classes for this ClassLoader's base class type
         * @return A vector of strings corresponding to the names of all available classes
         */
        std::vector<std::string> getDeclaredClasses();

        /**
         * @brief  Strips the package name off of a lookup name
         * @param lookup_name The name of the plugin
         * @return The name of the plugin stripped of the package name
         */
        virtual std::string getName(const std::string& lookup_name);

        /**
         * @brief  Given the lookup name of a class, returns the type of the associated base class
         * @return The type of the associated base class
         */
        virtual std::string getBaseClassType() const;

        /**
         * @brief  Given the lookup name of a class, returns the type of the derived class associated with it
         * @param lookup_name The name of the class
         * @return The name of the associated derived class
         */
        virtual std::string getClassType(const std::string& lookup_name);

        /**
         * @brief  Given the lookup name of a class, returns its description
         * @param lookup_name The lookup name of the class
         * @return The description of the class
         */
        virtual std::string getClassDescription(const std::string& lookup_name);

        /**
         * @brief  Given the name of a class, returns the path to its associated library
         * @param lookup_name The name of the class
         * @return The path to the associated library
         */
        virtual std::string getClassLibraryPath(const std::string& lookup_name);

        /**
         * @brief  Given the name of a class, returns name of the containing package
         * @param lookup_name The name of the class
         * @return The name of the containing package
         */
        virtual std::string getClassPackage(const std::string& lookup_name);

        /**
         * @brief  Given the name of a class, returns the path of the associated plugin manifest
         * @param lookup_name The name of the class
         * @return The path of the associated plugin manifest
         */
        virtual std::string getPluginManifestPath(const std::string& lookup_name);

        /**
         * @brief  Returns the libraries that are registered and can be loaded
         * @return A vector of strings corresponding to the names of registered libraries
         */
        virtual std::vector<std::string> getRegisteredLibraries();

        /**
         * @brief Checks if the library for a given class is currently loaded
         * @param  lookup_name The lookup name of the class to query
         * @return True if the class is loaded, false otherwise
         */
        bool isClassLoaded(const std::string& lookup_name);

        /**
         * @brief  Checks if the class associated with a plugin name is available to be loaded
         * @param lookup_name The name of the plugin
         * @return True if the plugin is available, false otherwise
         */
        virtual bool isClassAvailable(const std::string& lookup_name);

        /**
         * @brief  Attempts to load the library containing a class with a given name and increments a counter for the library
         * @param lookup_name The lookup name of the class to load
         * @exception pluginlib::LibraryLoadException Thrown if the library for the class cannot be loaded
         */
        virtual void loadLibraryForClass(const std::string & lookup_name);

        /**
         * @brief  Refreshs the list of all available classes for this ClassLoader's base class type
         * @exception pluginlib::LibraryLoadException Thrown if package manifest cannot be found
         */
        virtual void refreshDeclaredClasses();

        /**
         * @brief  Decrements the counter for the library containing a class with a given name and attempts to unload it if the counter reaches zero
         * @param lookup_name The lookup name of the class to unload
         * @exception pluginlib::LibraryUnloadException Thrown if the library for the class cannot be unloaded
         * @return The number of pending unloads until the library is removed from memory
         */
        virtual int unloadLibraryForClass(const std::string& lookup_name);

      private:
        /**
         * Calls a program from command line and returns output to stdout as a string
         */
        std::string callCommandLine(const char* cmd);

        /**
         * Returns the paths to plugin.xml files.
         * @exception pluginlib::LibraryLoadException Thrown if package manifest cannot be found
         * @return A vector of paths
         */
        std::vector<std::string> getPluginXmlPaths(const std::string& package, const std::string& attrib_name, bool force_recrawl=false);

        /**
         * @brief  Returns the available classes
         * @param plugin_xml_paths The vector of paths of plugin.xml files
         * @exception pluginlib::LibraryLoadException Thrown if package manifest cannot be found
         * @return A map of class names and the corresponding descriptions
         */
        std::map<std::string, ClassDesc> determineAvailableClasses(const std::vector<std::string>& plugin_xml_paths);

        /**
        * Opens a package.xml file and extracts the package name (i.e. contents of <name> tag)
        */
        std::string extractPackageNameFromPackageXML(const std::string& package_xml_path);

        /**
         * Gets a list of paths to try to find a library. As we transition from rosbuild to Catkin build systems, plugins can be found in the old rosbuild place (pkg_name/lib usually) or somewhere in the Catkin build space
         */
        std::vector<std::string> getAllLibraryPathsToTry(const std::string& library_name, const std::string& exporting_package_name);

        /**
         * Returns the paths where libraries are installed according to the Catkin build system.
         */
        std::vector<std::string> getCatkinLibraryPaths();

        /**
         * @brief  Returns an error message for an unknown class
         * @param lookup_name The name of the class
         * @return The error message
         */
        std::string getErrorStringForUnknownClass(const std::string& lookup_name);

        /**
         * Gets the standard path separator for the native OS (e.g. "/" on *nix, "\" on windows)
         */
        std::string getPathSeparator();

        /**
         * Gets the path where rosbuild build system thinks plugins are installed
         */
        std::string getROSBuildLibraryPath(const std::string& exporting_package_name);

        /**
        * Gets the package name from a path to a plugin XML file
        */
        std::string getPackageFromPluginXMLFilePath(const std::string & path);  

        /**
        * Joins two filesystem paths together utilzing appropriate path separator
        */
        std::string joinPaths(const std::string& path1, const std::string& path2);       

        /**
         *Parses a string delimited by newlines into a vector of strings
         */
        std::vector<std::string> parseToStringVector(std::string newline_delimited_str);

        /**
         * Parses a plugin XML file and inserts the appropriate ClassDesc entries into the passes classes_available map
         */
        void processSingleXMLPluginFile(const std::string& xml_file, std::map<std::string, ClassDesc>& class_available);

        /**
         * Strips all but the filename from an explicit file path.
         */
        std::string stripAllButFileFromPath(const std::string& path);


        /**
         * @brief  Helper function for unloading a shared library
         * @param  library_path The exact path to the library to unload
         * @return The number of pending unloads until the library is removed from memory
         */
        int unloadClassLibraryInternal(const std::string& library_path);        

     private:
        std::vector<std::string> plugin_xml_paths_;
        std::map<std::string, ClassDesc> classes_available_; //Map from library to class's descriptions described in XML
        std::string package_;
        std::string base_class_;
        std::string attrib_name_;
        class_loader::MultiLibraryClassLoader lowlevel_class_loader_; //The underlying classloader
    };
};

#include "class_loader_imp.h" //Note: The implementation of the methods is in a separate file for clarity

#endif //PLUGINLIB_CLASS_LOADER_H

