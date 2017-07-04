/*********************************************************************
*
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*
*********************************************************************/

//NOTE: this should really never be included on its own, but just in case someone is bad we'll guard

#ifndef PLUGINLIB_CLASS_LOADER_IMP_H_
#define PLUGINLIB_CLASS_LOADER_IMP_H_

#include "boost/bind.hpp"
#include "boost/filesystem.hpp"
#include <class_loader/class_loader.h>
#include <list>
#include "ros/package.h"
#include <sstream>
#include <stdexcept>

namespace pluginlib
{
  template <class T>
  ClassLoader<T>::ClassLoader(std::string package, std::string base_class, std::string attrib_name, std::vector<std::string> plugin_xml_paths) :
  plugin_xml_paths_(plugin_xml_paths),
  package_(package),
  base_class_(base_class),
  attrib_name_(attrib_name),
  lowlevel_class_loader_(false) //NOTE: The parameter to the class loader enables/disables on-demand class loading/unloading. Leaving it off for now...libraries will be loaded immediately and won't be unloaded until class loader is destroyed or force unload.
  /***************************************************************************/
  {
    ROS_DEBUG_NAMED("pluginlib.ClassLoader","Creating ClassLoader, base = %s, address = %p", base_class.c_str(), this);
    if (ros::package::getPath(package_).empty())
    {
      throw pluginlib::ClassLoaderException("Unable to find package: " + package_);
    }

    if (plugin_xml_paths_.size() == 0)
    {
      plugin_xml_paths_ = getPluginXmlPaths(package_, attrib_name_);
    }
    classes_available_ = determineAvailableClasses(plugin_xml_paths_);
    ROS_DEBUG_NAMED("pluginlib.ClassLoader","Finished constructring ClassLoader, base = %s, address = %p", base_class.c_str(), this);
  }

  template <class T>
  ClassLoader<T>::~ClassLoader()
  /***************************************************************************/
  {
    ROS_DEBUG_NAMED("pluginlib.ClassLoader","Destroying ClassLoader, base = %s, address = %p", getBaseClassType().c_str(), this);
  }

  template <class T>
  std::string ClassLoader<T>::callCommandLine(const char* cmd)
  /***************************************************************************/
  {
    FILE* pipe = popen(cmd, "r");
    if (!pipe)
      return "ERROR";
    char buffer[128];
    std::string result = "";
    while(!feof(pipe))
    {
      if(fgets(buffer, 128, pipe) != NULL)
              result += buffer;
    }
    pclose(pipe);
    return result;
  }

  template <class T>
  T* ClassLoader<T>::createClassInstance(const std::string& lookup_name, bool auto_load)
  /***************************************************************************/
  {
    //Note: This method is deprecated
    ROS_DEBUG_NAMED("pluginlib.ClassLoader","In deprecated call createClassInstance(), lookup_name = %s, auto_load = %i.", (lookup_name.c_str()), auto_load);

    if (auto_load && !isClassLoaded(lookup_name))
    {
      ROS_DEBUG_NAMED("pluginlib.ClassLoader","Autoloading class library before attempting to create instance.");
      loadLibraryForClass(lookup_name);
    }

    try
    {
      ROS_DEBUG_NAMED("pluginlib.ClassLoader","Attempting to create instance through low-level MultiLibraryClassLoader...");
      T* obj = lowlevel_class_loader_.createUnmanagedInstance<T>(getClassType(lookup_name));
      ROS_DEBUG_NAMED("pluginlib.ClassLoader","Instance created with object pointer = %p", obj);

      return obj;
    }
    catch (const class_loader::CreateClassException& ex)
    {
      ROS_DEBUG_NAMED("pluginlib.ClassLoader","CreateClassException about to be raised for class %s", lookup_name.c_str());
      throw pluginlib::CreateClassException(ex.what());
    }
  }

  template <class T>
  boost::shared_ptr<T> ClassLoader<T>::createInstance(const std::string& lookup_name)
  /***************************************************************************/
  {
    ROS_DEBUG_NAMED("pluginlib.ClassLoader","Attempting to create managed instance for class %s.", lookup_name.c_str());

    if (!isClassLoaded(lookup_name))
      loadLibraryForClass(lookup_name);

    try
    {
      std::string class_type = getClassType(lookup_name);
      ROS_DEBUG_NAMED("pluginlib.ClassLoader","%s maps to real class type %s", lookup_name.c_str(), class_type.c_str());

      boost::shared_ptr<T> obj = lowlevel_class_loader_.createInstance<T>(class_type);

      ROS_DEBUG_NAMED("pluginlib.ClassLoader","boost::shared_ptr to object of real type %s created.", class_type.c_str());

      return obj;
    }
    catch (const class_loader::CreateClassException& ex)
    {
      ROS_DEBUG_NAMED("pluginlib.ClassLoader","Exception raised by low-level multi-library class loader when attempting to create instance of class %s.", lookup_name.c_str());
      throw pluginlib::CreateClassException(ex.what());
    }
  }

#if __cplusplus >= 201103L
  template <class T>
  UniquePtr<T> ClassLoader<T>::createUniqueInstance(const std::string& lookup_name)
  {
    ROS_DEBUG_NAMED("pluginlib.ClassLoader","Attempting to create managed (unique) instance for class %s.", lookup_name.c_str());

    if (!isClassLoaded(lookup_name))
      loadLibraryForClass(lookup_name);

    try
    {
      std::string class_type = getClassType(lookup_name);
      ROS_DEBUG_NAMED("pluginlib.ClassLoader","%s maps to real class type %s", lookup_name.c_str(), class_type.c_str());

      UniquePtr<T> obj = lowlevel_class_loader_.createUniqueInstance<T>(class_type);

      ROS_DEBUG_NAMED("pluginlib.ClassLoader","std::unique_ptr to object of real type %s created.", class_type.c_str());

      return obj;
    }
    catch (const class_loader::CreateClassException& ex)
    {
      ROS_DEBUG_NAMED("pluginlib.ClassLoader","Exception raised by low-level multi-library class loader when attempting to create instance of class %s.", lookup_name.c_str());
      throw pluginlib::CreateClassException(ex.what());
    }

  }
#endif

  template <class T>
  T* ClassLoader<T>::createUnmanagedInstance(const std::string& lookup_name)
  /***************************************************************************/
  {
    ROS_DEBUG_NAMED("pluginlib.ClassLoader","Attempting to create UNMANAGED instance for class %s.", lookup_name.c_str());

    if (!isClassLoaded(lookup_name))
      loadLibraryForClass(lookup_name);

    T* instance = 0;
    try
    {
      ROS_DEBUG_NAMED("pluginlib.ClassLoader","Attempting to create instance through low level multi-library class loader.");
      std::string class_type = getClassType(lookup_name);
      ROS_DEBUG_NAMED("pluginlib.ClassLoader","%s maps to real class type %s", lookup_name.c_str(), class_type.c_str());
      instance = lowlevel_class_loader_.createUnmanagedInstance<T>(class_type);
      ROS_DEBUG_NAMED("pluginlib.ClassLoader","Instance of type %s created.", class_type.c_str());
    }
    catch (const class_loader::CreateClassException& ex) //mas - change exception type here (DONE)
    {
      ROS_DEBUG_NAMED("pluginlib.ClassLoader","Exception raised by low-level multi-library class loader when attempting to create UNMANAGED instance of class %s.", lookup_name.c_str());
      throw pluginlib::CreateClassException(ex.what());
    }
    return instance;
  }

  template <class T>
  std::vector<std::string> ClassLoader<T>::getPluginXmlPaths(const std::string& package, const std::string& attrib_name, bool force_recrawl)
  /***************************************************************************/
  {
    //Pull possible files from manifests of packages which depend on this package and export class
    std::vector<std::string> paths;
    ros::package::getPlugins(package, attrib_name, paths, force_recrawl);
    return paths;
  }

  template <class T>
  std::map<std::string, ClassDesc> ClassLoader<T>::determineAvailableClasses(const std::vector<std::string>& plugin_xml_paths)
  /***************************************************************************/
  {
    //mas - This method requires major refactoring...not only is it really long and confusing but a lot of the comments do not seem to be correct. With time I keep correcting small things, but a good rewrite is needed.

    ROS_DEBUG_NAMED("pluginlib.ClassLoader","Entering determineAvailableClasses()...");
    std::map<std::string, ClassDesc> classes_available;

    //Walk the list of all plugin XML files (variable "paths") that are exported by the build system
    for (std::vector<std::string>::const_iterator it = plugin_xml_paths.begin(); it != plugin_xml_paths.end(); ++it)
    {
      processSingleXMLPluginFile(*it, classes_available);
    }

    ROS_DEBUG_NAMED("pluginlib.ClassLoader","Exiting determineAvailableClasses()...");
    return classes_available;
  }

  template <class T>
  std::string ClassLoader<T>::extractPackageNameFromPackageXML(const std::string& package_xml_path)
 /***************************************************************************/
  {
      TiXmlDocument document;
      document.LoadFile(package_xml_path);
      TiXmlElement* doc_root_node = document.FirstChildElement("package");
      if (doc_root_node == NULL)
      {
        ROS_ERROR_NAMED("pluginlib.ClassLoader","Could not find a root element for package manifest at %s.", package_xml_path.c_str());
        return "";
      }

      assert(doc_root_node == document.RootElement());

      TiXmlElement* package_name_node = doc_root_node->FirstChildElement("name");
      if(package_name_node == NULL)
      {
        ROS_ERROR_NAMED("pluginlib.ClassLoader","package.xml at %s does not have a <name> tag! Cannot determine package which exports plugin.", package_xml_path.c_str());
        return "";
      }

      return(package_name_node->GetText());
  }

  template <class T>
  std::vector<std::string> ClassLoader<T>::getCatkinLibraryPaths()
  /***************************************************************************/
  {
    //TODO: This needs to be replaced with an api call
    return(parseToStringVector(callCommandLine("catkin_find --lib")));
  }

  template <class T>
  std::vector<std::string> ClassLoader<T>::getAllLibraryPathsToTry(const std::string& library_name, const std::string& exporting_package_name)
  /***************************************************************************/
  {
    //Catkin-rosbuild Backwards Compatability Rules - Note library_name may be prefixed with relative path (e.g. "/lib/libFoo")
    //1. Try catkin library paths (catkin_find --libs) + library_name + extension
    //2. Try catkin library paths (catkin_find -- libs) + stripAllButFileFromPath(library_name) + extension
    //3. Try export_pkg/library_name + extension

    std::vector<std::string> all_paths;
    std::vector<std::string> all_paths_without_extension = getCatkinLibraryPaths();
    all_paths_without_extension.push_back(getROSBuildLibraryPath(exporting_package_name));
    bool debug_library_suffix = (class_loader::systemLibrarySuffix().compare(0, 1, "d") == 0);
    std::string non_debug_suffix;
    if(debug_library_suffix) {
        non_debug_suffix = class_loader::systemLibrarySuffix().substr(1);
    } else {
        non_debug_suffix = class_loader::systemLibrarySuffix();
    }
    std::string library_name_with_extension = library_name + non_debug_suffix;
    std::string stripped_library_name = stripAllButFileFromPath(library_name);
    std::string stripped_library_name_with_extension = stripped_library_name + non_debug_suffix;

    const std::string path_separator = getPathSeparator();

    for(unsigned int c = 0; c < all_paths_without_extension.size(); c++)
    {
      std::string current_path = all_paths_without_extension.at(c);
      all_paths.push_back(current_path + path_separator + library_name_with_extension);
      all_paths.push_back(current_path + path_separator + stripped_library_name_with_extension);
      // We're in debug mode, try debug libraries as well
      if(debug_library_suffix) {
          all_paths.push_back(current_path + path_separator + library_name + class_loader::systemLibrarySuffix());
          all_paths.push_back(current_path + path_separator + stripped_library_name + class_loader::systemLibrarySuffix());
      }
    }

    return(all_paths);
  }

  template <class T>
  bool ClassLoader<T>::isClassLoaded(const std::string& lookup_name)
  /***************************************************************************/
  {
    return lowlevel_class_loader_.isClassAvailable<T>(getClassType(lookup_name)); //mas - (DONE)
  }

  template <class T>
  std::string ClassLoader<T>::getBaseClassType() const
  /***************************************************************************/
  {
    return base_class_;
  }

    template <class T>
  std::string ClassLoader<T>::getClassDescription(const std::string& lookup_name)
  /***************************************************************************/
  {
    ClassMapIterator it = classes_available_.find(lookup_name);
    if (it != classes_available_.end())
      return it->second.description_;
    return "";
  }

    template <class T>
  std::string ClassLoader<T>::getClassType(const std::string& lookup_name)
  /***************************************************************************/
  {
    ClassMapIterator it = classes_available_.find(lookup_name);
    if (it != classes_available_.end())
      return it->second.derived_class_;
    return "";
  }

  template <class T>
  std::string ClassLoader<T>::getClassLibraryPath(const std::string& lookup_name)
  /***************************************************************************/
  {
    if (classes_available_.find(lookup_name) == classes_available_.end())
    {
      ROS_DEBUG_NAMED("pluginlib.ClassLoader","Class %s has no mapping in classes_available_.", lookup_name.c_str());
      return "";
    }
    ClassMapIterator it = classes_available_.find(lookup_name);
    std::string library_name = it->second.library_name_;
    ROS_DEBUG_NAMED("pluginlib.ClassLoader","Class %s maps to library %s in classes_available_.", lookup_name.c_str(), library_name.c_str());

    std::vector<std::string> paths_to_try = getAllLibraryPathsToTry(library_name, it->second.package_);

    ROS_DEBUG_NAMED("pluginlib.ClassLoader","Iterating through all possible paths where %s could be located...", library_name.c_str());
    for(std::vector<std::string>::const_iterator it = paths_to_try.begin(); it != paths_to_try.end(); it++)
    {
      ROS_DEBUG_NAMED("pluginlib.ClassLoader","Checking path %s ", it->c_str());
      if (boost::filesystem::exists(*it))
      {
        ROS_DEBUG_NAMED("pluginlib.ClassLoader","Library %s found at explicit path %s.", library_name.c_str(), it->c_str());
        return *it;
      }
    }
    return "";
  }

  template <class T>
  std::string ClassLoader<T>::getClassPackage(const std::string& lookup_name)
  /***************************************************************************/
  {
    ClassMapIterator it = classes_available_.find(lookup_name);
    if (it != classes_available_.end())
      return it->second.package_;
    return "";
  }

  template <class T>
  std::vector<std::string> ClassLoader<T>::getPluginXmlPaths()
  /***************************************************************************/
  {
    return plugin_xml_paths_;
  }

  template <class T>
  std::vector<std::string> ClassLoader<T>::getDeclaredClasses()
  /***************************************************************************/
  {
    std::vector<std::string> lookup_names;
    for (ClassMapIterator it = classes_available_.begin(); it != classes_available_.end(); ++it)
      lookup_names.push_back(it->first);

    return lookup_names;
  }

  template <class T>
  std::string ClassLoader<T>::getErrorStringForUnknownClass(const std::string& lookup_name)
  /***************************************************************************/
  {
    std::string declared_types;
    std::vector<std::string> types = getDeclaredClasses();
    for ( unsigned int i = 0; i < types.size(); i ++)
    {
      declared_types = declared_types + std::string(" ") + types[i];
    }
    return "According to the loaded plugin descriptions the class " + lookup_name
      + " with base class type " + base_class_ + " does not exist. Declared types are " + declared_types;
  }

  template <class T>
  std::string ClassLoader<T>::getName(const std::string& lookup_name)
  /***************************************************************************/
  {
    //remove the package name to get the raw plugin name
    std::vector<std::string> split;
    boost::split(split, lookup_name, boost::is_any_of("/:"));
    return split.back();
  }

  template <class T>
  std::string ClassLoader<T>::getPackageFromPluginXMLFilePath(const std::string & plugin_xml_file_path)
 /***************************************************************************/
  {
    //Note: This method takes an input a path to a plugin xml file and must determine which
    //package the XML file came from. This is not necessariliy the same thing as the member
    //variable "package_". The plugin xml file can be located anywhere in the source tree for a
    //package

    //rosbuild:
    //1. Find nearest encasing manifest.xml
    //2. Once found, the name of the folder containg the manifest should be the package name we are looking for
    //3. Confirm package is findable with rospack

    //catkin:
    //1. Find nearest encasing package.xml
    //2. Extract name of package from package.xml

    std::string package_name;
    boost::filesystem::path p(plugin_xml_file_path);
    boost::filesystem::path parent = p.parent_path();

    //Figure out exactly which package the passed XML file is exported by.
    while (true)
    {
      if(boost::filesystem::exists(parent / "package.xml"))
      {
        std::string package_file_path = (boost::filesystem::path(parent / "package.xml")).string();
        return(extractPackageNameFromPackageXML(package_file_path));
      }
      else if (boost::filesystem::exists(parent / "manifest.xml"))
      {
#if BOOST_FILESYSTEM_VERSION >= 3
        std::string package = parent.filename().string();
#else
        std::string package = parent.filename();
#endif
        std::string package_path = ros::package::getPath(package);

        if (plugin_xml_file_path.find(package_path) == 0) //package_path is a substr of passed plugin xml path
        {
          package_name = package;
          break;
        }
      }

      //Recursive case - hop one folder up
      parent = parent.parent_path().string();

      //Base case - reached root and cannot find what we're looking for
      if (parent.string().empty())
        return "";
    }

    return package_name;
  }

  template <class T>
  std::string ClassLoader<T>::getPathSeparator()
  /***************************************************************************/
  {
#if BOOST_FILESYSTEM_VERSION >= 3
    return(boost::filesystem::path("/").native());
#else
    return(boost::filesystem::path("/").external_file_string());
#endif
  }


  template <class T>
  std::string ClassLoader<T>::getPluginManifestPath(const std::string& lookup_name)
  /***************************************************************************/
  {
    ClassMapIterator it = classes_available_.find(lookup_name);
    if (it != classes_available_.end())
      return it->second.plugin_manifest_path_;
    return "";
  }


  template <class T>
  std::vector<std::string> ClassLoader<T>::getRegisteredLibraries()
  /***************************************************************************/
  {
    return(lowlevel_class_loader_.getRegisteredLibraries());
  }

  template <class T>
  std::string ClassLoader<T>::getROSBuildLibraryPath(const std::string& exporting_package_name)
  /***************************************************************************/
  {
    return(ros::package::getPath(exporting_package_name));
  }

  template <class T>
  bool ClassLoader<T>::isClassAvailable(const std::string& lookup_name)
  /***************************************************************************/
  {
    return classes_available_.find(lookup_name) != classes_available_.end();
  }

  template <class T>
  std::string ClassLoader<T>::joinPaths(const std::string& path1, const std::string& path2)
  /***************************************************************************/
  {
    boost::filesystem::path p1(path1);
    return (p1 / path2).string();
  }

  template <class T>
  void ClassLoader<T>::loadLibraryForClass(const std::string& lookup_name)
  /***************************************************************************/
  {
    ClassMapIterator it = classes_available_.find(lookup_name);
    if (it == classes_available_.end())
    {
      ROS_DEBUG_NAMED("pluginlib.ClassLoader","Class %s has no mapping in classes_available_.", lookup_name.c_str());
      throw pluginlib::LibraryLoadException(getErrorStringForUnknownClass(lookup_name));
    }

    std::string library_path = getClassLibraryPath(lookup_name);
    if (library_path == "")
    {
      ROS_DEBUG_NAMED("pluginlib.ClassLoader","No path could be found to the library containing %s.", lookup_name.c_str());
      std::ostringstream error_msg;
      error_msg << "Could not find library corresponding to plugin " << lookup_name << ". Make sure the plugin description XML file has the correct name of the library and that the library actually exists.";
      throw pluginlib::LibraryLoadException(error_msg.str());
    }

    try
    {
      lowlevel_class_loader_.loadLibrary(library_path);
      it->second.resolved_library_path_ = library_path;
    }
    catch(const class_loader::LibraryLoadException& ex)
    {
      std::string error_string = "Failed to load library " + library_path + ". Make sure that you are calling the PLUGINLIB_EXPORT_CLASS macro in the library code, and that names are consistent between this macro and your XML. Error string: " + ex.what();
      throw pluginlib::LibraryLoadException(error_string);
    }
  }

  template <class T>
  std::vector<std::string> ClassLoader<T>::parseToStringVector(std::string newline_delimited_str)
  /***************************************************************************/
  {
    std::string next;
    std::vector<std::string> parse_result;
    for(unsigned int c = 0; c < newline_delimited_str.size(); c++)
    {
      char ch = newline_delimited_str.at(c);
      if(ch == '\n')
      {
        parse_result.push_back(next);
        next = "";
      }
      else
        next.push_back(ch);
    }
    return(parse_result);
  }


  template <class T>
  void ClassLoader<T>::processSingleXMLPluginFile(const std::string& xml_file, std::map<std::string, ClassDesc>& classes_available)
  /***************************************************************************/
  {
    ROS_DEBUG_NAMED("pluginlib.ClassLoader","Processing xml file %s...", xml_file.c_str());
    TiXmlDocument document;
    document.LoadFile(xml_file);
    TiXmlElement * config = document.RootElement();
    if (config == NULL)
    {
      ROS_ERROR_NAMED("pluginlib.ClassLoader","Skipping XML Document \"%s\" which had no Root Element.  This likely means the XML is malformed or missing.", xml_file.c_str());
      return;
    }
    if (config->ValueStr() != "library" &&
        config->ValueStr() != "class_libraries")
    {
      ROS_ERROR_NAMED("pluginlib.ClassLoader","The XML document \"%s\" given to add must have either \"library\" or \
          \"class_libraries\" as the root tag", xml_file.c_str());
      return;
    }
    //Step into the filter list if necessary
    if (config->ValueStr() == "class_libraries")
    {
      config = config->FirstChildElement("library");
    }

    TiXmlElement* library = config;
    while ( library != NULL)
    {
      std::string library_path = library->Attribute("path");
      if (library_path.size() == 0)
      {
        ROS_ERROR_NAMED("pluginlib.ClassLoader","Failed to find Path Attirbute in library element in %s", xml_file.c_str());
        continue;
      }

      std::string package_name = getPackageFromPluginXMLFilePath(xml_file);
      if (package_name == "")
        ROS_ERROR_NAMED("pluginlib.ClassLoader","Could not find package manifest (neither package.xml or deprecated manifest.xml) at same directory level as the plugin XML file %s. Plugins will likely not be exported properly.\n)", xml_file.c_str());

      TiXmlElement* class_element = library->FirstChildElement("class");
      while (class_element)
      {
        std::string derived_class;
        if (class_element->Attribute("type") != NULL) {
          derived_class = std::string(class_element->Attribute("type"));
        } else {
          throw pluginlib::ClassLoaderException("Class could not be loaded. Attribute 'type' in class tag is missing.");
        }

        std::string base_class_type;
        if (class_element->Attribute("base_class_type") != NULL) {
          base_class_type = std::string(class_element->Attribute("base_class_type"));
        } else {
          throw pluginlib::ClassLoaderException("Class could not be loaded. Attribute 'base_class_type' in class tag is missing.");
        }

        std::string lookup_name;
        if(class_element->Attribute("name") != NULL)
        {
          lookup_name = class_element->Attribute("name");
          ROS_DEBUG_NAMED("pluginlib.ClassLoader","XML file specifies lookup name (i.e. magic name) = %s.", lookup_name.c_str());
        }
        else
        {
          ROS_DEBUG_NAMED("pluginlib.ClassLoader","XML file has no lookup name (i.e. magic name) for class %s, assuming lookup_name == real class name.", derived_class.c_str());
          lookup_name = derived_class;
        }

        //make sure that this class is of the right type before registering it
        if(base_class_type == base_class_){

          // register class here
          TiXmlElement* description = class_element->FirstChildElement("description");
          std::string description_str;
          if (description)
            description_str = description->GetText() ? description->GetText() : "";
          else
            description_str = "No 'description' tag for this plugin in plugin description file.";

          classes_available.insert(std::pair<std::string, ClassDesc>(lookup_name, ClassDesc(lookup_name, derived_class, base_class_type, package_name, description_str, library_path, xml_file)));
        }

        //step to next class_element
        class_element = class_element->NextSiblingElement( "class" );
      }
      library = library->NextSiblingElement( "library" );
    }
  }

  template <class T>
  void ClassLoader<T>::refreshDeclaredClasses()
  /***************************************************************************/
  {
    ROS_DEBUG_NAMED("pluginlib.ClassLoader","Refreshing declared classes.");
    // determine classes not currently loaded for removal
    std::list<std::string> remove_classes;
    for (std::map<std::string, ClassDesc>::const_iterator it = classes_available_.begin(); it != classes_available_.end(); it++)
    {

      std::string resolved_library_path = it->second.resolved_library_path_;
      std::vector<std::string> open_libs = lowlevel_class_loader_.getRegisteredLibraries();
      if(std::find(open_libs.begin(), open_libs.end(), resolved_library_path) != open_libs.end())
        remove_classes.push_back(it->first);
    }

    while (!remove_classes.empty())
    {
      classes_available_.erase(remove_classes.front());
      remove_classes.pop_front();
    }

    // add new classes
    plugin_xml_paths_ = getPluginXmlPaths(package_, attrib_name_, true);
    std::map<std::string, ClassDesc> updated_classes = determineAvailableClasses(plugin_xml_paths_);
    for (std::map<std::string, ClassDesc>::const_iterator it = updated_classes.begin(); it != updated_classes.end(); it++)
    {
      if (classes_available_.find(it->first) == classes_available_.end())
        classes_available_.insert(std::pair<std::string, ClassDesc>(it->first, it->second));
    }
  }

  template <class T>
  std::string ClassLoader<T>::stripAllButFileFromPath(const std::string& path)
  /***************************************************************************/
  {
    std::string only_file;
    size_t c = path.find_last_of(getPathSeparator());
    if(c == std::string::npos)
      return(path);
    else
      return(path.substr(c, path.size()));
  }

  template <class T>
  int ClassLoader<T>::unloadLibraryForClass(const std::string& lookup_name)
  /***************************************************************************/
  {
    ClassMapIterator it = classes_available_.find(lookup_name);
    if (it != classes_available_.end() && it->second.resolved_library_path_ != "UNRESOLVED")
    {
      std::string library_path = it->second.resolved_library_path_;
      ROS_DEBUG_NAMED("pluginlib.ClassLoader","Attempting to unload library %s for class %s", library_path.c_str(), lookup_name.c_str());
      return unloadClassLibraryInternal(library_path);
    }
    else
    {
      throw pluginlib::LibraryUnloadException(getErrorStringForUnknownClass(lookup_name));
    }
  }

  template <class T>
  int ClassLoader<T>::unloadClassLibraryInternal(const std::string& library_path)
  /***************************************************************************/
  {
    return(lowlevel_class_loader_.unloadLibrary(library_path));
  }

}




#endif
