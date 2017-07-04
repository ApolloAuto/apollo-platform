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

#ifndef PLUGINLIB_CLASS_LIST_MACROS_H_
#define PLUGINLIB_CLASS_LIST_MACROS_H_

#include <class_loader/class_loader.h> 

/** 
 * @macro This version was deprecated in favor of PLUGINLIB_DECLARE_CLASS
 * @param - class_name - An alias for the class (no special characters allowed)  (IGNORED AS OF PLUGINLIB 1.9)
 * @param - class_type - The real class name with namespace qualifier (e.g. Animals::Lion)
 * @param - base_class_type - The real base class type from which class_type inherits
 */
#define PLUGINLIB_REGISTER_CLASS(class_name, class_type, base_class_type) \
  CLASS_LOADER_REGISTER_CLASS_WITH_MESSAGE(class_type, base_class_type, "In file " __FILE__ " pluginlib WARNING: PLUGINLIB_REGISTER_CLASS is deprecated, please use PLUGINLIB_EXPORT_CLASS instead. You can run the script 'plugin_macro_update' provided with pluginlib in your package source folder to automatically and recursively update legacy macros. Base = base_class_type, Derived = derived_class_type")

/** 
 * @macro This version is the most in use and requires package name in addition to fields in PLUGINLIB_REGISTER_CLASS 
 * @param - pkg - The package that exports the plugin (IGNORED AS OF PLUGINLIB 1.9)
 * @param - class_name - An alias for the class (no special characters allowed)  (IGNORED AS OF PLUGINLIB 1.9)
 * @param - class_type - The real class name with namespace qualifier (e.g. Animals::Lion)
 * @param - base_class_type - The real base class type from which class_type inherits
 */
#define PLUGINLIB_DECLARE_CLASS(pkg, class_name, class_type, base_class_type) \
  CLASS_LOADER_REGISTER_CLASS_WITH_MESSAGE(class_type, base_class_type, "pluginlib WARNING: In file " __FILE__ " PLUGINLIB_DECLARE_CLASS is deprecated, please use PLUGINLIB_EXPORT_CLASS instead. You can run the script 'plugin_macro_update' provided with pluginlib in your package source folder to automatically and recursively update legacy macros.  Base = base_class_type, Derived = derived_class_type")            
   
/** 
 * @macro This version was only made possible with pluginlib 1.9 series. It's the easiest to use and now the official way of exporting classes.
 * @param - class_type - The real class name with namespace qualifier (e.g. Animals::Lion)
 * @param - base_class_type - The real base class type from which class_type inherits
 */
#define PLUGINLIB_EXPORT_CLASS(class_type, base_class_type) \
  CLASS_LOADER_REGISTER_CLASS(class_type, base_class_type);

#endif
