CMake coding standards
======================

In order to make your ROS packages most maintainable and avoid several
frequent errors, follow the standards below.

Do care about the style your cmake code is written in.
For inspiration, look at the guideline examples here:
`CMake Style recommendations <http://www.cmake.org/cgi-bin/viewcvs.cgi/Modules/readme.txt?root=CMake&view=markup>`_
`KDE cmake coding guidelines <http://techbase.kde.org/Policies/CMake_Coding_Style>`_

**Call catkin_package before any targets**

The following lines must always appear the CMakeLists.txt in this order::

  cmake_minimum_required(VERSION 2.8.3)
  project(myproject)
  find_package(catkin REQUIRED <COMPONENTS ...>)
  catkin_package(<...>)

While there might be additional function calls before catkin_package()
(e.g. for finding other libraries or generating messages)
it must be invoked before any targets are added.

**Use ${PROJECT_NAME} wherever possible**

Use ``${PROJECT_NAME}`` for global variables, targets and labels instead of
repeating the project name manually or using fixed names.

You can use ${PROJECT_NAME} as prefix for global variable names as well, as shown in examples below. Variables become global if they are set using the CACHE argument or created using the option() macro.

After you defined your project name like this::

   project(myproject)

dont do this::

  catkin_add_gtest(test ...)
  add_executable(myproject ...)
  set(use_feature 42 CACHE STRING "description")
  option(use_feature "on or off" OFF)
  macro(xyz) ...

do this instead::

  catkin_add_gtest(${PROJECT_NAME}_test ...)
  add_executable(${PROJECT_NAME} ...)
  set(${PROJECT_NAME}_use_feature 42 CACHE STRING "description")
  option(${PROJECT_NAME}_use_feature "on or off" OFF)
  macro(${PROJECT_NAME}_xyz) ...

This will avoid conflicts between packages and errors due to copy&paste.

**find_package(... REQUIRED)**

Use ``REQUIRED`` on all calls to ``find_package`` if they aren't
actually optional (i.e. you're not going to check ``thing_FOUND``
and enable/disable features).

**Avoid Pkg_config**

CMake allows to use pkg-config::

  find_package(PkgConfig)
  pkg_check_modules(XXX xxx)

However you should (in CMakeLists.txt files) if possible always prefer the method::

  find_package(xxx)

This is usually the most flexible and portable way.

**Keep lists sorted**

Whenever using a list of items (i.e. in find_package(COMPONENTS ...)
or files which should be build or installed) keep them alphabetically
sorted.  This improves readability when looking for specific items.
(There are exceptions which require a specific custom order like the
list of projects inside a stack).

**Lowercase keywords**

Keywords like ``if``, ``for`` etc. are all lowercase.

**Upper arguments**

Use UPPER_CASE for arguments to CMake functions and macros (e.g. REQUIRED, NO_MODULE, QUIET)

**Closing keyword should have empty parenthesis**

The closing keywords like ``endif()`` and ``endforeach()`` should not repeat the condition of the opening keyword.

**Indentation**

Indent all code correctly, i.e. the body of

*    if/else/endif
*    foreach/endforeach
*    while/endwhile
*    macro/endmacro
*    function/endfunction

Use spaces for indenting, 2 spaces preferably

**Variable names**

For custom variables, avoid the following list of suffixes in any CMakeLists.txt, those should only be set by FindXXX.cmake or XXXConfig.cmake files:

* XXX_DEFINITIONS
* XXX_EXECUTABLE
* XXX_INCLUDE_DIRS
* XXX_INCLUDE_DIR
* XXX_YY_INCLUDE_DIR
* XXX_LIBRARIES
* XXX_LIBRARY_DIRS
* XXX_LIBRARY
* XXX_YY_LIBRARY
* XXX_ROOT_DIR
* XXX_FOUND
* XXX_YY_FOUND
* XXX_RUNTIME_LIBRARY_DIRS
* XXX_VERSION_STRING
* XXX_VERSION_MAJOR
* XXX_VERSION_MINOR
* XXX_VERSION_PATCH
* XXX_VERSION_YY
* XXX_WRAP_YY

You may use such variables of course by reading their value after calling find_package(), but do not manually change them.

**Forbidden variables**

Do not set

* CMAKE_CXX_FLAGS
* CMAKE_FIND_ROOT_PATH
* CMAKE_MODULE_PATH

**Conditions and Variables**

Always quote variable that represent a string::

  set(myvar "foo")
  if("${myvar}" STREQUAL "bar")
  # ...
  endif()

Do not quote variable that are booleans::

  set(mybvar ON)
  set(mybvar OFF)
  if(${myvar})
  # ...
  endif()

When storing paths in variables, do NOT have the cmake variables end up with a slash::

  # YES:
  set(_my_path "path/to/foo")
  set(_my_other_path "${_my_path}/${_my_var}")
  # NO:
  set(my_path "path/to/foo/")
  set(_my_other_path "${_my_path}${_my_var}")   # wrong: this is ugly

Use if(DEFINED varname) to check if a variable is set::

  if(DEFINED myvar)
  #  ...
  endif()

Use if(varname) to check it a variable has a non-empty value::

  if(myvar)
  #  ...
  endif()
