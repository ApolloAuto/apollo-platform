/*
 * Copyright (C) 2008, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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

#include "rospack/rospack.h"
#include "utils.h"
#include "rospack_cmdline.h"

#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <algorithm>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

namespace po = boost::program_options;

namespace rospack
{

bool parse_args(int argc, char** argv,
                rospack::Rosstackage& rp,
                po::variables_map& vm);

bool
rospack_run(int argc, char** argv, rospack::Rosstackage& rp, std::string& output)
{
  po::variables_map vm;

  if(!parse_args(argc, argv, rp, vm))
    return false;

  bool quiet = (vm.count("quiet")==1);
  rp.setQuiet(quiet);

  std::string command;
  std::string package;
  bool package_given = false;
  bool deps_only = false;
  std::string lang;
  std::string attrib;
  std::string top;
  std::string target;
  bool zombie_only = false;
  std::string length_str;
  int length;
  if(vm.count("command"))
    command = vm["command"].as<std::string>();

  if(vm.count("-h") && command.empty())
    command = "help";

  if(!command.size())
  {
    rp.logError( std::string("no command given.  Try '") + rp.getName() + " help'");
    return true;
  }
  // For some commands, we force a crawl.  Definitely anything that does a
  // depends-on calculation.
  bool force = false;
  if((command == "profile") ||
     (command == "depends-on") ||
     (command == "depends-on1") ||
     (command == "langs") ||
     (command == "list-duplicates"))
    force = true;

  if(vm.count("package"))
  {
    package = vm["package"].as<std::string>();
    package_given = true;
  }
  else
  {
    // try to determine package from directory context
    rp.inStackage(package);
  }
  if(vm.count("deps-only"))
    deps_only = true;
  if(vm.count("lang"))
    lang = vm["lang"].as<std::string>();
  if(vm.count("attrib"))
    attrib = vm["attrib"].as<std::string>();
  if(vm.count("top"))
    top = vm["top"].as<std::string>();
  if(vm.count("target"))
    target = vm["target"].as<std::string>();
  if(vm.count("zombie-only"))
    zombie_only = true;
  if(vm.count("length"))
  {
    length_str = vm["length"].as<std::string>();
    length = atoi(length_str.c_str());
  }
  else
  {
    if(zombie_only)
      length = -1;
    else
      length = 20;
  }

  // COMMAND: help
  if(command == "help" || vm.count("help"))
  {
    if(package_given || (command != "help" && vm.count("help"))) {
      if (command == "help") {
        command = vm["package"].as<std::string>();
      }
      output.append("Usage: rospack ");
      output.append(command);
      if(command == "help")
        output.append("[command]\n\nPrint help message.");
      else if(command == "find")
        output.append("\n\nPrint absolute path to the package");
      else if(command == "list")
        output.append("\n\nPrint newline-separated list <package-name> <package-dir> for all packages.");
      else if(command == "list-names")
        output.append("\n\nPrint newline-separated list of packages names for all packages.");
      else if(command == "list-duplicates")
        output.append("\n\nPrint newline-separated list of names of packages that are found more than once during the search.");
      else if(command == "langs")
        output.append("\n\nPrint space-separated list of available language-specific client libraries.");
      else if(command == "depends" || command == "deps")
        output.append("[package]\n\nPrint newline-separated, ordered list of all dependencies of the package.");
      else if(command == "depends1" || command == "deps1")
        output.append("[package]\n\nPrint newline-separated, ordered list of immediate dependencies of the package.");
      else if(command == "depends-manifest" || command == "deps-manifest")
        output.append("[package]\n\nPrint space-separated, ordered list of manifest.xml files for all dependencies of the package. Used internally by rosbuild.");
      else if(command == "depends-indent" || command == "deps-indent")
        output.append("[package]\n\nPrint newline-separated, indented list of the entire dependency chain for the package.");
      else if(command == "depends-why" || command == "deps-why")
        output.append("--target=TARGET [package]\n\nPrint newline-separated presentation of all dependency chains from the package to TARGET. ");
      else if(command == "depends-msgsrv" || command == "deps-msgsrv")
        output.append("[package]\n\nPrint space-separated list of message-generation marker files for all dependencies of the package.  Used internally by rosbuild.");
      else if(command == "rosdep" || command == "rosdeps")
        output.append("[package]\n\nPrint newline-separated list of all [rosdep] tags from the manifest.xml of the package and all of its dependencies.");
      else if(command == "rosdep0" || command == "rosdeps0")
        output.append("[package]\n\nPrint newline-separated list of all [rosdep] tags from the manifest.xml of just the package itself.");
      else if(command == "vcs")
        output.append("[package]\n\nPrint newline-separated list of all [versioncontrol] tags from the manifest.xml of the package and all of its dependencies.");
      else if(command == "vcs0")
        output.append("[package]\n\nPrint newline-separated list of all [versioncontrol] tags from the manifest.xml of just the package itself.");
      else if(command == "depends-on")
        output.append("[package]\n\nPrint newline-separated list of all packages that depend on the package. ");
      else if(command == "depends-on1")
        output.append("[package]\n\nPrint newline-separated list of all packages that directly depend on the package.");
      else if(command == "export")
        output.append("[--deps-only] --lang=<lang> --attrib=<attrib> [package]\n\nPrint Space-separated list of [export][LANGUAGE ATTRIBUTE=\"\"/][/export] values from the manifest.xml of the package and its dependencies.\n\nIf --deps-only is provided, then the package itself is excluded.");
      else if(command == "plugins")
        output.append("--attrib=<attrib> [--top=<toppkg>] [package]\n\nExamine packages that depend directly on the given package, giving name and the exported attribute with the name <attrib>\n\nIf --top=<toppkg> is given, then in addition to depending directly on the given package, to be scanned for exports, a package must also be a dependency of <toppkg>, or be <toppkg> itself.");
      else if(command == "cflags-only-I")
        output.append("[--deps-only] [package]\n\nPrint Space-separated list of [export][LANGUAGE ATTRIBUTE=\"\"/][/export] values from the manifest.xml of the package and its dependencies.\n\nIf --deps-only is provided, then the package itself is excluded.");
      else if(command == "cflags-only-other")
        output.append("[--deps-only] [package]\n\nPrint space-separated list of export/cpp/cflags that don't start with -I.\n\nIf --deps-only is provided, then the package itself is excluded.");
      else if(command == "libs-only-L")
        output.append("[--deps-only] [package]\n\nPrint space-separated list of export/cpp/libs that start with -L.\n\nIf --deps-only is provided, then the package itself is excluded.");
      else if(command == "libs-only-l")
        output.append("[--deps-only] [package]\n\nPrint space-separated list of export/cpp/libs that start with -l.\n\nIf --deps-only is provided, then the package itself is excluded.");
      else if(command == "libs-only-other")
        output.append("[--deps-only] [package]\n\nPrint space-separated list of export/cpp/libs that don't start with -l or -L.\n\nIf --deps-only is provided, then the package itself is excluded.");
      else if(command == "profile")
        output.append("[--length=<length>] [--zombie-only]\n\nForce a full crawl of package directories and report the directories that took the longest time to crawl.\n\n--length=N how many directories to display\n\n--zombie-only Only print directories that do not have any manifests.");
      output.append("\n");
    } else {
        output.append(rp.usage());
    }
    return true;
  }

  std::vector<std::string> search_path;
  if(!rp.getSearchPathFromEnv(search_path))
    return false;

  // COMMAND: profile
  if(command == "profile")
  {
    if(package_given || target.size() || top.size() ||
       deps_only || lang.size() || attrib.size())
    {
      rp.logError( "invalid option(s) given");
      return false;
    }
    std::vector<std::string> dirs;
    if(rp.profile(search_path, zombie_only, length, dirs))
      return false;
    for(std::vector<std::string>::const_iterator it = dirs.begin();
        it != dirs.end();
        ++it)
      output.append((*it) + "\n");
    return true;
  }

  // We crawl here because profile (above) does its own special crawl.
  rp.crawl(search_path, force);

  // COMMAND: find [package]
  if(command == "find")
  {
    if(!package.size())
    {
      rp.logError(std::string("no ") + rp.get_manifest_type() + " given");
      return false;
    }
    if(target.size() || top.size() || length_str.size() ||
       zombie_only || deps_only || lang.size() || attrib.size())
    {
      rp.logError( "invalid option(s) given");
      return false;
    }
    std::string path;
    if(!rp.find(package, path))
      return false;
    output.append(path + "\n");
    return true;
  }
  // COMMAND: list
  else if(command == "list")
  {
    if(package_given || target.size() || top.size() || length_str.size() ||
       zombie_only || deps_only || lang.size() || attrib.size())
    {
      rp.logError( "invalid option(s) given");
      return false;
    }
    std::set<std::pair<std::string, std::string> > list;
    rp.list(list);
    for(std::set<std::pair<std::string, std::string> >::const_iterator it = list.begin();
        it != list.end();
        ++it)
    {
      output.append(it->first + " " + it->second + "\n");
    }
    return true;
  }
  // COMMAND: list-names
  else if(command == "list-names")
  {
    if(package_given || target.size() || top.size() || length_str.size() ||
       zombie_only || deps_only || lang.size() || attrib.size())
    {
      rp.logError( "invalid option(s) given");
      return false;
    }
    std::set<std::pair<std::string, std::string> > list;
    rp.list(list);
    for(std::set<std::pair<std::string, std::string> >::const_iterator it = list.begin();
        it != list.end();
        ++it)
    {
      output.append(it->first + "\n");
    }
    return true;
  }
  // COMMAND: list-duplicates
  else if(command == "list-duplicates")
  {
    if(package_given || target.size() || top.size() || length_str.size() ||
       zombie_only || deps_only || lang.size() || attrib.size())
    {
      rp.logError( "invalid option(s) given");
      return false;
    }
    std::map<std::string, std::vector<std::string> > dups;
    rp.listDuplicatesWithPaths(dups);
    // if there are dups, list-duplicates prints them and returns non-zero
    for(std::map<std::string, std::vector<std::string> >::const_iterator it = dups.begin();
        it != dups.end();
        ++it)
    {
      output.append(it->first + "\n");
      for(std::vector<std::string>::const_iterator jt = it->second.begin();
          jt != it->second.end();
          ++jt)
      {
        output.append("- " + *jt + "\n");
      }
    }
    return true;
  }
  // COMMAND: langs
  else if(rp.getName() == ROSPACK_NAME && command == "langs")
  {
    if(package_given || target.size() || top.size() || length_str.size() ||
       zombie_only || deps_only || lang.size() || attrib.size())
    {
      rp.logError( "invalid option(s) given");
      return false;
    }
    std::vector<std::string> deps;
    if(!rp.depsOn("roslang", true, deps))
      return false;
    const char* ros_lang_disable;
    if((ros_lang_disable = getenv("ROS_LANG_DISABLE")))
    {
      std::vector<std::string> disable_langs;
    // I can't see that boost filesystem has an elegant cross platform
    // representation for this anywhere like qt/python have.
    #if defined(WIN32)
      const char *path_delim = ";";
    #else //!defined(WIN32)
      const char *path_delim = ":";
    #endif
      boost::split(disable_langs, ros_lang_disable,
                   boost::is_any_of(path_delim),
                   boost::token_compress_on);
      std::vector<std::string>::iterator it = deps.begin();
      while(it != deps.end())
      {
        if(std::find(disable_langs.begin(), disable_langs.end(), *it) !=
           disable_langs.end())
          it = deps.erase(it);
        else
          ++it;
      }
    }
    for(std::vector<std::string>::const_iterator it = deps.begin();
        it != deps.end();
        ++it)
    {
      if(it != deps.begin())
        output.append(" ");
      output.append(*it);
    }
    output.append("\n");
    return true;
  }
  // COMMAND: depends [package] (alias: deps)
  else if(command == "depends" || command == "deps" ||
          command == "depends1" || command == "deps1")
  {
    if(!package.size())
    {
      rp.logError(std::string("no ") + rp.get_manifest_type() + " given");
      return false;
    }
    if(target.size() || top.size() || length_str.size() ||
       zombie_only || deps_only || lang.size() || attrib.size())
    {
      rp.logError( "invalid option(s) given");
      return false;
    }
    std::vector<std::string> deps;
    if(!rp.deps(package, (command == "depends1" || command == "deps1"), deps))
      return false;
    for(std::vector<std::string>::const_iterator it = deps.begin();
        it != deps.end();
        ++it)
      output.append(*it + "\n");
    return true;
  }
  // COMMAND: depends-manifests [package] (alias: deps-manifests)
  else if(command == "depends-manifests" || command == "deps-manifests")
  {
    if(!package.size())
    {
      rp.logError(std::string("no ") + rp.get_manifest_type() + " given");
      return false;
    }
    if(target.size() || top.size() || length_str.size() ||
       zombie_only || deps_only || lang.size() || attrib.size())
    {
      rp.logError( "invalid option(s) given");
      return false;
    }
    std::vector<std::string> manifests;
    if(!rp.depsManifests(package, false, manifests))
      return false;
    for(std::vector<std::string>::const_iterator it = manifests.begin();
        it != manifests.end();
        ++it)
    {
      if(it != manifests.begin())
        output.append(" ");
      output.append(*it);
    }
    output.append("\n");
    return true;
  }
  // COMMAND: depends-msgsrv [package] (alias: deps-msgsrv)
  else if(rp.getName() == ROSPACK_NAME &&
          (command == "depends-msgsrv" || command == "deps-msgsrv"))
  {
    if(!package.size())
    {
      rp.logError( "no package given");
      return false;
    }
    if(target.size() || top.size() || length_str.size() ||
       zombie_only || deps_only || lang.size() || attrib.size())
    {
      rp.logError( "invalid option(s) given");
      return false;
    }
    std::vector<std::string> gens;
    if(!rp.depsMsgSrv(package, false, gens))
      return false;
    for(std::vector<std::string>::const_iterator it = gens.begin();
        it != gens.end();
        ++it)
    {
      if(it != gens.begin())
        output.append(" ");
      output.append(*it);
    }
    output.append("\n");
    return true;
  }
  // COMMAND: depends-indent [package] (alias: deps-indent)
  else if(command == "depends-indent" || command == "deps-indent")
  {
    if(!package.size())
    {
      rp.logError(std::string("no ") + rp.get_manifest_type() + " given");
      return false;
    }
    if(target.size() || top.size() || length_str.size() ||
       zombie_only || deps_only || lang.size() || attrib.size())
    {
      rp.logError( "invalid option(s) given");
      return false;
    }
    std::vector<std::string> deps;
    if(!rp.depsIndent(package, false, deps))
      return false;
    for(std::vector<std::string>::const_iterator it = deps.begin();
        it != deps.end();
        ++it)
      output.append(*it + "\n");
    return true;
  }
  // COMMAND: depends-why [package] (alias: deps-why)
  else if(command == "depends-why" || command == "deps-why")
  {
    if(!package.size() || !target.size())
    {
      rp.logError(std::string("no ") + rp.get_manifest_type() + " or target given");
      return false;
    }
    if(top.size() || length_str.size() ||
       zombie_only || deps_only || lang.size() || attrib.size())
    {
      rp.logError( "invalid option(s) given");
      return false;
    }
    std::string why_output;
    if(!rp.depsWhy(package, target, why_output))
      return false;
    output.append(why_output);
    return true;
  }
  // COMMAND: rosdep [package] (alias: rosdeps)
  // COMMAND: rosdep0 [package] (alias: rosdeps0)
  else if(rp.getName() == ROSPACK_NAME &&
          (command == "rosdep" || command == "rosdeps" ||
           command == "rosdep0" || command == "rosdeps0"))
  {
    if(!package.size())
    {
      rp.logError( "no package given");
      return false;
    }
    if(target.size() || top.size() || length_str.size() ||
       zombie_only || deps_only || lang.size() || attrib.size())
    {
      rp.logError( "invalid option(s) given");
      return false;
    }
    std::set<std::string> rosdeps;
    if(!rp.rosdeps(package, (command == "rosdep0" || command == "rosdeps0"), rosdeps))
      return false;
    for(std::set<std::string>::const_iterator it = rosdeps.begin();
        it != rosdeps.end();
        ++it)
      output.append(*it + "\n");
    return true;
  }
  // COMMAND: vcs [package]
  // COMMAND: vcs0 [package]
  else if(rp.getName() == ROSPACK_NAME &&
          (command == "vcs" || command == "vcs0"))
  {
    if(!package.size())
    {
      rp.logError( "no package given");
      return false;
    }
    if(target.size() || top.size() || length_str.size() ||
       zombie_only || deps_only || lang.size() || attrib.size())
    {
      rp.logError( "invalid option(s) given");
      return false;
    }
    std::vector<std::string> vcs;
    if(!rp.vcs(package, (command == "vcs0"), vcs))
      return false;
    for(std::vector<std::string>::const_iterator it = vcs.begin();
        it != vcs.end();
        ++it)
      output.append(*it + "\n");
    return true;
  }
  // COMMAND: depends-on [package]
  // COMMAND: depends-on1 [package]
  else if(command == "depends-on" || command == "depends-on1")
  {
    if(!package.size())
    {
      rp.logError(std::string("no ") + rp.get_manifest_type() + " given");
      return false;
    }
    if(target.size() || top.size() || length_str.size() ||
       zombie_only || deps_only || lang.size() || attrib.size())
    {
      rp.logError( "invalid option(s) given");
      return false;
    }
    std::vector<std::string> deps;
    if(!rp.depsOn(package, (command == "depends-on1"), deps))
      return false;
    for(std::vector<std::string>::const_iterator it = deps.begin();
        it != deps.end();
        ++it)
      output.append(*it + "\n");
    return true;
  }
  // COMMAND: export [--deps-only] --lang=<lang> --attrib=<attrib> [package]
  else if(rp.getName() == ROSPACK_NAME && command == "export")
  {
    if(!package.size() || !lang.size() || !attrib.size())
    {
      rp.logError( "no package / lang / attrib given");
      return false;
    }
    if(target.size() || top.size() || length_str.size() || zombie_only)
    {
      rp.logError( "invalid option(s) given");
      return false;
    }
    std::vector<std::string> flags;
    if(!rp.exports(package, lang, attrib, deps_only, flags))
      return false;
    for(std::vector<std::string>::const_iterator it = flags.begin();
        it != flags.end();
        ++it)
    {
      if(it != flags.begin())
        output.append(" ");
      output.append(*it);
    }
    output.append("\n");
    return true;
  }
  // COMMAND: plugins --attrib=<attrib> [--top=<toppkg>] [package]
  else if(rp.getName() == ROSPACK_NAME && command == "plugins")
  {
    if(!package.size() || !attrib.size())
    {
      rp.logError( "no package / attrib given");
      return false;
    }
    if(target.size() || length_str.size() || zombie_only)
    {
      rp.logError( "invalid option(s) given");
      return false;
    }
    std::vector<std::string> flags;
    if(!rp.plugins(package, attrib, top, flags))
      return false;
    for(std::vector<std::string>::const_iterator it = flags.begin();
        it != flags.end();
        ++it)
      output.append(*it + "\n");
    return true;
  }
  // COMMAND: cflags-only-I [--deps-only] [package]
  else if(rp.getName() == ROSPACK_NAME && command == "cflags-only-I")
  {
    if(!package.size())
    {
      rp.logError( "no package given");
      return false;
    }
    if(target.size() || top.size() || length_str.size() || zombie_only)
    {
      rp.logError( "invalid option(s) given");
      return false;
    }
    std::vector<std::pair<std::string, bool> > flags;
    if(!rp.cpp_exports(package, "--cflags-only-I", "cflags", deps_only, flags))
      return false;

    std::string dry_combined;
    std::string wet_combined;
    for(std::vector<std::pair<std::string, bool> >::const_iterator it = flags.begin();
        it != flags.end();
        ++it)
    {
      std::string& combined = it->second ? wet_combined : dry_combined;
      if(!combined.empty())
        combined.append(" ");
      combined.append(it->first);
    }

    std::string dry_result;
    parse_compiler_flags(dry_combined, "-I", true, false, dry_result);
    output.append(dry_result);

    std::string wet_result;
    parse_compiler_flags(wet_combined, "-I", true, false, wet_result);
    if(!dry_result.empty() && !wet_result.empty())
      output.append(" ");
    if(!rp.reorder_paths(wet_result, wet_result))
      return false;
    output.append(wet_result + "\n");
    return true;
  }
  // COMMAND: cflags-only-other [--deps-only] [package]
  else if(rp.getName() == ROSPACK_NAME && command == "cflags-only-other")
  {
    if(!package.size())
    {
      rp.logError( "no package given");
      return false;
    }
    if(target.size() || top.size() || length_str.size() || zombie_only)
    {
      rp.logError( "invalid option(s) given");
      return false;
    }
    std::vector<std::pair<std::string, bool> > flags;
    if(!rp.cpp_exports(package, "--cflags-only-other", "cflags", deps_only, flags))
      return false;
    std::string combined;
    for(std::vector<std::pair<std::string, bool> >::const_iterator it = flags.begin();
        it != flags.end();
        ++it)
    {
      if(it != flags.begin())
        combined.append(" ");
      combined.append(it->first);
    }
    std::string result;
    parse_compiler_flags(combined, "-I", false, false, result);
    output.append(result + "\n");
    return true;
  }
  // COMMAND: libs-only-L [--deps-only] [package]
  else if(rp.getName() == ROSPACK_NAME && command == "libs-only-L")
  {
    if(!package.size())
    {
      rp.logError( "no package given");
      return false;
    }
    if(target.size() || top.size() || length_str.size() || zombie_only)
    {
      rp.logError( "invalid option(s) given");
      return false;
    }
    std::vector<std::pair<std::string, bool> > flags;
    if(!rp.cpp_exports(package, "--libs-only-L", "lflags", deps_only, flags))
      return false;

    std::string dry_combined;
    std::string wet_combined;
    for(std::vector<std::pair<std::string, bool> >::const_iterator it = flags.begin();
        it != flags.end();
        ++it)
    {
      std::string& combined = it->second ? wet_combined : dry_combined;
      if(!combined.empty())
        combined.append(" ");
      combined.append(it->first);
    }

    std::string dry_result;
    parse_compiler_flags(dry_combined, "-L", true, false, dry_result);
    output.append(dry_result);

    std::string wet_result;
    parse_compiler_flags(wet_combined, "-L", true, false, wet_result);
    if(!dry_result.empty() && !wet_result.empty())
      output.append(" ");
    if(!rp.reorder_paths(wet_result, wet_result))
      return false;
    output.append(wet_result + "\n");
    return true;
  }
  // COMMAND: libs-only-l [--deps-only] [package]
  else if(rp.getName() == ROSPACK_NAME && command == "libs-only-l")
  {
    if(!package.size())
    {
      rp.logError( "no package given");
      return false;
    }
    if(target.size() || top.size() || length_str.size() || zombie_only)
    {
      rp.logError( "invalid option(s) given");
      return false;
    }
    std::vector<std::pair<std::string, bool> > flags;
    if(!rp.cpp_exports(package, "--libs-only-l", "lflags", deps_only, flags))
      return false;
    std::string combined;
    for(std::vector<std::pair<std::string, bool> >::const_iterator it = flags.begin();
        it != flags.end();
        ++it)
    {
      if(it != flags.begin())
        combined.append(" ");
      combined.append(it->first);
    }
    std::string result;
    parse_compiler_flags(combined, "-l", true, true, result);
    output.append(result + "\n");
    return true;
  }
  // COMMAND: libs-only-other [--deps-only] [package]
  else if(rp.getName() == ROSPACK_NAME && command == "libs-only-other")
  {
    if(!package.size())
    {
      rp.logError( "no package given");
      return false;
    }
    if(target.size() || top.size() || length_str.size() || zombie_only)
    {
      rp.logError( "invalid option(s) given");
      return false;
    }
    std::vector<std::pair<std::string, bool> > flags;
    if(!rp.cpp_exports(package, "--libs-only-other", "lflags", deps_only, flags))
      return false;
    std::string combined;
    for(std::vector<std::pair<std::string, bool> >::const_iterator it = flags.begin();
        it != flags.end();
        ++it)
    {
      if(it != flags.begin())
        combined.append(" ");
      combined.append(it->first);
    }
    std::string intermediate;
    parse_compiler_flags(combined, "-L", false, false, intermediate);
    std::string result;
    parse_compiler_flags(intermediate, "-l", false, false, result);
    output.append(result + "\n");
    return true;
  }
  // COMMAND: contents [stack]
  else if(rp.getName() == ROSSTACK_NAME && command == "contents")
  {
    if(!package.size())
    {
      rp.logError( "no stack given");
      return false;
    }
    if(target.size() || top.size() || length_str.size() ||
       zombie_only || deps_only || lang.size() || attrib.size())
    {
      rp.logError( "invalid option(s) given");
      return false;
    }

    std::set<std::string> packages;
    rp.contents(package, packages);
    for(std::set<std::string>::const_iterator it = packages.begin();
        it != packages.end();
        ++it)
      output.append(*it + "\n");
    return true;
  }
  // COMMAND: contains [package]
  else if(rp.getName() == ROSSTACK_NAME &&
          ((command == "contains") || (command == "contains-path")))
  {
    if(!package.size())
    {
      rp.logError( "no package given");
      return false;
    }
    if(target.size() || top.size() || length_str.size() ||
       zombie_only || deps_only || lang.size() || attrib.size())
    {
      rp.logError( "invalid option(s) given");
      return false;
    }
    std::string name, path;
    if(!rp.contains(package, name, path))
      return false;
    if(command == "contains")
      output.append(name + "\n");
    else // command == "contains-path"
      output.append(path + "\n");
    return true;
  }
  else
  {
    rp.logError(std::string("command ") + command + " not implemented");
    return false;
  }
}

bool
parse_args(int argc, char** argv,
           rospack::Rosstackage& rp, po::variables_map& vm)
{
  po::options_description desc("Allowed options");
  desc.add_options()
          ("command", po::value<std::string>(), "command")
          ("package", po::value<std::string>(), "package")
          ("target", po::value<std::string>(), "target")
          ("deps-only", "deps-only")
          ("lang", po::value<std::string>(), "lang")
          ("attrib", po::value<std::string>(), "attrib")
          ("top", po::value<std::string>(), "top")
          ("length", po::value<std::string>(), "length")
          ("zombie-only", "zombie-only")
          ("help", "help")
          ("-h", "help")
          ("quiet,q", "quiet");

  po::positional_options_description pd;
  pd.add("command", 1).add("package", 1);
  try
  {
    po::store(po::command_line_parser(argc, argv).options(desc).positional(pd).run(), vm);
  }
  catch(boost::program_options::error e)
  {
    rp.logError( std::string("failed to parse command-line options: ") + e.what());
    return false;
  }
  po::notify(vm);

  return true;
}

}
