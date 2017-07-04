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

/**

\mainpage

\section overview Overview

\b librospack is the ROS stackage (stack/package) management library.

librospack is part dpkg, part pkg-config.  The main function of librospack is
to crawl through the stackages in ROS_ROOT and ROS_PACKAGE_PATH, read and
parse the \b manifest for each stackage, and assemble a complete
dependency tree for all stackages.

Using this tree, librospack can answer a number of queries about stackages and
their dependencies.  Common queries include:
 - find : return the absolute path to a stackage
 - depends : return a list of all of a stackage's dependencies
 - depends-on : return a list of stackages that depend on the given stackage
 - export : return flags necessary for building and linking against a package

librospack is intended to be cross-platform.

\subsection crawling Crawling algorithm
librospack crawls in the following order: the directory ROS_ROOT, followed by
the colon-separated list of directories ROS_PACKAGE_PATH, in the order they
are listed.

During the crawl, librospack examines the contents of each directory, looking
for a file called @b package.xml / manifest.xml (for packages) or @b stack.xml (for stacks).
If such a file is found, the directory
containing it is considered to be a ROS stackage, with the stackage name
equal to the directory name.  The crawl does not descend further once a
manifest is found (i.e., stackages cannot be nested inside one another).

If a manifest is not found in a given directory, each subdirectory
is searched.  This subdirectory search is prevented if a file called @b
rospack_nosubdirs is found.  The directory itself is still searched for a
manifest, but its subdirectories are not crawled.

If multiple stackages by the same name exist within the search path, the
first one found wins.  It is strongly recommended that you keep stackages by
the same name in separate trees, each having its own element within
ROS_PACKAGE_PATH.  That way, you can deterministically control the search
order by the way that you specify ROS_PACKAGE_PATH.  The search order
within a given element of ROS_PACKAGE_PATH can be unpredictably affected by
the details of how files are laid out on disk.

\subsection efficiency Efficiency considerations
librospack re-parses the manifest files and rebuilds the dependency tree
on each execution.  However, it maintains a cache of stackage directories in
ROS_HOME/rospack_cache (or ROS_HOME/rosstack_cache).
This cache is updated whenever there is a cache
miss, or when the cache is 60 seconds old.  You can change this timeout by
setting the environment variable ROS_CACHE_TIMEOUT, in seconds.  Set it to
0.0 to force a cache rebuild on every invocation of librospack.

librospack's performance can be adversely affected by the presence of very
broad and/or deep directory structures that don't contain manifest files.
If such directories are in librospack's search path, it can spend a lot of
time crawling them only to discover that there are no stackages to be found.
You can prevent this latency by creating a @b rospack_nosubdirs file in
such directories. If librospack seems to be running annoyingly slowly, you
can call profile(), which will print out the slowest trees
to crawl.

\subsection dependencies Minimal dependencies
Because librospack is the tool that determines dependencies, it must
have minimal dependencies.  librospack contains a copy of the TinyXML library,
and depends on Boost.  If gtest is available, then tests can be run.

\section codeapi Code API

librospack is used primarily in command-line tools (rospack and rosstack),
but can be used in other contexts.  See the API documentation for Rospack
and Rosstack.
*/

#ifndef ROSPACK_ROSPACK_H
#define ROSPACK_ROSPACK_H

#include <boost/tr1/unordered_set.hpp>
#include <boost/tr1/unordered_map.hpp>
#include <list>
#include <map>
#include <set>
#include <string>
#include <vector>
#include "macros.h"

//#ifdef ROSPACK_API_BACKCOMPAT_V1
#if 1 // def ROSPACK_API_BACKCOMPAT_V1
  #include "rospack/rospack_backcompat.h"
#endif


namespace rospack
{

typedef enum
{
  POSTORDER,
  PREORDER
} traversal_order_t;

// Forward declarations
class Stackage;
class DirectoryCrawlRecord;

/**
 * @brief The base class for package/stack ("stackage") crawlers.  Users of the library should
 * use the functionality provided here through one of the derived classes,
 * Rosstack or Rospack.
 */
class ROSPACK_DECL Rosstackage
{
  private:
    std::string manifest_name_;
    std::string cache_prefix_;
    bool crawled_;
    std::string name_;
    std::string tag_;
    bool quiet_;
    std::vector<std::string> search_paths_;
    std::tr1::unordered_map<std::string, std::vector<std::string> > dups_;
    std::tr1::unordered_map<std::string, Stackage*> stackages_;
    Stackage* findWithRecrawl(const std::string& name);
    void log(const std::string& level, const std::string& msg, bool append_errno);
    void clearStackages();
    void addStackage(const std::string& path);
    void crawlDetail(const std::string& path,
                     bool force,
                     int depth,
                     bool collect_profile_data,
                     std::vector<DirectoryCrawlRecord*>& profile_data,
                     std::tr1::unordered_set<std::string>& profile_hash);
    bool isStackage(const std::string& path);
    void loadManifest(Stackage* stackage);
    void computeDeps(Stackage* stackage, bool ignore_errors=false, bool ignore_missing=false);
    void computeDepsInternal(Stackage* stackage, bool ignore_errors, const std::string& depend_tag, bool ignore_missing=false);
    bool isSysPackage(const std::string& pkgname);
    void gatherDeps(Stackage* stackage, bool direct,
                    traversal_order_t order,
                    std::vector<Stackage*>& deps,
                    bool no_recursion_on_wet=false);
    void gatherDepsFull(Stackage* stackage, bool direct,
                        traversal_order_t order, int depth,
                        std::tr1::unordered_set<Stackage*>& deps_hash,
                        std::vector<Stackage*>& deps,
                        bool get_indented_deps,
                        std::vector<std::string>& indented_deps,
                        bool no_recursion_on_wet=false);
    std::string getCachePath();
    std::string getCacheHash();
    bool readCache();
    void writeCache();
    FILE* validateCache();
    bool expandExportString(Stackage* stackage,
                            const std::string& instring,
                            std::string& outstring);
    void depsWhyDetail(Stackage* from,
                       Stackage* to,
                       std::list<std::list<Stackage*> >& acc_list);

    void initPython();

  protected:
    /**
     * @brief Constructor, only used by derived classes.
     * @param manifest_name What the manifest is called (e.g., "manifest.xml or stack.xml")
     * @param cache_prefix What the cache is called (e.g., "rospack_cache" or "rosstack_cache") excluding the appended search path hash
     * @param name Name of the tool we're building (e.g., "rospack" or "rosstack")
     * @param tag Name of the attribute we look for in a "depend" tag in a
     *            manifest (e.g., "package" or "stack")
     */
    Rosstackage(const std::string& manifest_name,
                const std::string& cache_prefix,
                const std::string& name,
                const std::string& tag);

  public:
    /**
     * @brief Destructor.
     */
    virtual ~Rosstackage();

    /**
     * @brief Usage string, to be overridden by derived classes.
     * @return Command-line usage statement.
     */
    virtual const char* usage() { return ""; }
    /**
     * @brief Crawl the filesystem, accumulating a database of
     *        stackages.  May read results from a cache file instead
     *        of crawling.  This method should be called before any
     *        making any queries (find, list, etc.).
     * @param search_path List of directories to crawl, in precenence
     *                    order.  Directories should be absolute paths.
     *                    It's passed by value to allow callers (e.g.,
     *                    find()) to safely pass in search_paths_.
     * @param force If true, then crawl even if the cache looks valid
     */
    void crawl(std::vector<std::string> search_path, bool force);
    /**
     * @brief Is the current working directory a stackage?
     * @param name If in a stackage, then the stackage's name is written here.
     * @return True if the current working directory contains a manifest
     *         file
     */
    bool inStackage(std::string& name);
    /**
     * @brief Control warning and error console output.
     * @param quiet If true, then don't output any warnings or errors to
     *              console.  If false, then output warnings and errors to
     *              stderr (default behavior).
     */
    void setQuiet(bool quiet);
    /**
     * @brief Get the name of the tool that's in use (e.g., "rospack" or "rosstack")
     * @return The name of the tool.
     */
    const std::string& getName() {return name_;}
    /**
     * @brief Helper method to construct a directory search path by looking
     *        at relevant environment variables.  The value of ROS_ROOT goes
     *        first, followed by each element of a colon-separated
     *        ROS_PACKAGE_PATH.
     * @param sp The computed search path is written here.
     * @return True if a search path was computed, false otherwise (e.g., ROS_ROOT not set).
     */
    bool getSearchPathFromEnv(std::vector<std::string>& sp);
    /**
     * @brief Look for a stackage.
     * @param name The stackage to look for.
     * @param path If found, the absolute path to the stackage is written here.
     * @return True if the stackage is found, false otherwise.
     */
    bool find(const std::string& name, std::string& path);
    /**
     * @brief Compute the packages that are contained in a stack.
     * @param name The stack to work on.
     * @param packages The stack's constituent packages are written here.
     * @return True if the contents could be computed, false otherwise.
     */
    bool contents(const std::string& name, std::set<std::string>& packages);
    /**
     * @brief Find the stack that contains a package.
     * @param name The package to work on.
     * @param stack If the containing stack is found, its name is written here.
     * @param path If the containing stack is found, its absolute path is written here.
     * @return True if the containing stack could be found, false otherwise.
     */
    bool contains(const std::string& name,
                  std::string& stack,
                  std::string& path);

    /**
     * @brief List names and paths of all stackages.
     * @param list Pairs of (name,path) are written here.
     */
    void list(std::set<std::pair<std::string, std::string> >& list);
    /**
     * @brief Identify duplicate stackages.  Forces crawl.
     * @param dups Names of stackages that are found more than once while
     *             crawling are written here.
     */
    void listDuplicates(std::vector<std::string>& dups);
    /**
     * @brief Identify duplicate stackages and provide their paths.  Forces crawl.
     * @param dups Names of stackages that are found more than once while
     *             crawling are mapped to the found paths of these packages.
     */
    void listDuplicatesWithPaths(std::map<std::string, std::vector<std::string> >& dups);
    /**
     * @brief Compute dependencies of a stackage (i.e., stackages that this
     *        stackages depends on).
     * @param name The stackage to work on.
     * @param direct If true, then compute only direct dependencies.  If
     *               false, then compute full (including indirect)
     *               dependencies.
     * @param deps If dependencies are computed, then they're written here.
     * @return True if dependencies were computed, false otherwise.
     */
    bool deps(const std::string& name, bool direct, std::vector<std::string>& deps);
    /**
     * @brief Compute reverse dependencies of a stackage (i.e., stackages
     *        that depend on this stackage).  Forces crawl.
     * @param name The stackage to work on.
     * @param direct If true, then compute only direct dependencies.  If
     *               false, then compute full (including indirect)
     *               dependencies.
     * @param deps If dependencies are computed, then they're written here.
     * @return True if dependencies were computed, false otherwise.
     */
    bool depsOn(const std::string& name, bool direct,
                   std::vector<std::string>& deps);
     /**
     * @brief Compute dependencies of a stackage (i.e., stackages that this
     *        stackages depends on), taking and returning stackage objects..
     * @param name The stackage to work on.
     * @param direct If true, then compute only direct dependencies.  If
     *               false, then compute full (including indirect)
     *               dependencies.
     * @param deps If dependencies are computed, then they're written here in stackage objects.
     * @return True if dependencies were computed, false otherwise.
     */
    bool depsDetail(const std::string& name, bool direct, std::vector<Stackage*>& deps);
    /**
     * @brief Compute reverse dependencies of a stackage (i.e., stackages
     *        that depend on this stackage), taking and returning stackage objects. Forces crawl.
     * @param name The stackage to work on.
     * @param direct If true, then compute only direct dependencies.  If
     *               false, then compute full (including indirect) dependencies.
     * @param deps List of Stackage objects. If dependencies are computed, then they're written here in stackage objects.
     * @return True if dependencies were computed, false otherwise.
     */
    bool depsOnDetail(const std::string& name, bool direct,
                      std::vector<Stackage*>& deps, bool ignore_missing=false);
    /**
     * @brief List the manifests of a stackage's dependencies.  Used by rosbuild.
     * @param name The stackage to work on.
     * @param direct If true, then compute only direct dependencies.  If
     *               false, then compute full (including indirect) dependencies.
     * @param manifests The list of absolute paths to manifests of stackages
     *                  that the given stackage depends on is written here.
     * @return True if the manifest list was computed, false otherwise.
     */
    bool depsManifests(const std::string& name, bool direct,
                       std::vector<std::string>& manifests);
    /**
     * @brief List the marker files in a packages's dependencies that
     * indicate that those packages contain auto-generated message
     * and/or service code.  Used by rosbuild.
     * @param name The package to work on.
     * @param direct If true, then compute only direct dependencies.  If
     *               false, then compute full (including indirect)
     *               dependencies.
     * @param gens The list of absolute paths to marker files (e.g.,
     * "/opt/ros/electric/stacks/ros_comm/messages/std_msgs/msg_gen/generated")
     * is written here.
     * @return True if the list of files was generated, false otherwise.
     */
    bool depsMsgSrv(const std::string& name, bool direct,
                    std::vector<std::string>& gens);
    /**
     * @brief Generate indented list of a stackage's dependencies,
     * including duplicates.  Intended for visual debugging of dependency
     * structures.
     * @param name The stackage to work on.
     * @param direct If true, then compute only direct dependencies.  If
     *               false, then compute full (including indirect)
     *               dependencies.
     * @param deps List of the stackage's dependencies, with leading spaces
     * to indicate depth, is written here.  Print this list to console,
     * with newlines separating each element.  Example output:
@verbatim
roscpp_traits
  cpp_common
cpp_common
rostime
  cpp_common
@endverbatim
     * @return True if the indented dependencies were computed, false
     * otherwise.
     */
    bool depsIndent(const std::string& name, bool direct,
                    std::vector<std::string>& deps);
    /**
     * @brief Compute all dependency chains from one stackage to another.
     * Intended for visual debugging of dependency structures.
     * @param from The stackage that depends on.
     * @param to The stackage that is depended on.
     * @param output A list of dependency chains. Print this list to console,
     * with newlines separating each element.  Example output:
@verbatim
Dependency chains from roscpp to roslib:
* roscpp -> roslib
* roscpp -> std_msgs -> roslib
* roscpp -> rosgraph_msgs -> std_msgs -> roslib
@endverbatim
     * @return True if the dependency chains were computed, false
     * otherwise.
     */
    bool depsWhy(const std::string& from,
                 const std::string& to,
                 std::string& output);
    /**
     * @brief Compute rosdep entries that are declared in manifest of a package
     * and its dependencies.  Used by rosmake.
     * @param name The package to work on.
     * @param direct If true, then compute only direct dependencies.  If
     *               false, then compute full (including indirect)
     *               dependencies.
     * @param rosdeps List of rosdep entries found in the package and its
     * dependencies is written here.
     * @return True if the rosdep list is computed, false otherwise.
     */
    bool rosdeps(const std::string& name, bool direct,
                 std::set<std::string>& rosdeps);
    void _rosdeps(Stackage* stackage, std::set<std::string>& rosdeps, const char* tag_name);
    /**
     * @brief Compute vcs entries that are declared in manifest of a package
     * and its dependencies.  Was used by Hudson build scripts; might not
     * be needed.
     * @param name The package to work on.
     * @param direct If true, then compute only direct dependencies.  If
     *               false, then compute full (including indirect)
     *               dependencies.
     * @param vcs List of vcs entries found in the package and its
     * dependencies is written here.
     * @return True if the vcs list is computed, false otherwise.
     */
    bool vcs(const std::string& name, bool direct,
             std::vector<std::string>& vcs);
    /**
     * @brief Compute cpp exports declared in a package and its dependencies.
     * Used by rosbuild.
     * @param name The package to work on.
     * @param type The option to pass to pkg-config for wet packages.
     * @param attrib The value of the 'attrib' attribute to search for.
     * @param deps_only If true, then only return information from the
     * pacakge's dependencies; if false, then also include the package's
     * own export information.
     * @param flags The pairs of export flags and is-wet are written here.
     * @return True if the flags were computed, false otherwise.
     */
    bool cpp_exports(const std::string& name, const std::string& type,
                 const std::string& attrib, bool deps_only,
                 std::vector<std::pair<std::string, bool> >& flags);
    /**
     * @brief Reorder the paths according to the workspace chaining.
     * @param paths The paths.
     * @param reordered The reordered paths are written here.
     * @return True if the pathswere reordered, false otherwise.
     */
    bool reorder_paths(const std::string& paths, std::string& reordered);
    /**
     * @brief Compute exports declared in a package and its dependencies.
     * Used by rosbuild.
     * @param name The package to work on.
     * @param lang The value of the 'lang' attribute to search for.
     * @param attrib The value of the 'attrib' attribute to search for.
     * @param deps_only If true, then only return information from the
     * pacakge's dependencies; if false, then also include the package's
     * own export information.
     * @param flags The accumulated flags are written here.
     * @return True if the flags were computed, false otherwise.
     */
    bool exports(const std::string& name, const std::string& lang,
                 const std::string& attrib, bool deps_only,
                 std::vector<std::string>& flags);
    /**
     * @brief Compute exports declared in a dry package.
     * @param name The package to work on.
     * @param lang The value of the 'lang' attribute to search for.
     * @param attrib The value of the 'attrib' attribute to search for.
     * @param flags The accumulated flags are written here.
     * @return True if the flags were computed, false otherwise.
     */
    bool exports_dry_package(Stackage* stackage, const std::string& lang,
                         const std::string& attrib,
                         std::vector<std::string>& flags);
    /**
     * @brief Compute exported plugins declared in packages that depend
     * on a package.  Forces crawl. Used by rosbuild and roslib.
     * @param name The package to work on.
     * @param attrib The value of the 'attrib' attribute to search for.
     * @param top If non-empty, then limit the reverse dependency search to
     * packages that 'top' depends on.  Otherwise, examine all packages
     * that depend on 'name'.
     * @param flags The accumulated flags are written here.
     * @return True if the flags were computed, false otherwise.
     */
    bool plugins(const std::string& name, const std::string& attrib,
                 const std::string& top,
                 std::vector<std::string>& flags);
    /**
     * @brief Report on time taken to crawl for stackages.  Intended for
     * use in debugging misconfigured stackage trees.  Forces crawl.
     * @param search_path Directories to search; passed to crawl().
     * @param zombie_only If false, then produce formatted output, with
     * timing information.  Example output:
@verbatim
Full tree crawl took 0.014954 seconds.
Directories marked with (*) contain no manifest.  You may
want to delete these directories.
To get just of list of directories without manifests,
re-run the profile with --zombie-only
-------------------------------------------------------------
0.013423   /opt/ros/electric/stacks
0.002989   /opt/ros/electric/stacks/ros_comm
@endverbatim If true, then produce a list of absolute paths that contain no stackages ("zombies"); these directories can likely be safely deleted.  Example output:
@verbatim
/opt/ros/electric/stacks/pr2_controllers/trajectory_msgs
/opt/ros/electric/stacks/pr2_controllers/trajectory_msgs/msg
@endverbatim
     * @param length Limit on how many directories to include in report
     * (ordered in decreasing order of time taken to crawl).
     * @param dirs Profile output. Print this list to console,
     * with newlines separating each element.
     */
    bool profile(const std::vector<std::string>& search_path,
                 bool zombie_only,
                 int length,
                 std::vector<std::string>& dirs);
    /**
     * @brief Log a warning (usually goes to stderr).
     * @param msg The warning.
     * @param append_errno If true, then append a colon, a space, and
     * the return from 'sterror(errno)'.
     */
    void logWarn(const std::string& msg,
                 bool append_errno = false);
    /**
     * @brief Log a error (usually goes to stderr).
     * @param msg The error.
     * @param append_errno If true, then append a colon, a space, and
     * the return from 'sterror(errno)'.
     */
    void logError(const std::string& msg,
                  bool append_errno = false);

    /*
     * @brief The manifest type.
     * @return Either "package" or "stack".
     */
    virtual std::string get_manifest_type() { return ""; }
};

/**
 * @brief Package crawler.  Create one of these to operate on a package
 * tree.  Call public methods inherited from Rosstackage.
 */
class ROSPACK_DECL Rospack : public Rosstackage
{
  public:
    /**
     * @brief Constructor
     */
    Rospack();
    /**
     * @brief Usage statement.
     * @return Command-line usage for the rospack tool.
     */
    virtual const char* usage();

    virtual std::string get_manifest_type();
};

/**
 * @brief Stack crawler.  Create one of these to operate on a stack
 * tree.  Call public methods inherited from Rosstackage.
 */
class ROSPACK_DECL Rosstack : public Rosstackage
{
  public:
    /**
     * @brief Constructor
     */
    Rosstack();
    /**
     * @brief Usage statement.
     * @return Command-line usage for the rosstack tool.
     */
    virtual const char* usage();

    virtual std::string get_manifest_type();
};

} // namespace rospack

#endif
