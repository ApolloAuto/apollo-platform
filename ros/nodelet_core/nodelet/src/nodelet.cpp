/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id$
 *
 */

/**
@mainpage

\author Radu Bogdan Rusu

@b nodeletcpp is a tool for loading/unloading nodelets to/from a Nodelet manager.
**/
#include <signal.h>
#include <uuid/uuid.h>

#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <bondcpp/bond.h>
#include "nodelet/loader.h"
#include "nodelet/NodeletList.h"
#include "nodelet/NodeletLoad.h"
#include "nodelet/NodeletUnload.h"

std::string genId()
{
  uuid_t uuid;
  uuid_generate_random(uuid);
  char uuid_str[40];
  uuid_unparse(uuid, uuid_str);
  return std::string(uuid_str);
}

class NodeletArgumentParsing
{
  private:
    std::string command_;
    std::string type_;
    std::string name_;
    std::string default_name_;
    std::string manager_;
    std::vector<std::string> local_args_;
    bool is_bond_enabled_;
  
  public:
    //NodeletArgumentParsing() { };
    bool
      parseArgs(int argc, char** argv)
    {
      is_bond_enabled_ = true;
      std::vector<std::string> non_ros_args;
      ros::removeROSArgs (argc, argv, non_ros_args);
      size_t used_args = 0;

      if (non_ros_args.size() > 1)
        command_ = non_ros_args[1];
      else
        return false;

      if (command_ == "load" && non_ros_args.size() > 3)
      {
        type_ = non_ros_args[2];
        manager_ = non_ros_args[3];
        used_args = 4;

        if (non_ros_args.size() > used_args)
        {
          if (non_ros_args[used_args] == "--no-bond")
          {
            is_bond_enabled_ = false;
            ++used_args;
          }
        }
      }

      else if (command_ == "unload" && non_ros_args.size() > 3)
      {
        type_    = "nodelet_unloader";
        name_    = non_ros_args[2];
        manager_ = non_ros_args[3];
        used_args = 4;
      }
      else if (command_ == "standalone" && non_ros_args.size() > 2)
      {
        type_ = non_ros_args[2];
        printf("type is %s\n", type_.c_str());
        used_args = 3;
      }

      if (command_ == "manager")
        used_args = 2;

      for (size_t i = used_args; i < non_ros_args.size(); i++)
        local_args_.push_back(non_ros_args[i]);


      if (used_args > 0) return true;
      else return false;

    };


    std::string getCommand () const { return (command_); }
    std::string getType () const    { return (type_);    }
    std::string getName () const    { return (name_);    }
    std::string getManager() const  { return (manager_); }
    bool isBondEnabled() const { return is_bond_enabled_; }

    std::vector<std::string> getMyArgv () const {return local_args_;};
    std::string getDefaultName()
    {
      std::string s = type_;
      replace(s.begin(), s.end(), '/', '_');
      return s;
    };

};


class NodeletInterface
{
  public:
    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Unload the nodelet */
    bool
      unloadNodelet (const std::string &name, const std::string &manager)
    {
      ROS_INFO_STREAM ("Unloading nodelet " << name << " from manager " << manager);

      std::string service_name = manager + "/unload_nodelet";
      // Check if the service exists and is available
      if (!ros::service::exists (service_name, true))
      {
        // Probably the manager has shut down already, which is fine
        ROS_WARN("Couldn't find service %s, perhaps the manager is already shut down",
                 service_name.c_str());
        return (false);
      }

      ros::ServiceClient client = n_.serviceClient<nodelet::NodeletUnload> (service_name);
      //client.waitForExistence ();

      // Call the service
      nodelet::NodeletUnload srv;
      srv.request.name = name;
      if (!client.call (srv))
      {
        // Maybe service shut down in the meantime, which isn't an error
        if (ros::service::exists(service_name, false))
          ROS_FATAL_STREAM("Failed to unload nodelet '" << name << "` from manager `" << manager << "'");
        return (false);
      }
      return (true);
    }

    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Load the nodelet */
    bool
      loadNodelet (const std::string &name, const std::string &type,
                   const std::string &manager, const std::vector<std::string> &args,
                   const std::string &bond_id)
    {
      ros::M_string remappings = ros::names::getRemappings ();
      std::vector<std::string> sources (remappings.size ()), targets (remappings.size ());
      ROS_INFO_STREAM ("Loading nodelet " << name << " of type " << type << " to manager " << manager << " with the following remappings:");
      int i = 0;
      for (ros::M_string::iterator it = remappings.begin (); it != remappings.end (); ++it, ++i)
      {
        sources[i] = (*it).first;
        targets[i] = (*it).second;
        ROS_INFO_STREAM (sources[i] << " -> " << targets[i]);
      }

      // Get and set the parameters
      XmlRpc::XmlRpcValue param;
      std::string node_name = ros::this_node::getName ();
      n_.getParam (node_name, param);
      n_.setParam (name, param);

      std::string service_name = std::string (manager) + "/load_nodelet";

      // Wait until the service is advertised
      ROS_DEBUG ("Waiting for service %s to be available...", service_name.c_str ());
      ros::ServiceClient client = n_.serviceClient<nodelet::NodeletLoad> (service_name);
      client.waitForExistence ();

      // Call the service
      nodelet::NodeletLoad srv;
      srv.request.name = std::string (name);
      srv.request.type = std::string (type);
      srv.request.remap_source_args = sources;
      srv.request.remap_target_args = targets;
      srv.request.my_argv = args;
      srv.request.bond_id = bond_id;
      if (!client.call (srv))
      {
        ROS_FATAL_STREAM("Failed to load nodelet '" << name << "` of type `" << type << "` to manager `" << manager << "'");
        return false;
      }
      return true;
    }
  private:
    ros::NodeHandle n_;
};

void print_usage(int argc, char** argv)
{
  printf("Your usage: \n");
  for (int i = 0; i < argc; i++)
    printf("%s ", argv[i]);
  printf("\nnodelet usage:\n");
  printf("nodelet load pkg/Type manager [--no-bond]  - Launch a nodelet of type pkg/Type on manager manager\n");
  printf("nodelet standalone pkg/Type   - Launch a nodelet of type pkg/Type in a standalone node\n");
  printf("nodelet unload name manager   - Unload a nodelet by name from manager\n");
  printf("nodelet manager               - Launch a nodelet manager node\n");

};

sig_atomic_t volatile request_shutdown = 0;

void nodeletLoaderSigIntHandler(int)
{
  request_shutdown = 1;
}

// Shutdown can be triggered externally by an XML-RPC call, this is how "rosnode kill"
// works. When shutting down a "nodelet load" we always want to unload the nodelet
// before shutting down our ROS comm channels, so we override the default roscpp
// handler for a "shutdown" XML-RPC call.
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    request_shutdown = 1;
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}

/* ---[ */
int
  main (int argc, char** argv)
{
  NodeletArgumentParsing arg_parser;

  if (!arg_parser.parseArgs(argc, argv))
  {
    print_usage(argc, argv);
    return (-1);
  }
  std::string command = arg_parser.getCommand();


  if (command == "manager")
  {
    ros::init (argc, argv, "manager");
    nodelet::Loader n;
    ros::spin ();
  }
  else if (command == "standalone")
  {
    ros::init (argc, argv, arg_parser.getDefaultName());

    ros::NodeHandle nh;
    nodelet::Loader n(false);
    ros::M_string remappings; //Remappings are already applied by ROS no need to generate them.
    std::string nodelet_name = ros::this_node::getName ();
    std::string nodelet_type = arg_parser.getType();
    if(!n.load(nodelet_name, nodelet_type, remappings, arg_parser.getMyArgv()))
      return -1;

    ROS_DEBUG("Successfully loaded nodelet of type '%s' into name '%s'\n", nodelet_name.c_str(), nodelet_name.c_str());

    ros::spin();
  }
  else if (command == "load")
  {
    ros::init (argc, argv, arg_parser.getDefaultName (), ros::init_options::NoSigintHandler);
    NodeletInterface ni;
    ros::NodeHandle nh;
    std::string name = ros::this_node::getName ();
    std::string type = arg_parser.getType();
    std::string manager = arg_parser.getManager();
    std::string bond_id;
    if (arg_parser.isBondEnabled())
      bond_id = name + "_" + genId();
    bond::Bond bond(manager + "/bond", bond_id);
    if (!ni.loadNodelet(name, type, manager, arg_parser.getMyArgv(), bond_id))
      return -1;

    // Override default exit handlers for roscpp
    signal(SIGINT, nodeletLoaderSigIntHandler);
    ros::XMLRPCManager::instance()->unbind("shutdown");
    ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);
    
    if (arg_parser.isBondEnabled())
      bond.start();
    // Spin our own loop
    ros::AsyncSpinner spinner(1);
    spinner.start();
    while (!request_shutdown)
    {
      if (arg_parser.isBondEnabled() && bond.isBroken())
      {
        ROS_INFO("Bond broken, exiting");
        goto shutdown;
      }
      usleep(100000);
    }
    // Attempt to unload the nodelet before shutting down ROS
    ni.unloadNodelet(name, manager);
    if (arg_parser.isBondEnabled())
      bond.breakBond();
  shutdown:
    ros::shutdown();
  }
  else if (command == "unload")
  {
    ros::init (argc, argv, arg_parser.getDefaultName ());
    std::string name = arg_parser.getName ();
    std::string manager = arg_parser.getManager();
    NodeletInterface ni;
    if (!ni.unloadNodelet(name, manager))
      return -1;
  }
  else
  {
    ros::init(argc, argv, "nodelet");
    ROS_ERROR("Command %s unknown", command.c_str());
    return -1;
  }

  return (0);
}
/* ]--- */
