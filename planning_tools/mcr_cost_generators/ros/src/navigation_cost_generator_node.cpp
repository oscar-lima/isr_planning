/* 
 * Copyright [2016] <Bonn-Rhein-Sieg University>  
 * 
 * Author: Oscar Lima (olima_84@yahoo.com)
 * 
 * Generate navigation costs for task planning based on distance between locations
 * 
 */

#include <mcr_cost_generators/navigation_cost_generator_node.h>
#include <string>
#include <vector>

NavigationCostGeneratorNode::NavigationCostGeneratorNode() : nh_("~"), is_event_in_received_(false), path_length_(0.0),
is_path_length_received_(false), cost_multiplier_(2.0), self_location_cost_(100.0)
{
    // subscriptions
    sub_event_in_ = nh_.subscribe("event_out", 1, &NavigationCostGeneratorNode::eventInCallBack, this);
    sub_path_length_ = nh_.subscribe("path_length", 1, &NavigationCostGeneratorNode::pathLengthCallBack, this);

    // publications
    pub_trigger_path_length_calc_ = nh_.advertise<std_msgs::String>("event_in", 1);

    // give some time for publishers to register in network
    ros::Duration(1).sleep();

    // querying parameters from parameter server
    getParams();

    // combine locations
    combineLocations();

    // loop over all available locations, calculate path lenght and write cost
    costGenerationLoop();
}

NavigationCostGeneratorNode::~NavigationCostGeneratorNode()
{
    // shut down publishers and subscribers
    sub_event_in_.shutdown();
    sub_path_length_.shutdown();
    pub_trigger_path_length_calc_.shutdown();
}

void NavigationCostGeneratorNode::getParams()
{
    std::string global_frame;
    std::string param_server_namespace;

    nh_.param<std::string>("global_frame", global_frame, "map");
    nh_.param<std::string>("make_plan_service", service_name_, "/move_base/GlobalPlannerWithOrientations/make_plan");
    nh_.param<std::string>("cost_file_path", cost_file_path_, "/home/user/.ros/cost.pddl");
    nh_.param<std::string>("param_server_namespace", param_server_namespace, "/script_server/base");
    nh_.param<double>("self_location_cost", self_location_cost_, 100.0);
    nh_.param<double>("cost_multiplier", cost_multiplier_, 2.0);

    ROS_INFO("The following parameters will be used : ");
    ROS_INFO("global reference frame : %s", global_frame.c_str());
    ROS_INFO("make plan service name : %s", service_name_.c_str());
    ROS_INFO("cost file path : %s", cost_file_path_.c_str());
    ROS_INFO("self location cost : %lf", self_location_cost_);

    // reading from parameter server available locations
    XmlRpc::XmlRpcValue locations;
    nh_.getParam(param_server_namespace.c_str(), locations);
    XmlRpc::XmlRpcValue::iterator locations_iterator = locations.begin();

    // iterate trough available locations and store them in member variable
    while (locations_iterator != locations.end())
    {
        location_names_.push_back(locations_iterator->first);

        // creating pose stamped message from data comming from param server
        geometry_msgs::PoseStamped location;
        location.header.frame_id = global_frame;
        location.pose.position.x = locations_iterator->second[0];
        location.pose.position.y = locations_iterator->second[1];
        location.pose.orientation = tf::createQuaternionMsgFromYaw(locations_iterator->second[2]);

        locations_.push_back(std::pair<std::string, geometry_msgs::PoseStamped>(locations_iterator->first, location));

        ++locations_iterator;
    }
}

void NavigationCostGeneratorNode::combineLocations()
{
    int n = locations_.size();

    std::vector<bool> v(n);
    std::fill(v.begin() + n - 2, v.end(), true);

    do
    {
        std::vector<int> single_combination;
        for (int i = 0; i < n; ++i)
        {   
            if (v[i])
            {
                single_combination.push_back(i+1);
            }
        }

        combinations_.push_back(single_combination);
    } while (std::next_permutation(v.begin(), v.end()));
}

void NavigationCostGeneratorNode::costGenerationLoop()
{
    ROS_INFO("Starting cost generation");

    pFile_.open(cost_file_path_.c_str());
    pFile_ << ";This PDDL costs were generated automatically from navigation cost generator node" << std::endl;

    for (int i = 0; i < combinations_.size() ; i++)
    {
        int start, goal;

        start = combinations_.at(i).at(0) - 1;
        goal = combinations_.at(i).at(1) - 1;

        makePlanServiceCall(locations_.at(start).second, locations_.at(goal).second);

        // Give some time to visualize paths in rviz
        ros::Duration(0.05).sleep();

        while (!getPathLengthCalculatorResponse());
        
        ROS_INFO_STREAM(locations_.at(start).first << " -> " << locations_.at(goal).first << " : " << path_length_);
        writeToFile(locations_.at(start).first, locations_.at(goal).first, path_length_);
        writeToFile(locations_.at(goal).first, locations_.at(start).first, path_length_);
    }

    // write distances between locations itself, this is required by planners for completition
    for (int i = 0 ; i < location_names_.size() ; i++)
    {
        writeToFile(location_names_.at(i), location_names_.at(i), self_location_cost_);
    }

    pFile_.close();
}

bool NavigationCostGeneratorNode::makePlanServiceCall(geometry_msgs::PoseStamped base_position, geometry_msgs::PoseStamped target_pose)
{
    ROS_DEBUG("Calling move_base make plan service");

    // wait for make plan service to become available
    while (!ros::service::waitForService(service_name_, ros::Duration(.5)) && ros::ok())
    {
        ROS_WARN("Waiting for make plan service to become available");
    }

    ROS_DEBUG("Make plan service is available");

    ros::ServiceClient serviceClient = nh_.serviceClient<nav_msgs::GetPlan>(service_name_, true);

    if (!serviceClient)
    {
        ROS_FATAL("Could not initialize get plan service from %s", serviceClient.getService().c_str());
        return false;
    }

    nav_msgs::GetPlan srv;
    srv.request.start = base_position;
    srv.request.goal = target_pose;
    srv.request.tolerance = 0.1;

    if (serviceClient.call(srv))
    {
        if (!srv.response.plan.poses.empty())
        {
            return true;
        }
        else
        {
            ROS_ERROR("Got empty plan");
            return false;
        }
    }
    else
    {
        ROS_ERROR("Failed to call service %s", serviceClient.getService().c_str());
    }

    return false;
}

bool NavigationCostGeneratorNode::getPathLengthCalculatorResponse()
{
    std_msgs::String trigger;
    trigger.data = "e_trigger";

    // trigger path length calculator
    pub_trigger_path_length_calc_.publish(trigger);

    // listen to response from path length calculator at 60 hz
    ros::Rate loop_rate(60.0);

    // wait for path length calculator response
    while (!is_path_length_received_ && ros::ok() && !is_event_in_received_)
    {
        // listen to callbacks
        ros::spinOnce();

        // sleep
        loop_rate.sleep();
    }

    // reset flags
    is_path_length_received_ = false;
    is_event_in_received_ = false;

    if(event_in_msg_.data != "e_success")
        return false;
    
    ROS_DEBUG("path_length_calculator response received");
    return true;
}

void NavigationCostGeneratorNode::writeToFile(std::string source, std::string dest, double cost)
{
    pFile_ << "(= (path-length " << source << " " << dest << ") " << (int)ceil(cost * cost_multiplier_) << ")" << std::endl;
}

void NavigationCostGeneratorNode::eventInCallBack(const std_msgs::String::ConstPtr& msg)
{
    event_in_msg_ = *msg;
    is_event_in_received_ = true;
}

void NavigationCostGeneratorNode::pathLengthCallBack(const std_msgs::Float64::ConstPtr& msg)
{
    path_length_ = msg->data;
    is_path_length_received_ = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_cost_generator_node");

    ROS_INFO("Node is going to initialize...");

    // create object of the node class (NavigationCostGeneratorNode)
    NavigationCostGeneratorNode navigation_cost_generator_node;

    return 0;
}
