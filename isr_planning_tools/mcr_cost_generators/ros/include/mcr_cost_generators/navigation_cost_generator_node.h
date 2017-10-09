/*
 * Copyright [2016] <Bonn-Rhein-Sieg University>
 *
 * Author: Oscar Lima (olima_84@yahoo.com)
 * 
 * Generate navigation costs for task planning based on distance between locations
 * 
 */

#ifndef MCR_NAVIGATION_COST_GENERATOR_NAVIGATION_COST_GENERATOR_NODE_H
#define MCR_NAVIGATION_COST_GENERATOR_NAVIGATION_COST_GENERATOR_NODE_H

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/GetPlan.h>

#include <iostream>
#include <sstream>
#include <fstream>

class NavigationCostGeneratorNode
{
    public:
        NavigationCostGeneratorNode();
        ~NavigationCostGeneratorNode();

        // get parameters from param server
        void getParams();
        
        // generate combinations based on the number of locations
        void combineLocations();

        // calls move base make plan service
        bool makePlanServiceCall(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal);
        
        // loop over all combinations of locations and make cost based on path length
        void costGenerationLoop();
        
        // get path length calculator node response
        bool getPathLengthCalculatorResponse();
        
        // write navigation costs based on path length to file
        void writeToFile(std::string source, std::string dest, double cost);
        
        // callback for event_in received msg
        void eventInCallBack(const std_msgs::String::ConstPtr& msg);
        
        // callback to receive the path length
        void pathLengthCallBack(const std_msgs::Float64::ConstPtr& msg);

    private:
        // ros related variables
        ros::NodeHandle nh_;
        ros::Publisher pub_trigger_path_length_calc_;
        ros::Subscriber sub_path_length_;
        ros::Subscriber sub_event_in_;

        // flag used to know when we have received a callback
        bool is_event_in_received_;
        bool is_path_length_received_;
        
        double path_length_;

        // stores the received msg in event_in callback (runScriptCallBack)
        std_msgs::String event_in_msg_;

        // stores the message that will be published on event_out topic
        std_msgs::String even_out_msg_;
        
        // stores available locations
        std::vector<std::pair<std::string, geometry_msgs::PoseStamped> > locations_;
        
        // stores combinations
        std::vector<std::vector<int> > combinations_;
        
        // stores the make plan service name
        std::string service_name_;
        
        // writes cost information to file
        std::ofstream pFile_;
        
        // stores the path in which the cost will be saved
        std::string cost_file_path_;
        
        // stores the names of the locations
        std::vector<std::string> location_names_;
        
        // to assign cost for cost of driving from the same locations
        double self_location_cost_;
        
        // to control the scale of the costs
        double cost_multiplier_;

};
#endif  // MCR_NAVIGATION_COST_GENERATOR_NAVIGATION_COST_GENERATOR_NODE_H
