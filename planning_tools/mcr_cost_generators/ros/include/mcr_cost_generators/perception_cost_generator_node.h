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
#include <tf/transform_datatypes.h>

#include <iostream>
#include <sstream>
#include <fstream>

class PerceptionCostGeneratorNode
{
    public:
        PerceptionCostGeneratorNode();
        ~PerceptionCostGeneratorNode();

        // get and setup parameters from parameter server
        void getParams();
        
        // write costs to file in pddl format
        bool generateCosts();
        
        // decrease the object recognition rate by specified rate value
        bool decreaseRecognitionRate(std::string object, double rate);
        
        // callback to receive the object name to decrease the recognition rate
        void objectNameCallback(const std_msgs::String::ConstPtr& msg);
        
        // to receive e_trigger and make the costs
        void eventInCallBack(const std_msgs::String::ConstPtr& msg);
        
        // main loop function, wait for e_trigger then generate costs
        void update();

    private:
        // ros related variables
        ros::NodeHandle nh_;
        ros::Subscriber sub_event_in_;
        ros::Subscriber sub_object_name_;
        ros::Publisher pub_event_out_;

        // flag used to know when we have received a callback
        bool is_event_in_received_;

        // stores the received msg in event_in callback (runScriptCallBack)
        std_msgs::String event_in_msg_;

        // stores the message that will be published on event_out topic
        std_msgs::String even_out_msg_;
        
        // writes cost information to file
        std::ofstream pFile_;
        
        // stores the path in which the cost will be saved
        std::string cost_file_path_;
        
        // to control the range of the generated costs, costs values are normalized in this range
        double minimum_cost_;
        double maximum_cost_;
        
        // to store th desired value to decrease the percentage of the perception recognition
        double decrease_rate_;
        
        // to store the perception recognition rates
        std::map<std::string, double> perception_recognition_rates_;
        
        // to store the name of the object from which the recognition rate will be reduced
        std::string object_name_;

};
#endif  // MCR_NAVIGATION_COST_GENERATOR_NAVIGATION_COST_GENERATOR_NODE_H
