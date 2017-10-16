/* 
 * Copyright [2016] <Bonn-Rhein-Sieg University>  
 * 
 * Author: Oscar Lima (olima_84@yahoo.com)
 * 
 * Generate navigation costs for task planning based on distance between perception_recognition_rates_
 * 
 */

#include <mcr_cost_generators/perception_cost_generator_node.h>
#include <string>
#include <vector>

PerceptionCostGeneratorNode::PerceptionCostGeneratorNode() : nh_("~"), is_event_in_received_(false), minimum_cost_(0.0), maximum_cost_(0.0),
object_name_(std::string(""))
{
    // subscriptions
    sub_event_in_ = nh_.subscribe("event_in", 1, &PerceptionCostGeneratorNode::eventInCallBack, this);
    sub_object_name_ = nh_.subscribe("decrease_recognition_rate/object_name", 1, &PerceptionCostGeneratorNode::objectNameCallback, this);

    // publications
    pub_event_out_ = nh_.advertise<std_msgs::String>("event_out", 1);

    // querying parameters from parameter server
    getParams();
}

PerceptionCostGeneratorNode::~PerceptionCostGeneratorNode()
{
    // shut down publishers and subscribers
    sub_event_in_.shutdown();
    pub_event_out_.shutdown();
    sub_object_name_.shutdown();
}

void PerceptionCostGeneratorNode::getParams()
{
    // read from param server
    nh_.param<double>("minimum_cost", minimum_cost_, 2.0);
    nh_.param<double>("maximum_cost", maximum_cost_, 20.0);
    nh_.param<double>("decrease_rate", decrease_rate_, 20.0); // decrease 20% by default the recognition rate every time the robot fails to detect object
    nh_.param<std::string>("cost_file_path", cost_file_path_, "/home/user/.ros/perception_costs.pddl");

    ROS_INFO("The following parameters will be used : ");
    ROS_INFO("minimum_cost : %lf", minimum_cost_);
    ROS_INFO("maximum_cost : %lf", maximum_cost_);
    ROS_INFO("decrease_rate : %lf", decrease_rate_);
    ROS_INFO("cost_file_path : %s", cost_file_path_.c_str());

    // reading from parameter server object recognition rate
    XmlRpc::XmlRpcValue perception_costs;
    nh_.getParam("objects_recognition_rate", perception_costs);
    XmlRpc::XmlRpcValue::iterator perception_recognition_rates_iterator = perception_costs.begin();

    // iterate trough available perception costs and store them in member variable
    while (perception_recognition_rates_iterator != perception_costs.end())
    {
        perception_recognition_rates_.insert(std::pair<std::string, double>(perception_recognition_rates_iterator->first, perception_recognition_rates_iterator->second));
        ++perception_recognition_rates_iterator;
    }
}

bool PerceptionCostGeneratorNode::generateCosts()
{
    ROS_INFO("Starting perception cost generation");

    pFile_.open(cost_file_path_.c_str());
    pFile_ << ";This PDDL costs were generated automatically from perception cost generator node" << std::endl;
    
    std::map<std::string, double>::iterator perception_costs_iterator = perception_recognition_rates_.begin();
    
    while(perception_costs_iterator != perception_recognition_rates_.end())
    {   
        double recog_rate = perception_costs_iterator->second;
        
        // normalize complementary recognition rate (100 - recog rate)
        double cost = (maximum_cost_ - minimum_cost_) * ((100.0 - recog_rate) / 100.0) + minimum_cost_;
        
        // clamp cost value to allowed range
        if (cost > maximum_cost_)
        {
            cost = maximum_cost_;
        }
        
        if (cost < minimum_cost_)
        {
            cost = minimum_cost_;
        }
        
        pFile_ << "(= (perception-complexity " << perception_costs_iterator->first << ") " << (int)ceil(cost) << ")" << std::endl;
        
        perception_costs_iterator++;
    }

    pFile_.close();
    
    return true;
}

bool PerceptionCostGeneratorNode::decreaseRecognitionRate(std::string object, double rate)
{
    std::map<std::string, double>::iterator it;
    
    // find object current data
    it = perception_recognition_rates_.find(object);
    
    if (it == perception_recognition_rates_.end())
    {
        even_out_msg_.data = "e_not_found";
        pub_event_out_.publish(even_out_msg_);
        return false;
    }
    
    ROS_INFO("Recognition rate for object %s will be reduced from %lf to %lf", object_name_.c_str(), it->second, it->second - decrease_rate_);
    it->second -= decrease_rate_;
    
    even_out_msg_.data = "e_decreased_rate";
    pub_event_out_.publish(even_out_msg_);
    
    return true;
}

void PerceptionCostGeneratorNode::objectNameCallback(const std_msgs::String::ConstPtr& msg)
{
    object_name_ = msg->data;
    ROS_INFO("Received object name : %s, ready to reduce recognition rate", object_name_.c_str());
}

void PerceptionCostGeneratorNode::eventInCallBack(const std_msgs::String::ConstPtr& msg)
{
    event_in_msg_ = *msg;
    is_event_in_received_ = true;
}

void PerceptionCostGeneratorNode::update()
{
    // listen to callbacks
    ros::spinOnce();

    if (!is_event_in_received_) return;

    // reset flag
    is_event_in_received_ = false;

    
    if(event_in_msg_.data == "e_decrease_rate")
    {
        if(object_name_ != "")
        {
            decreaseRecognitionRate(object_name_, decrease_rate_);
        }
        else
        {
            even_out_msg_.data = std::string("e_failure");
            pub_event_out_.publish(even_out_msg_);

            ROS_ERROR("Could not decrease recognition rate, no object name was received previously");
        }
        
        return;
    }
    
    if(event_in_msg_.data == "e_gen_costs")
    {
         // write costs to file
        if (generateCosts())
        {
            even_out_msg_.data = std::string("e_success");
            pub_event_out_.publish(even_out_msg_);

            ROS_INFO("Perception costs were generated succesfully !");
        }
        else
        {
            even_out_msg_.data = std::string("e_failure");
            pub_event_out_.publish(even_out_msg_);

            ROS_ERROR("A failure occurred while generating perception costs");
        }
        
        return;
    }
    
    ROS_ERROR("Received unsupported event: %s, supported events are (e_gen_costs, e_decrease_rate)", event_in_msg_.data.c_str());
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "perception_cost_generator_node");

    ROS_INFO("Node is going to initialize...");

    // create object of the node class (PerceptionCostGeneratorNode)
    PerceptionCostGeneratorNode perception_cost_generator_node;

    // setup node frequency
    double node_frequency = 10.0;
    ros::NodeHandle nh("~");
    nh.param("node_frequency", node_frequency, 10.0);
    ROS_INFO("Node will run at : %lf [hz]", node_frequency);
    ros::Rate loop_rate(node_frequency);

    ROS_INFO("Node initialized.");

    while (ros::ok())
    {
        // main loop function
        perception_cost_generator_node.update();

        // sleep to control the node frequency
        loop_rate.sleep();
    }

    return 0;
}
