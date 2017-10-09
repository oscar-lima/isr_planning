/*
 * Copyright [2016] <Bonn-Rhein-Sieg University>
 *
 * Author: Oscar Lima (olima_84@yahoo.com)
 * Based on: https://github.com/KCL-Planning/ROSPlan/blob/master/rosplan_planning_system/src/PDDLProblemGenerator.cpp
 * 
 * Generic class used to generate a PDDL problem file
 * 
 */

#ifndef MCR_PDDL_PROBLEM_GENERATOR_PDDL_PROBLEM_GENERATOR_H
#define MCR_PDDL_PROBLEM_GENERATOR_PDDL_PROBLEM_GENERATOR_H

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <rosplan_planning_system/PlanningEnvironment.h>

class PDDLProbGenCost
{
    public:
        PDDLProbGenCost();
        
        /**
        * @brief Constructor - select this one for no cost PDDL problem
        * @param problem_path the path for the PDDL problem file to be created
        */
        explicit PDDLProbGenCost(std::string& problem_path);

        /**
        * @brief Constructor - select this one for PDDL problem with cost information
        * @param problem_path the path for the PDDL problem file to be created
        * @param cost_file_path the path for the cost information to be read as file
        */
        PDDLProbGenCost(std::string& problem_path, std::vector<std::string>& cost_file_path);

        /**
        * @brief function needed if user creates object of this class with an empty constructor
        * Use this function for creating PDDL problems without cost information
        * @param problem_path the path for the PDDL problem file to be created
        */
        void configure(std::string& problem_path);

        /**
        * @brief function needed if user creates object of this class with an empty constructor
        * Use this function for creating PDDL problems with cost information and default
        * minimize cost metric
        * @param problem_path the path for the PDDL problem file to be created
        * @param cost_file_path the path for the cost information to be read as file
        */
        void configure(std::string& problem_path, std::vector<std::string>& cost_file_path);

        /**
        * @brief function needed if user creates object of this class with an empty constructor
        * Use this function for creating PDDL problems with cost information and custom metric
        * @param problem_path the path for the PDDL problem file to be created
        * @param cost_file_path the path for the cost information to be read as file
        * @param metric the metric that you want your planner to follow
        */
        void configure(std::string& problem_path, std::vector<std::string>& cost_file_path, std::string& metric);

        /**
        * @brief Generate PDDL problem definition from Knowledge base snapshot
        * with or without cost information for planners which can handle cost
        * @param environment kcl_rosplan environment class which holds the knowledge
        * base items, needs to be called externally as it uses a ros nodehandle to work
        */
        bool generatePDDLProblemFile(KCL_rosplan::PlanningEnvironment& environment);

        /**
        * @brief Reads define and domain initial PDDL tags from environment and writes it to file
        * @param environment kcl_rosplan environment class which holds the knowledge
        * base items
        * @param pFile the stream file object to write the information in
        */
        bool makeHeader(KCL_rosplan::PlanningEnvironment& environment, std::ofstream &pFile);

        /**
        * @brief Read instance type and objects from environment (knowledge base snapshot)
        * @param environment kcl_rosplan environment class which holds the knowledge
        * base items
        * @param pFile the stream file object to write the information in
        */
        bool makeObjects(KCL_rosplan::PlanningEnvironment& environment, std::ofstream &pFile);

        /**
        * @brief Creates init tag from PDDL problem definition, this needs to be a separate function
        * because there might be the possibility to include cost information
        * @param pFile the stream file object to write the information in
        */
        bool makeInitialStateHeader(std::ofstream &pFile);

        /**
        * @brief Reads from cost_file_path_ file location and writes whatever is in there to the 
        * PDDL problem definition stream
        * @param environment kcl_rosplan environment class which holds the knowledge
        * base items
        * @param pFile the stream file object to write the information in
        */
        bool makeInitialStateCost(KCL_rosplan::PlanningEnvironment& environment, std::ofstream &pFile);

        /**
        * @brief Reads predicates (facts) from environment (knowledge base snapshot) and writes
        * to ppdl problem definition stream
        * @param environment kcl_rosplan environment class which holds the knowledge
        * base items
        * @param pFile the stream file object to write the information in
        */
        bool makeInitialStateFacts(KCL_rosplan::PlanningEnvironment& environment, std::ofstream &pFile);

        /**
        * @brief Reads goals from environment (knowledge base snapshot) and writes the information
        * to the PDDL problem definition stream
        * @param environment kcl_rosplan environment class which holds the knowledge
        * base items
        * @param pFile the stream file object to write the information in
        */
        bool makeGoals(KCL_rosplan::PlanningEnvironment& environment, std::ofstream &pFile);

        /**
        * @brief A planner which is able to handle cost information needs a metric to be
        * specified in the PDDL file. i.e = (:metric minimize (total-cost))
        * @param pFile the stream file object to write the information in
        */
        bool makeMetric(std::ofstream& pFile);

        /**
        * @brief Ends the PDDL file by writing )
        * @param pFile the stream file object to write the information in
        */
        bool finalizePDDLFile(std::ofstream& pFile);

    private:
        // flag to determine whether cost information is required or not
        bool cost_required_;

        // flag to indicate that required setup function has been called
        bool ready_to_generate_;

        // Stores the location in which the PDDL problem will be saved
        std::string problem_path_;

        // Stores the location in which the cost information is available
        std::vector<std::string> cost_file_path_;

        // Stores the metric to be written in the PDDL file
        std::string metric_;
};
#endif  // MCR_PDDL_PROBLEM_GENERATOR_PDDL_PROBLEM_GENERATOR_H
