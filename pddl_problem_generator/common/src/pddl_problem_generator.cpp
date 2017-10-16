/*
 * Copyright [2016] <Bonn-Rhein-Sieg University>
 *
 * Author: Oscar Lima (olima_84@yahoo.com)
 * Based on: https://github.com/KCL-Planning/ROSPlan/blob/master/rosplan_planning_system/src/generatePDDLProblem.cpp
 * 
 * Generic class used to generate a PDDL problem file with or without cost information
 * 
 */

#include <pddl_problem_generator/pddl_problem_generator.h>
#include <map>
#include <string>
#include <vector>

PDDLProbGenCost::PDDLProbGenCost()
: problem_path_(std::string("")), cost_required_(false),
metric_(std::string("")), ready_to_generate_(false) {}

PDDLProbGenCost::PDDLProbGenCost(std::string& problem_path)
: problem_path_(problem_path), cost_required_(false),
metric_(std::string("")) {}

PDDLProbGenCost::PDDLProbGenCost(std::string& problem_path, std::vector<std::string>& cost_file_path)
: problem_path_(problem_path), cost_file_path_(cost_file_path), cost_required_(true),
metric_(std::string("(:metric minimize (total-cost))")) {}

void PDDLProbGenCost::configure(std::string& problem_path)
{
    cost_required_ = false;
    problem_path_ = problem_path;
    metric_ = std::string("");
    ready_to_generate_ = true;
}

void PDDLProbGenCost::configure(std::string& problem_path, std::vector<std::string>& cost_file_path)
{
    cost_required_ = true;
    problem_path_ = problem_path;
    cost_file_path_ = cost_file_path;
    metric_ = std::string("(:metric minimize (total-cost))");
    ready_to_generate_ = true;
}

void PDDLProbGenCost::configure(std::string& problem_path, std::vector<std::string>& cost_file_path, std::string& metric)
{
    cost_required_ = true;
    problem_path_ = problem_path;
    cost_file_path_ = cost_file_path;
    metric_ = metric;
    ready_to_generate_ = true;
}

bool PDDLProbGenCost::generatePDDLProblemFile(KCL_rosplan::PlanningEnvironment& environment)
{
    // check if configure method was called, if not then exit without doing anything
    if (!ready_to_generate_) return false;

    std::ofstream pFile;
    pFile.open((problem_path_).c_str());

    if (!makeHeader(environment, pFile))
    {
        std::cerr << "Error : Could not make header" << std::endl;
        pFile.close();
        return false;
    }

    if (!makeObjects(environment, pFile))
    {
        std::cerr << "Error : Could not make objects" << std::endl;
        pFile.close();
        return false;
    }

    if (!makeInitialStateHeader(pFile))
    {
        std::cerr << "Error : Could not make initial state header" << std::endl;
        pFile.close();
        return false;
    }

    if (cost_required_)
    {
        if (!makeInitialStateCost(environment, pFile))
        {
            std::cerr << "Error : Could not make initial state costs" << std::endl;
            pFile.close();
            return false;
        }
    }

    if (!makeInitialStateFacts(environment, pFile))
    {
        std::cerr << "Error : Could not make state facts" << std::endl;
        pFile.close();
        return false;
    }

    if (!makeGoals(environment, pFile))
    {
        std::cerr << "Error : Could not make goals" << std::endl;
        pFile.close();
        return false;
    }

    if (cost_required_)
    {
        if (!makeMetric(pFile))
        {
            std::cerr << "Error : Could not make metric" << std::endl;
            pFile.close();
            return false;
        }
    }

    if (!finalizePDDLFile(pFile))
    {
        std::cerr << "Error : Could not finalize PDDL file" << std::endl;
        pFile.close();
        return false;
    }

    pFile.close();
    return true;
}

bool PDDLProbGenCost::makeHeader(KCL_rosplan::PlanningEnvironment& environment, std::ofstream &pFile)
{
    // check if configure method was called, if not then exit without doing anything
    if (!ready_to_generate_) return false;

    pFile << ";This PDDL problem definition was made automatically from a KB snapshot" << std::endl;
    pFile << "(define (problem " << environment.domainName << "_task)" << std::endl;
    pFile << "(:domain " << environment.domainName << ")" << std::endl;
    pFile << "" << std::endl;

    return true;
}

bool PDDLProbGenCost::makeObjects(KCL_rosplan::PlanningEnvironment& environment, std::ofstream &pFile)
{
    // check if configure method was called, if not then exit without doing anything
    if (!ready_to_generate_) return false;

    bool is_there_objects = false;

    // objects
    pFile << "(:objects" << std::endl;

    for (std::map<std::string, std::vector<std::string> >::iterator iit=environment.type_object_map.begin();
         iit != environment.type_object_map.end(); ++iit)
    {
        if (iit->second.size() > 0)
        {
            pFile << "    ";

            for (size_t i = 0; i < iit->second.size(); i++)
            {
                pFile << iit->second[i] << " ";
            }

            pFile << "- " << iit->first << std::endl;
            if (!is_there_objects) is_there_objects = true;
        }
    }
    pFile << ")" << std::endl;
    pFile << "" << std::endl;

    return is_there_objects;
}

bool PDDLProbGenCost::makeInitialStateHeader(std::ofstream &pFile)
{
    // check if configure method was called, if not then exit without doing anything
    if (!ready_to_generate_) return false;

    pFile << "(:init" << std::endl;

    return true;
}

bool PDDLProbGenCost::makeInitialStateCost(KCL_rosplan::PlanningEnvironment& environment, std::ofstream &pFile)
{
    // check if configure method was called, if not then exit without doing anything
    if (!ready_to_generate_) return false;

    bool is_there_cost_information = false;

    pFile << "    ;Cost information starts" << std::endl;
    pFile << "    (= (total-cost) 0)" << std::endl;

    for (int i = 0; i < cost_file_path_.size() ; i++)
    {
        std::ifstream infile(cost_file_path_.at(i).c_str());

        // read cost information from file and add it to pddl problem
        std::string line;
        while (std::getline(infile, line))
        {
            if (!is_there_cost_information) is_there_cost_information = true;
            pFile << "    " << line << std::endl;
        }
        
        // empty line between different cost files
        pFile << "" << std::endl;
    }
    
    pFile << "    ;Cost information ends" << std::endl;
    pFile << "" << std::endl;

    return true;
}

bool PDDLProbGenCost::makeInitialStateFacts(KCL_rosplan::PlanningEnvironment& environment, std::ofstream &pFile)
{
    // check if configure method was called, if not then exit without doing anything
    if (!ready_to_generate_) return false;

    bool is_there_facts = false;

    // add knowledge to the initial state
    for (size_t i = 0; i < environment.domain_attributes.size(); i++)
    {
        std::stringstream ss;
        ss << "    (";

        if (environment.domain_attributes[i].knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FUNCTION)
        {
            ss << "= (";
        }

        ss << environment.domain_attributes[i].attribute_name;

        // fetch the corresponding symbols from domain
        std::map<std::string, std::vector<std::string> >::iterator ait;
        ait = environment.domain_predicates.find(environment.domain_attributes[i].attribute_name);

        if (ait == environment.domain_predicates.end())
        {
            ait = environment.domain_functions.find(environment.domain_attributes[i].attribute_name);
        }

        if (ait == environment.domain_functions.end())
        {
            continue;
        }

        // find the PDDL parameters in the KnowledgeItem
        bool writeAttribute = true;

        for (size_t j=0; j < ait->second.size(); j++)
        {
            bool found = false;
            for (size_t k = 0; k < environment.domain_attributes[i].values.size(); k++)
            {
                if (0 == environment.domain_attributes[i].values[k].key.compare(ait->second[j]))
                {
                    ss << " " << environment.domain_attributes[i].values[k].value;
                    found = true;
                }
            }
            if (!found) writeAttribute = false;
        }

        ss << ")";

        if (!is_there_facts) is_there_facts = true;

        // output function value
        if (environment.domain_attributes[i].knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FUNCTION)
        {
            ss << " " << environment.domain_attributes[i].function_value << ")";
        }

        if (writeAttribute) pFile << ss.str() << std::endl;
    }

    // add knowledge to the initial state
    for (size_t i = 0; i < environment.instance_attributes.size(); i++)
    {
        std::stringstream ss;
        bool writeAttribute = false;

        // check if attribute is a PDDL predicate
        std::map<std::string, std::vector<std::string> >::iterator ait;

        ait = environment.domain_predicates.find(environment.instance_attributes[i].attribute_name);

        if (ait != environment.domain_predicates.end())
        {
            writeAttribute = true;

            ss << "    (" + environment.instance_attributes[i].attribute_name;

            // find the PDDL parameters in the KnowledgeItem
            for (size_t j = 0; j < ait->second.size(); j++)
            {
                bool found = false;

                for (size_t k = 0; k < environment.instance_attributes[i].values.size(); k++)
                {
                    if (0 == environment.instance_attributes[i].values[k].key.compare(ait->second[j]))
                    {
                        ss << " " << environment.instance_attributes[i].values[k].value;
                        found = true;
                    }
                }
                if (!found) writeAttribute = false;
            };
            ss << ")";
        }
        if (writeAttribute) pFile << ss.str() << std::endl;
    }
    pFile << ")" << std::endl;

    // blank space between facts and goals
    pFile << "" << std::endl;

    return is_there_facts;
}

bool PDDLProbGenCost::makeGoals(KCL_rosplan::PlanningEnvironment& environment, std::ofstream &pFile)
{
    // check if configure method was called, if not then exit without doing anything
    if (!ready_to_generate_) return false;

    bool is_there_goals = false;

    pFile << "(:goal (and" << std::endl;

    // propositions in the initial state
    for (size_t i = 0; i < environment.goal_attributes.size(); i++)
    {
        std::stringstream ss;
        bool writeAttribute = true;

        // check if attribute belongs in the PDDL model
        std::map<std::string, std::vector<std::string> >::iterator ait;
        ait = environment.domain_predicates.find(environment.goal_attributes[i].attribute_name);
        if (ait != environment.domain_predicates.end())
        {
            ss << "    (" + environment.goal_attributes[i].attribute_name;

            // find the PDDL parameters in the KnowledgeItem
            bool found = false;
            for (size_t j = 0; j < ait->second.size(); j++) {
                for (size_t k = 0; k < environment.goal_attributes[i].values.size(); k++)
                {
                    if (0 == environment.goal_attributes[i].values[k].key.compare(ait->second[j]))
                    {
                        ss << " " << environment.goal_attributes[i].values[k].value;
                        found = true;
                    }
                }
            }
            if (!found) writeAttribute = false;

            if (!is_there_goals) is_there_goals = true;

            ss << ")";
        }
        else
        {
            writeAttribute = false;
        }

        if (writeAttribute) pFile << ss.str() << std::endl;
    }

    pFile << "    )" << std::endl;
    pFile << ")" << std::endl;
    pFile << "" << std::endl;

    return is_there_goals;
}

bool PDDLProbGenCost::makeMetric(std::ofstream& pFile)
{
    // check if configure method was called, if not then exit without doing anything
    if (!ready_to_generate_) return false;

    // metric specification
    pFile << metric_ << std::endl;
    pFile << "" << std::endl;

    return true;
}

bool PDDLProbGenCost::finalizePDDLFile(std::ofstream& pFile)
{
    // check if configure method was called, if not then exit without doing anything
    if (!ready_to_generate_) return false;

    // end of problem
    pFile << ")" << std::endl;

    return true;
}
