knowledge_base_analyzer_node
===

Provides with information:

a) is there unfinished goals in knowledge base?
b) is there new predicates/knowledge in the knowledge base?

Usage instructions.

1. Run the analyzer node and the knowledge base:

        roscore
        roslaunch isr_knowledge_base rosplan_knowledge_base_example.launch
        rosrun isr_knowledge_base knowledge_base_analyzer_node

2. Analyse if there is new knowledge:

        rostopic pub /knowledge_base_analyzer_node/new_knowledge/event_in std_msgs/String "data: 'e_start'"

You can query the result on the topic:

        rostopic echo /knowledge_base_analyzer_node/new_knowledge/event_out

3. Analyse if there are unfinished goals:

        rostopic pub /knowledge_base_analyzer_node/pending_goals/event_in std_msgs/String "data: 'e_start'"

You can query the result on the topic:

        rostopic echo /knowledge_base_analyzer_node/pending_goals/event_out

4. Add some knowledge for testing purposes:

        rqt --standalone rosplan_rqt.dispatcher.ROSPlanDispatcher

Add some object instances and predicates and trigger the component again and check the result

Done! Now you should be able to get feedback wether the nowledge base has

unfinished goals or new knowledge.
