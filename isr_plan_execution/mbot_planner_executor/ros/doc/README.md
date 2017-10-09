planner_executor
================

Listens to /plan topic which is having this structure:

        ActionDispatch                              
        [rosplan_dispatch_msgs/ActionDispatch]:     
        int32 action_id                             
        string name                                 
        diagnostic_msgs/KeyValue[] parameters       
        string key                                
        string value                              
        float32 duration                            
        float32 dispatch_time                       
                                                    
        =========                                   
                                                    
        action_id: 0                                
        name: move_base                             
        parameters:                                 
        -                                         
            key: 1                                  
            value: start                            
        -                                         
            key: 2                                  
            value: assembly_station                 
        -                                         
            key: 3                                  
            value: youbot-brsu-3                    
        -                                         
            key: 4                                  
            value: dynamixel                        
        duration: 0.0                               
        dispatch_time: 0.0                          
                                            
And calls individual actionlib actions in order