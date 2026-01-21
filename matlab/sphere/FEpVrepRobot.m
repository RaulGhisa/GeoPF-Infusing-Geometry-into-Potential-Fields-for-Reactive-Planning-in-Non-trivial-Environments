classdef FEpVrepRobot < DQ_VrepRobot
    
    properties
        joint_names;
        base_frame_name;
    end
    
    methods 
        function obj = FEpVrepRobot(robot_name,vrep_interface)
            
            obj.robot_name = robot_name;
            obj.vrep_interface = vrep_interface;
            
            splited_name = strsplit(robot_name,'#');
            robot_label = splited_name{1};
            if ~strcmp(robot_label,'Franka')
                error('Franka')
            end
            if length(splited_name) > 1
                robot_index = splited_name{2};
            else
                robot_index = '';
            end
            
            %Initialize joint names and base frame
            obj.joint_names = {};
            for i=1:7
                current_joint_name = {robot_label,'_joint',int2str(i),robot_index};
                obj.joint_names{i} = strjoin(current_joint_name,'');
            end
            obj.base_frame_name = obj.joint_names{1};
        end            
   
        function send_q_to_vrep(obj,q)
            obj.vrep_interface.set_joint_positions(obj.joint_names,q)
        end
        
        function q = get_q_from_vrep(obj)
            q = obj.vrep_interface.get_joint_positions(obj.joint_names,65536);
        end
        
        
        
        function kin = kinematics(obj)
           
            % DH parameters are in FramkaEmikaPandaRobot
            kin = FrankaEmikaPandaRobot.kinematics();
            
            % set reference frame and base frame
            kin.set_reference_frame(obj.vrep_interface.get_object_pose(obj.base_frame_name));
            kin.set_base_frame(obj.vrep_interface.get_object_pose(obj.base_frame_name));
        end
    end
end