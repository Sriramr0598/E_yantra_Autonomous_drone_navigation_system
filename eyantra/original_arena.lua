-- This script is used for realtime emulation of the environment in V-REP

function sysCall_init()

    -- Add required handles here
      -- Add required handles here

    obstacle = sim.getObjectHandle('obstacle_1');
    emulation = sim.getObjectHandle('emulation_script');
    -- position_2 = sim.getObjectHandle('Position_hoop2')
    position_1 = sim.getObjectHandle('Position_hoop1')
    position_3 = sim.getObjectHandle('Position_hoop3')
    -- angle = sim.getObjectHandle('Orientation_hoop2')
    hoop1_angle = sim.getObjectHandle('Orientation_hoop1')
    hoop3_angle = sim.getObjectHandle('Orientation_hoop3')




    -- Subscribing to the required topics 
    aruco_sub = simROS.subscribe('/aruco_marker_publisher/markers', 'aruco_msgs/MarkerArray', 'aruco_callback')
    whycon_sub = simROS.subscribe('/whycon/poses', 'geometry_msgs/PoseArray', 'whycon_callback')
    key_input = simROS.subscribe('/input_key', 'std_msgs/Int16', 'key_callback')
end


function sysCall_actuation()

end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end



function aruco_callback(msg)
    -- Get the orientation(quaternion) of the ArUco marker and set the orientation of the hoop using Orientation_hoop dummy
    -- Hint : Go through the regular API - sim.setObjectQuaternion


end

function whycon_callback(msg)
    -- Get the position of the whycon marker and set the position of the food tree and non-food tree using Position_hoop dummy
    -- Hint : Go through the regular API - sim.setObjectPosition

end

function key_callback(msg)
    -- Read key input to set or unset position and orientation of food and non-food trees

end