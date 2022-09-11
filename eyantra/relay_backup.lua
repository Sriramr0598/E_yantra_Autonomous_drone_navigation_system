-- This script is used for realtime emulation of the environment in V-REP

function sysCall_init()

    -- Add required handles here

    whycon_msgs = 0;
    emulation = sim.getObjectHandle('emulation_script');
    position_2 = sim.getObjectHandle('Position_hoop2')
    angle = sim.getObjectHandle('Orientation_hoop2')
    -- Subscribing to the required topics 
    aruco_sub = simROS.subscribe('/chatter123', 'geometry_msgs/Pose', 'aruco_callback')
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
    -- msg.position.x = (path[i]*-7.55)
    -- msg.position.y = (path[i+1]*-7.55)
    -- msg.position.z = (((-55.6/3.048))*path[i+2]) + 55.6

    -- x = msg.position.x*-7.55;
    -- y = msg.position.y*-7.55;

    print(msg)
    sim.setObjectQuaternion(angle,-1,{-1*msg.orientation.y,-1*msg.orientation.x,msg.orientation.w,msg.orientation.z})
end

function whycon_callback(msg)
    -- Get the position of the whycon marker and set the position of the food tree and non-food tree using Position_hoop dummy
    -- Hint : Go through the regular API - sim.setObjectPosition

    sim.setObjectPosition(emulation,-1,{0.5*msg.poses[1].position.x,-0.5*msg.poses[1].position.y,0.67})
    sim.setObjectPosition(position_2,emulation,{0,0,0})
    whycon_msgs = msg
    print(whycon_msgs)

    -- sim.setObjectPosition(position_2,-1,{msg.poses[1].position.x,msg.poses[1].position.y,msg.poses[1].position.z})
    -- print(msg.poses[1].position.x)
end

function key_callback(msg)
    -- Read key input to set or unset position and orientation of food and non-food trees

end