-- This script is used for realtime emulation of the environment in V-REP

function sysCall_init()

    -- Add required handles here


    emulation = sim.getObjectHandle('emulation_script');
    -- position_2 = sim.getObjectHandle('Position_hoop2')
    position_1 = sim.getObjectHandle('Position_hoop1')
    position_3 = sim.getObjectHandle('Position_hoop3')
    -- angle = sim.getObjectHandle('Orientation_hoop2')
    hoop1_angle = sim.getObjectHandle('Orientation_hoop1')
    hoop3_angle = sim.getObjectHandle('Orientation_hoop3')
    -- Subscribing to the required topics 



    -- aruco_sub = simROS.subscribe('/chatter123', 'geometry_msgs/Pose', 'aruco_callback')


    aruco_1 = simROS.subscribe('/hoop_1', 'geometry_msgs/Pose', 'aruco_1_callback')
    aruco_3 = simROS.subscribe('/hoop_3', 'geometry_msgs/Pose', 'aruco_3_callback')




    whycon_sub = simROS.subscribe('/whycon/poses', 'geometry_msgs/PoseArray', 'whycon_callback')
    key_input = simROS.subscribe('/input_key', 'std_msgs/Int16', 'key_callback')


    aruco_1_msg = 0;
    aruco_3_msg = 0;
    whycon_msg = 0;
end


function sysCall_actuation()

end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end



-- function aruco_callback(msg)
--     -- Get the orientation(quaternion) of the ArUco marker and set the orientation of the hoop using Orientation_hoop dummy
--     -- Hint : Go through the regular API - sim.setObjectQuaternion

--     -- print(msg)
--     -- sim.setObjectQuaternion(angle,-1,{-1*msg.orientation.y,-1*msg.orientation.x,msg.orientation.w,msg.orientation.z})
-- end

function aruco_1_callback( msg )

    aruco_1_msg = msg
    -- sim.setObjectQuaternion(hoop1_angle,-1,{-1*msg.orientation.y,-1*msg.orientation.x,msg.orientation.w,msg.orientation.z})


    -- body
end

function aruco_3_callback( msg )
    aruco_3_msg = msg
    -- sim.setObjectQuaternion(hoop3_angle,-1,{-1*msg.orientation.y,-1*msg.orientation.x,msg.orientation.w,msg.orientation.z})


    -- body
end




function whycon_callback(msg)
    -- Get the position of the whycon marker and set the position of the food tree and non-food tree using Position_hoop dummy
    -- Hint : Go through the regular API - sim.setObjectPosition

    -- sim.setObjectPosition(emulation,-1,{0.5*msg.poses[1].position.x,-0.5*msg.poses[1].position.y,0.67})
    -- sim.setObjectPosition(position_2,emulation,{0,0,0})

    whycon_msg = msg
    -- sim.setObjectPosition(position_2,-1,{msg.poses[1].position.x,msg.poses[1].position.y,msg.poses[1].position.z})
    -- print(msg.poses[1].position.x)
end

function key_callback(msg)
    -- Read key input to set or unset position and orientation of food and non-food trees
    if msg.data == 500 then
        sim.setObjectPosition(emulation,-1,{0.5*whycon_msg.poses[1].position.x,-0.5*whycon_msg.poses[1].position.y,0.67})
        sim.setObjectPosition(position_1,emulation,{0,0,0})
    end

    if msg.data == 700 then
        sim.setObjectPosition(emulation,-1,{0.5*whycon_msg.poses[1].position.x,-0.5*whycon_msg.poses[1].position.y,0.67})
        sim.setObjectPosition(position_3,emulation,{0,0,0})
    end

    if msg.data == 900 then
        sim.setObjectQuaternion(hoop1_angle,-1,{-1*aruco_1_msg.orientation.y,-1*aruco_1_msg.orientation.x,aruco_1_msg.orientation.w,aruco_1_msg.orientation.z})

    end

    if msg.data == 1000 then
        sim.setObjectQuaternion(hoop3_angle,-1,{-1*aruco_3_msg.orientation.y,-1*aruco_3_msg.orientation.x,aruco_3_msg.orientation.w,aruco_3_msg.orientation.z})

    end

end