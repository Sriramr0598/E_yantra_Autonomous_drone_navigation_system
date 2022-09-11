function sysCall_init()

    -- Add required handles here

    -- obstacle = sim.getObjectHandle('obstacle_1'); --Obstacle Handle
    emulation = sim.getObjectHandle('emulation_script'); -- Script dummy handle
    position_1 = sim.getObjectHandle('Position_hoop1') --  Hoop 1 Handle
    position_3 = sim.getObjectHandle('Position_hoop3') -- Hoop 3 Handle
    hoop1_angle = sim.getObjectHandle('Orientation_hoop1') -- Hoop 1 Orientation Handle
    hoop3_angle = sim.getObjectHandle('Orientation_hoop3') -- Hoop 3 Orientation Handle

    front_handle3 = sim.getObjectHandle('Front_hoop3')
    back_handle3 = sim.getObjectHandle('Back_hoop3')

    front_handle1 = sim.getObjectHandle('Front_hoop1')
    back_handle1 = sim.getObjectHandle('Back_hoop1')   
    

    -- Subscribing to the required topics 

    aruco_sub = simROS.subscribe('/hoop_3', 'geometry_msgs/Pose', 'aruco_callback3')
    aruco_sub = simROS.subscribe('/hoop_1', 'geometry_msgs/Pose', 'aruco_callback1')

    whycon_sub = simROS.subscribe('/whycon/poses', 'geometry_msgs/PoseArray', 'whycon_callback')
    key_input = simROS.subscribe('/input_key', 'std_msgs/Int16', 'key_callback')

    aruco_msg1 = 0 -- Variable to store message from Aruco Callback
    aruco_msg3 = 0
    whycon_msg = 0 -- Variable to store message from Whycon Callback
end


function sysCall_actuation()

end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end



function aruco_callback3(msg)
    -- Get the orientation(quaternion) of the ArUco marker and set the orientation of the hoop using Orientation_hoop dummy
    -- Hint : Go through the regular API - sim.setObjectQuaternion

    aruco_msg3 = msg -- Variable to store message from Aruco Callback
    --print(aruco_msg)
end
function aruco_callback1(msg)
    -- Get the orientation(quaternion) of the ArUco marker and set the orientation of the hoop using Orientation_hoop dummy
    -- Hint : Go through the regular API - sim.setObjectQuaternion

    aruco_msg1 = msg -- Variable to store message from Aruco Callback
    --print(aruco_msg)
end


function whycon_callback(msg)
    -- Get the position of the whycon marker and set the position of the food tree and non-food tree using Position_hoop dummy
    -- Hint : Go through the regular API - sim.setObjectPosition
  

    whycon_msg = msg -- Variable to store message from Whycon Callback


end

function key_callback(msg)
    -- Read key input to set or unset position and orientation of food and non-food trees


---------------------------------------------- SET POSITION OF HOOP 1 ------------------------------------------------------
    if msg.data == 500 then
        sim.setObjectPosition(emulation,-1,{0.13*whycon_msg.poses[1].position.x,-0.13*whycon_msg.poses[1].position.y,1.1229}) -- 1.12 is the z value of hoop1
        sim.setObjectPosition(position_1,emulation,{0,0,0})
    end
-------------------------------------------------------------------------------------------------------------------------------




---------------------------------------------- SET POSITION OF HOOP 3 --------------------------------------------------------

    if msg.data == 700 then
        sim.setObjectPosition(emulation,-1,{0.13*whycon_msg.poses[1].position.x,-0.13*whycon_msg.poses[1].position.y,0.47087}) -- 0.62 is the z value of hoop3
        sim.setObjectPosition(position_3,emulation,{0,0,0})
    end
-------------------------------------------------------------------------------------------------------------------------------    





---------------------------------------------- SET POSITION OF OBSTACLE 1  ------------------------------------------------------

    if msg.data == 2000 then
        sim.setObjectPosition(obstacle,-1,{0.13*whycon_msg.poses[1].position.x,-0.13*whycon_msg.poses[1].position.y,1.3345}) -- 1.3 is the z value of obstacle1
        sim.setObjectPosition(position_3,emulation,{0,0,0})
    end
-----------------------------------------------------------------------------------------------------------------------------------




---------------------------------------------- SET ORIENTATION OF HOOP 1 ---------------------------------------------------------

    if msg.data == 900 then
        sim.setObjectQuaternion(hoop1_angle,-1,{-1*aruco_msg1.orientation.y,   -1*aruco_msg1.orientation.x,   aruco_msg1.orientation.w,  aruco_msg1.orientation.z})
        sim.setObjectOrientation(front_handle1,-1,{0,0,0})
        sim.setObjectOrientation(back_handle1,-1,{0,0,0})
    end
---------------------------------------------------------------------------------------------------------------------------------



---------------------------------------------- SET ORIENTATION OF HOOP 3 ------------------------------------------------------


    if msg.data == 1000 then
        sim.setObjectQuaternion(hoop3_angle,-1,{-1*aruco_msg3.orientation.y,   -1*aruco_msg3.orientation.x,   aruco_msg3.orientation.w,  aruco_msg3.orientation.z})
        sim.setObjectOrientation(front_handle3,-1,{0,0,0})
        sim.setObjectOrientation(back_handle3,-1,{0,0,0})
    end
-------------------------------------------------------------------------------------------------------------------------------
end








function sysCall_init()

    -- Add required handles here

    -- obstacle = sim.getObjectHandle('obstacle_1'); --Obstacle Handle
    emulation = sim.getObjectHandle('emulation_script'); -- Script dummy handle
    -- position_1 = sim.getObjectHandle('Position_hoop1') --  Hoop 1 Handle
    position_3 = sim.getObjectHandle('Position_hoop3') -- Hoop 3 Handle
    -- hoop1_angle = sim.getObjectHandle('Orientation_hoop1') -- Hoop 1 Orientation Handle
    hoop3_angle = sim.getObjectHandle('Orientation_hoop3') -- Hoop 3 Orientation Handle

    front_handle = sim.getObjectHandle('Front_hoop3')
    back_handle = sim.getObjectHandle('Back_hoop3')   
    

    -- Subscribing to the required topics 

    aruco_sub = simROS.subscribe('/hoop_3', 'geometry_msgs/Pose', 'aruco_callback')
    whycon_sub = simROS.subscribe('/whycon/poses', 'geometry_msgs/PoseArray', 'whycon_callback')
    key_input = simROS.subscribe('/input_key', 'std_msgs/Int16', 'key_callback')

    aruco_msg = 0 -- Variable to store message from Aruco Callback
    whycon_msg = 0 -- Variable to store message from Whycon Callback
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

    aruco_msg = msg -- Variable to store message from Aruco Callback
    --print(aruco_msg)
end



function whycon_callback(msg)
    -- Get the position of the whycon marker and set the position of the food tree and non-food tree using Position_hoop dummy
    -- Hint : Go through the regular API - sim.setObjectPosition
  

    whycon_msg = msg -- Variable to store message from Whycon Callback


end

function key_callback(msg)
    -- Read key input to set or unset position and orientation of food and non-food trees


---------------------------------------------- SET POSITION OF HOOP 1 ------------------------------------------------------
    -- if msg.data == 500 then
    --     sim.setObjectPosition(emulation,-1,{0.13*whycon_msg.poses[1].position.x,-0.13*whycon_msg.poses[1].position.y,1.1229}) -- 1.12 is the z value of hoop1
    --     sim.setObjectPosition(position_1,emulation,{0,0,0})
    -- end
-------------------------------------------------------------------------------------------------------------------------------




---------------------------------------------- SET POSITION OF HOOP 3 --------------------------------------------------------

    if msg.data == 700 then
        sim.setObjectPosition(emulation,-1,{0.13*whycon_msg.poses[1].position.x,-0.13*whycon_msg.poses[1].position.y,0.47087}) -- 0.62 is the z value of hoop3
        sim.setObjectPosition(position_3,emulation,{0,0,0})
    end
-------------------------------------------------------------------------------------------------------------------------------    





---------------------------------------------- SET POSITION OF OBSTACLE 1  ------------------------------------------------------

    -- if msg.data == 2000 then
    --     sim.setObjectPosition(obstacle,-1,{0.13*whycon_msg.poses[1].position.x,-0.13*whycon_msg.poses[1].position.y,1.3345}) -- 1.3 is the z value of obstacle1
    --     sim.setObjectPosition(position_3,emulation,{0,0,0})
    -- end
-----------------------------------------------------------------------------------------------------------------------------------




---------------------------------------------- SET ORIENTATION OF HOOP 1 ---------------------------------------------------------

    -- if msg.data == 900 then
    --     sim.setObjectQuaternion(hoop1_angle,-1,{-1*aruco_msg.markers[1].pose.pose.orientation.y,  -1*aruco_msg.markers[1].pose.pose.orientation.x,  aruco_msg.markers[1].pose.pose.orientation.w,   aruco_msg.markers[1].pose.pose.orientation.z})

    -- end
---------------------------------------------------------------------------------------------------------------------------------



---------------------------------------------- SET ORIENTATION OF HOOP 3 ------------------------------------------------------


    if msg.data == 1000 then
        sim.setObjectQuaternion(hoop3_angle,-1,{-1*aruco_msg.orientation.y,   -1*aruco_msg.orientation.x,   aruco_msg.orientation.w,  aruco_msg.orientation.z})
        sim.setObjectOrientation(front_handle,-1,{0,0,0})
        sim.setObjectOrientation(back_handle,-1,{0,0,0})
    end
-------------------------------------------------------------------------------------------------------------------------------
end





