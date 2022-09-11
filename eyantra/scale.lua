--[[
This child script is used to determine the scale factor ie a scale factor by which you can transform the coordinate points from world frame (vrep) to whycon frame and vice-versa
Here a basic method is suggested (there are other efficient ways like using tf in ros). you are free to experiment and discover your own method. Your target is that you should transform coordinate from one frame of reference to the other
--]]

function sysCall_init()
    --get handles of whycons and test dummy
    dummy_handle = sim.getObjectHandle('Dummy')
    whycon_1_handle = sim.getObjectHandle('whycon_1')
    whycon_2_handle = sim.getObjectHandle('whycon_2')

    -- getting the position of vision sensor and the value of z coordinate of whycon marker when placed on the floor.
    vision_sensor_position = sim.getObjectPosition(sim.getObjectHandle('Vision_sensor'),-1)
    -- whycon_ground_z_value = --uncomment add the value

    whycon_scale = {nil,nil,nil}
    whycon_1 = {}
    whycon_2 = {}

    whycon_sub = simROS.subscribe('/whycon/poses', 'geometry_msgs/PoseArray', 'whycon_callback')
    whycon_sub = simROS.subscribe('/check_whycon_to_real', 'geometry_msgs/Vector3', 'whycon_to_real')
    whycon_sub = simROS.subscribe('/check_real_to_whycon', 'geometry_msgs/Vector3', 'real_to_whycon')
end

-- Function for whycon callback and the function where you can compute your scale factor.
function whycon_callback(msg)
    x1 = msg.poses[1].position.x
    y1 = msg.poses[1].position.y
    z1 = msg.poses[1].position.z

    whycon_1 = {x1,y1,z1}

    x2 = msg.poses[2].position.x
    y2 = msg.poses[2].position.y
    z2 = msg.poses[2].position.z

    whycon_2 = {x2,y2,z2}

    position_whycon_1 = sim.getObjectPosition(whycon_1_handle,-1)
    position_whycon_2 = sim.getObjectPosition(whycon_2_handle,-1)

    --------------------Compute the scale factor here------------------





    -------------------------------------------------------------------

end

-- Function to check if the scale factor is correct by converting vrep coordinates to whycon coordinates.
-- Once this callback is called, ideally the following should happen
-- (1) 'Dummy' gets translated to the input real_world value which you have published
-- (2) whycon_1 marker gets translated to the same position as that of the 'Dummy'
-- (3) Compare the accuracy of value by comparing the whycon_1 coordinate in the image and the output whycon coordinates printing in the vrep console (code_line : 64).

function real_to_whycon(msg)

    -- whycon_world_value = {,,,}   -- uncomment and compute the whycon_world value

    sim.setObjectPosition(dummy_handle,-1,{msg.x,msg.y,msg.z})
    sim.setObjectPosition(whycon_1_handle,dummy_handle,{0,0,0})
    print ("Corresponding whycon_world_value: ",whycon_world_value)

end


-- Function to check if the scale factor is correct by converting whycon coordinates to vrep coordinates.
-- Once this callback is called, ideally the following should happen
-- (1) 'Dummy' gets set at the computed real_world value
-- (2) whycon_1 marker gets translated to the same position as that of the 'Dummy'
-- (3) Compare the accuracy of value by comparing the whycon_1 coordinate in the image and the input whycon coordinate you have published

function whycon_to_real(msg)

    -- compute the real_world_value by multiplying or dividing the input msg.x,msg.y and msg.z with the scale factor.
    -- Hint : You will have to consider the height of the vision sensor to compute the corresponding z value.

    -- real_world_value = {,,,} --uncomment and compute the real world value
    sim.setObjectPosition(dummy_handle,-1,{real_world_value[1] , real_world_value[2], real_world_value[3]})
    sim.setObjectPosition(whycon_1_handle,dummy_handle,{0,0,0})

end

function sysCall_actuation()
    -- put your actuation code here
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end