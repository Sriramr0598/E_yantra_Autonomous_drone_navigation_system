-- This script is used for realtime emulation of the environment in V-REP

function sysCall_init()

    -- Add required handles here
    drone_handle = sim.getObjectHandle('Drone_Pos_Emulation');




    -- Subscribing to the required topic
     whycon_sub = simROS.subscribe('/filtered_position', 'geometry_msgs/Pose', 'filter_callback')
end


function sysCall_actuation()

end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end


function filter_callback(msg)
    -- Get the position of the real-world whycon marker and set the position of the drone in the simulator.
    x = 0.1033*msg.position.x
    y = -0.1033*msg.position.y
    z = ((msg.position.z-40)*((3.2)/(-1*40))) --38.5
    sim.setObjectPosition(drone_handle,-1,{x,y,z})

end
