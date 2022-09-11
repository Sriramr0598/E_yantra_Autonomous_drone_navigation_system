function sysCall_init()
    -- This child script contains computing path and publishing it into the path_planning.py for waypoint navigation.
    

    -- Declaring required handles
    drone_handle = sim.getObjectHandle('Dummy')
    collection_handles= sim.getCollectionHandle('Obstacles')

    --Assigning obstacles handles
    no_of_obstacles = 6
    obstacles_handles = {}
    for i=1,no_of_obstacles do
        table.insert(obstacles_handles,sim.getObjectHandle('obstacle_'..tostring(i)))
    end

    -----------Add other required handles here----------------

    ------------- Goal Handles -----------------------------
    goal_1_handle = sim.getObjectHandle('goal_1')
    goal_2_handle = sim.getObjectHandle('goal_2')
    goal_3_handle = sim.getObjectHandle('goal_3')

    initial_waypoint_handle = sim.getObjectHandle('initial_waypoint')

    goal_id = 0 -- Id to identify which goal to set path to





    --Hint : Goal handles and other required handles
    ----------------------------------------------------------
    t=simOMPL.createTask('t')
    ss={simOMPL.createStateSpace('6d',simOMPL.StateSpaceType.pose3d,drone_handle,{-2,-2,0},{2,2,2},1)}
    simOMPL.setStateSpace(t,ss)
    simOMPL.setAlgorithm(t,simOMPL.Algorithm.RRTConnect)
    simOMPL.setCollisionPairs(t,{sim.getObjectHandle('eDrone_outer'),collection_handles})


    ------------Add the path planning task initial details------------------
    no_of_path_points_required = 50

    compute_path_flag = false


    --Hint : Creating task, statespace creation, algorithm setting and setting collision pairs
    -- Carefull about the bounds and the collision pairs you set.
    --------------------------------------------------------------------------


    --------------------Add your publisher and subscriber nodes here ---------------------


    -------------- Publishes to Python file -------------------------------------------
    path_pub=simROS.advertise('/vrep/waypoints', 'geometry_msgs/PoseArray')    -- Publish the computed path under the topic /vrep/waypoints


   -------------------------- Subscribes to Python file ------------------------------------------
    path_request_sub = simROS.subscribe('/path_request','std_msgs/Int64','path_callback')





    -- Hint : You will require to subscribe to a topic published by path_planning.py indicating the need of new path once all waypoints are covered. THIS IS IMPORTANT
    ---------------------------------------------------------------------------------------


    -- scale_factor = {} -- Add the scale_factor you computed learned from the tutorial of whycon transformation
    -- no_of_path_points_required = -- Add no of path points you want from one point to another

end

function getpose(handle,ref_handle)
    position = sim.getObjectPosition(handle,ref_handle)
    orientation = sim.getObjectQuaternion(handle,ref_handle)
    pose = {position[1],position[2],position[3],orientation[1],orientation[2],orientation[3],orientation[4]}
    return pose
end
-- This function can be used to visualize the path you compute. This function takes path points as the argument...
-- GO through the code segment for better understanding
function visualizePath( path )
    if not _lineContainer then
        _lineContainer=sim.addDrawingObject(sim.drawing_lines,1,0,-1,99999,{0.2,0.2,1})
    end
    sim.addDrawingObjectItem(_lineContainer,nil)
    if path then
        local pc=#path/7
        for i=1,pc-1,1 do
            lineDat={path[(i-1)*7+1],path[(i-1)*7+2],path[(i-1)*7+3],path[i*7+1],path[i*7+2],path[i*7+3]}
            sim.addDrawingObjectItem(_lineContainer,lineDat)
        end
    end
end

function path_callback(msg)
    --print(msg)
    compute_path_flag = true
    goal_id = msg.data
    
end



-- This function is used to send the Path computed in the real_world to whycon_world after transformation
-- Message is to be sent in geometry_msgs/PoseArray as WhyCon node subscribes to that message
function packdata(path)

    local sender = {header = {}, poses = {}}
    
    sender['header']={seq=123, stamp=simROS.getTime(), frame_id="drone"}
    sender['poses'] = {}

    for i=1,((no_of_path_points_required-1)*7)+1,7 do
        a = {x = 0, y = 0, w = 0, z = 0}
        b = {x = 0, y = 0, z = 0}
        pose = {position = b, orientation = a, }
--------- Converting from real to whycon ------------------------
        pose.position.x = (path[i]*-7.55)
        pose.position.y = (path[i+1]*-7.55)
        pose.position.z = (((-55.6/3.048))*path[i+2]) + 55.6  ------ 55.6 is the ground z value in whycon, 3.048 is height of camera in real world
        sender.poses[math.floor(i/7) + 1] = pose
    end
    return sender
end


--- This function is used to compute and publish the path to path_planninglpy
function compute_and_send_path(task)
    --print('hey')
    local r
    local path

    r,path=simOMPL.compute(t,10,-1,no_of_path_points_required) -- Provide the correct arguments here.. Make sure you check the no of path points it actually computes
    --print(r,path)
    if(r == true) then
        visualizePath(path)
        message = packdata(path)  
        simROS.publish(path_pub,message)
        -- Provide slots for debug to cross check the message or path points you recieve after computing
    end
    return r
end


function sysCall_actuation()
    
    ---- Add your code to set start and goal state after getting present poses of the drone and the new goal when path_planning.py request you a path
    ---------------------------------------------------------------------------------------------------------------
    if compute_path_flag == true then
 ----------- To choose which path to compute based on goal_id --------------------------------------
        if goal_id == 0 then
            goal_pose = getpose(goal_1_handle,-1)
        end
        if goal_id == 1 then
            goal_pose = getpose(goal_2_handle,-1)
        end
        if goal_id == 2 then
            goal_pose = getpose(goal_3_handle,-1)
        end
        if goal_id == 3 then
            goal_pose = getpose(initial_waypoint_handle,-1)
        end





        drone_pose = getpose(drone_handle,-1)
        -- Getting the goalpose
        -- Setting start state
        simOMPL.setStartState(t,drone_pose)
        -- Setting goal state but the orientation is set same as that of startpose
        simOMPL.setGoalState(t,{goal_pose[1],goal_pose[2],goal_pose[3],drone_pose[4],drone_pose[5],drone_pose[6],drone_pose[7]})
        -- Computing path and publishing path points
        status = compute_and_send_path(t)
        if(status == true) then -- path computed
            compute_path_flag = false
        end
    end 








    ----------------------------------------------------------------------------------------------------------------
    --Hint : Set orientation of the goal as same as that of the start pose as you don't want the drone to rotate on its yaw

end


function sysCall_sensing()

end



function sysCall_cleanup()

end

