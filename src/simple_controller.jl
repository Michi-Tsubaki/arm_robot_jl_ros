#!/usr/bin/env julia

using RobotOS
@rosimport geometry_msgs.msg: Pose
rostypegen()
using .geometry_msgs.msg

function circle_trajectory_publisher()
    init_node("circle_trajectory_publisher")
    pub = Publisher{Pose}("larm_target_pose", queue_size=10)
    center_x = 0.4
    center_y = 0.0
    center_z = 0.1
    radius = 0.15
    angular_velocity = 0.5  # rad/s
    publish_rate = 20.0     # Hz
    loop_rate = Rate(publish_rate)
    start_time = time()
    while !is_shutdown()
        current_time = time() - start_time
        angle = angular_velocity * current_time
        x = center_x + radius * cos(angle)
        y = center_y + radius * sin(angle)
        z = center_z
        pose_msg = Pose()
        pose_msg.position.x = x
        pose_msg.position.y = y
        pose_msg.position.z = z
        pose_msg.orientation.w = 1.0
        pose_msg.orientation.x = 0.0
        pose_msg.orientation.y = 0.0
        pose_msg.orientation.z = 0.0
        publish(pub, pose_msg)
        if current_time % 1.0 < (1.0 / publish_rate)
            println("Time: $(round(current_time, digits=1))s, Angle: $(round(rad2deg(angle), digits=1))°, Position: ($(round(x, digits=3)), $(round(y, digits=3)), $(round(z, digits=3)))")
        end
        rossleep(loop_rate)
    end
    println("Circle trajectory publisher stopped.")
end


function figure_eight_trajectory_publisher()
    init_node("figure_eight_trajectory_publisher")
    pub = Publisher{Pose}("larm_target_pose", queue_size=10)
    center_x = 0.4
    center_y = 0.0
    center_z = 0.1
    scale = 0.1
    angular_velocity = 0.4
    publish_rate = 20.0
    loop_rate = Rate(publish_rate)
    start_time = time()
    while !is_shutdown()
        current_time = time() - start_time
        t = angular_velocity * current_time
        x = center_x + scale * sin(t)
        y = center_y + scale * sin(2*t)
        z = center_z
        pose_msg = Pose()
        pose_msg.position.x = x
        pose_msg.position.y = y
        pose_msg.position.z = z
        pose_msg.orientation.w = 1.0
        pose_msg.orientation.x = 0.0
        pose_msg.orientation.y = 0.0
        pose_msg.orientation.z = 0.0
        publish(pub, pose_msg)
        if current_time % 1.0 < (1.0 / publish_rate)
            println("Time: $(round(current_time, digits=1))s, Position: ($(round(x, digits=3)), $(round(y, digits=3)), $(round(z, digits=3)))")
        end
        rossleep(loop_rate)
    end
end

function main()
    if length(ARGS) == 0
        println("Usage: julia circle_publisher.jl [circle|eight]")
        println("Default: circle")
        circle_trajectory_publisher()
    elseif ARGS[1] == "circle"
        circle_trajectory_publisher()
    elseif ARGS[1] == "eight"
        figure_eight_trajectory_publisher()
    else
        circle_trajectory_publisher()
        println("Unknown trajectory type: $(ARGS[1])")
    end
end

if !isinteractive()
    main()
end
