#!/usr/bin/env julia

using Pkg
Pkg.activate(joinpath(@__DIR__, ".."))

# Calculation Packages
using CoordinateTransformations
using GeometryBasics: Rect, Point3f, Vec3f
using LinearAlgebra
using RigidBodyDynamics
using RigidBodyDynamics: Transform3D, CartesianFrame3D
using Rotations
using StaticArrays

# Visualization Packages
using MeshCat
using MeshCatMechanisms
using MechanismGeometries
using Colors

# ROS Packages
using RobotOS
@rosimport geometry_msgs.msg: Pose
rostypegen()
using .geometry_msgs.msg

# rospkg for finding files
using PyCall
rospkg = pyimport("rospkg")
rp = rospkg.RosPack()

# Simulator state structure
mutable struct MeshcatSimulator
    vis::Visualizer
    robot::Mechanism
    mvis::MechanismVisualizer
    current_config::Vector{Float64}
    # Field to store the target pose received from ROS
    target_pose::Union{Transform3D, Nothing}
    # Add a counter to track messages received
    message_count::Int
end

"""
Creates environment objects (desk).
"""
function create_environment(vis::Visualizer)
    println("Creating environment objects...")
    desk_w, desk_d, desk_h = 0.8, 0.8, 0.1
    desk = Rect(Point3f(-desk_w/2, -desk_d/2, -desk_h), Vec3f(desk_w, desk_d, desk_h))
    setobject!(vis["environment"]["desk"], desk,
              MeshPhongMaterial(color=RGBA(0.8, 0.8, 0.8, 0.9)))
    settransform!(vis["environment"]["desk"], Translation(0.4, 0.0, -0.1))
    println("Environment created.")
end

"""
Sets the robot to an initial pose suitable for the task.
"""
function initialize_robot_pose!(sim::MeshcatSimulator)
    println("Initializing robot to a good starting pose...")
    reset_pose_dict = Dict(
        "CHEST_JOINT0" => deg2rad(0.0),
        "RARM_JOINT0" => deg2rad(-10.0), "RARM_JOINT1" => deg2rad(0.0),
        "RARM_JOINT2" => deg2rad(-100.0), "RARM_JOINT3" => deg2rad(25.0),
        "RARM_JOINT4" => deg2rad(-10.0), "RARM_JOINT5" => deg2rad(0.0),
        "LARM_JOINT0" => deg2rad(10.0), "LARM_JOINT1" => deg2rad(0.0),
        "LARM_JOINT2" => deg2rad(-100.0), "LARM_JOINT3" => deg2rad(-25.0),
        "LARM_JOINT4" => deg2rad(-10.0), "LARM_JOINT5" => deg2rad(0.0),
        "HEAD_JOINT0" => deg2rad(0.0), "HEAD_JOINT1" => deg2rad(0.0)
    )
    state = MechanismState(sim.robot)
    for (joint_name, value) in reset_pose_dict
        try
            joint = findjoint(sim.robot, joint_name)
            set_configuration!(state, joint, value)
        catch e
             println("Warning: Could not find or set joint '$joint_name'. Error: $e")
        end
    end
    final_config = configuration(state)
    sim.current_config = final_config
    set_configuration!(sim.mvis, final_config)
    sleep(1.0)
    println("Robot initialization complete.")
end


function jacobian_transpose_ik!(state::MechanismState,
                               target_body::RigidBody,
                               desired_pose::Transform3D;
                               α=0.8, λ=0.1, iterations=50, tolerance=1e-3)
    mechanism = state.mechanism
    world = root_frame(mechanism)
    tcp = Point3D(default_frame(target_body), 0.0, 0.0, 0.0)
    p = path(mechanism, root_body(mechanism), target_body)
    Jp = point_jacobian(state, p, transform(state, tcp, world))

    for i in 1:iterations
        pos_error = translation(desired_pose) - translation(transform_to_root(state, target_body))
        if norm(pos_error) < tolerance
            #println("IK converged in $i iterations.")
            return configuration(state)
        end
        point_jacobian!(Jp, state, p, transform(state, tcp, world))
        Jp_mat = Array(Jp)
        Δq = Jp_mat' * ((Jp_mat * Jp_mat' + (λ^2) * I) \ pos_error)
        q = configuration(state) .+ α * Δq
        set_configuration!(state, q)
    end
    println("IK did not converge after $iterations iterations.")
    return configuration(state)
end

function find_end_effector(robot::Mechanism, arm_name::String)
    candidates = arm_name == "RARM" ? ["RARM_JOINT5_Link", "RARM_JOINT6_Link"] : ["LARM_JOINT5_Link", "LARM_JOINT6_Link"]
    for name_candidate in candidates
        try
            return findbody(robot, name_candidate)
        catch
            continue
        end
    end
    error("Could not find any suitable end-effector body for '$arm_name'")
end

function solve_arm_ik(sim::MeshcatSimulator, arm_name::String, target_pose::Transform3D)
    state = MechanismState(sim.robot)
    set_configuration!(state, sim.current_config)
    target_body = find_end_effector(sim.robot, arm_name)
    try
        return jacobian_transpose_ik!(state, target_body, target_pose)
    catch e
        println("Error during Jacobian IK for $arm_name: ", e)
        return sim.current_config
    end
end

function pose_callback(msg::Pose, sim::MeshcatSimulator)
    try
        sim.message_count += 1
        pos = msg.position
        orient = msg.orientation        
        translation_vec = SVector(pos.x, pos.y, pos.z)
        rotation = QuatRotation(orient.w, orient.x, orient.y, orient.z)
        world_frame = CartesianFrame3D("world")
        target_frame = CartesianFrame3D("target")        
        new_pose = Transform3D(world_frame, target_frame, rotation, translation_vec)
        sim.target_pose = new_pose
    catch e
        println("ERROR IN CALLBACK: $e")
        println("Stack trace:")
        for (exc, bt) in Base.catch_stack()
            showerror(stdout, exc, bt)
            println()
        end
    end
end

# main
function main()
    vis = Visualizer()
    open(vis)

    package_paths = [rp.get_path("nextage_description"), joinpath(rp.get_path("nextage_description"), "..")]
    urdf_path = joinpath(package_paths[1], "urdf", "NextageOpen.urdf")
    
    if !isfile(urdf_path)
        println("URDF file not found at: ", urdf_path)
        return
    end

    robot = parse_urdf(urdf_path, remove_fixed_tree_joints=false)
    delete!(vis)
    urdf_visuals = URDFVisuals(urdf_path, package_path=package_paths)
    mvis = MechanismVisualizer(robot, urdf_visuals, vis)
    
    sim = MeshcatSimulator(vis, robot, mvis, zeros(num_positions(robot)), nothing, 0)
    
    initialize_robot_pose!(sim)
    create_environment(vis)

    init_node("robot_sim_controller")
    sub = Subscriber{Pose}("larm_target_pose", pose_callback, (sim,), queue_size=1)
    rate = Rate(10)
    loop_count = 0
    
    while !is_shutdown()
        if sim.target_pose !== nothing
            currentTarget = sim.target_pose
            sim.target_pose = nothing 
            try
                new_config = solve_arm_ik(sim, "LARM", currentTarget)
                sim.current_config = new_config
                set_configuration!(sim.mvis, new_config)
            catch e
                for (exc, bt) in Base.catch_stack()
                    showerror(stdout, exc, bt)
                    println()
                end
            end
        end
        rossleep(rate)
    end
end

# main
main()
