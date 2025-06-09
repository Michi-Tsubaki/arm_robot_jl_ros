#!/usr/bin/env julia

using Pkg
Pkg.activate(joinpath(@__DIR__, ".."))

# --- Calculation Packages ---
using CoordinateTransformations
using GeometryBasics: Rect, Point3f, Vec3f
using LinearAlgebra
using RigidBodyDynamics
using RigidBodyDynamics: Transform3D, CartesianFrame3D
using Rotations
using StaticArrays

# --- Visualization Packages ---
using MeshCat
using MeshCatMechanisms
using MechanismGeometries
using Colors

# --- ROS Packages ---
using RobotOS
@rosimport geometry_msgs.msg: Pose
rostypegen()
using .geometry_msgs.msg

# --- rospkg for finding files ---
using PyCall
rospkg = pyimport("rospkg")
rp = rospkg.RosPack()

# Simulator state structure
mutable struct DesktopManipulationDemo
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
function initialize_robot_pose!(sim::DesktopManipulationDemo)
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

"""
Robust Inverse Kinematics (IK) using RigidBodyDynamics.
"""
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
            println("IK converged in $i iterations.")
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

"""
Safely finds the end-effector body for a given arm.
"""
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

"""
Generic IK solver wrapper.
"""
function solve_arm_ik(sim::DesktopManipulationDemo, arm_name::String, target_pose::Transform3D)
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

"""
Enhanced callback with proper Transform3D creation.
"""
function pose_callback(msg::Pose, sim::DesktopManipulationDemo)
    try
        sim.message_count += 1
        println("=== CALLBACK TRIGGERED === (Message #$(sim.message_count))")
        println("Received pose:")
        println("  Position: x=$(msg.position.x), y=$(msg.position.y), z=$(msg.position.z)")
        println("  Orientation: w=$(msg.orientation.w), x=$(msg.orientation.x), y=$(msg.orientation.y), z=$(msg.orientation.z)")
        
        # Check current state before modification
        println("  Current target_pose state: $(sim.target_pose === nothing ? "nothing" : "has_value")")
        
        # Create the transform using RigidBodyDynamics approach
        pos = msg.position
        orient = msg.orientation
        
        println("  Creating translation vector...")
        translation_vec = SVector(pos.x, pos.y, pos.z)
        println("  Translation vector created: $translation_vec")
        
        println("  Creating rotation...")
        rotation = QuatRotation(orient.w, orient.x, orient.y, orient.z)
        println("  Rotation created: $rotation")
        
        println("  Creating Transform3D...")
        # Create a proper Transform3D for RigidBodyDynamics
        # We need to specify frames - using a generic frame for now
        world_frame = CartesianFrame3D("world")
        target_frame = CartesianFrame3D("target")
        
        # Create Transform3D properly
        new_pose = Transform3D(world_frame, target_frame, rotation, translation_vec)
        println("  Transform3D created successfully")
        
        println("  Setting target_pose...")
        sim.target_pose = new_pose
        println("  target_pose set successfully")
        
        # Verify it was set
        println("  Verification - target_pose is now: $(sim.target_pose === nothing ? "nothing" : "has_value")")
        
        println("Target pose received and stored. Processing will happen in main loop.")
        println("=== CALLBACK END ===")
        
    catch e
        println("ERROR IN CALLBACK: $e")
        println("Stack trace:")
        for (exc, bt) in Base.catch_stack()
            showerror(stdout, exc, bt)
            println()
        end
        println("=== CALLBACK ERROR END ===")
    end
end

"""
Main function with enhanced debugging but safer ROS calls.
"""
function main()
    # --- Simulation Setup ---
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
    
    # Initialize the simulation state with `nothing` for the target pose and 0 message count
    sim = DesktopManipulationDemo(vis, robot, mvis, zeros(num_positions(robot)), nothing, 0)
    
    initialize_robot_pose!(sim)
    create_environment(vis)
    println("Robot visualization setup complete.")

    # --- ROS Setup ---
    println("Initializing ROS node...")
    init_node("robot_sim_controller")
    println("ROS node 'robot_sim_controller' initialized successfully.")
    
    # Enhanced subscriber setup with debugging
    println("Creating subscriber for '/larm_target_pose'...")
    sub = Subscriber{Pose}("larm_target_pose", pose_callback, (sim,), queue_size=1)
    println("Subscriber created successfully.")
    
    println("\n" * "="^50)
    println("ROS node initialized. Starting main processing loop...")
    println("To move the left arm, publish a geometry_msgs/Pose to '/larm_target_pose'")
    println("Example command:")
    println("rostopic pub -1 /larm_target_pose geometry_msgs/Pose '{position: {x: 0.4, y: 0.2, z: 0.1}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}'")
    println("="^50)

    # This main loop does the heavy lifting (IK calculation)
    rate = Rate(10) # Set loop frequency to 10 Hz
    loop_count = 0
    
    while !is_shutdown()
        loop_count += 1
        
        # Print status every 50 iterations (every 5 seconds at 10 Hz)
        if loop_count % 50 == 0
            println("Main loop running... (iteration $loop_count) - Messages received: $(sim.message_count)")
            println("  - is_shutdown(): $(is_shutdown())")
            println("  - target_pose is nothing: $(sim.target_pose === nothing)")
        end
        
        # Check if the callback has stored a new target
        if sim.target_pose !== nothing
            println("*** NEW TARGET DETECTED IN MAIN LOOP ***")
            println("Processing target pose...")
            println("  Target pose type: $(typeof(sim.target_pose))")
            
            # Make a local copy of the target
            currentTarget = sim.target_pose
            println("  Local copy created")
            
            # Reset the shared target to `nothing` to indicate it has been processed
            sim.target_pose = nothing 
            println("  Shared target reset to nothing")
            
            # Now, call the IK solver from the main loop, not the callback
            println("Calling IK solver...")
            try
                new_config = solve_arm_ik(sim, "LARM", currentTarget)
                println("  IK solver completed successfully")
                
                # Update the robot's state
                sim.current_config = new_config
                set_configuration!(sim.mvis, new_config)
                println("*** IK PROCESSING COMPLETE - Robot pose updated in visualizer ***")
            catch e
                println("  ERROR in IK solver: $e")
                println("  Stack trace:")
                for (exc, bt) in Base.catch_stack()
                    showerror(stdout, exc, bt)
                    println()
                end
            end
        end
        
        # rossleep allows ROS callbacks to be processed while controlling the loop rate
        rossleep(rate)
    end
end

# Main execution block
if !isinteractive()
    main()
end