# ------------------------------------------------------------------------
# Main Driver Script for Dual-Slider Crank Simulation
# ------------------------------------------------------------------------
# Includes modules, runs simulations, generates plots/animations
# ------------------------------------------------------------------------

include("src/constraints.jl")
include("src/kinematics.jl")
include("src/visualize.jl")

using .Constraints
using .Kinematics
using .Visualize
using Plots

# ------------------------------------------------------------------------
# Main Simulation
# ------------------------------------------------------------------------

function main()

    # ----- Simulation Parameters ----------------------------------------
    N          = 500                            # number of time steps
    n_rot      = 2                              # number of full rotations to simulate
    t_end      = n_rot * 2*pi / omega_drive     # total simulation time for two full rotations (approx. 6.28 s)

    println("= ^ 120")
    println("ME 5180 | Project 02 | Dual-Slider Crank Kinematics")
    println("= ^ 120")
    println("Bar Length:          L = $(L) m"
    println("Driving Speed:       omega = $(omega_drive) rad/s")
    println("Number of steps:     N = $(N)")
    println("Simulation time:     t_end = $(round(t_end, digits=4)) s")
    println("Full rotations:      $(n_rot)")
    println("= ^ 120")

    # ----- Create Results Directory (if one doesnt exist) ---------------
    mkpath("results")

    # ----- Run Kinematic Analysis ---------------------------------------
    println("\nRunning kinematic analysis...")
    time, q_all, dq_all, ddq_all = run_kinematics(N, t_end)
    println("Kinematic analysis complete.")

    # ----- Validate Against Analytical Solution -------------------------
    println("\nValidating against analytical solution...")
    max_q_err, max_dq_err, max_ddq_err = compute_errors(time, q_all, dq_all, ddq_all)
    println(" Max position error:              $(max_q_err)")
    println(" Max velocity error:              $(max_dq_err)")
    println(" Max acceleration error:          $(max_ddq_err)")

    # check that errors are effectively zero
    if max_q_err < 1e-10 && max_dq_err < 1e-10 && max_ddq_err < 1e-10
        println(" All arrors are within floating point tolerance.")
    else
        println(" Errors exceed expected tolerance, check implementation!")
    end

    # ----- Verify Constraint Satisfaction -------------------------------
    println("\nChecking constraint satisfaction...")
    max_residual = 0.0 
    for i in 1:length(time)
        C_val = C_eqs(q_all[i, :], time[i])          # evaluate constraint equations
        residual = sqrt(sum(C_val .^2))              # Euclidean norm
        max_residual = max(max_residual, residual)   # update max
    end
    println(" Max constraint residual: $(max_residual)")

    # ----- Generate Static Plots ----------------------------------------
    println("\nGenerating plots...")

    # position vs time
    plot_positions(time, q_all, filename="results/positions_vs_time.png")

    # velocity vs time
    plot_velocities(time, dq_all, filename="results/velocities_vs_time.png")

    # acceleration vs time
    plot_accelerations(time, ddq_all, filename="results/accelerations_vs_time.png")

    # piston center trajectories
    plot_piston_paths(time, q_all, filename="results/piston_paths.png")

    # bar center trajectory
    plot_bar_path(time, q_all, filename="results/bar_center_path.png")

    # ----- Generate Animations ------------------------------------------
    println("\nGenerating animations...")

    # mechanism animation
    animate_mechanism(time, q_all, filename="results/mechanism.gif", fps=30)

    # dashboard animation
    animate_dashboard(time, q_all, dq_all, ddq_all; filename="results/dashboard.gif", fps=30)

    # ----- Summary ------------------------------------------------------
    println("\n" * "=" ^ 120)
    println("All outputs saved to results/ directory:")
    println("     Static Plots:")
    println("          - positions_vs_time.png")
    println("          - velocities_vs_time.png")
    println("          - accelerations_vs_time.png")
    println("          - piston_paths.png")
    println("          - bar_center_path.png")
    println("     Animations:")
    println("          - mechanism.gif")
    println("          - dashboard.gif")
    println("=" ^ 120)
    
end

# ------------------------------------------------------------------------
# Main Driver Call
# ------------------------------------------------------------------------

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end