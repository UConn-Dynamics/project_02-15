module Visualize

using Plots
using Measures

using ..Constraints: L, half_L, omega_drive
using ..Constraints: C_eqs, q_analytical, dq_analytical, ddq_analytical

# ----------------------------------------------------------------
# Mechanism Drawing Helpers
# ----------------------------------------------------------------

"""
    draw_mechanism!(p, q; track_len=0.12)

Draw the dual-slider crank mechanism at a given configuration.

# Arguments
- 'p':         existing plot object
- 'q':         generalzied coordinate vector
- 'track_len': half length of the track lines to draw (meters)
"""
 function draw_mechanism!(p, q; track_len=0.12)

    # unpack coordinates
    x_1, y_1, theta_1 = q[1], q[2], q[3]     # piston 1 position and orientation
    x_2, y_2, theta_2 = q[4], q[5], q[6]     # piston 2 position and orientation
    x_3, y_3, theta_3 = q[7], q[8], q[9]     # bar center position and orientation

    # ----- Draw Track Lines --------
    # +45 deg track
    alpha_1 = pi / 4

    # x-coordinates of endpoints
    track1_x = [-track_len * cos(alpha_1), track_len * cos(alpha_1)]

    # y-coordinates of endpoints
    track1_y = [-track_len * sin(alpha_1), track_len * cos(alpha_1)]

    # -45 deg track
    alpha_2 = =-pi / 4

    # x-coordinates of endpoints
    track2_x = [-track_len * cos(alpha_2), track_len * cos(alpha_2)]

    # y-coordinates of endpoints
    track2_y = [-track_len * sin(alpha_2), track_len * cos(alpha_2)]

    # plot the tracks
    plot!(p, track1_x, track1_y, color=:gray, lw=2, ls=:dash, label="")
    plot!(p, track2_x, track2_y, color=:gray, lw=2, ls=:dash, label="")

    # plot origin
    scatter!(p, [0.0], [0.0], color=:gray, ms=3, label="")

    # ----- Draw Rigid Bar ---------
    bar_x = [x_1, x_2] # x-coordinates of bar endpoints
    bar_y = [y_1, y_2] # y-coordinates of bar endpoints

    plot!(p, bar_x, bar_y, color=:blue, lw=4, label="")

    # ----- Draw Pistons -----------
    piston_w = 0.025 
    piston_h = 0.008

    draw_piston!(p, x_1, y_1, theta_1, pistom_w, piston_h, :orange)     # draw piston 01
    draw_piston!(p, x_2, y_2, theta_2, pistom_w, piston_h, :green)     # draw piston 02

    # ----- Draw Joint Markers -----
    scatter!(p, [x_1], [y_1], color=:red,   ms=5, markershape=:circle,  label="")     # piston 1
    scatter!(p, [x_2], [y_2], color=:red,   ms=5, markershape=:circle,  label="")     # piston 2
    scatter!(p, [x_3], [y_3], color=:black, ms=5, markershape=:diamond, label="")     # bar COM

    return p

end

"""
    draw_piston!(p, cx, cy, theta, hw, hh, color)

Draw an oriented rectangle resembling a piston centered at (cx, cy) with orientation theta.

# Arguments
- 'p':          plot object
- 'cx, cy':     center coordinates
- 'theta':      orientation angle
- 'hw':         half-width along the pistons local x-axis
- 'hh':         half-height along the pistons local y-axis
- 'color':      fill color
"""

function draw_piston!(p, cx, cy, theta, hw, hh, color)

    local_x = [-hw, hw, hw, -hw]     # x-coordinates of four corners (ccw)
    local_y = [-hh, -hh, hh, hh]     # y-coordinates of four corners

    # transform local coordinates to global coordinates using rotation matrix A(theta)
    global_x = cx .+ cos(theta) .* local_x .- sin(theta) .* local_y
    global_y = cy .+ sin(theta) .* local_x .+ cos(theta) .* local_y
    
    # create a filled rectangle and plot it
    piston_shape = Shape(global_x, global_y)
    plot!(p, piston_shape, c=color, linecolor=:black, alpha=0.5, label="")

end

# ----------------------------------------------------------------
# Animation Function
# ----------------------------------------------------------------

"""
    animate_mechanism(time, q_all; filename="results/mechanism.gif", fps=30)

Animate the dual-slider crank mechanism over time.

# Arguments
- 'time':         vector of all time values
- 'q_all':        matrix of generalized coordinates
- 'filename':     output GIF file path
- 'fps':          frames per second for the animation
"""

function animate_mechanism(time, q_all; filename="results/mechanism.gif", fps=30)

    N = length(time)     # numer of time steps/frames

    # compute axis limits
    all_x = vcat(q_all[:, 1], q_all[:, 4], q_all[:, 7])     # all x-coordinates
    all_y = vcat(q_all[:, 2], q_all[:, 5], q_all[:, 8])     # all y-coordinates
    pad   = 0.04                                            # padding around dq_analytical
    xl    =(minimum(all_x) - pad, maximum(all_x) + pad)     # x-axis limits
    yl    =(minimum(all_y) - pad, maximum(all_y) + pad)     # y-axis limits

    anim = @animate for i in 1:N

        # extract coordinates at each time step
        q_i = q_all[i, :]

        # create plot
        p = plot(
            xlim=xl, ylim=yl, 
            aspect_ratio=:equal,                    # equal scaling on both axes
            xlabel="x (m)", ylabel="y (m)",
            title="Dual-Slider Crank  |  t = $(round(time[i], digits=3)) s",
            legend=false,
            size=(600, 600),
            left_margin=5mm, right_margin=5mm,
            bottom_margin=5mm, top_margin=8mm 
        )

        # draw mechanism at each time step
        draw_mechanism(p, q_i)

    end

    # save animation as GIF
    gif(anim, filename, fps=fps)
    println("Saved animation: $filename")

end
            
# ----------------------------------------------------------------
# Static Kinematic Plots
# ----------------------------------------------------------------

"""
    plot_positions(time, q_all; filename="results/positions_vs_time.png")

Plot generalized coordinates vs. time.
"""

function plot_positions(time, q_all; filename="results/positions_vs_time.png")

    theta_pad = 0.5     # padding for theta axis

    # ----- Panel 1: Piston 1 -----
    p1 = plot(time, q_all[:, 1], label="x_1", lw=2, color=:blue, legend=:topleft)
    plot!(p1, time, q_all[:, 2], label="y_1", lw=2, color=:orange)
    ylabel!(p1, "Position (m)")
    p1r = twinx(p1)
    plot!(p1r, time, q_all[:, 3], label="theta_1", lw=2, ls=:dash, color=:green, legend=:topright)
    theta_1_val = q_all[1, 3]          # pull constant value of theta
    ylims!(p1r, (theta_1_val - pad, theta_1_val + pad))
    ylabel!(p1r, "Angle (rad)")
    title!(p1, "Piston 1 Coordinates")

     # ----- Panel 2: Piston 2 -----
    p2 = plot(time, q_all[:, 4], label="x_2", lw=2, color=:blue, legend=:topleft)
    plot!(p2, time, q_all[:, 5], label="y_2", lw=2, color=:orange)
    ylabel!(p2, "Position (m)")
    p2r = twinx(p2)
    plot!(p2r, time, q_all[:, 6], label="theta_2", lw=2, ls=:dash, color=:green, legend=:topright)
    theta_2_val = q_all[1, 6]          # pull constant value of theta
    ylims!(p2r, (theta_2_val - pad, theta_2_val + pad))
    ylabel!(p2r, "Angle (rad)")
    title!(p2, "Piston 2 Coordinates")

     # ----- Panel 3: Piston 3 -----
    p3 = plot(time, q_all[:, 7], label="x_3", lw=2, color=:blue, legend=;:topleft)
    plot!(p1, time, q_all[:, 8], label="y_3", lw=2, color=:orange)
    ylabel!(p3, "Position (m)")
    p3r = twinx(p3)
    plot!(p3r, time, q_all[:, 9], label="theta_3", lw=2, ls=:dash, color=:green, legend=:topright)
    ylabel!(p3r, "Angle (rad)")
    title!(p3, "Bar Coordinates")

    # combine into 3-row layout
    p = plot(p1, p2, p3, layout=(3,1), size=(800,900),
             left_margin=10mm, right_margin=15mm,
             bottom_margin=5mm, top_margin=5mm)
    
    savefig(p, filename)
    # display(p)
    println("Saved: $filename")

end

"""
    plot_velocities(time, dq_all; filename="results/velocities_vs_time.png")

Plot generalized velocities vs. time.
"""

function plot_velocities(time, dq_all; filename="results/velocities_vs_time.png")

    theta_pad = 0.5     # padding for theta axis

    # ----- Panel 1: Piston 1 -----
    p1 = plot(time, dq_all[:, 1], label="x_dot_1", lw=2, color=:blue, legend=:topleft)
    plot!(p1, time, dq_all[:, 2], label="y_dot_1", lw=2, color=:orange)
    ylabel!(p1, "Velocity (m/s)")
    p1r = twinx(p1)
    plot!(p1r, time, dq_all[:, 3], label="theta_dot_1", lw=2, ls=:dash, color=:green, legend=:topright)
    theta_1_val = dq_all[1, 3]          # pull constant value of theta
    ylims!(p1r, (theta_1_val - pad, theta_1_val + pad))
    ylabel!(p1r, "Anglular Velocity (rad/s)")
    title!(p1, "Piston 1 Velocities")

     # ----- Panel 2: Piston 2 -----
    p2 = plot(time, dq_all[:, 4], label="x_dot_2", lw=2, color=:blue, legend=:topleft)
    plot!(p2, time, dq_all[:, 5], label="y_dot_2", lw=2, color=:orange)
    ylabel!(p2, "Velocity (m/s)")
    p2r = twinx(p2)
    plot!(p2r, time, dq_all[:, 6], label="theta_dot_2", lw=2, ls=:dash, color=:green, legend=:topright)
    theta_2_val = dq_all[1, 6]          # pull constant value of theta
    ylims!(p2r, (theta_2_val - pad, theta_2_val + pad))
    ylabel!(p2r, "Anglular Velocity (rad/s)")
    title!(p2, "Piston 2 Velocities")

     # ----- Panel 3: Piston 3 -----
    p3 = plot(time, dq_all[:, 7], label="x_dot_3", lw=2, color=:blue, legend=;:topleft)
    plot!(p1, time, dq_all[:, 8], label="y_dot_3", lw=2, color=:orange)
    ylabel!(p3, "Velocity (m/s)")
    p3r = twinx(p3)
    plot!(p3r, time, dq_all[:, 9], label="theta_dot_3", lw=2, ls=:dash, color=:green, legend=:topright)
    theta_3_val = dq_all[1, 9]          # pull constant value of theta
    ylims!(p3r, (theta_3_val - pad, theta_3_val + pad))
    ylabel!(p3r, "Anglular Velocity (rad/s)")
    title!(p3, "Bar Velocities")

    # combine into 3-row layout
    p = plot(p1, p2, p3, layout=(3,1), size=(800,900),
             left_margin=10mm, right_margin=15mm,
             bottom_margin=5mm, top_margin=5mm)
    
    savefig(p, filename)
    # display(p)
    println("Saved: $filename")

end

"""
    plot_accelerations(time, ddq_all; filename="results/accelerations_vs_time.png")

Plot generalized accelerations vs. time.
"""

function plot_accelerations(time, ddq_all; filename="results/accelerations_vs_time.png")

    theta_pad = 0.5     # padding for theta axis

    # ----- Panel 1: Piston 1 -----
    p1 = plot(time, ddq_all[:, 1], label="x_ddot_1", lw=2, color=:blue, legend=:topleft)
    plot!(p1, time, ddq_all[:, 2], label="y_ddot_1", lw=2, color=:orange)
    ylabel!(p1, "Acceleration (m/s^2)")
    p1r = twinx(p1)
    plot!(p1r, time, ddq_all[:, 3], label="theta_ddot_1", lw=2, ls=:dash, color=:green, legend=:topright)
    theta_1_val = ddq_all[1, 3]          # pull constant value of theta
    ylims!(p1r, (theta_1_val - pad, theta_1_val + pad))
    ylabel!(p1r, "Anglular Acceleration (rad/s^2)")
    title!(p1, "Piston 1 Accelerations")

     # ----- Panel 2: Piston 2 -----
    p2 = plot(time, ddq_all[:, 4], label="x_ddot_2", lw=2, color=:blue, legend=:topleft)
    plot!(p2, time, ddq_all[:, 5], label="y_ddot_2", lw=2, color=:orange)
    ylabel!(p2, "Acceleration (m/s^2)")
    p2r = twinx(p2)
    plot!(p2r, time, ddq_all[:, 6], label="theta_ddot_2", lw=2, ls=:dash, color=:green, legend=:topright)
    theta_2_val = ddq_all[1, 6]          # pull constant value of theta
    ylims!(p2r, (theta_2_val - pad, theta_2_val + pad))
    ylabel!(p2r, "Anglular Acceleration (rad/s^2)")
    title!(p2, "Piston 2 Accelerations")

     # ----- Panel 3: Piston 3 -----
    p3 = plot(time, ddq_all[:, 7], label="x_ddot_3", lw=2, color=:blue, legend=;:topleft)
    plot!(p1, time, ddq_all[:, 8], label="y_ddot_3", lw=2, color=:orange)
    ylabel!(p3, "Acceleration (m/s^2)")
    p3r = twinx(p3)
    plot!(p3r, time, ddq_all[:, 9], label="theta_ddot_3", lw=2, ls=:dash, color=:green, legend=:topright)
    theta_3_val = ddq_all[1, 9]          # pull constant value of theta
    ylims!(p3r, (theta_3_val - pad, theta_3_val + pad))
    ylabel!(p3r, "Anglular Acceleration (rad/s^2)")
    title!(p3, "Bar Accelerations")

    # combine into 3-row layout
    p = plot(p1, p2, p3, layout=(3,1), size=(800,900),
             left_margin=10mm, right_margin=15mm,
             bottom_margin=5mm, top_margin=5mm)
    
    savefig(p, filename)
    # display(p)
    println("Saved: $filename")

end

# ----------------------------------------------------------------
# Piston Trajectory Plot
# ----------------------------------------------------------------

"""
    plot_piston_paths(time, q_all; filename="results/piston_paths.png")

Plot the (x, y) trajectories of both piston centers over time.
"""

function plot_piston_paths(time, q_all; filename="results/piston_paths.png")

    p = plot(
        aspect_ratio=:equal,
        xlabel="x (m)", ylabel="y (m)",
        title="Piston Center Trajectories",
        legend=:outertopright
    )

    # draw track lines
    # +45 deg track
    alpha_1 = pi / 4

    # x-coordinates of endpoints
    track1_x = [-track_len * cos(alpha_1), track_len * cos(alpha_1)]

    # y-coordinates of endpoints
    track1_y = [-track_len * sin(alpha_1), track_len * cos(alpha_1)]

    # -45 deg track
    alpha_2 = =-pi / 4

    # x-coordinates of endpoints
    track2_x = [-track_len * cos(alpha_2), track_len * cos(alpha_2)]

    # y-coordinates of endpoints
    track2_y = [-track_len * sin(alpha_2), track_len * cos(alpha_2)]

    # plot the tracks
    plot!(p, track1_x, track1_y, color=:gray, lw=2, ls=:dash, label="")
    plot!(p, track2_x, track2_y, color=:gray, lw=2, ls=:dash, label="")

    # plot origin
    scatter!(p, [0.0], [0.0], color=:black, ms=3, markershape=:cross, label="")

    # plot piston 1 trajectory
    plot!(p, q_all[:, 1], q_all[:, 2], lw=2, color=:orange, label="Piston 1")

    # plot piston 1 trajectory
    plot!(p, q_all[:, 4], q_all[:, 5], lw=2, color=:green, label="Piston 2")

    # mark start positions
    scatter!(p, [q_all[1,1]], [q_all[1, 2]], color=:orange, ms=6, label="Piston 1 Start")
    scatter!(p, [q_all[1,4]], [q_all[1, 5]], color=:green, ms=6, label="Piston 2 Start")

    savefig(p, filename)
    # display(p)
    println("Saved: $filename")

end

# ----------------------------------------------------------------
# Bar Center Trajectory Plot
# ----------------------------------------------------------------

"""
    plot_bar_path(time, q_all; filename="results/bar_center_path.png")

Plot the (x, y) trajectory of the bar center, which traces a circle of radius L/2 centered at the orign.
"""

function plot_bar_path(time, q_all; filename="results/bar_center_path.png")

    p = plot(
        q_all[:, 7], q_all[:, 8],
        lw=2, color=:blue, label="Bar Center",
        xlabel="x_3 (m)", ylabel="y_3 (m)",
        title="Bar Center Trajectory"
    )

    # mark start position
    scatter!(p, [q_all[1, 7]], [q_all[1, 8]], color:=red, ms=6, label="Start")

    # draw reference circle of radius L/2
    theta_ref = LinRange(0, 2*pi, 100)         # generate angles for reference circle
    ref_circ_x = half_L .* sin.(theta_ref)     # x-coordinates for reference circle
    ref_circ_y = half_L .* cos.(theta_ref)     # y-coordinates for reference circle
    plot!(p, ref_circ_x, ref_circ_y, color=:gray, ls=:dot, lw=1, label="R = L/2 Circle")

    savefig(p, filename)
    # display(p)
    println("Saved: $filename")

end

# ----------------------------------------------------------------
# Dashboard Animation
# ----------------------------------------------------------------

"""
    animate_dashboard(time, q_all, dq_all, ddq_all; filename="results/dashboard.gif", fps=30)

Create a combined dashboard animation.
"""

function animate_dashboard(time, q_all, dq_all, ddq_all; filename="results/dashboard.gif", fps=30)

    N = length(time)     # numer of time steps/frames

    # compute axis limits for mechanism panel
    all_x = vcat(q_all[:, 1], q_all[:, 4], q_all[:, 7])     # all x-coordinates
    all_y = vcat(q_all[:, 2], q_all[:, 5], q_all[:, 8])     # all y-coordinates
    pad   = 0.04                                            # padding around dq_analytical
    xl    =(minimum(all_x) - pad, maximum(all_x) + pad)     # x-axis limits
    yl    =(minimum(all_y) - pad, maximum(all_y) + pad)     # y-axis limits

    # compute axis limits for other panels (NOT USED, REMOVE THIS)
    x_1_lim = (minimum(q_all[:, 1]) - 0.005, maximum(q_all[:, 1]) + 0.005)     # piston 1 position
    x_2_lim = (minimum(q_all[:, 4]) - 0.005, maximum(q_all[:, 4]) + 0.005)     # piston 2 position
    v_1_lim = (minimum(dq_all[:, 1]) - 0.01, maximum(dq_all[:, 1]) + 0.01)     # piston 1 velocity
    v_2_lim = (minimum(dq_all[:, 4]) - 0.01, maximum(dq_all[:, 4]) + 0.01)     # piston 2 velocity

    anim = @animate for i in 1:N

        # ----- Panel 1: Mechanism Animation -----
        p1 = plot(xlim=xl, ylim=yl, aspect_ratio=:equal, legend=false,
                  xlabel="x (m)", ylabel="y (m)", title="Mechanism")
        draw_mechanism!(p1, q_all[i, :])

        # ----- Panel 2: Piston Positions --------
        p2 = plot(time[1:i], q_all[1:i, 1], label="x_1", lw=2, color=:orange, xlabel="t (s)", ylabel="Position (m)", title="Piston Positions",
                  xlim=(time[1], time[end]))
        
        plot!(p2, time[1:i], q_all[1:i, 4], label="x_2", lw=2, color=:green)
        scatter!(p2, [time[i]], [q_all[i, 1]], color=:orange, ms=4, label="")
        scatter!(p2, [time[i]], [q_all[i, 4]], color=:green, ms=4, label="")
        plot!(p2, legend=:outertopright)

        # ----- Panel 3: Piston Velocities -------
        p3 = plot(time[1:i], dq_all[1:i, 1], label="x_dot_1", lw=2, color=:orange, xlabel="t (s)", ylabel="Velocity (m/s)", title="Piston Velocities",
                  xlim=(time[1], time[end]))
        
        plot!(p3, time[1:i], dq_all[1:i, 4], label="x_dot_2", lw=2, color=:green)
        scatter!(p3, [time[i]], [dq_all[i, 1]], color=:orange, ms=4, label="")
        scatter!(p3, [time[i]], [dq_all[i, 4]], color=:green, ms=4, label="")
        plot!(p3, legend=:outertopright)

        # ----- Panel 4: Piston Accelerations ----
        p4 = plot(time[1:i], ddq_all[1:i, 1], label="x_ddot_1", lw=2, color=:orange, xlabel="t (s)", ylabel="Acceleration (m/s^2)", title="Piston Accelerations",
                  xlim=(time[1], time[end]))
        
        plot!(p4, time[1:i], ddq_all[1:i, 4], label="x_ddot_2", lw=2, color=:green)
        scatter!(p4, [time[i]], [ddq_all[i, 1]], color=:orange, ms=4, label="")
        scatter!(p4, [time[i]], [ddq_all[i, 4]], color=:green, ms=4, label="")
        plot!(p4, legend=:outertopright)

        # combine all 4 panels into a 2x2 generalized
        plot(p1, p2, p3, p4, layout=(2, 2), size=(1000, 800),
             left_margin=8mm, right_margin=5mm,
             bottom_margin=5mm, top_margin=5mm,
             suptitle="Dual-Slider Crank Dashboard  |  t = $(round(time[i], digits=3)) s")

    end

    # save animation as GIF
    gif(anim, filename, fps=fps)
    println("Saved dashboard animation: $filename")

end

# ----------------------------------------------------------------
# Exported Functions/Parameters
# ----------------------------------------------------------------

export draw_mechanism, animate_mechanism
export plot_positions, plot_velocities, plot_accelerations
export plot_piston_paths, plot_bar_path
export animate_dashboard

end


