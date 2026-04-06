module Kinematics

using NonlinearSolve
using LinearAlgebra
using ..Constraints

# ----------------------------------------------------------------
# Single-Step Solvers
# ----------------------------------------------------------------

"""
    solve_position(q_guess, t)

Solve the nonlinear constraint equations for the generalized coordinates at a given time t.

# Arguments
- 'q_guess': initial guess for q, typically the solution from the previous time step
- 't':       current time step

# Returns
- 'q_sol':   9-element soltion vector q that satisfies all constraints
"""

function solve_position(q_guess, t)
   
    # create the nonlinear problem
    prob = NonlinearProblem(C_nonlinear, q_guess, t)

    # solve using the default Newton-Raphson method
    sol = solve(prob)

    return Vector(sol.u)     # extract the solution vector and return it as a plain vector

end

"""
    solve_velocity(q, t)

Solve the velocity constraint equation for the generalized velocities.

# Arguments
- 'q': current generalized coordinate vector
- 't': current time step

# Returns
- 'dq': 9-element velocity vector
"""

function solve_velocity(q, t)

    # build constraint Jacobian 
    Cq = Cq_jacobian(q)

    # build RHS of the velocity constraint equation
    rhs = -Ct_partial(q, t)

    # solve the linear system
    dq = Cq \ rhs

    return dq

end

"""
    solve_acceleration(q, dq, t)

Solve the acceleration constraint equation for the generalized accelerations.

# Arguments
- 'q':  current generalized coordinate vector
- 'dq': current generalized velocity vector
- 't':  current time step

# Returns
- 'ddq': 9-element acceleration vector
"""

function solve_acceleration(q, dq, t)

    # build constraint Jacobian 
    Cq = Cq_jacobian(q)

    # build RHS of the velocity constraint equation
    rhs = gamma_accel(q, dq)

    # solve the linear system
    ddq = Cq \ rhs

    return ddq

end

# ----------------------------------------------------------------
# Full Kinematic Analysis
# ----------------------------------------------------------------

"""
    run_kinematics(N, t_end)

Run the complete kinematic analysis over one or more full rotations.

# Arguments
- 'N':         number of time steps
- 't_end':     final time (s), one full rotation at omega=2 rad/s takes pi = 3.14 s

# Returns
- 'time':      vector of tme values (length N)
- 'q_all':     Nx9 matrix of generalized coordinates at each time step
- 'dq_all':    Nx9 matrix of generalized velocities at each time step
- 'ddq_all':   Nx9 matrix of generalized accelerations at each time step
"""

function run_kinematics(N, t_end)

    # create a uniformly spaced time vector from 0 to t_end with N points
    time = LinRange(0, t_end, N)

    # initialize storage matrices
    q_all     = zeros(N, 9)
    dq_all    = zeros(N, 9)
    ddq_all   = zeros(N, 9)

    # use the analytical solution at t=0 as the initial guess for the solver
    q_guess = q_analytical(0.0)

    # loop over every time step
    for i in 1:N

        t = time[i]     # current time step

        # ----- Step 1: Solve for Positions ---------
        q = solve_position(q_guess, t)

        # ----- Step 2: Solve for velocities --------
        dq = solve_velocity(q, t)

        # ----- Step 3: Solve for Accelerations -----
        ddq = solve_acceleration(q, dq, t)

        # ----- Step 4: Store Results ---------------
        q_all[i, :]     = q     # save positions for current time step
        dq_all[i, :]    = dq    # save velocities for current time step
        ddq_all[i, :]   = ddq   # save accelerations for current time step

        # ----- Step 5: Update Initial Guess --------
        q_guess         = q     # update initial guess for next time step

    end

    # collect converts time (type LinRange) to type Vector
    return collect(time), q_all, dq_all, ddq_all

end

# ----------------------------------------------------------------
# Validation Against Analytical Solution
# ----------------------------------------------------------------

"""
    compute_errors(time, q_all, dq_all, ddq_all)

Compute the maximum absolute error between numerical and analytical solutions for positions, velocities, and accelerations.

# Arguments
- 'time':      vector of tme values (length N)
- 'q_all':     Nx9 matrix of generalized coordinates at each time step
- 'dq_all':    Nx9 matrix of generalized velocities at each time step
- 'ddq_all':   Nx9 matrix of generalized accelerations at each time step

# Returns
- '(max_q_err, max_dq_err, max_ddq_err)':     maximum absolute compute_errors
"""

function compute_errors(time, q_all, dq_all, ddq_all)

    N = length(time)     # number of time steps

    # initialize error storage
    max_q_err     = 0.0     # max position error across all time steps and coordinates
    max_dq_err    = 0.0     # max velocity error across all time steps and coordinates
    max_ddq_err   = 0.0     # max acceleration error across all time steps and coordinates

    for i in 1:N

        t = time[i]     # current time step

        # compute the analytical solutions at current time step
        q_exact     = q_analytical(t)     # analytical positions
        dq_exact    = dq_analytical(t)    # analytical velocities
        ddq_exact   = ddq_analytical(t)   # analytical accelerations

        # update maximum errors
        max_q_err     = max(max_q_err,     maximum(abs.(q_all[i, :] - q_exact)))
        max_dq_err    = max(max_dq_err,    maximum(abs.(dq_all[i, :] - dq_exact)))
        max_ddq_err   = max(max_ddq_err,   maximum(abs.(ddq_all[i, :] - ddq_exact)))

    end

    return max_q_err, max_dq_err, max_ddq_err

end

# ----------------------------------------------------------------
# Exported Functions/Parameters
# ----------------------------------------------------------------

export solve_position, solve_velocity, solve_acceleration
export run_kinematics, compute_errors

end