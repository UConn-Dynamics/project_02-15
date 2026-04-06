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

 