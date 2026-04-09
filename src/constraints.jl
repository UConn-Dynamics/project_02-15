module Constraints

# ----------------------------------------------------------------
# Physical Constants
# ----------------------------------------------------------------

const L               = 0.10     # rigid bar length (m)
const omega_drive     = 2.0      # driving angular velocity for theta_3 (rad/s)
const half_L          = L / 2    # half-length of the bar, used for endpoint offsets

# ----------------------------------------------------------------
# Constraint Equations
# ----------------------------------------------------------------

"""
     C_eqs(q, t)

Evaluate the 9 constraint equations.

# Arguments
- 'q': generalized coordinate vector [x_1, y_1, theta_1, x_2, y_2, theta_2, x_3, y_3, theta_3]
- 't': current time step

# Returns
- A 9-element vector; each element should be zero when constraints are satisfied.
"""

function C_eqs(q, t)

    # unpack generalized coordinates
    x_1        = q[1]     # x-position of Piston 01 center
    y_1        = q[2]     # y-position of Piston 01 center
    theta_1    = q[3]     # orientation of Piston 01
    x_2        = q[4]     # x-position of Piston 02 center
    y_2        = q[5]     # y-position of Piston 02 center
    theta_2    = q[6]     # orientation of Piston 01
    x_3        = q[7]     # x-position of bar center
    y_3        = q[8]     # y-position of barcenter
    theta_3    = q[9]     # orientation of the bar

    return [
        y_1 - x_1,
        theta_1 - pi/4,
        y_2 + x_2,
        theta_2 + pi/4,
        x_1 - x_3 + half_L * cos(theta_3),
        y_1 - y_3 + half_L * sin(theta_3),
        x_2 - x_3 - half_L * cos(theta_3),
        y_2 - y_3 - half_L * sin(theta_3),
        theta_3 - omega_drive * t
    ]

end

# ----------------------------------------------------------------
# Wrapper for NonlinearSolve
# ----------------------------------------------------------------

"""
     C_nonlinear(q, t)

Wrapper around C_eqs for use with NonlinearSolve, wich expects the signature f(u, p) where u is the unknown vector and p is a parameter. Here p = t (time).
"""
function C_nonlinear(q, t)

    return C_eqs(q, t)

end

# ----------------------------------------------------------------
# Jacobian: C_q = dC/dq
# ----------------------------------------------------------------

"""
    Cq_jacobian(q)

Compute the 9x9 constraint Jacobian matrix.

# Arguments
- 'q': generalized coordinate vector

# Returns
- 9x9 matric of partial derivatives
"""

function Cq_jacobian(q)
    
    theta_3 = q[9]

    return [
        -1.0  1.0  0.0  0.0  0.0  0.0  0.0  0.0  0.0                     ; # dC_1/dq
        0.0   0.0  1.0  0.0  0.0  0.0  0.0  0.0  0.0                     ; # dC_2/dq
        0.0   0.0  0.0  1.0  1.0  0.0  0.0  0.0  0.0                     ; # dC_3/dq
        0.0   0.0  0.0  0.0  0.0  1.0  0.0  0.0  0.0                     ; # dC_4/dq
        1.0   0.0  0.0  0.0  0.0  0.0 -1.0  0.0 -half_L*sin(theta_3)     ; # dC_5/dq
        0.0   1.0  0.0  0.0  0.0  0.0  0.0 -1.0  half_L*cos(theta_3)     ; # dC_6/dq
        0.0   0.0  0.0  1.0  0.0  0.0 -1.0  0.0  half_L*sin(theta_3)     ; # dC_7/dq
        0.0   0.0  0.0  0.0  1.0  0.0  0.0 -1.0 -half_L*cos(theta_3)     ; # dC_8/dq
        0.0   0.0  0.0  0.0  0.0  0.0  0.0  0.0  1.0                     ; # dC_9/dq         
         
    ]

end

# ----------------------------------------------------------------
# Partial Time Derivative: C_t = dC/dt
# ----------------------------------------------------------------

"""
    Ct_partial(q, t)

Compute the partial time derivative vector.

# Arguments
- 'q': generalized coordinate vector (not used here, but kept for generality)
- 't': current time step (not used here, but kept for generality)

# Returns
- 9-element vector of partial time derivatives
"""

function Ct_partial(q, t)

    return [
        0.0,               # dC_1/dt
        0.0,               # dC_2/dt
        0.0,               # dC_3/dt
        0.0,               # dC_4/dt
        0.0,               # dC_5/dt
        0.0,               # dC_6/dt
        0.0,               # dC_7/dt
        0.0,               # dC_8/dt
        -omega_drive       # dC_9/dt
    ]

end

# ----------------------------------------------------------------
# Acceleration RHS: gamma = -(dC_q/dt)q_dot - dC_t/dt
# ----------------------------------------------------------------

"""
    gamma_accel(q, dq)

Compute the RHS vector (gamma) of the acceleration equation.

# Arguments
- 'q':  generalized coordinate vector
- 'dq': generalized velocity vector 

# Returns
- 9-element vector, gamma
"""

function gamma_accel(q, dq)

    theta_3  = q[9]
    theta_3d = dq[9]

    return [
        0.0,                                    # gamma_1
        0.0,                                    # gamma_2
        0.0,                                    # gamma_3
        0.0,                                    # gamma_4
        half_L * cos(theta_3) * theta_3d^2,     # gamma_5
        half_L * sin(theta_3) * theta_3d^2,     # gamma_6
        -half_L * cos(theta_3) * theta_3d^2,    # gamma_7
        -half_L * sin(theta_3) * theta_3d^2,    # gamma_8
        0.0                                     # gamma_9
    ]

end

# ----------------------------------------------------------------
# Analytical Position Solver
# ----------------------------------------------------------------

"""
    q_analytical(t)

Compute the exact analytical solution for q at a given time step.
"""
function q_analytical(t)

    theta_3 = omega_drive * t 
    stheta = sin(theta_3)               # pre-compute sin(theta_3) for efficiency
    ctheta = cos(theta_3)               # pre-compute cos(theta_3) for efficiency

    x_3 = -half_L * stheta              # bar center x-position
    y_3 = -half_L * ctheta              # bar center y-position

    x_1 = -half_L * (stheta + ctheta)   # piston 1 center x-position
    y_1 = -half_L * (stheta + ctheta)   # piston 1 center y-position (= x_1)
    theta_1 = pi / 4                    # piston 1 orientation 

    x_2 = half_L * (ctheta - stheta)    # piston 2 center x-position
    y_2 = half_L * (stheta - ctheta)    # piston 2 center y-position (= -x_2)
    theta_2 = -pi / 4                   # piston 3 orientation

    return [x_1, y_1, theta_1, x_2, y_2, theta_2, x_3, y_3, theta_3] # return full coordinate vector

end

# ----------------------------------------------------------------
# Analytical Velocity Solver
# ----------------------------------------------------------------

"""
    dq_analytical(t)

Compute the exact analytical solution for dq at a given time step.
"""

function dq_analytical(t)

    theta_3 = omega_drive * t 
    stheta = sin(theta_3)               # pre-compute sin(theta_3) for efficiency
    ctheta = cos(theta_3)               # pre-compute cos(theta_3) for efficiency

    dx_3 = -half_L * ctheta * omega_drive     # bar center x-velocity
    dy_3 = half_L * stheta * omega_drive      # bar center y-velocity
    dtheta_3 = omega_drive
    
    dx_1 = half_L * (stheta - ctheta) * omega_drive     # piston 1 center x-velocity
    dy_1 = half_L * (stheta - ctheta) * omega_drive     # piston 1 center y-velocity (= dx_1)
    dtheta_1 = 0.0
    
    dx_2 = -half_L * (stheta + ctheta) * omega_drive    # piston 2 center x-velocity
    dy_2 = half_L * (stheta + ctheta) * omega_drive     # piston 2 center y-velocity (= -dx_2)
    dtheta_2 = 0.0

    return [dx_1, dy_1, dtheta_1, dx_2, dy_2, dtheta_2, dx_3, dy_3, dtheta_3]

end

# ----------------------------------------------------------------
# Analytical Acceleration Solver
# ----------------------------------------------------------------

"""
    ddq_analytical(t)

Compute the exact analytical solution for ddq at a given time step.
"""

function ddq_analytical(t)

    theta_3 = omega_drive * t 
    stheta = sin(theta_3)               # pre-compute sin(theta_3) for efficiency
    ctheta = cos(theta_3)               # pre-compute cos(theta_3) for efficiency

    ddx_3 = half_L * stheta * omega_drive^2     # bar center x-acceleration
    ddy_3 = half_L * ctheta * omega_drive^2     # bar center y-acceleration
    ddtheta_3 = 0.0

    ddx_1 = half_L * (ctheta + stheta) * omega_drive^2     # piston 1 center x-acceleration
    ddy_1 = half_L * (ctheta + stheta) * omega_drive^2     # piston 1 center y-acceleration (= ddx_1)
    ddtheta_1 = 0.0

    ddx_2 = half_L * (stheta - ctheta) * omega_drive^2     # piston 2 center x-acceleration
    ddy_2 = -half_L * (stheta - ctheta) * omega_drive^2    # piston 2 center y-acceleration (= -ddx_2)
    ddtheta_2 = 0.0

    return [ddx_1, ddy_1, ddtheta_1, ddx_2, ddy_2, ddtheta_2, ddx_3, ddy_3, ddtheta_3]

end

# ----------------------------------------------------------------
# Exported Functions/Parameters
# ----------------------------------------------------------------

export C_eqs, C_nonlinear, Cq_jacobian, Ct_partial, gamma_accel
export q_analytical, dq_analytical, ddq_analytical
export L, omega_drive, half_L

end

