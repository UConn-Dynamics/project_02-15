


# ------------------------------------------------------------------------
# Main Driver Function
# ------------------------------------------------------------------------

include("src/constraints.jl")
include("src/kinematics.jl")
include("src/visualize.jl")

using .Constraints
using .Kinematics
using .Visualize
using Plots


function main()

end

# ------------------------------------------------------------------------
# Main Driver Call
# ------------------------------------------------------------------------

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end