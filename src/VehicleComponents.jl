module VehicleComponents

using LinearAlgebra

# Hand-written helpers must come before the generated code that calls them
include("vehicle_properties.jl")
include("damper_maps.jl")
include("motor_maps.jl")
include("motec.jl")
include("telemetry.jl")
include("road_surface.jl")

include("../generated/module.jl")

end # module VehicleComponents
