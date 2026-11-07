if ARGS == ["--precompile-only"]
    @info "Test environment prepared; skipping simulations"
    exit(0)
end

using VehicleComponents
using Test
using ADTypes: AutoFiniteDiff
using DyadInterface: ODEAlg
using OrdinaryDiffEqRosenbrock: Rodas5P

selected_test = isempty(ARGS) ? nothing : basename(only(ARGS))

function __dyad_run_test_case!(
    rig::Function, name::AbstractString; component_stem, solver, kwargs...)
    if selected_test !== nothing && selected_test != component_stem * "_test.jl"
        return
    end

    if solver == ODEAlg.Rodas5P() && rig in (
        FullVehicleTestStraightLine, FullVehicleTestConstantSteer,
        FullVehicleTestInertiaYaw, FullVehicleTestInertiaPitch, FullVehicleTestInertiaRoll)
        solver = ODEAlg.NativeAlgorithm(Rodas5P(autodiff=AutoFiniteDiff()), @__MODULE__)
    end
    invoke(__dyad_run_test_case!, Tuple{Any,AbstractString}, rig, name; component_stem, solver, kwargs...)
end

if selected_test in (nothing, "damper_maps.jl")
    include("damper_maps.jl")
end

if selected_test in (nothing, "hub_motor.jl")
    include("hub_motor.jl")
end

if !(selected_test in ("damper_maps.jl", "hub_motor.jl"))
    include("../generated/tests.jl")
end
