using VehicleComponents
using ModelingToolkit, MultibodyComponents, OrdinaryDiffEqRosenbrock
using ADTypes: AutoFiniteDiff

# Let the suspension settle, then measure the 75 m time during a full-throttle run.
t_drive = 1.5
t_end = t_drive + 4.0
distance_m = 75.0

@named model = FullVehicleTestStraightLine()
ssys = multibody(model)
D = Differential(ModelingToolkit.get_iv(ssys))
# TODO: Remove guesses once MTK avoids symbolic guess expansion for linear initialization:
# https://github.com/SciML/ModelingToolkit.jl/issues/5272
prob = ODEProblem(ssys, [
    ssys.drive_start_time => t_drive,
    ssys.drive_duration => t_end - t_drive,
    ssys.brake_level => 0.0,
], (0.0, t_end); guesses=[D(D(ssys.vehicle.body.r_0[i])) => 0.0 for i in 1:3])

sol = solve(prob, Rodas5P(autodiff=AutoFiniteDiff()); tstops=[t_drive], saveat=0.01)

# Interpolate between the saved samples bracketing 75 m from throttle application.
x = ssys.vehicle.outputs.body.position[1]
travel = sol[x] .- sol(t_drive; idxs=x)
i = findfirst(>=(distance_m), travel)
isnothing(i) && error("75 m finish not reached.")
fraction = (distance_m - travel[i - 1]) / (travel[i] - travel[i - 1])
time_75m = sol.t[i - 1] + fraction * (sol.t[i] - sol.t[i - 1]) - t_drive
println("75 m acceleration time: $(round(time_75m; digits=3)) s")

# Export the full run to MoTeC.
VehicleComponents.Telemetry.write_telemetry(
    model, sol;
    run="acceleration_75m",
    venue="Straight Line",
    event="FSAE Acceleration (75 m)",
    comment="75 m in $(round(time_75m; digits=3)) s. LMUX=$(prob.ps[ssys.vehicle.corner_fl.wheel_assembly.tire.LMUX])."
)
