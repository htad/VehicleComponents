using Revise
using VehicleComponents
using ADTypes: AutoFiniteDiff
using GLMakie, ModelingToolkit, MultibodyComponents, OrdinaryDiffEqRosenbrock, Plots

const DEG = 180 / π
const MM = 1000

@named model = VehicleComponents.TireTestRig()
ssys = multibody(model)

# Slip sweep. The carriage rolls the wheel forward at `speed` while the spin drive
# ramps wheel speed either side of free rolling, at a held vertical load.
speed = 10.0
static_deflection = 0.005
kappa_start = -0.2
kappa_end = 0.2
stop = 2.0

parameter_map = Dict([
    ssys.speed => speed,
    ssys.static_deflection => static_deflection,
    ssys.stroke => 0.0,
    ssys.kappa_start => kappa_start,
    ssys.kappa_end => kappa_end,
    ssys.slip_ramp_time => stop,
])

prob = ODEProblem(ssys, parameter_map, (0.0, stop))
vertical_stiffness = prob.ps[ssys.tire.VERTICAL_STIFFNESS]
sol = solve(prob, Rodas5P(autodiff=AutoFiniteDiff()))

plot_slip_curve = Plots.plot(
    sol,
    idxs=(model.tire.kappa, model.tire.Fx),
    ylabel="Fx [N]",
    xlabel="slip ratio [-]",
    title="Slip curve"
);
plot_kappa = Plots.plot(
    sol,
    idxs=model.tire.kappa,
    ylabel="kappa [-]",
    xlabel="t [s]",
    title="Slip ratio"
);
plot_fx = Plots.plot(
    sol,
    idxs=model.tire.Fx,
    ylabel="Fx [N]",
    xlabel="t [s]",
    title="Longitudinal force"
);
plot_fz = Plots.plot(
    sol,
    idxs=model.tire.Fz,
    ylabel="Fz [N]",
    xlabel="t [s]",
    title="Vertical load"
);
Plots.plot(
    plot_slip_curve, plot_kappa, plot_fx, plot_fz;
    layout=(2, 2),
    legend=false,
    lw=2,
    size=(1200, 800),
    left_margin=5Plots.PlotMeasures.mm,
    bottom_margin=5Plots.PlotMeasures.mm
)

# Slip angle sweep, at a family of vertical loads. Every load term in the fit is
# linear in dfz, so one sweep says nothing about whether load sensitivity holds.
# The mount preload sets the load: Fz = vertical_stiffness * static_deflection.
alpha_start = -0.3
alpha_end = 0.3
sweep_loads = [400.0, 700.0, 1000.0, 4850.0]

lateral_map(deflection) = Dict([
    ssys.speed => speed,
    ssys.static_deflection => deflection,
    ssys.stroke => 0.0,
    ssys.alpha_start => alpha_start,
    ssys.alpha_end => alpha_end,
    ssys.steer_ramp_time => stop,
])

lateral_sols = map(sweep_loads) do fz
    solve(ODEProblem(ssys, lateral_map(fz / vertical_stiffness), (0.0, stop)),
        Rodas5P(autodiff=AutoFiniteDiff()))
end

# Fy needs a few relaxation lengths to catch up with the commanded slip angle, so
# everything before 50 ms is startup rather than tyre behaviour.
settled(sol) = findfirst(>(0.05), sol.t):length(sol.t)

plot_cornering_curve = Plots.plot(
    ylabel="Fy [N]",
    xlabel="slip angle [deg]",
    title="Cornering curve",
    legend=:topright
);
plot_mu_y = Plots.plot(
    ylabel="Fy / Fz [-]",
    xlabel="slip angle [deg]",
    title="Lateral friction used"
);
plot_alpha = Plots.plot(
    ylabel="alpha [deg]",
    xlabel="t [s]",
    title="Slip angle"
);
plot_fz_lat = Plots.plot(
    ylabel="Fz [N]",
    xlabel="t [s]",
    title="Vertical load"
);
for (fz, lateral_sol) in zip(sweep_loads, lateral_sols)
    k = settled(lateral_sol)
    alpha_deg = DEG * lateral_sol[model.tire.alpha]
    fy = lateral_sol[model.tire.Fy]
    fz_model = lateral_sol[model.tire.Fz]

    Plots.plot!(plot_cornering_curve, alpha_deg[k], fy[k]; label="Fz = $(round(Int, fz)) N")
    Plots.plot!(plot_mu_y, alpha_deg[k], fy[k] ./ fz_model[k])
    Plots.plot!(plot_alpha, lateral_sol.t, alpha_deg)
    Plots.plot!(plot_fz_lat, lateral_sol.t, fz_model)
end
Plots.plot(
    plot_cornering_curve, plot_mu_y, plot_alpha, plot_fz_lat;
    layout=(2, 2),
    lw=2,
    size=(1200, 800),
    left_margin=5Plots.PlotMeasures.mm,
    bottom_margin=5Plots.PlotMeasures.mm
)

# Vertical sweep. Same rig with the carriage and spin drive parked, which is what
# the `sweep` test exercises.
prob_vertical = ODEProblem(ssys, [], (0.0, 2.0))
sol_vertical = solve(prob_vertical, Rodas5P(autodiff=AutoFiniteDiff()))

plot_height = Plots.plot(
    sol_vertical,
    idxs=MM * model.tire.z_w,
    ylabel="wheel centre [mm]"
);
plot_deflection = Plots.plot(
    sol_vertical,
    idxs=MM * model.tire.rho,
    ylabel="deflection [mm]"
);
plot_force = Plots.plot(
    sol_vertical,
    idxs=model.tire.Fz,
    ylabel="contact force [N]",
    xlabel="time [s]"
);
Plots.plot(
    plot_height, plot_deflection, plot_force;
    layout=(3, 1),
    link=:x,
    legend=false,
    lw=2,
    size=(800, 900)
)

render(model, sol; filename="output/tire.gif", up=[0, 0, 1], x=1, y=0.1, z=1, lookat=[0, 0, 0.3])
