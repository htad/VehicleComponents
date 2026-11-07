using Revise
using VehicleComponents
using GLMakie, ModelingToolkit, MultibodyComponents, OrdinaryDiffEqRosenbrock, Plots
using ADTypes: AutoFiniteDiff

t_drive = 1.5
t_steer = 1.5
steer_angle = 0.2
t_end = 24.0

@named model = FullVehicleTestStep()
ssys = multibody(model)
D = Differential(ModelingToolkit.get_iv(ssys))
# TODO: Remove guesses once MTK avoids symbolic guess expansion for linear initialization:
# https://github.com/SciML/ModelingToolkit.jl/issues/5272
prob = ODEProblem(ssys, [
    ssys.steer_angle => steer_angle,
    ssys.throttle.height => 0.1,
    ssys.throttle.duration => 22.5,
], (0.0, t_end); guesses=[D(D(ssys.vehicle.body.r_0[i])) => 0.0 for i in 1:3])
sol = solve(prob, Rodas5P(autodiff=AutoFiniteDiff()); tstops=[t_drive, t_steer])

# ===========================================================================
# MoTeC export
# ===========================================================================
VehicleComponents.Telemetry.write_telemetry(
    model, sol;
    run="step_steer",
    venue="Step steer",
    event="FullVehicleTestStep",
    comment="drive at $(t_drive) s, $(steer_angle) rad step at $(t_steer) s"
)

# ===========================================================================
# Animation
# ===========================================================================
cam_offset = GLMakie.Vec3f(-2.5, -0.35, 0.4)
framerate = 25
timevec = range(sol.t[1], sol.t[end], step=1 / framerate)

fig, tobs, scene = render(model, sol, sol.t[1];
    x=cam_offset[1], y=cam_offset[2], z=cam_offset[3],
    up=[0, 0, 1], slider=false, size=(800, 600))

body_position = collect(ssys.vehicle.outputs.body.position)
body_R = vec(ssys.vehicle.body.frame_a.R)

GLMakie.record(fig, "output/full_vehicle_step.gif", timevec; framerate) do time
    tobs[] = time

    R = reshape(sol(time, idxs=body_R), 3, 3)

    offset_world = MultibodyComponents.resolve1(R, collect(cam_offset))

    target = GLMakie.Vec3f(sol(time, idxs=body_position)...)
    camera = target + GLMakie.Vec3f(offset_world...)

    GLMakie.update_cam!(
        scene.scene,
        GLMakie.cameracontrols(scene.scene),
        camera,
        target,
        GLMakie.Vec3f(0, 0, 1),
    )
end
