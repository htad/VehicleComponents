using VehicleComponents, ModelingToolkit, MultibodyComponents, OrdinaryDiffEqRosenbrock, Plots
using ADTypes: AutoFiniteDiff

motor_type = "AMK"
@named model = HubMotorTestRig(; motor_type)
ssys = multibody(model)
prob = ODEProblem(ssys, [], (0.0, 1.0))
sol = solve(prob, Rodas5P(autodiff=AutoFiniteDiff()))

motor = ssys.motor.outputs;
t = range(0, 1, length=401);
rpm = sol(t, idxs=motor.motor_speed).u .* (30/π);
plotly();
torque = plot(rpm, sol(t, idxs=motor.tau_motor).u; xlabel="Motor speed [rpm]", ylabel="Shaft torque [N·m]", label="Rig", title=motor_type);
map_data = VehicleComponents.motor_map(motor_type);
scatter!(torque, map_data.rpm, [last(c.torque) for c in map_data.curves]; label="Source envelope", markersize=2);
power = plot(rpm, sol(t, idxs=motor.electrical_power).u ./ 1000; xlabel="Motor speed [rpm]", ylabel="Power [kW]", label="Electrical input");
plot!(power, rpm, sol(t, idxs=motor.shaft_power).u ./ 1000; label="Shaft output");
plot(torque, power; layout=(2, 1), size=(800, 600))
