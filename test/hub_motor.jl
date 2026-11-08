using Test, VehicleComponents, ModelingToolkit, MultibodyComponents, OrdinaryDiffEqRosenbrock
using ADTypes: AutoFiniteDiff

@testset "Hub motor dynamometer" begin
    # Source anchors at full and partial load. Each loss is winding plus iron.
    samples = (
        ("AMK", 5000.0, 20.9916181227, 2676.44195112 + 364.48709873,
            10.70320395, 429.49230156 + 210.29459434),
        ("F-MOT", 5000.0, 31.5929981472, 101 + 1597*(66/65)^2, 10.64196, 101 + 168),
    )
    for (name, rpm, peak, peak_loss, partial, partial_loss) in samples
        @testset "$name" begin
            @named model = HubMotorTestRig(motor_map_file="public/motors/$name.csv")
            sys = multibody(model)
            motor = sys.motor.outputs
            ω, τ, wheel, mechanical, electrical = (
                motor.motor_speed, motor.tau_motor, motor.tau_wheel,
                motor.shaft_power, motor.electrical_power,
            )
            for (demand, expected_torque, loss) in ((1., peak, peak_loss), (partial/peak, partial, partial_loss))
                prob = ODEProblem(sys, [sys.demand => demand], (0., 1.))
                sol = solve(prob, Rodas5P(autodiff=AutoFiniteDiff()); abstol=1e-8, reltol=1e-8)
                @test ModelingToolkit.SciMLBase.successful_retcode(sol)
                sweep_rate_rpm = prob.ps[sys.final_motor_speed] * (30/π) / prob.ps[sys.sweep_duration]
                state = sol(rpm/sweep_rate_rpm, idxs=[ω, τ, wheel, mechanical, electrical])
                speed = rpm * π/30
                @test state ≈ [speed, expected_torque, 14 * expected_torque,
                    speed*expected_torque, speed*expected_torque + loss] atol=0.002
            end
        end
    end
end

@testset "Shared electrical limit" begin
    # Front motors above the map endpoint, rears inside it.
    speeds = [21000., 21000., 17500., 17500.] .* (π/30)
    for name in ("AMK", "F-MOT")
        file = "public/motors/$name.csv"
        available = VehicleComponents.motor_torque_limit.(file, speeds)
        requests = [0.05, 0.05, 1., 1.] .* available
        k = VehicleComponents.motor_power_scale(file, 80000., speeds, requests)
        powers = VehicleComponents.motor_input_power.(file, speeds, k .* requests)
        @test 0 < k < 1
        @test 79999.99 <= sum(powers) <= 80000.000001
        @test powers[3] > 20000
        @test VehicleComponents.motor_power_scale(file, 80000., speeds, requests/10) == 1
        @test VehicleComponents.motor_input_power(file, 0., 0.) == 0
        @test VehicleComponents.motor_input_power(file, speeds[1], 0.) > 0
    end
    @test_throws ArgumentError VehicleComponents.motor_map("public/motors/missing.csv")
end

@testset "Above the final mapped speed" begin
    for name in ("AMK", "F-MOT")
        file = "public/motors/$name.csv"
        map = VehicleComponents.motor_map(file)
        endpoint = last(map.rpm) * (π / 30)
        speed = 1.05 * endpoint
        curve = last(map.curves)
        @test VehicleComponents.motor_torque_limit(file, speed) ≈ last(curve.torque)
        for (torque, loss) in zip(curve.torque, curve.watts)
            @test VehicleComponents.motor_input_power(file, speed, torque) ≈ speed * torque + loss
        end
    end
end

@testset "Native torque and loss map knots" begin
    for name in ("AMK", "F-MOT")
        file = "public/motors/$name.csv"
        map = VehicleComponents.motor_map(file)
        @test all(VehicleComponents.motor_torque_limit(file, rpm*π/30) ≈ last(curve.torque)
                  for (rpm, curve) in zip(map.rpm, map.curves))
        error = maximum(
            abs(VehicleComponents.motor_input_power(file, rpm*π/30, t) - t*rpm*π/30 - loss)
            for (rpm, curve) in zip(map.rpm, map.curves)
            for (t, loss) in zip(curve.torque, curve.watts)
        )
        @test error < 0.001  # watts
    end
    # F-MOT field weakening, 56 A and -20 degrees at 14,910 rpm.
    rpm, torque = 14910., 26.869530831809485
    iron = 376 + (377 - 376) * (rpm - 14500) / 500
    copper = 1143 + (1360 - 1143) / 5
    @test VehicleComponents.motor_input_power("public/motors/F-MOT.csv", rpm*π/30, torque) ≈
        torque*rpm*π/30 + iron + copper atol=0.01
end
