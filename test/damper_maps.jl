using Test
using DyadData: DyadTimeseries, build_table
using VehicleComponents
using ModelingToolkit

@testset "Multimatic damper catalog" begin
    asset_dir = joinpath(@__DIR__, "..", "assets", "public", "dampers")
    names = [
        "$(architecture)_$(code)_$(direction)_$(lpad(position, 2, '0')).csv"
        for architecture in ("heave", "roll")
        for code in ("VC01", "VC02", "VC03")
        for direction in ("compression", "extension")
        for position in 0:11
    ]
    @test Set(filter(name -> endswith(name, ".csv"), readdir(asset_dir))) == Set(names)
    speeds = [0, 10, 25, 50, 100, 150, 200, 250, 300, 350, 400, 450, 500] ./ 1000
    for name in names
        curve = build_table(DyadTimeseries("dyad://VehicleComponents/public/dampers/$name";
            independent_var="velocity", dependent_vars=["force"]))
        @test curve.velocity == speeds
        @test all(isfinite, curve.force)
        @test first(curve.force) == 0
        @test all(occursin("compression", name) ? >=(0) : <=(0), curve.force)
    end
end

@testset "Independent damper directions" begin
    compression = "public/dampers/heave_VC03_compression_02.csv"
    extension = "public/dampers/heave_VC03_extension_02.csv"
    baseline = build_table(VehicleComponents.damper_dataset(compression, extension))
    rebound = build_table(VehicleComponents.damper_dataset(
        compression, "public/dampers/heave_VC03_extension_07.csv"))
    bump = build_table(VehicleComponents.damper_dataset(
        "public/dampers/heave_VC03_compression_07.csv", extension))

    @test baseline.velocity == rebound.velocity == bump.velocity
    @test all(>(0), diff(baseline.velocity))
    @test count(iszero, baseline.velocity) == 1
    negative = baseline.velocity .< 0
    positive = baseline.velocity .> 0
    @test rebound.force[negative] == baseline.force[negative]
    @test bump.force[positive] == baseline.force[positive]
    @test rebound.force[positive] != baseline.force[positive]
    @test bump.force[negative] != baseline.force[negative]

    # Published heave VC03 samples at 100 mm/s, converted to model signs.
    @test baseline.force[findfirst(==(-0.1), baseline.velocity)] == -104
    @test baseline.force[findfirst(==(0.1), baseline.velocity)] == 90
    @test rebound.force[findfirst(==(0.1), rebound.velocity)] == 299
    @test bump.force[findfirst(==(-0.1), bump.velocity)] == -279
    @test_throws ArgumentError VehicleComponents.damper_dataset(extension, compression)
end

@testset "Damper fixtures and vehicle selection" begin
    for constructor in (DamperTestRig, InboardTestRig, SuspensionTestRig)
        @test constructor(; name=:fixture) isa ModelingToolkit.System
    end
    model = FullVehicleTestStraightLine(; name=:model, __overrides=Dict(
        "vehicle.heave_damper_extension_front" => "public/dampers/heave_VC03_extension_07.csv",
    ))
    force_map = model.vehicle.front_suspension.inboard.heave_strut.damper.force_map
    velocity = ModelingToolkit.getdefault(force_map.independent_var)
    force = ModelingToolkit.getdefault(force_map.data)
    @test force[findfirst(==(-0.1), velocity)] == -104
    @test force[findfirst(==(0.1), velocity)] == 299
end
