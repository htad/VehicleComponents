"""
    VehicleComponents.Telemetry

Maps a `FullVehicle` solution onto MoTeC i2 channels.

Naming follows a MoTeC export of Bosch channels: lower case, a quantity
prefix, then a position suffix, `_fl _fr _rl _rr` for a corner and `_f _r` for an
axle. `acc_` acceleration, `a_` angle, `f_` force, `h_` height, `n_` rotational
speed, `p_` pressure, `r_` ratio, `s_` displacement, `tq_` torque, `v_` velocity.
"""
module Telemetry

using ..Motec

const G = 9.80665
const DEG = 180 / pi
const KMH = 3.6
const MM = 1000
const METERS_TO_DEGREES = 9e-6
const GPS_LATITUDE_ORIGIN = 1.0
const GPS_LONGITUDE_ORIGIN = 1.0
const GPS_RATE = 20

function vehicle_channels(model, sol; rate::Int=100)
    car = model.vehicle
    outputs = car.outputs
    times = range(0.0, sol.t[end], step=1 / rate)
    channels = Motec.Channel[]
    skipped = String[]

    sample(signal) = sol(times, idxs=signal).u

    specs = Tuple{String,Any,Float64,String}[]
    add(name, signal, scale, unit) = push!(specs, (name, signal, scale, unit))

    # Body
    add("acc_x", () -> outputs.body.a_cg[1], 1 / G, "G")
    add("acc_y", () -> outputs.body.a_cg[2], 1 / G, "G")
    add("acc_z", () -> outputs.body.a_cg[3], 1 / G, "G")
    add("a_yaw", () -> model.yaw_joint.phi, DEG, "deg")
    add("a_pitch", () -> model.pitch_joint.phi, DEG, "deg")
    add("a_roll", () -> model.roll_joint.phi, DEG, "deg")
    add("a_yaw_rate", () -> model.yaw_joint.w, DEG, "deg/s")
    add("a_pitch_rate", () -> model.pitch_joint.w, DEG, "deg/s")
    add("a_roll_rate", () -> model.roll_joint.w, DEG, "deg/s")
    add("v_vx", () -> outputs.body.v_cg[1], KMH, "km/h")
    add("v_vy", () -> outputs.body.v_cg[2], KMH, "km/h")
    add("a_beta", () -> atan.(sample(outputs.body.v_cg[2]), sample(outputs.body.v_cg[1])), DEG, "deg")

    # Driver inputs
    add("r_throttle", () -> car.inputs.throttle, 100, "%")
    add("r_brake", () -> car.inputs.brake, 100, "%")
    add("a_steer", () -> car.inputs.steer, DEG, "deg")

    corners = (outputs.corner_fl, outputs.corner_fr, outputs.corner_rl, outputs.corner_rr)
    add("pwr_motor_elec_total", () -> sum(corner.electrical_power for corner in corners), 1 / 1000, "kW")
    add("pwr_motor_shaft_total", () -> sum(corner.shaft_power for corner in corners), 1 / 1000, "kW")

    # Brake control
    add("r_brake_abs", () -> outputs.control.brake_demand, 100, "%")
    add("r_abs_intervention", () -> outputs.control.abs_intervention, 100, "%")

    # Aero
    add("f_aero_drag", () -> outputs.aero.drag, 1, "N")
    add("f_aero_downforce", () -> outputs.aero.downforce, 1, "N")
    add("f_aero_downforce_f", () -> outputs.aero.downforce_front, 1, "N")
    add("f_aero_downforce_r", () -> outputs.aero.downforce_rear, 1, "N")
    add("p_dynamic", () -> outputs.aero.q, 1 / 100, "mbar")

    # Corner and suspension measurements use the grouped vehicle outputs.
    for (tag, corner, suspension, side) in (
        ("fl", outputs.corner_fl, outputs.front_suspension, "left"),
        ("fr", outputs.corner_fr, outputs.front_suspension, "right"),
        ("rl", outputs.corner_rl, outputs.rear_suspension, "left"),
        ("rr", outputs.corner_rr, outputs.rear_suspension, "right"),
    )
        linkage = name -> getproperty(suspension, Symbol(name, "_", side))
        add("v_wheel_$tag", () -> sample(corner.omega) .* sample(corner.Re), KMH, "km/h")
        add("n_wheel_$tag", () -> corner.omega, 60 / 2pi, "rpm")
        add("n_motor_$tag", () -> corner.motor_speed, 60 / 2pi, "rpm")
        add("f_tire_x_$tag", () -> corner.Fx, 1, "N")
        add("f_tire_y_$tag", () -> corner.Fy, 1, "N")
        add("f_tire_z_$tag", () -> corner.Fz, 1, "N")
        add("r_tire_long_slip_$tag", () -> corner.slip, 1, "ratio")
        add("a_tire_lateral_slip_$tag", () -> corner.alpha, DEG, "deg")
        add("a_tire_inclination_$tag", () -> linkage("camber"), DEG, "deg")
        add("a_tire_toe_$tag", () -> linkage("toe"), DEG, "deg")
        add("s_wheel_$tag", () -> linkage("wc_height"), MM, "mm")
        add("tq_motor_$tag", () -> corner.tau_motor, 1, "N.m")
        add("pwr_motor_elec_$tag", () -> corner.electrical_power, 1 / 1000, "kW")
        add("pwr_motor_$tag", () -> corner.shaft_power, 1 / 1000, "kW")
        add("tq_drive_wheel_$tag", () -> corner.tau_wheel, 1, "N.m")
        add("tq_brake_wheel_$tag", () -> corner.tau_brake, 1, "N.m")
        for (axis, i) in (("x", 1), ("y", 2), ("z", 3))
            add("f_lca_$(axis)_$tag", () -> linkage("f_lca")[i], 1, "N")
            add("f_uca_ball_$(axis)_$tag", () -> linkage("f_uca_ball")[i], 1, "N")
            add("f_uca_mount_$(axis)_$tag", () -> linkage("f_uca")[i], 1, "N")
            add("f_pushrod_$(axis)_$tag", () -> linkage("f_push")[i], 1, "N")
        end
    end

    for (tag, suspension) in (("f", outputs.front_suspension), ("r", outputs.rear_suspension))
        add("s_heave_$tag", () -> suspension.wc_heave, MM, "mm")
        add("s_roll_$tag", () -> suspension.wc_roll, MM, "mm")
        add("s_damper_heave_$tag", () -> suspension.s_heave, MM, "mm")
        add("s_damper_roll_$tag", () -> suspension.s_roll, MM, "mm")
        add("f_damper_heave_$tag", () -> suspension.f_heave, 1, "N")
        add("f_damper_roll_$tag", () -> suspension.f_roll, 1, "N")
    end

    signals = Any[(try spec[2]() catch; nothing end) for spec in specs]
    direct = [i for (i, s) in enumerate(signals) if !(s === nothing || s isa AbstractVector{<:Real})]
    series = Dict{Int,Vector{Float64}}()

    if !isempty(direct)
        try
            batch = sol(times, idxs=[signals[i] for i in direct])
            for (k, i) in enumerate(direct)
                series[i] = Float64[u[k] for u in batch.u]
            end
        catch
            for i in direct
                try
                    series[i] = sample(signals[i])
                catch
                end
            end
        end
    end

    for (i, (name, _, scale, unit)) in enumerate(specs)
        values = signals[i] isa AbstractVector{<:Real} ? signals[i] : get(series, i, nothing)
        if values === nothing
            push!(skipped, name)
        else
            push!(channels, Motec.Channel(name, values .* scale; freq=rate, unit))
        end
    end

    isempty(skipped) ||
        @warn "telemetry: $(length(skipped)) channels are not in the solution" skipped

    # GPS coordinates in degrees, with a nonzero "pretend" origin.
    gps_times = range(0.0, sol.t[end], step=1 / GPS_RATE)
    gps_x = Float64[sol(time, idxs=outputs.body.position[1]) for time in gps_times]
    gps_y = Float64[sol(time, idxs=outputs.body.position[2]) for time in gps_times]
    gps_step = hypot.(diff(gps_x), diff(gps_y))
    gps_speed = vcat(0.0, gps_step .* GPS_RATE) .* KMH
    lap_distance = vcat(0.0, cumsum(gps_step))
    gps_latitude = GPS_LATITUDE_ORIGIN .+ gps_x .* METERS_TO_DEGREES
    gps_longitude = GPS_LONGITUDE_ORIGIN .+ gps_y .* (METERS_TO_DEGREES / cosd(GPS_LATITUDE_ORIGIN))
    for (name, values, unit) in (
        ("GPS Latitude", gps_latitude, "deg"),
        ("GPS Longitude", gps_longitude, "deg"),
        ("GPS Speed", gps_speed, "km/h"),
        ("Lap Distance", lap_distance, "m"),
    )
        push!(channels, Motec.Channel(name, values; freq=GPS_RATE, unit=unit))
    end

    return channels
end

function write_telemetry(model, sol; run::AbstractString, dir="output/telemetry",
    rate::Int=100, driver="Sim", vehicle_id=sol.prob.ps[model.vehicle.vehicle_name],
    vehicle_desc="Full vehicle", venue="", event="", session="S1",
    comment="")
    stamp = Base.time()
    parts = filter(!isempty, [run, String(vehicle_id), Base.Libc.strftime("%Y%m%d_%H%M%S", stamp)])
    meta = Motec.LogMeta(; driver, vehicle_id, vehicle_desc, venue, event, session, comment,
        date=Base.Libc.strftime("%d/%m/%Y", stamp),
        time=Base.Libc.strftime("%H:%M:%S", stamp))
    return Motec.write_ld(joinpath(dir, join(parts, "_") * ".ld"),
        vehicle_channels(model, sol; rate), meta)
end

end
