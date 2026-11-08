using ModelingToolkit.Symbolics: @register_symbolic

struct MotorLossCurve
    torque::Vector{Float64}
    watts::Vector{Float64}
end

struct MotorMap
    rpm::Vector{Float64}
    curves::Vector{MotorLossCurve}
end

"""Read a motor CSV from an assets-relative path or an absolute file path."""
function read_motor_map(file::AbstractString)
    path = joinpath(@__DIR__, "..", "assets", file)
    isfile(path) || throw(ArgumentError("Motor map file not found: $path"))
    Base.include_dependency(path)
    rpm = Float64[]
    curves = MotorLossCurve[]
    for line in Iterators.drop(eachline(path), 1)
        speed, torque, watts = parse.(Float64, split(line, ','))
        if isempty(rpm) || speed != last(rpm)
            push!(rpm, speed)
            push!(curves, MotorLossCurve(Float64[], Float64[]))
        end
        push!(last(curves).torque, torque)
        push!(last(curves).watts, watts)
    end
    first(rpm) == 0 && all(>(0), diff(rpm)) || error("Invalid speed axis in $path")
    for curve in curves
        first(curve.torque) == 0 && last(curve.torque) > 0 &&
            all(>(0), diff(curve.torque)) && all(x -> isfinite(x) && x >= 0, curve.watts) ||
            error("Invalid torque or loss data in $path")
    end
    return MotorMap(rpm, curves)
end

const MOTOR_MAPS = Dict{String,MotorMap}()

function motor_map(file::AbstractString)
    # Read only the selected CSV and reuse it during solver evaluations.
    return get!(MOTOR_MAPS, file) do
        read_motor_map(file)
    end
end

# Each speed has its own measured torque knots, including its full-load point.
# Interpolate at equal fractions of the two adjacent envelopes.
function motor_operating_speed(map, omega)
    rpm = abs(omega) * (30 / π)
    i = clamp(searchsortedlast(map.rpm, rpm), 1, length(map.rpm) - 1)
    mix = clamp((rpm - map.rpm[i]) / (map.rpm[i + 1] - map.rpm[i]), 0, 1)
    left, right = map.curves[i], map.curves[i + 1]
    # mix clamps the lookup to the last row above the mapped speed. Keep
    # actual shaft speed for torque × speed in the electrical power balance.
    limit = (1 - mix) * last(left.torque) + mix * last(right.torque)
    return (; omega=abs(omega), left, right, mix, limit)
end

motor_torque_limit(file, omega) = motor_operating_speed(motor_map(file), omega).limit
@register_symbolic motor_torque_limit(file::String, omega)

function curve_loss(curve, fraction)
    torque = fraction * last(curve.torque)
    i = clamp(searchsortedlast(curve.torque, torque), 1, length(curve.torque) - 1)
    mix = (torque - curve.torque[i]) / (curve.torque[i + 1] - curve.torque[i])
    return (1 - mix) * curve.watts[i] + mix * curve.watts[i + 1]
end

function motor_input_power(point, torque)
    torque = clamp(torque, 0, point.limit)
    fraction = iszero(point.limit) ? zero(torque) : torque / point.limit
    loss = (1 - point.mix) * curve_loss(point.left, fraction) + point.mix * curve_loss(point.right, fraction)
    return point.omega * torque + loss
end

motor_input_power(file, omega, torque) = motor_input_power(motor_operating_speed(motor_map(file), omega), torque)
@register_symbolic motor_input_power(file::String, omega, torque)

"""Common torque multiplier under a shared electrical input limit, in watts."""
function motor_power_scale(file, limit, speeds, requests)
    map = motor_map(file)
    points = ntuple(i -> motor_operating_speed(map, speeds[i]), 4)
    draw(k) = sum(i -> motor_input_power(points[i], k * requests[i]), 1:4)
    draw(1) <= limit && return 1.0
    draw(0) <= limit || throw(DomainError(limit, "Power limit is below the motors' no-load losses"))
    low, high = 0.0, 1.0
    for _ in 1:40
        mid = (low + high) / 2
        if draw(mid) <= limit
            low = mid
        else
            high = mid
        end
    end
    return low
end
@register_symbolic motor_power_scale(file::String, limit, speeds::AbstractArray, requests::AbstractArray)
