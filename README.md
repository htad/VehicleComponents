# VehicleComponents

VehicleComponents is a Julia/Dyad multibody vehicle-dynamics modeling library for building and simulating modular full-vehicle assemblies. It integrates suspension linkages and inboard heave/roll systems, hub motors with torque control, aerodynamic loads, and a Magic Formula 6.1 tire model to support parameterized performance studies.

<p align="center">
  <img src="assets/full_vehicle_straight_line.gif" alt="Straight-line acceleration simulation" width="44%" align="middle">
  <img src="assets/motec_data_view.png" alt="MoTeC i2 Data View of the simulation log" width="55%" align="middle">
</p>

<p align="center">
  <em>Each simulation run is exported as a MoTeC log, so its inputs, vehicle states, suspension and tire loads, and GPS track data can be reviewed in MoTeC.</em>
</p>

## Getting Started

This library was created with the Dyad Studio VS Code extension. Your Dyad models should be placed in the `dyad` directory and the files should be given the `.dyad` extension. The Dyad compiler will compile the Dyad models into Julia code and place it in the `generated` folder. Do not edit the files in that directory or remove/rename that directory.

1. Run `Julia: Start REPL` from the command palette.

1. Type `]`.  This will take you to the package manager prompt.

1. At the `pkg>` prompt, type `instantiate` (this downloads all the Julia libraries you will need, and the very first time you do it it might take a while).

1. From the same `pkg>` prompt, type `test`.  This will test to make sure the models are working as expected.  It may also take some time but you should eventually see a result that indicates 2 of 2 tests passed.

1. Use the `Backspace`/`Delete` key to return to the normal Julia REPL, it should look like this: `julia>`.

1. Run any of the julia files in `scripts` directory.  This will load the model library and run an analysis.


## FullVehicle configuration

Select the profile in each Dyad test rig:

```dyad
vehicle = VehicleComponents.FullVehicle(
  apply "dyad://VehicleComponents/TestVehicle.toml"
)
```

Switch to `MR26.toml` or `MR27.toml` and regenerate. MR profiles are ignored by Git. Editing TOML values only requires a new model instance.

All four wheels load `assets/TestTire.toml` by default. To use the Hoosier R20, change the tire’s `apply` line in `dyad/WheelAssembly.dyad` to `apply "dyad://VehicleComponents/R20.toml"`, regenerate, and construct a new model. This selects the tire for TestVehicle, MR26, and MR27. Wheel masses and inertias also belong to the vehicle.

Select damper compression/extension CSVs independently, e.g. `heave_VC03_extension_07.csv`. Files use m/s and N; the loader converts published force signs. Source: Multimatic DSSV Valve Code Offerings, 2022-04-25.

## Dyad Conventions

### Variable Type

| Declaration | Purpose |
| --- | --- |
| Input | External signal, possibly time varying: throttle, brake, steering. |
| Parameter | Fixed per run, tunable between runs: mass, tire lambdas. |
| Final parameter | Derived configuration or fixed constant; cannot be overridden. |
| Structural parameter | Construction setting; changes require rebuilding. |
| Output | Measurement needed by components, telemetry, tests, or an FMU. |
| Variable | Internal state or calculation. |

### Order of Relations Section

This should be the order relation sections in a Dyad component.

1. Inputs
1. Outputs
1. Initialization
1. Equations
1. Connections

### Formatting and style

Use default Dyad icons/colors and two-space indentation for metadata and nested modifications.

## Modeling Conventions

Everything in this library, such as hardpoints, frames, outputs, and the JSON vehicle data in `assets/`, follows the conventions below.

### Coordinate system: ISO 8855

All geometry uses the **ISO 8855:2011** vehicle axis system:

| Axis | Index | Quantity | Positive direction |
| ----- | ----- | ----- | ----- |
| X | `[1]` | Longitudinal | **Forward** |
| Y | `[2]` | Lateral | **Left** |
| Z | `[3]` | Vertical | **Up** |

"Left" and "right" are always as seen by a driver seated in the vehicle facing forward.

#### NOTE: MultibodyComponents defaults to Y-up

`MultibodyComponents` inherits Modelica's convention where **Y** points up: `World` defaults to a gravity direction of `n = [0, -1, 0]`. Every `World` in this library must therefore be instantiated explicitly for ISO:

```
world = MultibodyComponents.World(n = [0, 0, -1])
```

and renders must pass `up = [0, 0, 1]`.

### Vehicle Origin

- `x = 0` at the front axle centreline, so rear-axle hardpoints are therefore **negative** x.
- `y = 0` on the longitudinal plane of symmetry.
- `z = 0` at the ground plane, so wheel-centre z is approximately the tyre's unloaded radius.

### Sign conventions for wheel outputs

ISO distinguishes the **camber angle** (wheel lean measured relative to the vehicle body) from the **inclination angle** (the same lean measured relative to the road).

Camber and toe are reported **side-symmetrically**, so a left and a right corner can be plotted on the same axes and compared directly:

| Output | Positive means |
| ----- | ----- |
| `camber` | Top of the wheel leans **outboard** |
| `toe` | Leading edge of the wheel points **inboard** (toe-in) |

### Naming conventions

#### Corner identifiers

Corners are named **axle first, then side**, lowercase, as a trailing suffix:

| Suffix | Corner      |
| ------ | ----------- |
| `_fl`  | Front left  |
| `_fr`  | Front right |
| `_rl`  | Rear left   |
| `_rr`  | Rear right  |

Rules:

1. **Axle-then-side, always.** Write `_fl`, never `_lf`.
1. **The suffix goes last**, after every other qualifier: `camber_fl`, `wheel_center_rr`, `spring_stiffness_fl`.
1. **Only use it where corners coexist in one scope** such as a full-vehicle or axle-level assembly. Inside a single-corner component, do not bake the corner into names.
1. **Abbreviate only as a fixed-width code, never as a standalone qualifier.** In `_fl` the first character is always the axle and the second always the side, so position tells the reader what each one means. A lone qualifier has no such anchor: `lca_r` gives no way to tell "rear" from "right". Spell those out — `_front` / `_rear` for the axle, `_left` / `_right` for the side.

#### Hardpoints

Hardpoints are named `<link>_<location>`, matching Adams/Car practice so geometry stays portable in both directions:

| Name | Meaning |
| ----- | ----- |
| `lca_front`, `lca_rear` | Lower control arm inboard ball joints |
| `lca_outer` | Lower control arm outboard ball joint |
| `uca_front`, `uca_rear` | Upper control arm inboard ball joints |
| `uca_outer` | Upper control arm outboard ball joint |
| `tierod_inner`, `tierod_outer`| Tie rod ends |
| `pushrod_outer` | Outboard pushrod pickup |
| `wheel_center` | Wheel centre |

#### Units and style parameters

Model interfaces are **SI, unprefixed** throughout: metres, radians, seconds, kilograms, newtons.

Shared visual styling lives in [Styles.dyad](dyad/Styles.dyad) and is prefixed `sty_`, so
appearance parameters are visually distinct from physical ones.

## Testing

### Testing in Julia

1. Run `Julia: Start REPL` from the command palette if it did not automatically start
1. Type `]`.  This will take you to the package manager prompt.
1. At the `pkg>` prompt, type `instantiate` (this downloads all the Julia libraries you will need, and the very first time you do it it might take a while).
1. From the same `pkg>` prompt, type `test`. This will test to make sure the models are working as expected. It may also take some time but you should eventually see a result that indicates X of Y tests passed.
1. Use the `Backspace`/`Delete` key to return to the normal Julia REPL, it should look like this: `julia>`.

### Update Test Reference Results

1. In Julia REPL, set `ENV["DYAD_UPDATE_REFS"] = "1"`
1. Follow the typical flow. This will overwrite exsisting test references
1. Exit the pacakge mamanger and in the Julia REPL run `delete!(ENV, "DYAD_UPDATE_REFS")`

### Testing in Docker

Runs the test suite in the same Linux environment CI uses. The image build instantiates the project and runs its tests against the committed Julia files in `generated/`.

1. Install [Docker Desktop](https://www.docker.com/products/docker-desktop/) and start it.

1. Build the image to run the tests:

   ```powershell
   docker buildx build `
      --progress=plain `
      --no-cache-filter tests `
      --output type=cacheonly `
      --secret id=juliahub_auth,src="$HOME\.julia\servers\juliahub.com\auth.toml" `
      .
   ```

The build uses your JuliaHub credentials to download Dyad packages; the secret is available only while dependencies are installed and is not retained in the image. The command exits non-zero if dependency installation or a test fails. It does not compile `.dyad` sources, so regenerate `generated/` in Dyad Studio and commit it whenever a model changes.

To run one suite, add `--build-arg TEST_FILE=generated/TireTestRig_test.jl`. The `--no-cache-filter tests` option reruns tests while preserving the setup cache.

The runner always includes the full generated test suite. When a file is selected,
the shared test wrapper skips other components before starting their simulations.

CI first builds and caches the `setup` stage, including precompilation of the Julia test environment. Setup passes `--precompile-only` to the test runner, which exits before running simulations. The parallel jobs reuse that cache and run the tests listed in `.github/workflows/regression.yml`, with up to 20 jobs in parallel. Add new test files to that list manually. Plain `Pkg.test()` still runs the full generated test suite. Test failures do not prevent the setup cache from being saved.

GitHub Actions passes the base64-encoded `JULIAHUB_TOKEN_ENCODED` repository secret directly to the Docker build.

### Refreshing the CI JuliaHub token

JuliaHub credentials expire, and CI fails at the dependency-install step when they do. The fix is to re-encode your local `auth.toml` and update the repository secret.

```powershell
[Convert]::ToBase64String([IO.File]::ReadAllBytes("$HOME\.julia\servers\juliahub.com\auth.toml")) | Set-Clipboard
```

```linux
base64 -w 0 "$HOME/.julia/servers/juliahub.com/auth.toml" | wl-copy
```