# wpml

**A DJI mission planned at 50 m can fly 50 m underground, from one wrong height field the file format never flags. wpml is the Rust library that catches it.**

```text
$ wpml from-plan ArduPilotSITL-ArduCopter.plan -o out.kmz --drone m3e
wpml: ArduPilotSITL-ArduCopter.plan: height: seq 4: frame GLOBAL is altitude above mean sea level (EGM96);
DJI executeHeightMode WGS84 is height above the ellipsoid. They differ by the geoid separation N at the
site. Pass it (ellipsoid minus EGM96, metres) or replan in GLOBAL_RELATIVE_ALT
```

It stopped at the one waypoint that would have flown off by the geoid separation. Most tools pick a datum and carry on.

**[Live demo](https://mattbusel.github.io/drone-demos/)**

[![crates.io](https://img.shields.io/crates/v/wpml.svg)](https://crates.io/crates/wpml)
[![downloads](https://img.shields.io/crates/d/wpml.svg)](https://crates.io/crates/wpml)
[![docs.rs](https://docs.rs/wpml/badge.svg)](https://docs.rs/wpml)
[![license](https://img.shields.io/crates/l/wpml.svg)](#license)

Read and write DJI WPML missions: the `.kmz` files that DJI Pilot 2, FlightHub 2, the Cloud API and MSDK v5 use for the Matrice 30/300/350, Mavic 3 Enterprise, Matrice 3D and the rest of the enterprise line. Every height datum is mapped explicitly instead of guessed, every `/wpmz/` namespace DJI has shipped is accepted, and elements wpml does not model are kept as raw XML and written back unchanged, so a round trip loses nothing. It is tested against the real KMZ files DJI publishes with MSDK v5 and the DJI Cloud API reference, and it converts an ArduPilot or PX4 mission into a DJI KMZ so one plan flies on both fleets. `MIT OR Apache-2.0`.

## The height problem

Most broken DJI missions are broken in one field. `executeHeightMode`
`WGS84` means height above the **ellipsoid**, not above sea level, and the
two differ by the geoid separation N, tens of metres in most places. Write a
sea level altitude into it and the aircraft flies N metres off. Write a
height above takeoff into it and a mission planned at 50 m flies at 50 m
above the ellipsoid, which on a hillside site is underground. Nothing in
the file format stops either. wpml does: every height datum is mapped
explicitly, and when the information to map it is missing you get an error
naming the waypoint, not a guess.

## What it does

* **Opens any WPML KMZ.** `wpmz/waylines.wpml` (what the aircraft
  executes) and `wpmz/template.kml` (what Pilot 2 and FlightHub 2 edit) are
  parsed into typed structs: mission config, wayline folders, waypoints,
  heading and turn parameters, action groups and actions (`takePhoto`,
  `startRecord`, `stopRecord`, `gimbalRotate`, `rotateYaw`, `hover`,
  `zoom`). Either file may be missing; `wpmz/res/` comes along as bytes.
* **Accepts every namespace DJI has shipped.** DJI's own published files
  disagree: the Cloud API reference uses `http://www.dji.com/wpmz/1.0.2`,
  the MSDK v5 waypoint sample `http://www.dji.com/wpmz/1.0.0`, and the MSDK
  v5 flight task sample `http://www.corp.com/wpmz/1.0.6`. A parser keyed to
  one URI or to the `wpml:` prefix fails on the others. wpml matches any
  `/wpmz/` namespace and writes the one it read back out.
* **Loses nothing.** Elements it does not model (`isRisky`,
  `waypointGimbalHeadingParam`, `orientedShoot`, `panoShot`, whatever DJI
  adds next) are kept as raw XML and written back. Both MSDK v5 sample
  files read, write and read back to the same model.
* **Writes a KMZ for both sides.** Pilot 2 and FlightHub 2 import
  `template.kml`; MSDK `startMission` and Cloud API flight tasks execute
  `waylines.wpml`. A KMZ with only one of them works in one place.
  `Kmz::from_waylines` writes the matching template next to the waylines,
  converting `WGS84` ellipsoid heights to the template's `EGM96` sea level
  heights with the geoid separation you give it, and refusing without one.
* **Bridges ArduPilot and PX4.** With the `mavmission` feature, a
  QGroundControl `.plan` or Mission Planner `.waypoints` mission becomes a
  DJI KMZ: waypoints, hold times, speed changes, camera triggers and gimbal
  commands. Anything that would change where the aircraft flies and has no
  WPML equivalent (DO_JUMP loops, terrain-following frames, mixed height
  frames, loiter turns, splines) is an error. Anything that only drops a
  payload detail is a note.

## Use

```toml
[dependencies]
wpml = "0.1"
```

```rust
let bytes = std::fs::read("mission.kmz")?;
let kmz = wpml::Kmz::from_bytes(&bytes)?;
let waylines = kmz.waylines.as_ref().expect("no waylines.wpml");
for folder in &waylines.folders {
    println!("wayline {:?}: heights are {:?}", folder.wayline_id, folder.execute_height_mode);
    for wp in &folder.placemarks {
        println!("  {} {},{} at {:?} m", wp.index, wp.lat, wp.lon, wp.execute_height);
        for group in &wp.action_groups {
            for action in &group.actions {
                println!("    {}", action.func.name());
            }
        }
    }
}
std::fs::write("copy.kmz", kmz.to_bytes()?)?;
# Ok::<(), Box<dyn std::error::Error>>(())
```

ArduPilot or PX4 to DJI (`features = ["mavmission"]`):

```rust
use wpml::bridge::{from_plan, BridgeOptions};

let plan = mavmission::plan::parse(&std::fs::read_to_string("survey.plan")?)?;
let (drone, payload) = wpml::drone_by_name("m3e").unwrap();
let mut opts = BridgeOptions::new(drone);
opts.payload_enum = payload;
opts.speed = 8.0;
// Only needed if the plan uses MAV_FRAME_GLOBAL (sea level) altitudes:
// opts.geoid_separation = Some(n_at_site);
let converted = from_plan(&plan, &opts)?;
for note in &converted.notes {
    eprintln!("note: {note}");
}
std::fs::write("survey.kmz", converted.kmz.to_bytes()?)?;
# Ok::<(), Box<dyn std::error::Error>>(())
```

## Command line

```sh
cargo install wpml --features cli
```

`wpml info` on the waypoint sample DJI ships with MSDK v5:

```text
$ wpml info msdk_waypointsample.kmz
msdk_waypointsample.kmz
  template.kml: namespace http://www.dji.com/wpmz/1.0.0
    drone: m30 (67:0)
    payload: 52 at position 0
    flyToWaylineMode safely, finishAction goHome, exitOnRCLost goContinue, executeRCLostAction -
    takeOffSecurityHeight 20 m, globalTransitionalSpeed 5 m/s
    folder: waypoint templateId 0, 4 waypoints, heights 100 to 100 m (relativeToStartPoint), speed 5 m/s
      actions: none
  waylines.wpml: namespace http://www.dji.com/wpmz/1.0.0
    ...
  heights: template relativeToStartPoint -> waylines relativeToStartPoint for wayline 0: consistent
```

`wpml from-plan` on QGroundControl's own ArduCopter SITL test plan, which
mixes three height frames:

```text
$ wpml from-plan ArduPilotSITL-ArduCopter.plan -o out.kmz --drone m3e
wpml: ArduPilotSITL-ArduCopter.plan: height: seq 4: frame GLOBAL is altitude above mean sea level (EGM96);
DJI executeHeightMode WGS84 is height above the ellipsoid. They differ by the geoid separation N at the
site. Pass it (ellipsoid minus EGM96, metres) or replan in GLOBAL_RELATIVE_ALT
```

With `--geoid-sep` given it still stops, at the mix of frames, because one
DJI wayline has one height mode. A converter that picks one and carries on
flies part of that mission at the wrong height.

## Height mapping

| ArduPilot / PX4 frame | waylines.wpml `executeHeightMode` | `executeHeight` | template.kml `heightMode` |
|---|---|---|---|
| `GLOBAL_RELATIVE_ALT` | `relativeToStartPoint` | `z` | `relativeToStartPoint` |
| `GLOBAL` (sea level) | `WGS84` (ellipsoid) | `z + N` | `EGM96`, `height = z` |
| `GLOBAL_TERRAIN_ALT` | none | error | none |
| local frames | none | error | none |

ArduPilot's relative altitude is relative to home, where it armed; DJI's
`relativeToStartPoint` is relative to where it took off. Fly the converted
mission from the same spot you planned home.

## What maps to what

| MAVLink | WPML |
|---|---|
| `NAV_WAYPOINT`, `NAV_LOITER_TIME` | `Placemark`; hold time becomes a `hover` action |
| `NAV_TAKEOFF` (relative frame) | `takeOffSecurityHeight` |
| `NAV_RETURN_TO_LAUNCH` / `NAV_LAND` last | `finishAction` `goHome` / `autoLand` (a LAND with its own position adds a waypoint there first) |
| `DO_CHANGE_SPEED` | `waypointSpeed` from that leg on |
| `DO_SET_CAM_TRIGG_DIST` | `takePhoto` group with a `multipleDistance` trigger over the waypoints it covers |
| `IMAGE_START_CAPTURE` / `IMAGE_STOP_CAPTURE` | `takePhoto`, single or `multipleTiming` |
| `DO_DIGICAM_CONTROL` shoot | `takePhoto` |
| `VIDEO_START_CAPTURE` / `VIDEO_STOP_CAPTURE` | `startRecord` / `stopRecord` |
| `DO_MOUNT_CONTROL`, `DO_GIMBAL_MANAGER_PITCHYAW` | `gimbalRotate` (yaw only when it is earth-referenced, as WPML's is) |
| `CONDITION_YAW` absolute | `rotateYaw` |
| `DO_JUMP`, other NAV commands | error |
| other DO and CONDITION commands | dropped, with a note |

Commands before the first waypoint go into the wayline's
`startActionGroup`.

## How it is tested

* The two KMZ files DJI publishes with the MSDK v5 sample code (MIT): every
  field checked against the file, then written and read back to an equal
  model, namespace included.
* The `waylines.wpml` sample from the DJI Cloud API WPML reference.
* QGroundControl's own test missions (Apache-2.0) through the bridge: a
  Mission Planner file that must stop at its DO_JUMP, and the ArduCopter
  SITL plan that must stop at its mix of height frames.

Sources and licenses: [tests/data/SOURCES.md](tests/data/SOURCES.md).

## Status

0.1. Reads and writes waypoint missions; mapping templates (`mapping2d`,
`mapping3d`, `mappingStrip`) are read and written back with their template
elements kept as raw XML, not typed. Not yet checked: loading the written
files on an aircraft. The writer follows the element order of DJI's own
files, and the default namespace for new files is
`http://www.dji.com/wpmz/1.0.6`, the newest version in DJI's published
samples; set `Document::namespace` if your app wants another.

## License

MIT OR Apache-2.0. The DJI sample KMZ files in `tests/data` are MIT,
Copyright (c) 2022 DJI (`tests/data/LICENSE-MIT-dji-msdk-v5-samples`); the
QGroundControl missions are Apache-2.0.
