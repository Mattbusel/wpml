//! `wpml info <mission.kmz>` and `wpml from-plan <mission.plan|.waypoints> -o <out.kmz> --drone <model>`.

use std::process::ExitCode;
use wpml::bridge::{self, BridgeOptions};
use wpml::*;

const USAGE: &str = "\
usage:
  wpml info <mission.kmz>
  wpml from-plan <mission.plan | mission.waypoints> -o <out.kmz> --drone <model>
                 [--payload <payloadEnumValue>] [--speed <m/s>] [--geoid-sep <m>]
                 [--finish goHome|autoLand|noAction|gotoFirstWaypoint] [--namespace <uri>]

  --drone      m350 m300 m30 m30t m3e m3t m3m m3d m3td, or droneEnumValue[:sub] such as 67:1
  --geoid-sep  geoid separation N at the site in metres (WGS84 ellipsoid minus EGM96);
               required when the mission uses MAV_FRAME_GLOBAL (sea level) altitudes";

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let r = match args.first().map(String::as_str) {
        Some("info") if args.len() == 2 => info(&args[1]),
        Some("from-plan") => from_plan(&args[1..]),
        Some("-h") | Some("--help") | Some("help") => {
            println!("{USAGE}");
            return ExitCode::SUCCESS;
        }
        _ => Err(USAGE.to_string()),
    };
    match r {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("wpml: {e}");
            ExitCode::FAILURE
        }
    }
}

fn info(path: &str) -> Result<(), String> {
    let bytes = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
    let kmz = Kmz::from_bytes(&bytes).map_err(|e| format!("{path}: {e}"))?;
    println!("{path}");
    match &kmz.template {
        Some(t) => describe("template.kml", t),
        None => println!("  template.kml: missing (Pilot 2 and FlightHub 2 import this file)"),
    }
    match &kmz.waylines {
        Some(w) => describe("waylines.wpml", w),
        None => println!("  waylines.wpml: missing (MSDK and the Cloud API execute this file)"),
    }
    if !kmz.resources.is_empty() {
        println!("  res/: {} file(s)", kmz.resources.len());
        for (n, b) in &kmz.resources {
            println!("    {n} ({} bytes)", b.len());
        }
    }
    if let (Some(t), Some(w)) = (&kmz.template, &kmz.waylines) {
        for wf in &w.folders {
            let tf = t.folders.iter().find(|f| f.template_id == wf.template_id);
            let th = tf
                .and_then(|f| f.coordinate_sys.as_ref())
                .and_then(|c| c.height_mode.clone());
            if let (Some(th), Some(eh)) = (th, wf.execute_height_mode.clone()) {
                let ok = matches!(
                    (&th, &eh),
                    (
                        HeightMode::RelativeToStartPoint,
                        ExecuteHeightMode::RelativeToStartPoint
                    ) | (HeightMode::Egm96, ExecuteHeightMode::Wgs84)
                        | (HeightMode::AboveGroundLevel, ExecuteHeightMode::Wgs84)
                        | (
                            HeightMode::RealTimeFollowSurface,
                            ExecuteHeightMode::RealTimeFollowSurface
                        )
                );
                let verdict = if ok { "consistent" } else { "MISMATCH" };
                println!(
                    "  heights: template {th} -> waylines {eh} for wayline {}: {verdict}",
                    wf.wayline_id
                        .map(|v| v.to_string())
                        .unwrap_or_else(|| "?".into())
                );
            }
        }
    }
    Ok(())
}

fn describe(name: &str, d: &Document) {
    println!(
        "  {name}: namespace {}",
        d.namespace.as_deref().unwrap_or("(none)")
    );
    let m = &d.mission_config;
    if let Some(di) = &m.drone_info {
        let label = drone_name(di).unwrap_or("unknown model");
        println!(
            "    drone: {} ({}:{})",
            label,
            di.drone_enum_value,
            di.drone_sub_enum_value
                .map(|v| v.to_string())
                .unwrap_or_default()
        );
    }
    for p in &m.payload_info {
        println!(
            "    payload: {} at position {}",
            p.payload_enum_value, p.payload_position_index
        );
    }
    let opt = |o: Option<String>| o.unwrap_or_else(|| "-".into());
    println!(
        "    flyToWaylineMode {}, finishAction {}, exitOnRCLost {}, executeRCLostAction {}",
        opt(m.fly_to_wayline_mode.as_ref().map(|v| v.to_string())),
        opt(m.finish_action.as_ref().map(|v| v.to_string())),
        opt(m.exit_on_rc_lost.as_ref().map(|v| v.to_string())),
        opt(m.execute_rc_lost_action.as_ref().map(|v| v.to_string())),
    );
    println!(
        "    takeOffSecurityHeight {} m, globalTransitionalSpeed {} m/s",
        opt(m.take_off_security_height.map(|v| v.to_string())),
        opt(m.global_transitional_speed.map(|v| v.to_string())),
    );
    for f in &d.folders {
        let mode = f
            .execute_height_mode
            .as_ref()
            .map(|m| m.to_string())
            .or_else(|| {
                f.coordinate_sys
                    .as_ref()
                    .and_then(|c| c.height_mode.as_ref())
                    .map(|m| m.to_string())
            })
            .unwrap_or_else(|| "-".into());
        let heights: Vec<f64> = f
            .placemarks
            .iter()
            .filter_map(|p| p.execute_height.or(p.height))
            .collect();
        let range = if heights.is_empty() {
            "-".to_string()
        } else {
            let lo = heights.iter().cloned().fold(f64::INFINITY, f64::min);
            let hi = heights.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
            format!("{lo} to {hi} m")
        };
        let mut actions: Vec<(String, usize)> = Vec::new();
        let groups = f
            .placemarks
            .iter()
            .flat_map(|p| p.action_groups.iter())
            .chain(f.start_action_group.iter());
        for g in groups {
            for a in &g.actions {
                let n = a.func.name().to_string();
                match actions.iter_mut().find(|(k, _)| *k == n) {
                    Some((_, c)) => *c += 1,
                    None => actions.push((n, 1)),
                }
            }
        }
        let acts = if actions.is_empty() {
            "none".to_string()
        } else {
            actions
                .iter()
                .map(|(k, c)| format!("{k} x{c}"))
                .collect::<Vec<_>>()
                .join(", ")
        };
        println!(
            "    folder: {}templateId {}{}, {} waypoints, heights {} ({}), speed {} m/s",
            f.template_type
                .as_ref()
                .map(|t| format!("{t} "))
                .unwrap_or_default(),
            opt(f.template_id.map(|v| v.to_string())),
            f.wayline_id
                .map(|v| format!(", waylineId {v}"))
                .unwrap_or_default(),
            f.placemarks.len(),
            range,
            mode,
            opt(f.auto_flight_speed.map(|v| v.to_string())),
        );
        println!("      actions: {acts}");
    }
}

fn from_plan(args: &[String]) -> Result<(), String> {
    let mut input = None;
    let mut out = None;
    let mut drone = None;
    let mut payload = None;
    let mut speed = None;
    let mut geoid = None;
    let mut finish = None;
    let mut ns = None;
    let mut i = 0;
    while i < args.len() {
        let a = args[i].as_str();
        let mut val = || -> Result<String, String> {
            i += 1;
            args.get(i)
                .cloned()
                .ok_or_else(|| format!("{a} needs a value"))
        };
        match a {
            "-o" | "--output" => out = Some(val()?),
            "--drone" => drone = Some(val()?),
            "--payload" => payload = Some(val()?),
            "--speed" => speed = Some(val()?),
            "--geoid-sep" => geoid = Some(val()?),
            "--finish" => finish = Some(val()?),
            "--namespace" => ns = Some(val()?),
            s if s.starts_with('-') => return Err(format!("unknown option {s}\n{USAGE}")),
            s => {
                if input.replace(s.to_string()).is_some() {
                    return Err(USAGE.into());
                }
            }
        }
        i += 1;
    }
    let input = input.ok_or(USAGE)?;
    let out = out.ok_or("missing -o <out.kmz>")?;
    let drone = drone.ok_or("missing --drone; DJI apps refuse a mission made for another model")?;
    let (info, default_payload) =
        drone_by_name(&drone).ok_or_else(|| format!("unknown drone {drone}"))?;
    let mut opts = BridgeOptions::new(info);
    opts.payload_enum = match payload {
        Some(p) => Some(p.parse().map_err(|_| format!("bad --payload {p}"))?),
        None => default_payload,
    };
    if let Some(s) = speed {
        opts.speed = s.parse().map_err(|_| format!("bad --speed {s}"))?;
    }
    if let Some(g) = geoid {
        opts.geoid_separation = Some(g.parse().map_err(|_| format!("bad --geoid-sep {g}"))?);
    }
    if let Some(f) = finish {
        opts.default_finish = match FinishAction::parse(&f) {
            FinishAction::Other(_) => return Err(format!("bad --finish {f}")),
            v => v,
        };
    }
    opts.namespace = ns;
    let text = std::fs::read_to_string(&input).map_err(|e| format!("{input}: {e}"))?;
    let converted = if mavmission::wpl::sniff(&text) {
        let w = mavmission::wpl::parse(&text).map_err(|e| format!("{input}: {e}"))?;
        bridge::from_wpl(&w, &opts)
    } else {
        let p = mavmission::plan::parse(&text).map_err(|e| format!("{input}: {e}"))?;
        bridge::from_plan(&p, &opts)
    }
    .map_err(|e| format!("{input}: {e}"))?;
    for n in &converted.notes {
        eprintln!("note: {n}");
    }
    let bytes = converted.kmz.to_bytes().map_err(|e| e.to_string())?;
    std::fs::write(&out, &bytes).map_err(|e| format!("{out}: {e}"))?;
    let wl = converted.kmz.waylines.as_ref().map(|w| &w.folders[0]);
    println!(
        "wrote {out}: {} waypoints, executeHeightMode {}",
        wl.map(|f| f.placemarks.len()).unwrap_or(0),
        wl.and_then(|f| f.execute_height_mode.as_ref())
            .map(|m| m.to_string())
            .unwrap_or_default()
    );
    Ok(())
}
