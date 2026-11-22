//! ArduPilot / PX4 mission to DJI WPML, from [`mavmission`] items.
//!
//! One plan, two fleets: read a QGroundControl `.plan` or a Mission Planner
//! `.waypoints` file with `mavmission`, convert it here, and fly the same
//! survey on a Matrice. The rule throughout is that anything that would
//! change *where the aircraft goes* is an error, and anything that only
//! drops a payload nicety is a note in [`Converted::notes`]. Nothing is
//! guessed silently.
//!
//! Heights get the most care, because that is how converted missions crash:
//!
//! | MAV_FRAME | executeHeightMode | executeHeight |
//! |---|---|---|
//! | `GLOBAL_RELATIVE_ALT` (3, 6) | `relativeToStartPoint` | `z` |
//! | `GLOBAL` (0, 5), above mean sea level | `WGS84` (ellipsoid) | `z + N`, needs [`BridgeOptions::geoid_separation`] |
//! | `GLOBAL_TERRAIN_ALT` (10, 11) | none | error |
//!
//! DJI's `WGS84` is the ellipsoid, not sea level, and the two differ by the
//! geoid separation N, which is tens of metres in most places. Writing an
//! AMSL altitude H as a `WGS84` height puts the aircraft at H - N above sea
//! level: too high where N is negative, too low where N is positive. So a
//! `GLOBAL` mission needs N, and a mission mixing frames is refused,
//! because one DJI wayline has one height mode.

use crate::kmz::Kmz;
use crate::model::*;
use crate::Error;
use mavmission::{command_name, frame, MissionItem, UNSET_XY};

/// Options for [`from_mission_items`].
#[derive(Debug, Clone, PartialEq)]
pub struct BridgeOptions {
    /// The DJI aircraft. Pilot 2 and MSDK refuse a mission for another model.
    pub drone: DroneInfo,
    /// `payloadEnumValue` of the main camera, when known (see [`crate::DRONES`]).
    pub payload_enum: Option<u32>,
    /// Cruise speed in m/s (`autoFlightSpeed`), until a `DO_CHANGE_SPEED`.
    pub speed: f64,
    /// Geoid separation N at the site in metres (WGS84 ellipsoid height minus
    /// EGM96 height). Required for `MAV_FRAME_GLOBAL` missions.
    pub geoid_separation: Option<f64>,
    /// `takeOffSecurityHeight` in metres when the mission has no relative-frame takeoff.
    pub take_off_security_height: f64,
    /// Action after the last waypoint when the mission does not end in RTL or LAND.
    pub default_finish: FinishAction,
    /// WPML namespace to write; `None` uses [`crate::DEFAULT_NAMESPACE`].
    pub namespace: Option<String>,
}

impl BridgeOptions {
    /// Options for a drone with defaults: 5 m/s, 20 m safe takeoff height
    /// (the value in DJI's samples), RTH at the end.
    pub fn new(drone: DroneInfo) -> Self {
        BridgeOptions {
            drone,
            payload_enum: None,
            speed: 5.0,
            geoid_separation: None,
            take_off_security_height: 20.0,
            default_finish: FinishAction::GoHome,
            namespace: None,
        }
    }
}

/// The converted mission and what did not carry over.
#[derive(Debug, Clone, PartialEq)]
pub struct Converted {
    /// `template.kml` + `waylines.wpml`.
    pub kmz: Kmz,
    /// One line per item that was dropped or approximated, with its seq.
    pub notes: Vec<String>,
}

mod cmd {
    pub const NAV_WAYPOINT: u16 = 16;
    pub const NAV_LOITER_TIME: u16 = 19;
    pub const NAV_RETURN_TO_LAUNCH: u16 = 20;
    pub const NAV_LAND: u16 = 21;
    pub const NAV_TAKEOFF: u16 = 22;
    pub const CONDITION_YAW: u16 = 115;
    pub const DO_JUMP: u16 = 177;
    pub const DO_CHANGE_SPEED: u16 = 178;
    pub const DO_DIGICAM_CONTROL: u16 = 203;
    pub const DO_MOUNT_CONTROL: u16 = 205;
    pub const DO_SET_CAM_TRIGG_DIST: u16 = 206;
    pub const DO_GIMBAL_MANAGER_PITCHYAW: u16 = 1000;
    pub const IMAGE_START_CAPTURE: u16 = 2000;
    pub const IMAGE_STOP_CAPTURE: u16 = 2001;
    pub const VIDEO_START_CAPTURE: u16 = 2500;
    pub const VIDEO_STOP_CAPTURE: u16 = 2501;
}

/// `GIMBAL_MANAGER_FLAGS_YAW_LOCK`: yaw is in the earth frame.
const GIMBAL_YAW_LOCK: u32 = 16;
/// `GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME`: yaw is relative to the vehicle.
const GIMBAL_YAW_IN_VEHICLE_FRAME: u32 = 32;
/// `GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME`: yaw is relative to north (the
/// newer flag for what YAW_LOCK means).
const GIMBAL_YAW_IN_EARTH_FRAME: u32 = 64;

fn name(c: u16) -> String {
    match command_name(c) {
        Some(n) => n.to_string(),
        None => format!("MAV_CMD {c}"),
    }
}

fn wrap180(mut a: f64) -> f64 {
    while a > 180.0 {
        a -= 360.0;
    }
    while a < -180.0 {
        a += 360.0;
    }
    a
}

/// An interval trigger (distance or time) that is running.
struct Open {
    start: u32,
    trigger: TriggerType,
    param: f64,
    seq: u16,
}

struct State<'a> {
    opts: &'a BridgeOptions,
    placemarks: Vec<Placemark>,
    mode: Option<(ExecuteHeightMode, u8, u16)>,
    speed: f64,
    /// reachPoint actions per waypoint index, and actions before the first waypoint.
    point_actions: Vec<(u32, ActionFunc)>,
    start_actions: Vec<ActionFunc>,
    intervals: Vec<(u32, u32, TriggerType, f64)>,
    open: Option<Open>,
    finish: Option<FinishAction>,
    takeoff_height: Option<f64>,
    ended_at: Option<u16>,
    notes: Vec<String>,
}

impl State<'_> {
    fn last_index(&self) -> Option<u32> {
        self.placemarks.last().map(|p| p.index)
    }

    fn action_here(&mut self, f: ActionFunc) {
        match self.last_index() {
            Some(i) => self.point_actions.push((i, f)),
            None => self.start_actions.push(f),
        }
    }

    fn height(&mut self, it: &MissionItem) -> Result<f64, Error> {
        let seq = it.seq;
        let (mode, h) = match it.frame {
            frame::GLOBAL_RELATIVE_ALT | frame::GLOBAL_RELATIVE_ALT_INT => {
                (ExecuteHeightMode::RelativeToStartPoint, it.z as f64)
            }
            frame::GLOBAL | frame::GLOBAL_INT => {
                let n = self.opts.geoid_separation.ok_or_else(|| {
                    Error::Height(format!(
                        "seq {seq}: frame GLOBAL is altitude above mean sea level (EGM96); DJI \
                         executeHeightMode WGS84 is height above the ellipsoid. They differ by \
                         the geoid separation N at the site. Pass it (ellipsoid minus EGM96, \
                         metres) or replan in GLOBAL_RELATIVE_ALT"
                    ))
                })?;
                (ExecuteHeightMode::Wgs84, it.z as f64 + n)
            }
            frame::GLOBAL_TERRAIN_ALT | frame::GLOBAL_TERRAIN_ALT_INT => {
                return Err(Error::Height(format!(
                    "seq {seq}: frame GLOBAL_TERRAIN_ALT follows a terrain database; WPML has no \
                     terrain-database height mode (realTimeFollowSurface uses the aircraft's own \
                     sensors and only the M3E/M3T/M3M support it). Replan in GLOBAL_RELATIVE_ALT \
                     or GLOBAL"
                )))
            }
            f => {
                return Err(Error::Height(format!(
                    "seq {seq}: frame {} is not a global frame; WPML waypoints need latitude, \
                     longitude and a height datum",
                    mavmission::frame_name(f).unwrap_or("unknown")
                )))
            }
        };
        match &self.mode {
            None => self.mode = Some((mode, it.frame, seq)),
            Some((m, f0, s0)) if *m != mode => {
                return Err(Error::Height(format!(
                    "seq {seq} uses frame {} but seq {s0} uses frame {}; one DJI wayline has one \
                     executeHeightMode, so a mission that mixes height frames cannot be \
                     converted without changing where it flies",
                    mavmission::frame_name(it.frame).unwrap_or("?"),
                    mavmission::frame_name(*f0).unwrap_or("?"),
                )))
            }
            _ => {}
        }
        if !h.is_finite() {
            return Err(Error::Height(format!(
                "seq {seq}: altitude is not a number"
            )));
        }
        Ok(h)
    }

    fn waypoint(&mut self, it: &MissionItem) -> Result<(), Error> {
        if it.x == UNSET_XY || it.y == UNSET_XY || (it.x == 0 && it.y == 0) {
            return Err(Error::Convert {
                seq: it.seq,
                reason: format!(
                    "{} has no position (0,0 means 'here' to ArduPilot); WPML waypoints need one",
                    name(it.command)
                ),
            });
        }
        let h = self.height(it)?;
        let index = self.placemarks.len() as u32;
        self.placemarks.push(Placemark {
            lon: it.y as f64 / 1e7,
            lat: it.x as f64 / 1e7,
            index,
            execute_height: Some(h),
            waypoint_speed: Some(self.speed),
            heading: Some(HeadingParam {
                mode: Some(HeadingMode::FollowWayline),
                angle: Some(0.0),
                poi_point: Some("0.000000,0.000000,0.000000".into()),
                angle_enable: Some(false),
                ..Default::default()
            }),
            turn: Some(TurnParam {
                mode: Some(TurnMode::ToPointAndStopWithDiscontinuityCurvature),
                damping_dist: Some(0.0),
                extra: Vec::new(),
            }),
            use_straight_line: Some(true),
            ..Default::default()
        });
        Ok(())
    }

    fn close_interval(&mut self, end: u32) {
        if let Some(o) = self.open.take() {
            self.intervals
                .push((o.start, end.max(o.start), o.trigger, o.param));
        }
    }

    fn open_interval(&mut self, it: &MissionItem, trigger: TriggerType, param: f64) {
        let start = self.last_index().unwrap_or(0);
        if let Some(prev) = &self.open {
            let s = prev.seq;
            self.notes.push(format!(
                "seq {}: {} replaces the interval capture started at seq {s}",
                it.seq,
                name(it.command)
            ));
            self.close_interval(start);
        }
        self.open = Some(Open {
            start,
            trigger,
            param,
            seq: it.seq,
        });
    }

    fn item(&mut self, it: &MissionItem) -> Result<(), Error> {
        if let Some(end) = self.ended_at {
            return Err(Error::Convert {
                seq: it.seq,
                reason: format!(
                    "{} after the mission ended at seq {end}; DJI finishAction must be last",
                    name(it.command)
                ),
            });
        }
        let p = |v: f32| v as f64;
        match it.command {
            cmd::NAV_WAYPOINT => {
                self.waypoint(it)?;
                if it.param1 > 0.0 {
                    self.action_here(ActionFunc::Hover {
                        hover_time: p(it.param1),
                    });
                }
                if it.param4.is_finite() && it.param4 != 0.0 {
                    self.notes.push(format!(
                        "seq {}: waypoint yaw {} dropped; the DJI waypoint follows the wayline",
                        it.seq, it.param4
                    ));
                }
            }
            cmd::NAV_LOITER_TIME => {
                self.waypoint(it)?;
                if it.param1 > 0.0 {
                    self.action_here(ActionFunc::Hover {
                        hover_time: p(it.param1),
                    });
                }
            }
            cmd::NAV_TAKEOFF => {
                if matches!(
                    it.frame,
                    frame::GLOBAL_RELATIVE_ALT | frame::GLOBAL_RELATIVE_ALT_INT
                ) && it.z > 0.0
                {
                    self.takeoff_height = Some(p(it.z).clamp(1.2, 1500.0));
                } else {
                    self.notes.push(format!(
                        "seq {}: NAV_TAKEOFF altitude not used; DJI climbs to \
                         takeOffSecurityHeight {} m",
                        it.seq, self.opts.take_off_security_height
                    ));
                }
            }
            cmd::NAV_RETURN_TO_LAUNCH => {
                self.finish = Some(FinishAction::GoHome);
                self.ended_at = Some(it.seq);
            }
            cmd::NAV_LAND => {
                let has_pos = it.x != UNSET_XY && it.y != UNSET_XY && !(it.x == 0 && it.y == 0);
                if has_pos {
                    let last = self.placemarks.last().ok_or_else(|| Error::Convert {
                        seq: it.seq,
                        reason: "NAV_LAND before any waypoint".into(),
                    })?;
                    let same = (
                        (last.lat * 1e7).round() as i32,
                        (last.lon * 1e7).round() as i32,
                    ) == (it.x, it.y);
                    if !same {
                        // Fly to the landing point at the last height, then land there,
                        // which is what ArduPilot and PX4 do with NAV_LAND.
                        let h = last.execute_height;
                        let mut wp = *it;
                        wp.frame = self.mode.as_ref().map(|m| m.1).unwrap_or(it.frame);
                        self.waypoint(&wp)?;
                        if let Some(l) = self.placemarks.last_mut() {
                            l.execute_height = h;
                        }
                    }
                }
                self.finish = Some(FinishAction::AutoLand);
                self.ended_at = Some(it.seq);
            }
            cmd::DO_JUMP => {
                return Err(Error::Convert {
                    seq: it.seq,
                    reason: "DO_JUMP loops have no WPML equivalent; unroll the loop first".into(),
                })
            }
            cmd::DO_CHANGE_SPEED => {
                if it.param2 > 0.0 {
                    self.speed = p(it.param2);
                    // MAVLink applies the new speed from here on, which in
                    // WPML is the leg that starts at the current waypoint.
                    if let Some(l) = self.placemarks.last_mut() {
                        l.waypoint_speed = Some(self.speed);
                    }
                } else {
                    self.notes.push(format!(
                        "seq {}: DO_CHANGE_SPEED without a speed ignored",
                        it.seq
                    ));
                }
            }
            cmd::DO_SET_CAM_TRIGG_DIST => {
                let end = self.last_index().unwrap_or(0);
                if it.param1 > 0.0 {
                    self.open_interval(it, TriggerType::MultipleDistance, p(it.param1));
                } else {
                    self.close_interval(end);
                }
            }
            cmd::IMAGE_START_CAPTURE => {
                let interval = p(it.param2);
                let count = it.param3;
                if count == 1.0 || interval <= 0.0 {
                    self.action_here(photo());
                } else {
                    if count > 1.0 {
                        self.notes.push(format!(
                            "seq {}: image count {count} not kept; DJI times photos until the \
                             interval ends",
                            it.seq
                        ));
                    }
                    self.open_interval(it, TriggerType::MultipleTiming, interval);
                }
            }
            cmd::IMAGE_STOP_CAPTURE => {
                let end = self.last_index().unwrap_or(0);
                self.close_interval(end);
            }
            cmd::DO_DIGICAM_CONTROL => {
                if it.params()[4] == 1.0 {
                    self.action_here(photo());
                } else {
                    self.notes.push(format!(
                        "seq {}: DO_DIGICAM_CONTROL without shoot=1 ignored",
                        it.seq
                    ));
                }
            }
            cmd::VIDEO_START_CAPTURE => self.action_here(ActionFunc::StartRecord {
                payload_position_index: 0,
                file_suffix: None,
                payload_lens_index: None,
            }),
            cmd::VIDEO_STOP_CAPTURE => self.action_here(ActionFunc::StopRecord {
                payload_position_index: 0,
                payload_lens_index: None,
            }),
            cmd::DO_MOUNT_CONTROL => {
                let yaw = p(it.param3);
                if yaw != 0.0 && yaw.is_finite() {
                    self.notes.push(format!(
                        "seq {}: mount yaw {yaw} dropped; DO_MOUNT_CONTROL yaw is relative to \
                         the vehicle, WPML gimbalRotate yaw is relative to north",
                        it.seq
                    ));
                }
                let roll = p(it.param2);
                self.action_here(gimbal(
                    Some(p(it.param1)),
                    (roll != 0.0 && roll.is_finite()).then_some(roll),
                    None,
                ));
            }
            cmd::DO_GIMBAL_MANAGER_PITCHYAW => {
                let flags = it.params()[4] as u32;
                let yaw = p(it.param2);
                let yaw = if yaw.is_finite() && yaw != 0.0 {
                    let earth = flags & (GIMBAL_YAW_LOCK | GIMBAL_YAW_IN_EARTH_FRAME) != 0
                        && flags & GIMBAL_YAW_IN_VEHICLE_FRAME == 0;
                    if earth {
                        Some(wrap180(yaw))
                    } else {
                        self.notes.push(format!(
                            "seq {}: gimbal yaw {yaw} dropped; without YAW_LOCK or \
                             YAW_IN_EARTH_FRAME it is relative to the vehicle, WPML \
                             gimbalRotate yaw is relative to north",
                            it.seq
                        ));
                        None
                    }
                } else {
                    None
                };
                let pitch = p(it.param1);
                self.action_here(gimbal(pitch.is_finite().then_some(pitch), None, yaw));
            }
            cmd::CONDITION_YAW => {
                if it.param4 == 0.0 {
                    self.action_here(ActionFunc::RotateYaw {
                        aircraft_heading: wrap180(p(it.param1)),
                        path_mode: Some(
                            if it.param3 < 0.0 {
                                "counterClockwise"
                            } else {
                                "clockwise"
                            }
                            .into(),
                        ),
                    });
                } else {
                    self.notes.push(format!(
                        "seq {}: relative CONDITION_YAW dropped; WPML rotateYaw takes a heading \
                         from north",
                        it.seq
                    ));
                }
            }
            c if c < 100 => {
                return Err(Error::Convert {
                    seq: it.seq,
                    reason: format!(
                        "{} changes the flight path and has no WPML equivalent in this version",
                        name(c)
                    ),
                })
            }
            c => self.notes.push(format!(
                "seq {}: {} dropped (no WPML action)",
                it.seq,
                name(c)
            )),
        }
        Ok(())
    }
}

fn photo() -> ActionFunc {
    ActionFunc::TakePhoto {
        payload_position_index: 0,
        file_suffix: None,
        payload_lens_index: None,
    }
}

fn gimbal(pitch: Option<f64>, roll: Option<f64>, yaw: Option<f64>) -> ActionFunc {
    ActionFunc::GimbalRotate(GimbalRotate {
        payload_position_index: 0,
        heading_yaw_base: Some("north".into()),
        rotate_mode: Some("absoluteAngle".into()),
        pitch,
        roll,
        yaw,
        time: None,
    })
}

/// Converts mission items (without the home row) to a KMZ with both
/// `waylines.wpml` and a matching `template.kml`.
///
/// Supported: `NAV_WAYPOINT` (hold time becomes a hover action),
/// `NAV_LOITER_TIME`, `NAV_TAKEOFF`, `NAV_LAND`, `NAV_RETURN_TO_LAUNCH`,
/// `DO_CHANGE_SPEED`, `DO_SET_CAM_TRIGG_DIST` (a `multipleDistance`
/// takePhoto group), `IMAGE_START_CAPTURE` / `IMAGE_STOP_CAPTURE`,
/// `DO_DIGICAM_CONTROL`, `VIDEO_START_CAPTURE` / `VIDEO_STOP_CAPTURE`,
/// `DO_MOUNT_CONTROL`, `DO_GIMBAL_MANAGER_PITCHYAW` and absolute
/// `CONDITION_YAW`. Other navigation commands and `DO_JUMP` are errors;
/// other DO and CONDITION commands are dropped with a note.
pub fn from_mission_items(items: &[MissionItem], opts: &BridgeOptions) -> Result<Converted, Error> {
    if !(opts.speed > 0.0 && opts.speed.is_finite()) {
        return Err(Error::Convert {
            seq: 0,
            reason: format!("speed {} m/s is not positive", opts.speed),
        });
    }
    let mut st = State {
        opts,
        placemarks: Vec::new(),
        mode: None,
        speed: opts.speed,
        point_actions: Vec::new(),
        start_actions: Vec::new(),
        intervals: Vec::new(),
        open: None,
        finish: None,
        takeoff_height: None,
        ended_at: None,
        notes: Vec::new(),
    };
    for it in items {
        st.item(it)?;
    }
    if st.placemarks.len() < 2 {
        return Err(Error::Convert {
            seq: items.last().map(|i| i.seq).unwrap_or(0),
            reason: format!(
                "{} waypoint(s); a DJI wayline needs at least 2",
                st.placemarks.len()
            ),
        });
    }
    let last = st.last_index().unwrap_or(0);
    st.close_interval(last);

    let mut group_id = 0u32;
    let mut next_group = || {
        let g = group_id;
        group_id += 1;
        g
    };
    let start_action_group = if st.start_actions.is_empty() {
        None
    } else {
        Some(ActionGroup {
            id: next_group(),
            start_index: 0,
            end_index: 0,
            mode: "sequence".into(),
            trigger: Some(TriggerType::ReachPoint),
            actions: actions(core::mem::take(&mut st.start_actions)),
            ..Default::default()
        })
    };
    // One reachPoint group per waypoint, in mission order.
    let mut by_point: Vec<(u32, Vec<ActionFunc>)> = Vec::new();
    for (i, f) in core::mem::take(&mut st.point_actions) {
        match by_point.iter_mut().find(|(j, _)| *j == i) {
            Some((_, v)) => v.push(f),
            None => by_point.push((i, vec![f])),
        }
    }
    for (i, fs) in by_point {
        let g = ActionGroup {
            id: next_group(),
            start_index: i,
            end_index: i,
            mode: "sequence".into(),
            trigger: Some(TriggerType::ReachPoint),
            actions: actions(fs),
            ..Default::default()
        };
        st.placemarks[i as usize].action_groups.push(g);
    }
    for (s, e, trigger, param) in core::mem::take(&mut st.intervals) {
        let g = ActionGroup {
            id: next_group(),
            start_index: s,
            end_index: e,
            mode: "sequence".into(),
            trigger: Some(trigger),
            trigger_param: Some(param),
            actions: actions(vec![photo()]),
            ..Default::default()
        };
        st.placemarks[s as usize].action_groups.push(g);
    }

    let (mode, _, _) = st
        .mode
        .clone()
        .ok_or_else(|| Error::Height("no waypoints".into()))?;
    let mission_config = MissionConfig {
        fly_to_wayline_mode: Some(FlyToWaylineMode::Safely),
        finish_action: Some(
            st.finish
                .clone()
                .unwrap_or_else(|| opts.default_finish.clone()),
        ),
        exit_on_rc_lost: Some(ExitOnRcLost::ExecuteLostAction),
        execute_rc_lost_action: Some(RcLostAction::GoBack),
        take_off_security_height: Some(st.takeoff_height.unwrap_or(opts.take_off_security_height)),
        global_transitional_speed: Some(opts.speed.min(15.0)),
        drone_info: Some(opts.drone),
        payload_info: opts
            .payload_enum
            .map(|e| PayloadInfo {
                payload_enum_value: e,
                payload_position_index: 0,
                ..Default::default()
            })
            .into_iter()
            .collect(),
        ..Default::default()
    };
    let waylines = Document {
        namespace: Some(
            opts.namespace
                .clone()
                .unwrap_or_else(|| crate::DEFAULT_NAMESPACE.to_string()),
        ),
        mission_config,
        folders: vec![Folder {
            template_id: Some(0),
            wayline_id: Some(0),
            execute_height_mode: Some(mode),
            auto_flight_speed: Some(opts.speed),
            start_action_group,
            placemarks: st.placemarks,
            ..Default::default()
        }],
        ..Default::default()
    };
    let kmz = Kmz::from_waylines(waylines, opts.geoid_separation)?;
    Ok(Converted {
        kmz,
        notes: st.notes,
    })
}

fn actions(fs: Vec<ActionFunc>) -> Vec<Action> {
    fs.into_iter()
        .enumerate()
        .map(|(i, func)| Action {
            id: i as u32,
            func,
            extra_params: Vec::new(),
            extra: Vec::new(),
        })
        .collect()
}

/// Converts a WPL file, skipping its home row.
pub fn from_wpl(wpl: &mavmission::wpl::Wpl, opts: &BridgeOptions) -> Result<Converted, Error> {
    let items = if wpl.has_home() && !wpl.items.is_empty() {
        &wpl.items[1..]
    } else {
        &wpl.items[..]
    };
    from_mission_items(items, opts)
}

/// Converts a QGroundControl `.plan` (complex items included, as their
/// generated mission items).
pub fn from_plan(plan: &mavmission::plan::Plan, opts: &BridgeOptions) -> Result<Converted, Error> {
    let wpl = plan.to_wpl().map_err(|e| Error::Convert {
        seq: 0,
        reason: format!(".plan to mission items: {e}"),
    })?;
    from_wpl(&wpl, opts)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn item(seq: u16, frame: u8, command: u16, p: [f64; 7]) -> MissionItem {
        MissionItem::from_params(seq, false, frame, command, p, true).unwrap()
    }

    fn opts() -> BridgeOptions {
        BridgeOptions::new(crate::drone_by_name("m30t").unwrap().0)
    }

    #[test]
    fn global_frame_needs_geoid_separation() {
        let items = [
            item(1, 0, 16, [0.0, 0.0, 0.0, 0.0, 40.0, -74.0, 120.0]),
            item(2, 0, 16, [0.0, 0.0, 0.0, 0.0, 40.001, -74.0, 120.0]),
        ];
        let e = from_mission_items(&items, &opts()).unwrap_err();
        assert!(matches!(e, Error::Height(_)), "{e}");
        let mut o = opts();
        o.geoid_separation = Some(-32.5);
        let c = from_mission_items(&items, &o).unwrap();
        let wl = c.kmz.waylines.unwrap();
        let f = &wl.folders[0];
        assert_eq!(f.execute_height_mode, Some(ExecuteHeightMode::Wgs84));
        // 120 m above sea level is 87.5 m above the ellipsoid where N = -32.5 m.
        assert_eq!(f.placemarks[0].execute_height, Some(87.5));
        let t = c.kmz.template.unwrap();
        let tf = &t.folders[0];
        assert_eq!(
            tf.coordinate_sys.as_ref().unwrap().height_mode,
            Some(HeightMode::Egm96)
        );
        assert_eq!(tf.placemarks[0].height, Some(120.0));
        assert_eq!(tf.placemarks[0].ellipsoid_height, Some(87.5));
    }

    #[test]
    fn mixed_and_terrain_frames_are_refused() {
        let mixed = [
            item(1, 3, 16, [0.0, 0.0, 0.0, 0.0, 40.0, -74.0, 50.0]),
            item(2, 0, 16, [0.0, 0.0, 0.0, 0.0, 40.001, -74.0, 50.0]),
        ];
        let mut o = opts();
        o.geoid_separation = Some(-30.0);
        assert!(matches!(
            from_mission_items(&mixed, &o),
            Err(Error::Height(_))
        ));
        let terrain = [
            item(1, 10, 16, [0.0, 0.0, 0.0, 0.0, 40.0, -74.0, 50.0]),
            item(2, 10, 16, [0.0, 0.0, 0.0, 0.0, 40.001, -74.0, 50.0]),
        ];
        assert!(matches!(
            from_mission_items(&terrain, &o),
            Err(Error::Height(_))
        ));
    }

    #[test]
    fn camera_and_gimbal_become_actions() {
        let items = [
            item(1, 3, 22, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 30.0]),
            item(2, 3, 16, [0.0, 0.0, 0.0, 0.0, 40.0, -74.0, 50.0]),
            item(3, 2, 205, [-90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0]),
            item(4, 2, 206, [25.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0]),
            item(5, 3, 16, [0.0, 0.0, 0.0, 0.0, 40.001, -74.0, 50.0]),
            item(6, 3, 16, [0.0, 0.0, 0.0, 0.0, 40.002, -74.0, 50.0]),
            item(7, 2, 206, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            item(8, 3, 16, [0.0, 0.0, 0.0, 0.0, 40.003, -74.0, 50.0]),
            item(9, 2, 20, [0.0; 7]),
        ];
        let c = from_mission_items(&items, &opts()).unwrap();
        let wl = c.kmz.waylines.unwrap();
        assert_eq!(wl.mission_config.take_off_security_height, Some(30.0));
        assert_eq!(wl.mission_config.finish_action, Some(FinishAction::GoHome));
        let f = &wl.folders[0];
        assert_eq!(f.placemarks.len(), 4);
        let g = &f.placemarks[0].action_groups;
        assert_eq!(g.len(), 2);
        assert!(matches!(
            g[0].actions[0].func,
            ActionFunc::GimbalRotate(GimbalRotate { pitch: Some(p), .. }) if p == -90.0
        ));
        assert_eq!(g[1].trigger, Some(TriggerType::MultipleDistance));
        assert_eq!(g[1].trigger_param, Some(25.0));
        assert_eq!((g[1].start_index, g[1].end_index), (0, 2));
    }

    #[test]
    fn gimbal_yaw_kept_only_in_earth_frame() {
        let wp = |seq, lat| item(seq, 3, 16, [0.0, 0.0, 0.0, 0.0, lat, -74.0, 50.0]);
        let pitchyaw =
            |seq, flags: f64| item(seq, 2, 1000, [-45.0, 200.0, 0.0, 0.0, flags, 0.0, 0.0]);
        let yaw_of = |flags: f64| {
            let items = [wp(1, 40.0), pitchyaw(2, flags), wp(3, 40.001)];
            let c = from_mission_items(&items, &opts()).unwrap();
            let wl = c.kmz.waylines.unwrap();
            match &wl.folders[0].placemarks[0].action_groups[0].actions[0].func {
                ActionFunc::GimbalRotate(g) => (g.pitch, g.yaw, c.notes.len()),
                other => panic!("{other:?}"),
            }
        };
        // YAW_LOCK (16) and YAW_IN_EARTH_FRAME (64): 200 degrees from north is -160.
        assert_eq!(yaw_of(16.0), (Some(-45.0), Some(-160.0), 0));
        assert_eq!(yaw_of(64.0), (Some(-45.0), Some(-160.0), 0));
        // No flag, or YAW_IN_VEHICLE_FRAME (32): body-relative yaw is dropped with a note.
        assert_eq!(yaw_of(0.0), (Some(-45.0), None, 1));
        assert_eq!(yaw_of(32.0), (Some(-45.0), None, 1));
    }

    #[test]
    fn do_jump_is_an_error() {
        let items = [
            item(1, 3, 16, [0.0, 0.0, 0.0, 0.0, 40.0, -74.0, 50.0]),
            item(2, 3, 16, [0.0, 0.0, 0.0, 0.0, 40.001, -74.0, 50.0]),
            item(3, 2, 177, [1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        ];
        assert!(matches!(
            from_mission_items(&items, &opts()),
            Err(Error::Convert { seq: 3, .. })
        ));
    }
}
