//! Element tree to typed model.

use crate::model::*;
use crate::xml::{self, Element};
use crate::Error;

fn bad(element: &str, value: &str) -> Error {
    Error::Value {
        element: element.to_string(),
        value: value.to_string(),
    }
}

fn f(e: &Element) -> Result<f64, Error> {
    let t = e.text.trim();
    t.parse::<f64>()
        .ok()
        .filter(|v| v.is_finite())
        .ok_or_else(|| bad(&e.name, t))
}

fn u(e: &Element) -> Result<u32, Error> {
    let t = e.text.trim();
    // Some writers emit integers as "0.0" or "1.000000".
    if let Ok(v) = t.parse::<u32>() {
        return Ok(v);
    }
    match t.parse::<f64>() {
        Ok(v) if v >= 0.0 && v <= u32::MAX as f64 && v.fract() == 0.0 => Ok(v as u32),
        _ => Err(bad(&e.name, t)),
    }
}

fn u64v(e: &Element) -> Result<u64, Error> {
    let t = e.text.trim();
    t.parse::<u64>().map_err(|_| bad(&e.name, t))
}

fn b(e: &Element) -> Result<bool, Error> {
    match e.text.trim() {
        "1" | "true" => Ok(true),
        "0" | "false" => Ok(false),
        t => Err(bad(&e.name, t)),
    }
}

fn s(e: &Element) -> String {
    e.text.trim().to_string()
}

/// Parses `template.kml` or `waylines.wpml` text.
pub fn parse_document(text: &str) -> Result<Document, Error> {
    let p = xml::parse(text)?;
    let root = p.root;
    if root.name != "kml" {
        return Err(Error::Structure(format!(
            "root element is <{}>, expected <kml>",
            root.name
        )));
    }
    let doc = root
        .children
        .iter()
        .find(|c| c.name == "Document")
        .ok_or_else(|| Error::Structure("no <Document> in <kml>".into()))?;
    let mut out = Document {
        namespace: p.wpml_ns,
        other_namespaces: p.other_ns,
        ..Default::default()
    };
    for c in &doc.children {
        match c.name.as_str() {
            "wpml:author" => out.author = Some(s(c)),
            "wpml:createTime" => out.create_time = Some(u64v(c)?),
            "wpml:updateTime" => out.update_time = Some(u64v(c)?),
            "wpml:missionConfig" => out.mission_config = mission_config(c)?,
            "Folder" => out.folders.push(folder(c)?),
            _ => out.extra.push(c.clone()),
        }
    }
    Ok(out)
}

fn mission_config(e: &Element) -> Result<MissionConfig, Error> {
    let mut m = MissionConfig::default();
    for c in &e.children {
        match c.name.as_str() {
            "wpml:flyToWaylineMode" => m.fly_to_wayline_mode = Some(FlyToWaylineMode::parse(&s(c))),
            "wpml:finishAction" => m.finish_action = Some(FinishAction::parse(&s(c))),
            "wpml:exitOnRCLost" => m.exit_on_rc_lost = Some(ExitOnRcLost::parse(&s(c))),
            "wpml:executeRCLostAction" => {
                m.execute_rc_lost_action = Some(RcLostAction::parse(&s(c)))
            }
            "wpml:takeOffSecurityHeight" => m.take_off_security_height = Some(f(c)?),
            "wpml:takeOffRefPoint" => m.take_off_ref_point = Some(s(c)),
            "wpml:takeOffRefPointAGLHeight" => m.take_off_ref_point_agl_height = Some(f(c)?),
            "wpml:globalTransitionalSpeed" => m.global_transitional_speed = Some(f(c)?),
            "wpml:globalRTHHeight" => m.global_rth_height = Some(f(c)?),
            "wpml:droneInfo" => {
                let mut d = DroneInfo::default();
                for g in &c.children {
                    match g.name.as_str() {
                        "wpml:droneEnumValue" => d.drone_enum_value = u(g)?,
                        "wpml:droneSubEnumValue" => d.drone_sub_enum_value = Some(u(g)?),
                        _ => {}
                    }
                }
                m.drone_info = Some(d);
            }
            "wpml:payloadInfo" => {
                let mut p = PayloadInfo::default();
                for g in &c.children {
                    match g.name.as_str() {
                        "wpml:payloadEnumValue" => p.payload_enum_value = u(g)?,
                        "wpml:payloadSubEnumValue" => p.payload_sub_enum_value = Some(u(g)?),
                        "wpml:payloadPositionIndex" => p.payload_position_index = u(g)?,
                        _ => p.extra.push(g.clone()),
                    }
                }
                m.payload_info.push(p);
            }
            _ => m.extra.push(c.clone()),
        }
    }
    Ok(m)
}

fn heading(e: &Element) -> Result<HeadingParam, Error> {
    let mut h = HeadingParam::default();
    for c in &e.children {
        match c.name.as_str() {
            "wpml:waypointHeadingMode" => h.mode = Some(HeadingMode::parse(&s(c))),
            "wpml:waypointHeadingAngle" => h.angle = Some(f(c)?),
            "wpml:waypointPoiPoint" => h.poi_point = Some(s(c)),
            "wpml:waypointHeadingAngleEnable" => h.angle_enable = Some(b(c)?),
            "wpml:waypointHeadingPathMode" => h.path_mode = Some(s(c)),
            _ => h.extra.push(c.clone()),
        }
    }
    Ok(h)
}

fn turn(e: &Element) -> Result<TurnParam, Error> {
    let mut t = TurnParam::default();
    for c in &e.children {
        match c.name.as_str() {
            "wpml:waypointTurnMode" => t.mode = Some(TurnMode::parse(&s(c))),
            "wpml:waypointTurnDampingDist" => t.damping_dist = Some(f(c)?),
            _ => t.extra.push(c.clone()),
        }
    }
    Ok(t)
}

fn coordinate_sys(e: &Element) -> Result<CoordinateSysParam, Error> {
    let mut p = CoordinateSysParam::default();
    for c in &e.children {
        match c.name.as_str() {
            "wpml:coordinateMode" => p.coordinate_mode = Some(s(c)),
            "wpml:heightMode" => p.height_mode = Some(HeightMode::parse(&s(c))),
            "wpml:positioningType" => p.positioning_type = Some(s(c)),
            _ => p.extra.push(c.clone()),
        }
    }
    Ok(p)
}

fn folder(e: &Element) -> Result<Folder, Error> {
    let mut fo = Folder::default();
    for c in &e.children {
        match c.name.as_str() {
            "wpml:templateType" => fo.template_type = Some(TemplateType::parse(&s(c))),
            "wpml:templateId" => fo.template_id = Some(u(c)?),
            "wpml:waylineId" => fo.wayline_id = Some(u(c)?),
            "wpml:executeHeightMode" => {
                fo.execute_height_mode = Some(ExecuteHeightMode::parse(&s(c)))
            }
            "wpml:waylineCoordinateSysParam" => fo.coordinate_sys = Some(coordinate_sys(c)?),
            "wpml:distance" => fo.distance = Some(f(c)?),
            "wpml:duration" => fo.duration = Some(f(c)?),
            "wpml:autoFlightSpeed" => fo.auto_flight_speed = Some(f(c)?),
            "wpml:globalHeight" => fo.global_height = Some(f(c)?),
            "wpml:caliFlightEnable" => fo.cali_flight_enable = Some(b(c)?),
            "wpml:gimbalPitchMode" => fo.gimbal_pitch_mode = Some(s(c)),
            "wpml:globalWaypointHeadingParam" => fo.global_heading = Some(heading(c)?),
            "wpml:globalWaypointTurnMode" => fo.global_turn_mode = Some(TurnMode::parse(&s(c))),
            "wpml:globalUseStraightLine" => fo.global_use_straight_line = Some(b(c)?),
            "wpml:startActionGroup" => fo.start_action_group = Some(action_group(c)?),
            "Placemark" => fo.placemarks.push(placemark(c)?),
            _ => fo.extra.push(c.clone()),
        }
    }
    Ok(fo)
}

/// Parses `lon,lat[,alt]` with any whitespace around the numbers.
pub(crate) fn coordinates(text: &str) -> Option<(f64, f64, Option<f64>)> {
    let mut it = text.split(',').map(|p| p.trim());
    let lon: f64 = it.next()?.parse().ok()?;
    let lat: f64 = it.next()?.parse().ok()?;
    let alt = match it.next() {
        Some(a) if !a.is_empty() => Some(a.parse().ok()?),
        _ => None,
    };
    if it.next().is_some() || !(-180.0..=180.0).contains(&lon) || !(-90.0..=90.0).contains(&lat) {
        return None;
    }
    Some((lon, lat, alt))
}

fn placemark(e: &Element) -> Result<Placemark, Error> {
    let mut p = Placemark::default();
    let mut have_point = false;
    let mut have_index = false;
    for c in &e.children {
        match c.name.as_str() {
            "Point" => {
                let co = c
                    .child("coordinates")
                    .ok_or_else(|| Error::Structure("Point without coordinates".into()))?;
                let (lon, lat, alt) =
                    coordinates(&co.text).ok_or_else(|| bad("coordinates", co.text.trim()))?;
                p.lon = lon;
                p.lat = lat;
                p.coord_alt = alt;
                have_point = true;
            }
            "wpml:index" => {
                p.index = u(c)?;
                have_index = true;
            }
            "wpml:executeHeight" => p.execute_height = Some(f(c)?),
            "wpml:height" => p.height = Some(f(c)?),
            "wpml:ellipsoidHeight" => p.ellipsoid_height = Some(f(c)?),
            "wpml:useGlobalHeight" => p.use_global_height = Some(b(c)?),
            "wpml:useGlobalSpeed" => p.use_global_speed = Some(b(c)?),
            "wpml:waypointSpeed" => p.waypoint_speed = Some(f(c)?),
            "wpml:useGlobalHeadingParam" => p.use_global_heading_param = Some(b(c)?),
            "wpml:waypointHeadingParam" => p.heading = Some(heading(c)?),
            "wpml:useGlobalTurnParam" => p.use_global_turn_param = Some(b(c)?),
            "wpml:waypointTurnParam" => p.turn = Some(turn(c)?),
            "wpml:useStraightLine" => p.use_straight_line = Some(b(c)?),
            "wpml:gimbalPitchAngle" => p.gimbal_pitch_angle = Some(f(c)?),
            "wpml:actionGroup" => p.action_groups.push(action_group(c)?),
            _ => p.extra.push(c.clone()),
        }
    }
    if !have_point {
        return Err(Error::Structure("Placemark without Point".into()));
    }
    if !have_index {
        return Err(Error::Structure(format!(
            "Placemark at {},{} without wpml:index",
            p.lon, p.lat
        )));
    }
    Ok(p)
}

fn action_group(e: &Element) -> Result<ActionGroup, Error> {
    let mut g = ActionGroup::default();
    for c in &e.children {
        match c.name.as_str() {
            "wpml:actionGroupId" => g.id = u(c)?,
            "wpml:actionGroupStartIndex" => g.start_index = u(c)?,
            "wpml:actionGroupEndIndex" => g.end_index = u(c)?,
            "wpml:actionGroupMode" => g.mode = s(c),
            "wpml:actionTrigger" => {
                for t in &c.children {
                    match t.name.as_str() {
                        "wpml:actionTriggerType" => g.trigger = Some(TriggerType::parse(&s(t))),
                        "wpml:actionTriggerParam" => g.trigger_param = Some(f(t)?),
                        _ => g.extra.push(t.clone()),
                    }
                }
            }
            "wpml:action" => g.actions.push(action(c)?),
            _ => g.extra.push(c.clone()),
        }
    }
    Ok(g)
}

/// Pulls typed params out of `actionActuatorFuncParam`, leaving the rest in `rest`.
struct Params {
    rest: Vec<Element>,
}

impl Params {
    fn take(&mut self, name: &str) -> Option<Element> {
        let i = self.rest.iter().position(|e| e.name == name)?;
        Some(self.rest.remove(i))
    }
    fn f(&mut self, name: &str) -> Result<Option<f64>, Error> {
        self.take(name).map(|e| f(&e)).transpose()
    }
    fn u(&mut self, name: &str) -> Result<Option<u32>, Error> {
        self.take(name).map(|e| u(&e)).transpose()
    }
    fn b(&mut self, name: &str) -> Result<Option<bool>, Error> {
        self.take(name).map(|e| b(&e)).transpose()
    }
    fn s(&mut self, name: &str) -> Option<String> {
        self.take(name).map(|e| s(&e))
    }
    fn req_f(&mut self, func: &str, name: &str) -> Result<f64, Error> {
        self.f(name)?
            .ok_or_else(|| Error::Structure(format!("{func} action without {name}")))
    }
    /// An `xxxEnable` flag and its value: the value counts only when enabled.
    /// A value with no flag counts as enabled.
    fn gated(&mut self, enable: &str, value: &str) -> Result<Option<f64>, Error> {
        let on = self.b(enable)?;
        let v = self.f(value)?;
        Ok(match on {
            Some(false) => None,
            _ => v,
        })
    }
}

fn action(e: &Element) -> Result<Action, Error> {
    let mut id = 0;
    let mut func_name = String::new();
    let mut params = Vec::new();
    let mut extra = Vec::new();
    for c in &e.children {
        match c.name.as_str() {
            "wpml:actionId" => id = u(c)?,
            "wpml:actionActuatorFunc" => func_name = s(c),
            "wpml:actionActuatorFuncParam" => params = c.children.clone(),
            _ => extra.push(c.clone()),
        }
    }
    let raw = params.clone();
    let mut p = Params { rest: params };
    let ppi = "wpml:payloadPositionIndex";
    let func = match func_name.as_str() {
        "takePhoto" => ActionFunc::TakePhoto {
            payload_position_index: p.u(ppi)?.unwrap_or(0),
            file_suffix: p.s("wpml:fileSuffix"),
            payload_lens_index: p.s("wpml:payloadLensIndex"),
        },
        "startRecord" => ActionFunc::StartRecord {
            payload_position_index: p.u(ppi)?.unwrap_or(0),
            file_suffix: p.s("wpml:fileSuffix"),
            payload_lens_index: p.s("wpml:payloadLensIndex"),
        },
        "stopRecord" => ActionFunc::StopRecord {
            payload_position_index: p.u(ppi)?.unwrap_or(0),
            payload_lens_index: p.s("wpml:payloadLensIndex"),
        },
        "gimbalRotate" => ActionFunc::GimbalRotate(GimbalRotate {
            payload_position_index: p.u(ppi)?.unwrap_or(0),
            heading_yaw_base: p.s("wpml:gimbalHeadingYawBase"),
            rotate_mode: p.s("wpml:gimbalRotateMode"),
            pitch: p.gated(
                "wpml:gimbalPitchRotateEnable",
                "wpml:gimbalPitchRotateAngle",
            )?,
            roll: p.gated("wpml:gimbalRollRotateEnable", "wpml:gimbalRollRotateAngle")?,
            yaw: p.gated("wpml:gimbalYawRotateEnable", "wpml:gimbalYawRotateAngle")?,
            time: p.gated("wpml:gimbalRotateTimeEnable", "wpml:gimbalRotateTime")?,
        }),
        "rotateYaw" => ActionFunc::RotateYaw {
            aircraft_heading: p.req_f("rotateYaw", "wpml:aircraftHeading")?,
            path_mode: p.s("wpml:aircraftPathMode"),
        },
        "hover" => ActionFunc::Hover {
            hover_time: p.req_f("hover", "wpml:hoverTime")?,
        },
        "zoom" => ActionFunc::Zoom {
            payload_position_index: p.u(ppi)?.unwrap_or(0),
            focal_length: p.req_f("zoom", "wpml:focalLength")?,
        },
        "" => return Err(Error::Structure("action without actionActuatorFunc".into())),
        other => {
            return Ok(Action {
                id,
                func: ActionFunc::Other {
                    func: other.to_string(),
                    params: raw,
                },
                extra_params: Vec::new(),
                extra,
            })
        }
    };
    Ok(Action {
        id,
        func,
        extra_params: p.rest,
        extra,
    })
}
