//! Typed model to XML text.
//!
//! Elements are written in the order DJI's own files use (DJI Cloud API
//! samples and the MSDK v5 sample KMZ files), followed by any elements kept
//! in `extra`.

use crate::model::*;
use crate::xml::{indent, Element, KML_NS};
use crate::DEFAULT_NAMESPACE;
use core::fmt::Write;

struct W {
    out: String,
    depth: usize,
}

impl W {
    fn open(&mut self, name: &str) {
        indent(&mut self.out, self.depth);
        let _ = writeln!(self.out, "<{name}>");
        self.depth += 1;
    }
    fn close(&mut self, name: &str) {
        self.depth -= 1;
        indent(&mut self.out, self.depth);
        let _ = writeln!(self.out, "</{name}>");
    }
    fn leaf(&mut self, name: &str, text: &str) {
        indent(&mut self.out, self.depth);
        let _ = writeln!(
            self.out,
            "<{name}>{}</{name}>",
            quick_xml::escape::escape(text)
        );
    }
    fn s(&mut self, name: &str, v: &Option<String>) {
        if let Some(v) = v {
            self.leaf(name, v);
        }
    }
    fn f(&mut self, name: &str, v: Option<f64>) {
        if let Some(v) = v {
            self.leaf(name, &num(v));
        }
    }
    fn u(&mut self, name: &str, v: Option<u32>) {
        if let Some(v) = v {
            self.leaf(name, &v.to_string());
        }
    }
    fn b(&mut self, name: &str, v: Option<bool>) {
        if let Some(v) = v {
            self.leaf(name, if v { "1" } else { "0" });
        }
    }
    fn extra(&mut self, extra: &[Element]) {
        for e in extra {
            e.write_to(&mut self.out, self.depth);
        }
    }
}

/// Shortest decimal that reads back as the same f64, never exponent form.
pub(crate) fn num(v: f64) -> String {
    if v == 0.0 {
        return "0".into();
    }
    let s = format!("{v}");
    if s.contains('e') || s.contains('E') {
        format!("{v:.10}")
            .trim_end_matches('0')
            .trim_end_matches('.')
            .to_string()
    } else {
        s
    }
}

/// Writes a document as `template.kml` or `waylines.wpml` text.
///
/// The `kind` decides only the order and which file-specific elements are
/// expected; fields that are `None` are not written.
pub fn write_document(doc: &Document, kind: FileKind) -> String {
    let ns = doc.namespace.as_deref().unwrap_or(DEFAULT_NAMESPACE);
    let mut w = W {
        out: String::with_capacity(4096),
        depth: 0,
    };
    w.out
        .push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    let _ = write!(w.out, "<kml xmlns=\"{KML_NS}\" xmlns:wpml=\"{ns}\"");
    for (p, uri) in &doc.other_namespaces {
        let _ = write!(
            w.out,
            " xmlns:{p}=\"{}\"",
            quick_xml::escape::escape(uri.as_str())
        );
    }
    w.out.push_str(">\n");
    w.depth = 1;
    w.open("Document");
    w.s("wpml:author", &doc.author);
    if let Some(t) = doc.create_time {
        w.leaf("wpml:createTime", &t.to_string());
    }
    if let Some(t) = doc.update_time {
        w.leaf("wpml:updateTime", &t.to_string());
    }
    mission_config(&mut w, &doc.mission_config);
    for fo in &doc.folders {
        folder(&mut w, fo, kind);
    }
    w.extra(&doc.extra);
    w.close("Document");
    w.out.push_str("</kml>\n");
    w.out
}

fn mission_config(w: &mut W, m: &MissionConfig) {
    w.open("wpml:missionConfig");
    if let Some(v) = &m.fly_to_wayline_mode {
        w.leaf("wpml:flyToWaylineMode", v.as_str());
    }
    if let Some(v) = &m.finish_action {
        w.leaf("wpml:finishAction", v.as_str());
    }
    if let Some(v) = &m.exit_on_rc_lost {
        w.leaf("wpml:exitOnRCLost", v.as_str());
    }
    if let Some(v) = &m.execute_rc_lost_action {
        w.leaf("wpml:executeRCLostAction", v.as_str());
    }
    w.f("wpml:takeOffSecurityHeight", m.take_off_security_height);
    w.s("wpml:takeOffRefPoint", &m.take_off_ref_point);
    w.f(
        "wpml:takeOffRefPointAGLHeight",
        m.take_off_ref_point_agl_height,
    );
    w.f("wpml:globalTransitionalSpeed", m.global_transitional_speed);
    w.f("wpml:globalRTHHeight", m.global_rth_height);
    if let Some(d) = &m.drone_info {
        w.open("wpml:droneInfo");
        w.u("wpml:droneEnumValue", Some(d.drone_enum_value));
        w.u("wpml:droneSubEnumValue", d.drone_sub_enum_value);
        w.close("wpml:droneInfo");
    }
    for p in &m.payload_info {
        w.open("wpml:payloadInfo");
        w.u("wpml:payloadEnumValue", Some(p.payload_enum_value));
        w.u("wpml:payloadSubEnumValue", p.payload_sub_enum_value);
        w.u("wpml:payloadPositionIndex", Some(p.payload_position_index));
        w.extra(&p.extra);
        w.close("wpml:payloadInfo");
    }
    w.extra(&m.extra);
    w.close("wpml:missionConfig");
}

fn heading(w: &mut W, name: &str, h: &HeadingParam) {
    w.open(name);
    if let Some(m) = &h.mode {
        w.leaf("wpml:waypointHeadingMode", m.as_str());
    }
    w.f("wpml:waypointHeadingAngle", h.angle);
    w.s("wpml:waypointPoiPoint", &h.poi_point);
    w.b("wpml:waypointHeadingAngleEnable", h.angle_enable);
    w.s("wpml:waypointHeadingPathMode", &h.path_mode);
    w.extra(&h.extra);
    w.close(name);
}

fn folder(w: &mut W, fo: &Folder, kind: FileKind) {
    w.open("Folder");
    if let Some(t) = &fo.template_type {
        w.leaf("wpml:templateType", t.as_str());
    }
    w.u("wpml:templateId", fo.template_id);
    if kind == FileKind::Waylines {
        if let Some(m) = &fo.execute_height_mode {
            w.leaf("wpml:executeHeightMode", m.as_str());
        }
        w.u("wpml:waylineId", fo.wayline_id);
        w.f("wpml:distance", fo.distance);
        w.f("wpml:duration", fo.duration);
    }
    if let Some(c) = &fo.coordinate_sys {
        w.open("wpml:waylineCoordinateSysParam");
        w.s("wpml:coordinateMode", &c.coordinate_mode);
        if let Some(m) = &c.height_mode {
            w.leaf("wpml:heightMode", m.as_str());
        }
        w.s("wpml:positioningType", &c.positioning_type);
        w.extra(&c.extra);
        w.close("wpml:waylineCoordinateSysParam");
    }
    if kind == FileKind::Template {
        // A template can still carry these if a caller set them.
        if let Some(m) = &fo.execute_height_mode {
            w.leaf("wpml:executeHeightMode", m.as_str());
        }
        w.u("wpml:waylineId", fo.wayline_id);
        w.f("wpml:distance", fo.distance);
        w.f("wpml:duration", fo.duration);
    }
    w.f("wpml:autoFlightSpeed", fo.auto_flight_speed);
    w.f("wpml:globalHeight", fo.global_height);
    w.b("wpml:caliFlightEnable", fo.cali_flight_enable);
    w.s("wpml:gimbalPitchMode", &fo.gimbal_pitch_mode);
    if let Some(h) = &fo.global_heading {
        heading(w, "wpml:globalWaypointHeadingParam", h);
    }
    if let Some(t) = &fo.global_turn_mode {
        w.leaf("wpml:globalWaypointTurnMode", t.as_str());
    }
    w.b("wpml:globalUseStraightLine", fo.global_use_straight_line);
    if let Some(g) = &fo.start_action_group {
        action_group(w, "wpml:startActionGroup", g);
    }
    for p in &fo.placemarks {
        placemark(w, p);
    }
    w.extra(&fo.extra);
    w.close("Folder");
}

fn placemark(w: &mut W, p: &Placemark) {
    w.open("Placemark");
    w.open("Point");
    let coords = match p.coord_alt {
        Some(a) => format!("{},{},{}", num(p.lon), num(p.lat), num(a)),
        None => format!("{},{}", num(p.lon), num(p.lat)),
    };
    w.leaf("coordinates", &coords);
    w.close("Point");
    w.u("wpml:index", Some(p.index));
    w.f("wpml:executeHeight", p.execute_height);
    w.f("wpml:ellipsoidHeight", p.ellipsoid_height);
    w.f("wpml:height", p.height);
    w.b("wpml:useGlobalHeight", p.use_global_height);
    w.b("wpml:useGlobalSpeed", p.use_global_speed);
    w.f("wpml:waypointSpeed", p.waypoint_speed);
    w.b("wpml:useGlobalHeadingParam", p.use_global_heading_param);
    if let Some(h) = &p.heading {
        heading(w, "wpml:waypointHeadingParam", h);
    }
    w.b("wpml:useGlobalTurnParam", p.use_global_turn_param);
    if let Some(t) = &p.turn {
        w.open("wpml:waypointTurnParam");
        if let Some(m) = &t.mode {
            w.leaf("wpml:waypointTurnMode", m.as_str());
        }
        w.f("wpml:waypointTurnDampingDist", t.damping_dist);
        w.extra(&t.extra);
        w.close("wpml:waypointTurnParam");
    }
    w.b("wpml:useStraightLine", p.use_straight_line);
    w.f("wpml:gimbalPitchAngle", p.gimbal_pitch_angle);
    for g in &p.action_groups {
        action_group(w, "wpml:actionGroup", g);
    }
    w.extra(&p.extra);
    w.close("Placemark");
}

fn action_group(w: &mut W, name: &str, g: &ActionGroup) {
    w.open(name);
    w.u("wpml:actionGroupId", Some(g.id));
    w.u("wpml:actionGroupStartIndex", Some(g.start_index));
    w.u("wpml:actionGroupEndIndex", Some(g.end_index));
    w.leaf(
        "wpml:actionGroupMode",
        if g.mode.is_empty() {
            "sequence"
        } else {
            &g.mode
        },
    );
    if g.trigger.is_some() || g.trigger_param.is_some() {
        w.open("wpml:actionTrigger");
        if let Some(t) = &g.trigger {
            w.leaf("wpml:actionTriggerType", t.as_str());
        }
        w.f("wpml:actionTriggerParam", g.trigger_param);
        w.close("wpml:actionTrigger");
    }
    for a in &g.actions {
        action(w, a);
    }
    w.extra(&g.extra);
    w.close(name);
}

fn enable(w: &mut W, flag: &str, value: &str, v: Option<f64>) {
    w.b(flag, Some(v.is_some()));
    w.f(value, Some(v.unwrap_or(0.0)));
}

fn action(w: &mut W, a: &Action) {
    w.open("wpml:action");
    w.u("wpml:actionId", Some(a.id));
    w.leaf("wpml:actionActuatorFunc", a.func.name());
    w.open("wpml:actionActuatorFuncParam");
    let ppi = "wpml:payloadPositionIndex";
    match &a.func {
        ActionFunc::TakePhoto {
            payload_position_index,
            file_suffix,
            payload_lens_index,
        }
        | ActionFunc::StartRecord {
            payload_position_index,
            file_suffix,
            payload_lens_index,
        } => {
            w.s("wpml:fileSuffix", file_suffix);
            w.u(ppi, Some(*payload_position_index));
            w.s("wpml:payloadLensIndex", payload_lens_index);
        }
        ActionFunc::StopRecord {
            payload_position_index,
            payload_lens_index,
        } => {
            w.u(ppi, Some(*payload_position_index));
            w.s("wpml:payloadLensIndex", payload_lens_index);
        }
        ActionFunc::GimbalRotate(g) => {
            w.s("wpml:gimbalHeadingYawBase", &g.heading_yaw_base);
            w.s("wpml:gimbalRotateMode", &g.rotate_mode);
            enable(
                w,
                "wpml:gimbalPitchRotateEnable",
                "wpml:gimbalPitchRotateAngle",
                g.pitch,
            );
            enable(
                w,
                "wpml:gimbalRollRotateEnable",
                "wpml:gimbalRollRotateAngle",
                g.roll,
            );
            enable(
                w,
                "wpml:gimbalYawRotateEnable",
                "wpml:gimbalYawRotateAngle",
                g.yaw,
            );
            enable(
                w,
                "wpml:gimbalRotateTimeEnable",
                "wpml:gimbalRotateTime",
                g.time,
            );
            w.u(ppi, Some(g.payload_position_index));
        }
        ActionFunc::RotateYaw {
            aircraft_heading,
            path_mode,
        } => {
            w.f("wpml:aircraftHeading", Some(*aircraft_heading));
            w.s("wpml:aircraftPathMode", path_mode);
        }
        ActionFunc::Hover { hover_time } => {
            w.f("wpml:hoverTime", Some(*hover_time));
        }
        ActionFunc::Zoom {
            payload_position_index,
            focal_length,
        } => {
            w.u(ppi, Some(*payload_position_index));
            w.f("wpml:focalLength", Some(*focal_length));
        }
        ActionFunc::Other { params, .. } => {
            w.extra(params);
        }
    }
    w.extra(&a.extra_params);
    w.close("wpml:actionActuatorFuncParam");
    w.extra(&a.extra);
    w.close("wpml:action");
}

#[cfg(test)]
mod tests {
    use super::num;

    #[test]
    fn numbers_are_short_and_plain() {
        assert_eq!(num(100.0), "100");
        assert_eq!(num(113.958298915883), "113.958298915883");
        assert_eq!(num(-0.5), "-0.5");
        assert_eq!(num(1e-7), "0.0000001");
        assert_eq!(num(0.0), "0");
    }
}
