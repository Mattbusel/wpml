//! Tests on published WPML files and spec examples. Sources and licenses
//! are in tests/data/SOURCES.md.

use std::io::{Cursor, Write};
use wpml::*;

fn data(name: &str) -> Vec<u8> {
    std::fs::read(format!("{}/tests/data/{name}", env!("CARGO_MANIFEST_DIR"))).unwrap()
}

fn round_trip(kmz: &Kmz) -> Kmz {
    Kmz::from_bytes(&kmz.to_bytes().unwrap()).unwrap()
}

/// MSDK v5 sample code, android-sdk-v5-sample/src/main/assets/waypointsample.kmz.
#[test]
fn msdk_waypoint_sample() {
    let kmz = Kmz::from_bytes(&data("msdk_waypointsample.kmz")).unwrap();
    let wl = kmz.waylines.as_ref().unwrap();
    assert_eq!(
        wl.namespace.as_deref(),
        Some("http://www.dji.com/wpmz/1.0.0")
    );
    let mc = &wl.mission_config;
    assert_eq!(mc.fly_to_wayline_mode, Some(FlyToWaylineMode::Safely));
    assert_eq!(mc.finish_action, Some(FinishAction::GoHome));
    assert_eq!(mc.exit_on_rc_lost, Some(ExitOnRcLost::GoContinue));
    assert_eq!(mc.take_off_security_height, Some(20.0));
    assert_eq!(mc.global_transitional_speed, Some(5.0));
    let di = mc.drone_info.unwrap();
    assert_eq!(
        (di.drone_enum_value, di.drone_sub_enum_value),
        (67, Some(0))
    );
    assert_eq!(drone_name(&di), Some("m30"));
    assert_eq!(mc.payload_info[0].payload_enum_value, 52);
    assert_eq!(mc.payload_info[0].payload_sub_enum_value, Some(0));

    let f = &wl.folders[0];
    assert_eq!((f.template_id, f.wayline_id), (Some(0), Some(0)));
    assert_eq!(
        f.execute_height_mode,
        Some(ExecuteHeightMode::RelativeToStartPoint)
    );
    assert_eq!(f.distance, Some(192.275848388672));
    assert_eq!(f.auto_flight_speed, Some(5.0));
    let p = &f.placemarks[0];
    assert_eq!((p.lon, p.lat), (113.958298915883, 22.5428601764565));
    assert_eq!(p.execute_height, Some(100.0));
    assert_eq!(p.waypoint_speed, Some(5.0));
    assert_eq!(
        p.heading.as_ref().unwrap().mode,
        Some(HeadingMode::FollowWayline)
    );
    assert_eq!(
        p.turn.as_ref().unwrap().mode,
        Some(TurnMode::ToPointAndStopWithDiscontinuityCurvature)
    );
    assert_eq!(p.use_straight_line, Some(true));
    assert_eq!(
        f.placemarks.iter().map(|p| p.index).collect::<Vec<_>>(),
        [0, 1, 2, 3]
    );

    let t = kmz.template.as_ref().unwrap();
    assert_eq!(t.create_time, Some(1646131934500));
    let tf = &t.folders[0];
    assert_eq!(tf.template_type, Some(TemplateType::Waypoint));
    let cs = tf.coordinate_sys.as_ref().unwrap();
    assert_eq!(cs.height_mode, Some(HeightMode::RelativeToStartPoint));
    // In this 1.0.0 file globalHeight sits inside waylineCoordinateSysParam; it is kept.
    assert_eq!(cs.extra[0].name, "wpml:globalHeight");
    assert_eq!(tf.placemarks[3].use_global_height, Some(true));

    assert_eq!(round_trip(&kmz), kmz);
}

/// MSDK v5 sample code, open-pilot-flighttask-sample/server/data/FLIGHT-TASK-DEMO-001.kmz.
/// Its namespace is http://www.corp.com/wpmz/1.0.6, not dji.com.
#[test]
fn msdk_flight_task_demo() {
    let kmz = Kmz::from_bytes(&data("msdk_flight_task_demo.kmz")).unwrap();
    let wl = kmz.waylines.as_ref().unwrap();
    assert_eq!(
        wl.namespace.as_deref(),
        Some("http://www.corp.com/wpmz/1.0.6")
    );
    assert_eq!(
        wl.mission_config.exit_on_rc_lost,
        Some(ExitOnRcLost::ExecuteLostAction)
    );
    assert_eq!(
        wl.mission_config.execute_rc_lost_action,
        Some(RcLostAction::GoBack)
    );
    assert_eq!(
        wl.mission_config.extra[0].name,
        "wpml:waylineAvoidLimitAreaMode"
    );
    let p0 = &wl.folders[0].placemarks[0];
    let g = &p0.action_groups[0];
    assert_eq!(g.trigger, Some(TriggerType::ReachPoint));
    let a = &g.actions[0];
    match &a.func {
        ActionFunc::TakePhoto {
            file_suffix,
            payload_lens_index,
            payload_position_index,
        } => {
            assert_eq!(file_suffix.as_deref(), Some("djitest"));
            assert_eq!(payload_lens_index.as_deref(), Some("wide"));
            assert_eq!(*payload_position_index, 0);
        }
        other => panic!("{other:?}"),
    }
    assert_eq!(a.extra_params[0].name, "wpml:useGlobalPayloadLensIndex");
    let extra: Vec<&str> = p0.extra.iter().map(|e| e.name.as_str()).collect();
    assert_eq!(
        extra,
        [
            "wpml:waypointGimbalHeadingParam",
            "wpml:isRisky",
            "wpml:waypointWorkType"
        ]
    );
    assert_eq!(
        p0.heading.as_ref().unwrap().path_mode.as_deref(),
        Some("followBadArc")
    );
    let back = round_trip(&kmz);
    assert_eq!(back, kmz);
    assert_eq!(
        back.waylines.unwrap().namespace.as_deref(),
        Some("http://www.corp.com/wpmz/1.0.6")
    );
}

/// The waylines.wpml sample from the DJI Cloud API WPML reference
/// (api-reference/dji-wpml/waylines-wpml), with its "longitude,latitude"
/// placeholders filled in.
const DJI_DOC_WAYLINES: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="http://www.dji.com/wpmz/1.0.2">
  <Document>
    <!-- Step 1: Setup Mission Configuration -->
    <wpml:missionConfig>
      <wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode>
      <wpml:finishAction>goHome</wpml:finishAction>
      <wpml:exitOnRCLost>goContinue</wpml:exitOnRCLost>
      <wpml:executeRCLostAction>hover</wpml:executeRCLostAction>
      <wpml:takeOffSecurityHeight>20</wpml:takeOffSecurityHeight>
      <wpml:globalTransitionalSpeed>10</wpml:globalTransitionalSpeed>
      <wpml:droneInfo>
        <wpml:droneEnumValue>67</wpml:droneEnumValue>
        <wpml:droneSubEnumValue>0</wpml:droneSubEnumValue>
      </wpml:droneInfo>
      <wpml:payloadInfo>
        <wpml:payloadEnumValue>52</wpml:payloadEnumValue>
        <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
      </wpml:payloadInfo>
    </wpml:missionConfig>
    <Folder>
      <wpml:templateId>0</wpml:templateId>
      <wpml:executeHeightMode>WGS84</wpml:executeHeightMode>
      <wpml:waylineId>0</wpml:waylineId>
      <wpml:autoFlightSpeed>10</wpml:autoFlightSpeed>
      <Placemark>
        <Point>
          <coordinates>
            113.943,22.577
          </coordinates>
        </Point>
        <wpml:index>0</wpml:index>
        <wpml:executeHeight>116.57</wpml:executeHeight>
        <wpml:waypointSpeed>10</wpml:waypointSpeed>
        <wpml:waypointHeadingParam>
          <wpml:waypointHeadingMode>followWayline</wpml:waypointHeadingMode>
        </wpml:waypointHeadingParam>
        <wpml:waypointTurnParam>
          <wpml:waypointTurnMode>toPointAndStopWithDiscontinuityCurvature</wpml:waypointTurnMode>
          <wpml:waypointTurnDampingDist>0</wpml:waypointTurnDampingDist>
        </wpml:waypointTurnParam>
      </Placemark>
      <Placemark>
        <Point>
          <coordinates>
            113.944,22.578
          </coordinates>
        </Point>
        <wpml:index>1</wpml:index>
        <wpml:executeHeight>116.57</wpml:executeHeight>
        <wpml:waypointSpeed>7</wpml:waypointSpeed>
        <wpml:waypointHeadingParam>
          <wpml:waypointHeadingMode>followWayline</wpml:waypointHeadingMode>
        </wpml:waypointHeadingParam>
        <wpml:waypointTurnParam>
          <wpml:waypointTurnMode>toPointAndStopWithDiscontinuityCurvature</wpml:waypointTurnMode>
          <wpml:waypointTurnDampingDist>0</wpml:waypointTurnDampingDist>
        </wpml:waypointTurnParam>
        <!-- Declare action group for waypoint 1# -->
        <wpml:actionGroup>
          <wpml:actionGroupId>0</wpml:actionGroupId>
          <wpml:actionGroupStartIndex>1</wpml:actionGroupStartIndex>
          <wpml:actionGroupEndIndex>1</wpml:actionGroupEndIndex>
          <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
          <wpml:actionTrigger>
            <wpml:actionTriggerType>reachPoint</wpml:actionTriggerType>
          </wpml:actionTrigger>
          <wpml:action>
            <wpml:actionId>0</wpml:actionId>
            <wpml:actionActuatorFunc>gimbalRotate</wpml:actionActuatorFunc>
            <wpml:actionActuatorFuncParam>
              <wpml:gimbalRotateMode>absoluteAngle</wpml:gimbalRotateMode>
              <wpml:gimbalPitchRotateEnable>0</wpml:gimbalPitchRotateEnable>
              <wpml:gimbalPitchRotateAngle>0</wpml:gimbalPitchRotateAngle>
              <wpml:gimbalRollRotateEnable>0</wpml:gimbalRollRotateEnable>
              <wpml:gimbalRollRotateAngle>0</wpml:gimbalRollRotateAngle>
              <wpml:gimbalYawRotateEnable>1</wpml:gimbalYawRotateEnable>
              <wpml:gimbalYawRotateAngle>30</wpml:gimbalYawRotateAngle>
              <wpml:gimbalRotateTimeEnable>0</wpml:gimbalRotateTimeEnable>
              <wpml:gimbalRotateTime>0</wpml:gimbalRotateTime>
              <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
            </wpml:actionActuatorFuncParam>
          </wpml:action>
          <wpml:action>
            <wpml:actionId>1</wpml:actionId>
            <wpml:actionActuatorFunc>takePhoto</wpml:actionActuatorFunc>
            <wpml:actionActuatorFuncParam>
              <wpml:fileSuffix>point1</wpml:fileSuffix>
              <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
            </wpml:actionActuatorFuncParam>
          </wpml:action>
        </wpml:actionGroup>
      </Placemark>
    </Folder>
  </Document>
</kml>
"#;

#[test]
fn dji_doc_waylines_example() {
    let d = parse_document(DJI_DOC_WAYLINES).unwrap();
    assert_eq!(
        d.mission_config.execute_rc_lost_action,
        Some(RcLostAction::Hover)
    );
    let f = &d.folders[0];
    assert_eq!(f.execute_height_mode, Some(ExecuteHeightMode::Wgs84));
    let p1 = &f.placemarks[1];
    assert_eq!(p1.waypoint_speed, Some(7.0));
    let g = &p1.action_groups[0];
    assert_eq!(
        (g.start_index, g.end_index, g.mode.as_str()),
        (1, 1, "sequence")
    );
    match &g.actions[0].func {
        ActionFunc::GimbalRotate(r) => {
            assert_eq!(r.rotate_mode.as_deref(), Some("absoluteAngle"));
            assert_eq!(
                (r.pitch, r.roll, r.yaw, r.time),
                (None, None, Some(30.0), None)
            );
        }
        other => panic!("{other:?}"),
    }
    assert!(matches!(
        &g.actions[1].func,
        ActionFunc::TakePhoto { file_suffix: Some(s), .. } if s == "point1"
    ));
    let again = parse_document(&write_document(&d, FileKind::Waylines)).unwrap();
    assert_eq!(again, d);

    // WGS84 is ellipsoid height. The template's absolute mode is EGM96 (sea
    // level), so making a template needs the geoid separation.
    assert!(matches!(
        template_from_waylines(&d, None),
        Err(Error::Height(_))
    ));
    let t = template_from_waylines(&d, Some(-3.5)).unwrap();
    let tp = &t.folders[0].placemarks[0];
    assert_eq!(tp.ellipsoid_height, Some(116.57));
    assert_eq!(tp.height, Some(116.57 + 3.5));
    assert_eq!(
        t.folders[0].coordinate_sys.as_ref().unwrap().height_mode,
        Some(HeightMode::Egm96)
    );
}

/// An action this crate does not model (orientedShoot, from the DJI common
/// elements table) is kept as raw elements and written back unchanged.
#[test]
fn unknown_action_kept_raw() {
    let x = DJI_DOC_WAYLINES.replace(
        "<wpml:actionActuatorFunc>takePhoto</wpml:actionActuatorFunc>",
        "<wpml:actionActuatorFunc>orientedShoot</wpml:actionActuatorFunc>",
    );
    let d = parse_document(&x).unwrap();
    let a = &d.folders[0].placemarks[1].action_groups[0].actions[1];
    match &a.func {
        ActionFunc::Other { func, params } => {
            assert_eq!(func, "orientedShoot");
            assert_eq!(params[0].name, "wpml:fileSuffix");
            assert_eq!(params[0].text, "point1");
        }
        other => panic!("{other:?}"),
    }
    let out = write_document(&d, FileKind::Waylines);
    assert!(out.contains("<wpml:actionActuatorFunc>orientedShoot</wpml:actionActuatorFunc>"));
    assert_eq!(parse_document(&out).unwrap(), d);
}

fn zip_of(files: &[(&str, &[u8])]) -> Vec<u8> {
    let mut c = Cursor::new(Vec::new());
    {
        let mut z = zip::ZipWriter::new(&mut c);
        for (n, b) in files {
            z.start_file(*n, zip::write::SimpleFileOptions::default())
                .unwrap();
            z.write_all(b).unwrap();
        }
        z.finish().unwrap();
    }
    c.into_inner()
}

#[test]
fn missing_template_and_resources() {
    let bytes = zip_of(&[
        ("wpmz/waylines.wpml", DJI_DOC_WAYLINES.as_bytes()),
        ("wpmz/res/ref.jpg", b"jpegbytes"),
    ]);
    let kmz = Kmz::from_bytes(&bytes).unwrap();
    assert!(kmz.template.is_none());
    assert_eq!(
        kmz.resources,
        vec![("ref.jpg".to_string(), b"jpegbytes".to_vec())]
    );
    assert_eq!(round_trip(&kmz), kmz);

    let empty = zip_of(&[("readme.txt", b"x")]);
    assert!(matches!(Kmz::from_bytes(&empty), Err(Error::Missing(_))));
}

#[test]
fn bad_value_names_the_element() {
    let x = DJI_DOC_WAYLINES.replace(
        "<wpml:executeHeight>116.57</wpml:executeHeight>",
        "<wpml:executeHeight>high</wpml:executeHeight>",
    );
    match parse_document(&x) {
        Err(Error::Value { element, value }) => {
            assert_eq!(element, "wpml:executeHeight");
            assert_eq!(value, "high");
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn written_kmz_layout() {
    let d = parse_document(DJI_DOC_WAYLINES).unwrap();
    let kmz = Kmz::from_waylines(d, Some(-3.5)).unwrap();
    let bytes = kmz.to_bytes().unwrap();
    let z = zip::ZipArchive::new(Cursor::new(&bytes)).unwrap();
    let names: Vec<String> = z.file_names().map(|n| n.unwrap().into_owned()).collect();
    assert_eq!(names, [TEMPLATE_PATH, WAYLINES_PATH]);
    let back = Kmz::from_bytes(&bytes).unwrap();
    assert_eq!(back, kmz);
    assert!(back.template.unwrap().create_time.is_some());
}

#[cfg(feature = "mavmission")]
mod bridge {
    use super::*;
    use wpml::bridge::{from_plan, from_wpl, BridgeOptions};

    fn text(name: &str) -> String {
        String::from_utf8(data(name)).unwrap()
    }

    fn m3e() -> BridgeOptions {
        let (d, payload) = drone_by_name("m3e").unwrap();
        let mut o = BridgeOptions::new(d);
        o.payload_enum = payload;
        o
    }

    /// QGroundControl test/MissionManager/MissionPlanner.waypoints.txt
    /// has a DO_JUMP at seq 7: a loop WPML cannot express.
    #[test]
    fn qgc_mission_planner_file() {
        let w = mavmission::wpl::parse(&text("qgc_MissionPlanner.waypoints.txt")).unwrap();
        let e = from_wpl(&w, &m3e()).unwrap_err();
        assert!(matches!(e, Error::Convert { seq: 7, .. }), "{e}");

        // Rows 0 to 6 (home, takeoff to 30 m, five waypoints at 100 m relative).
        let mut head = w.clone();
        head.items.truncate(7);
        let c = from_wpl(&head, &m3e()).unwrap();
        let wl = c.kmz.waylines.as_ref().unwrap();
        assert_eq!(wl.mission_config.take_off_security_height, Some(30.0));
        assert_eq!(wl.mission_config.drone_info.unwrap().drone_enum_value, 77);
        let f = &wl.folders[0];
        assert_eq!(
            f.execute_height_mode,
            Some(ExecuteHeightMode::RelativeToStartPoint)
        );
        assert_eq!(f.placemarks.len(), 5);
        assert_eq!(
            (f.placemarks[0].lat, f.placemarks[0].lon),
            (40.122993, -105.167717)
        );
        assert_eq!(f.placemarks[0].execute_height, Some(100.0));
        // And the written file reads back the same.
        assert_eq!(round_trip(&c.kmz), c.kmz);
    }

    /// QGroundControl test/plans/ArduPilotSITL-ArduCopter.plan mixes
    /// GLOBAL_RELATIVE_ALT, GLOBAL and GLOBAL_TERRAIN_ALT waypoints.
    #[test]
    fn qgc_sitl_plan_mixed_frames_refused() {
        let p = mavmission::plan::parse(&text("qgc_ArduPilotSITL-ArduCopter.plan")).unwrap();
        // Without N the GLOBAL (sea level) waypoint cannot become WGS84.
        let e = from_plan(&p, &m3e()).unwrap_err();
        assert!(matches!(&e, Error::Height(m) if m.contains("geoid")), "{e}");
        // With N, the mix of frames is still refused: one wayline, one height mode.
        let mut o = m3e();
        o.geoid_separation = Some(19.0);
        let e = from_plan(&p, &o).unwrap_err();
        assert!(
            matches!(&e, Error::Height(m) if m.contains("mixes height frames")),
            "{e}"
        );
    }
}
