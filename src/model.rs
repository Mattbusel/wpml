//! The typed WPML model shared by `template.kml` and `waylines.wpml`.
//!
//! Field names follow the DJI Cloud API WPML reference
//! (<https://developer.dji.com/doc/cloud-api-tutorial/en/api-reference/dji-wpml/overview.html>).
//! Every element is `Option` because the two files use different subsets,
//! and every struct keeps the elements it does not model in `extra`, so a
//! file read and written again loses nothing.

use crate::xml::Element;

macro_rules! string_enum {
    (
        $(#[$meta:meta])*
        $name:ident { $( $(#[$vmeta:meta])* $variant:ident = $text:literal, )* }
    ) => {
        $(#[$meta])*
        #[derive(Debug, Clone, PartialEq, Eq)]
        pub enum $name {
            $( $(#[$vmeta])* $variant, )*
            /// A value this version of the crate does not know, kept as written.
            Other(String),
        }

        impl $name {
            /// The value as written in the file.
            pub fn as_str(&self) -> &str {
                match self {
                    $( $name::$variant => $text, )*
                    $name::Other(s) => s.as_str(),
                }
            }

            /// Parses the file value; unknown values become `Other`.
            pub fn parse(s: &str) -> Self {
                match s {
                    $( $text => $name::$variant, )*
                    other => $name::Other(other.to_string()),
                }
            }
        }

        impl core::fmt::Display for $name {
            fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                f.write_str(self.as_str())
            }
        }
    };
}

string_enum! {
    /// `wpml:flyToWaylineMode`: how the aircraft gets to the first waypoint.
    FlyToWaylineMode {
        /// Climb to the first waypoint's altitude, then fly level.
        Safely = "safely",
        /// Climb to the safe takeoff height, then fly a slope to the first waypoint.
        PointToPoint = "pointToPoint",
    }
}

string_enum! {
    /// `wpml:finishAction`: what the aircraft does after the last waypoint.
    FinishAction {
        /// Return to home.
        GoHome = "goHome",
        /// Exit wayline mode and hover.
        NoAction = "noAction",
        /// Land where it is.
        AutoLand = "autoLand",
        /// Fly back to the first waypoint.
        GotoFirstWaypoint = "gotoFirstWaypoint",
    }
}

string_enum! {
    /// `wpml:exitOnRCLost`: continue the wayline or run the lost action.
    ExitOnRcLost {
        /// Keep flying the wayline.
        GoContinue = "goContinue",
        /// Exit and run `executeRCLostAction`.
        ExecuteLostAction = "executeLostAction",
    }
}

string_enum! {
    /// `wpml:executeRCLostAction`.
    RcLostAction {
        /// Fly back to the takeoff point.
        GoBack = "goBack",
        /// Land in place.
        Landing = "landing",
        /// Hover in place.
        Hover = "hover",
    }
}

string_enum! {
    /// `wpml:executeHeightMode` in `waylines.wpml`: the datum of `executeHeight`.
    ExecuteHeightMode {
        /// Height above the WGS84 ellipsoid. Not above mean sea level: the
        /// two differ by the geoid separation, which is tens of metres in
        /// most places.
        Wgs84 = "WGS84",
        /// Height above the takeoff point.
        RelativeToStartPoint = "relativeToStartPoint",
        /// Height above the surface, measured live by the aircraft (M3E/M3T/M3M).
        RealTimeFollowSurface = "realTimeFollowSurface",
    }
}

string_enum! {
    /// `wpml:heightMode` in `template.kml` (`waylineCoordinateSysParam`):
    /// the datum of a waypoint's `wpml:height`.
    HeightMode {
        /// Height above mean sea level (EGM96 geoid).
        Egm96 = "EGM96",
        /// Height above the takeoff point.
        RelativeToStartPoint = "relativeToStartPoint",
        /// Height above ground from terrain data (FlightHub 2 only).
        AboveGroundLevel = "aboveGroundLevel",
        /// Live surface following (mapping templates, M3E/M3T/M3M).
        RealTimeFollowSurface = "realTimeFollowSurface",
    }
}

string_enum! {
    /// `wpml:templateType`.
    TemplateType {
        /// Waypoint flight.
        Waypoint = "waypoint",
        /// 2D mapping.
        Mapping2d = "mapping2d",
        /// Oblique photography.
        Mapping3d = "mapping3d",
        /// Strip (linear) mapping.
        MappingStrip = "mappingStrip",
    }
}

string_enum! {
    /// `wpml:waypointHeadingMode`.
    HeadingMode {
        /// Nose follows the course to the next waypoint.
        FollowWayline = "followWayline",
        /// Pilot controls the heading.
        Manually = "manually",
        /// Keep the heading reached at the waypoint.
        Fixed = "fixed",
        /// Rotate evenly to `waypointHeadingAngle`.
        SmoothTransition = "smoothTransition",
        /// Face `waypointPoiPoint`.
        TowardPoi = "towardPOI",
    }
}

string_enum! {
    /// `wpml:waypointTurnMode`.
    TurnMode {
        /// Coordinated turn, cuts the corner.
        CoordinateTurn = "coordinateTurn",
        /// Straight line, stop at the point.
        ToPointAndStopWithDiscontinuityCurvature = "toPointAndStopWithDiscontinuityCurvature",
        /// Curve, stop at the point.
        ToPointAndStopWithContinuityCurvature = "toPointAndStopWithContinuityCurvature",
        /// Curve, pass through the point.
        ToPointAndPassWithContinuityCurvature = "toPointAndPassWithContinuityCurvature",
    }
}

string_enum! {
    /// `wpml:actionTriggerType`.
    TriggerType {
        /// Run on arrival at the waypoint.
        ReachPoint = "reachPoint",
        /// Run across the segment (with `gimbalEvenlyRotate`).
        BetweenAdjacentPoints = "betweenAdjacentPoints",
        /// Repeat every `actionTriggerParam` seconds.
        MultipleTiming = "multipleTiming",
        /// Repeat every `actionTriggerParam` metres.
        MultipleDistance = "multipleDistance",
    }
}

/// Which file a [`Document`] came from or is written as.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FileKind {
    /// `wpmz/template.kml`: what Pilot 2 and FlightHub 2 edit.
    Template,
    /// `wpmz/waylines.wpml`: what the aircraft executes (MSDK, Cloud API).
    Waylines,
}

/// One `template.kml` or `waylines.wpml` file.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Document {
    /// WPML namespace URI found in the file, such as `http://www.dji.com/wpmz/1.0.2`.
    /// `None` on a new document: the writer then uses [`crate::DEFAULT_NAMESPACE`].
    pub namespace: Option<String>,
    /// `wpml:author` (template only).
    pub author: Option<String>,
    /// `wpml:createTime`, Unix ms (template only).
    pub create_time: Option<u64>,
    /// `wpml:updateTime`, Unix ms (template only).
    pub update_time: Option<u64>,
    /// `wpml:missionConfig`.
    pub mission_config: MissionConfig,
    /// One `Folder` per template (template.kml) or per wayline (waylines.wpml).
    pub folders: Vec<Folder>,
    /// Other children of `Document`.
    pub extra: Vec<Element>,
    /// Extra `xmlns:*` declarations from the root, written back as found.
    pub other_namespaces: Vec<(String, String)>,
}

/// `wpml:droneInfo`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct DroneInfo {
    /// `wpml:droneEnumValue`: 89 M350 RTK, 60 M300 RTK, 67 M30/M30T,
    /// 77 M3E/M3T/M3M, 91 M3D/M3TD (DJI common elements table).
    pub drone_enum_value: u32,
    /// `wpml:droneSubEnumValue`.
    pub drone_sub_enum_value: Option<u32>,
}

/// `wpml:payloadInfo`.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct PayloadInfo {
    /// `wpml:payloadEnumValue`, such as 52 for the M30 camera.
    pub payload_enum_value: u32,
    /// `wpml:payloadSubEnumValue`, present in some DJI samples.
    pub payload_sub_enum_value: Option<u32>,
    /// `wpml:payloadPositionIndex`: 0 is the main (or front left) gimbal.
    pub payload_position_index: u32,
    /// Other children.
    pub extra: Vec<Element>,
}

/// `wpml:missionConfig`.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct MissionConfig {
    /// `wpml:flyToWaylineMode`.
    pub fly_to_wayline_mode: Option<FlyToWaylineMode>,
    /// `wpml:finishAction`.
    pub finish_action: Option<FinishAction>,
    /// `wpml:exitOnRCLost`.
    pub exit_on_rc_lost: Option<ExitOnRcLost>,
    /// `wpml:executeRCLostAction`.
    pub execute_rc_lost_action: Option<RcLostAction>,
    /// `wpml:takeOffSecurityHeight`, metres above the takeoff point.
    pub take_off_security_height: Option<f64>,
    /// `wpml:takeOffRefPoint` as written: `lat,lon,ellipsoid height` (template only).
    pub take_off_ref_point: Option<String>,
    /// `wpml:takeOffRefPointAGLHeight` (template only).
    pub take_off_ref_point_agl_height: Option<f64>,
    /// `wpml:globalTransitionalSpeed`, m/s.
    pub global_transitional_speed: Option<f64>,
    /// `wpml:globalRTHHeight`, metres.
    pub global_rth_height: Option<f64>,
    /// `wpml:droneInfo`.
    pub drone_info: Option<DroneInfo>,
    /// `wpml:payloadInfo` (one per mounted payload).
    pub payload_info: Vec<PayloadInfo>,
    /// Other children.
    pub extra: Vec<Element>,
}

/// `wpml:waylineCoordinateSysParam` (template only).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CoordinateSysParam {
    /// `wpml:coordinateMode`, `WGS84` in every published file.
    pub coordinate_mode: Option<String>,
    /// `wpml:heightMode`.
    pub height_mode: Option<HeightMode>,
    /// `wpml:positioningType`, such as `GPS` or `RTKBaseStation`.
    pub positioning_type: Option<String>,
    /// Other children (globalShootHeight, surfaceFollowModeEnable, ...).
    pub extra: Vec<Element>,
}

/// `wpml:waypointHeadingParam` or `wpml:globalWaypointHeadingParam`.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct HeadingParam {
    /// `wpml:waypointHeadingMode`.
    pub mode: Option<HeadingMode>,
    /// `wpml:waypointHeadingAngle`, degrees in [-180, 180].
    pub angle: Option<f64>,
    /// `wpml:waypointPoiPoint` as written: `lat,lon,alt`.
    pub poi_point: Option<String>,
    /// `wpml:waypointHeadingAngleEnable`.
    pub angle_enable: Option<bool>,
    /// `wpml:waypointHeadingPathMode`: `clockwise`, `counterClockwise` or `followBadArc`.
    pub path_mode: Option<String>,
    /// Other children.
    pub extra: Vec<Element>,
}

/// `wpml:waypointTurnParam`.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct TurnParam {
    /// `wpml:waypointTurnMode`.
    pub mode: Option<TurnMode>,
    /// `wpml:waypointTurnDampingDist`, metres.
    pub damping_dist: Option<f64>,
    /// Other children.
    pub extra: Vec<Element>,
}

/// One `Folder`: a template (template.kml) or an executable wayline (waylines.wpml).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Folder {
    /// `wpml:templateType` (template only).
    pub template_type: Option<TemplateType>,
    /// `wpml:templateId`: links a wayline to its template.
    pub template_id: Option<u32>,
    /// `wpml:waylineId` (waylines only).
    pub wayline_id: Option<u32>,
    /// `wpml:executeHeightMode` (waylines only).
    pub execute_height_mode: Option<ExecuteHeightMode>,
    /// `wpml:waylineCoordinateSysParam` (template only).
    pub coordinate_sys: Option<CoordinateSysParam>,
    /// `wpml:distance`, metres, as computed by the app (waylines only).
    pub distance: Option<f64>,
    /// `wpml:duration`, seconds, as computed by the app (waylines only).
    pub duration: Option<f64>,
    /// `wpml:autoFlightSpeed`, m/s.
    pub auto_flight_speed: Option<f64>,
    /// `wpml:globalHeight` (template only).
    pub global_height: Option<f64>,
    /// `wpml:caliFlightEnable` (template only).
    pub cali_flight_enable: Option<bool>,
    /// `wpml:gimbalPitchMode`: `manual` or `usePointSetting` (template only).
    pub gimbal_pitch_mode: Option<String>,
    /// `wpml:globalWaypointHeadingParam` (template only).
    pub global_heading: Option<HeadingParam>,
    /// `wpml:globalWaypointTurnMode` (template only).
    pub global_turn_mode: Option<TurnMode>,
    /// `wpml:globalUseStraightLine` (template only).
    pub global_use_straight_line: Option<bool>,
    /// `wpml:startActionGroup`: actions before the wayline starts.
    pub start_action_group: Option<ActionGroup>,
    /// The waypoints.
    pub placemarks: Vec<Placemark>,
    /// Other children.
    pub extra: Vec<Element>,
}

/// One waypoint (`Placemark`).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Placemark {
    /// Longitude, degrees (`Point/coordinates`, written `lon,lat`).
    pub lon: f64,
    /// Latitude, degrees.
    pub lat: f64,
    /// A third coordinate if the file had one. DJI does not use it for height.
    pub coord_alt: Option<f64>,
    /// `wpml:index`, from 0 and continuous within a wayline.
    pub index: u32,
    /// `wpml:executeHeight`, in the folder's `executeHeightMode` (waylines only).
    pub execute_height: Option<f64>,
    /// `wpml:height`, in the template's `heightMode` (template only).
    pub height: Option<f64>,
    /// `wpml:ellipsoidHeight`, WGS84 ellipsoid height (template only).
    pub ellipsoid_height: Option<f64>,
    /// `wpml:useGlobalHeight` (template only).
    pub use_global_height: Option<bool>,
    /// `wpml:useGlobalSpeed` (template only).
    pub use_global_speed: Option<bool>,
    /// `wpml:waypointSpeed`, m/s.
    pub waypoint_speed: Option<f64>,
    /// `wpml:useGlobalHeadingParam` (template only).
    pub use_global_heading_param: Option<bool>,
    /// `wpml:waypointHeadingParam`.
    pub heading: Option<HeadingParam>,
    /// `wpml:useGlobalTurnParam` (template only).
    pub use_global_turn_param: Option<bool>,
    /// `wpml:waypointTurnParam`.
    pub turn: Option<TurnParam>,
    /// `wpml:useStraightLine`.
    pub use_straight_line: Option<bool>,
    /// `wpml:gimbalPitchAngle` (template only).
    pub gimbal_pitch_angle: Option<f64>,
    /// `wpml:actionGroup` elements on this waypoint.
    pub action_groups: Vec<ActionGroup>,
    /// Other children (isRisky, waypointGimbalHeadingParam, ...).
    pub extra: Vec<Element>,
}

/// `wpml:actionGroup`.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ActionGroup {
    /// `wpml:actionGroupId`, unique within the file.
    pub id: u32,
    /// `wpml:actionGroupStartIndex`.
    pub start_index: u32,
    /// `wpml:actionGroupEndIndex`, at least `start_index`.
    pub end_index: u32,
    /// `wpml:actionGroupMode`, `sequence` in the spec.
    pub mode: String,
    /// `wpml:actionTrigger/actionTriggerType`.
    pub trigger: Option<TriggerType>,
    /// `wpml:actionTrigger/actionTriggerParam`: seconds or metres.
    pub trigger_param: Option<f64>,
    /// The actions, run in order.
    pub actions: Vec<Action>,
    /// Other children.
    pub extra: Vec<Element>,
}

/// `wpml:action`.
#[derive(Debug, Clone, PartialEq)]
pub struct Action {
    /// `wpml:actionId`, unique within the group.
    pub id: u32,
    /// `wpml:actionActuatorFunc` with its typed parameters.
    pub func: ActionFunc,
    /// Parameters of a known action that this crate does not model
    /// (for example `useGlobalPayloadLensIndex`), kept as found.
    pub extra_params: Vec<Element>,
    /// Other children of `wpml:action`, kept as found.
    pub extra: Vec<Element>,
}

/// `wpml:gimbalRotate` parameters.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct GimbalRotate {
    /// `wpml:payloadPositionIndex`.
    pub payload_position_index: u32,
    /// `wpml:gimbalHeadingYawBase`, `north` in the spec.
    pub heading_yaw_base: Option<String>,
    /// `wpml:gimbalRotateMode`, `absoluteAngle` in the spec.
    pub rotate_mode: Option<String>,
    /// Pitch in degrees when enabled (`gimbalPitchRotateEnable` = 1).
    pub pitch: Option<f64>,
    /// Roll in degrees when enabled.
    pub roll: Option<f64>,
    /// Yaw in degrees relative to north when enabled.
    pub yaw: Option<f64>,
    /// Rotation time in seconds when enabled.
    pub time: Option<f64>,
}

/// `wpml:actionActuatorFunc` and its `wpml:actionActuatorFuncParam`.
#[derive(Debug, Clone, PartialEq)]
pub enum ActionFunc {
    /// `takePhoto`.
    TakePhoto {
        /// `wpml:payloadPositionIndex`.
        payload_position_index: u32,
        /// `wpml:fileSuffix`.
        file_suffix: Option<String>,
        /// `wpml:payloadLensIndex`, such as `wide,ir`.
        payload_lens_index: Option<String>,
    },
    /// `startRecord`.
    StartRecord {
        /// `wpml:payloadPositionIndex`.
        payload_position_index: u32,
        /// `wpml:fileSuffix`.
        file_suffix: Option<String>,
        /// `wpml:payloadLensIndex`.
        payload_lens_index: Option<String>,
    },
    /// `stopRecord`.
    StopRecord {
        /// `wpml:payloadPositionIndex`.
        payload_position_index: u32,
        /// `wpml:payloadLensIndex`.
        payload_lens_index: Option<String>,
    },
    /// `gimbalRotate`.
    GimbalRotate(GimbalRotate),
    /// `rotateYaw`: turn the aircraft.
    RotateYaw {
        /// `wpml:aircraftHeading`, degrees from north in [-180, 180].
        aircraft_heading: f64,
        /// `wpml:aircraftPathMode`: `clockwise` or `counterClockwise`.
        path_mode: Option<String>,
    },
    /// `hover`.
    Hover {
        /// `wpml:hoverTime`, seconds.
        hover_time: f64,
    },
    /// `zoom`.
    Zoom {
        /// `wpml:payloadPositionIndex`.
        payload_position_index: u32,
        /// `wpml:focalLength`, mm.
        focal_length: f64,
    },
    /// Any other action (focus, customDirName, orientedShoot, panoShot, ...),
    /// with its parameter elements kept as found.
    Other {
        /// The `actionActuatorFunc` value.
        func: String,
        /// The children of `actionActuatorFuncParam`.
        params: Vec<Element>,
    },
}

impl ActionFunc {
    /// The `actionActuatorFunc` value.
    pub fn name(&self) -> &str {
        match self {
            ActionFunc::TakePhoto { .. } => "takePhoto",
            ActionFunc::StartRecord { .. } => "startRecord",
            ActionFunc::StopRecord { .. } => "stopRecord",
            ActionFunc::GimbalRotate(_) => "gimbalRotate",
            ActionFunc::RotateYaw { .. } => "rotateYaw",
            ActionFunc::Hover { .. } => "hover",
            ActionFunc::Zoom { .. } => "zoom",
            ActionFunc::Other { func, .. } => func,
        }
    }
}

/// Drone models from the DJI WPML common elements table
/// (`droneEnumValue`, `droneSubEnumValue`, default `payloadEnumValue`).
/// M300 RTK and M350 RTK take several payloads, so they have no default.
pub const DRONES: &[(&str, u32, Option<u32>, Option<u32>)] = &[
    ("m350", 89, None, None),
    ("m300", 60, None, None),
    ("m30", 67, Some(0), Some(52)),
    ("m30t", 67, Some(1), Some(53)),
    ("m3e", 77, Some(0), Some(66)),
    ("m3t", 77, Some(1), Some(67)),
    ("m3m", 77, Some(2), Some(68)),
    ("m3d", 91, Some(0), Some(80)),
    ("m3td", 91, Some(1), Some(81)),
];

/// Looks up a drone by short name (`m30t`, `m3e`, ...) or as `enum[:sub]`
/// numbers (`67:1`). Returns the drone info and the default payload enum.
pub fn drone_by_name(name: &str) -> Option<(DroneInfo, Option<u32>)> {
    let lower = name.to_ascii_lowercase();
    if let Some(&(_, e, sub, payload)) = DRONES.iter().find(|d| d.0 == lower) {
        return Some((
            DroneInfo {
                drone_enum_value: e,
                drone_sub_enum_value: sub,
            },
            payload,
        ));
    }
    let mut parts = lower.splitn(2, ':');
    let e = parts.next()?.trim().parse().ok()?;
    let sub = match parts.next() {
        Some(s) => Some(s.trim().parse().ok()?),
        None => None,
    };
    Some((
        DroneInfo {
            drone_enum_value: e,
            drone_sub_enum_value: sub,
        },
        None,
    ))
}

/// Name for a `droneEnumValue` / `droneSubEnumValue` pair, from [`DRONES`].
pub fn drone_name(info: &DroneInfo) -> Option<&'static str> {
    DRONES
        .iter()
        .find(|d| {
            d.1 == info.drone_enum_value
                && (d.2.is_none() || d.2 == info.drone_sub_enum_value.or(Some(0)))
        })
        .map(|d| d.0)
}
