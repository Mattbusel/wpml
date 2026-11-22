//! The `.kmz` container: `wpmz/template.kml`, `wpmz/waylines.wpml` and `wpmz/res/`.

use crate::model::*;
use crate::parse::parse_document;
use crate::write::write_document;
use crate::Error;
use std::io::{Cursor, Read, Seek, Write};
use zip::write::SimpleFileOptions;

/// Path of the template inside the KMZ.
pub const TEMPLATE_PATH: &str = "wpmz/template.kml";
/// Path of the execution file inside the KMZ.
pub const WAYLINES_PATH: &str = "wpmz/waylines.wpml";
/// Folder for resources inside the KMZ.
pub const RES_DIR: &str = "wpmz/res/";

/// A WPML mission file.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Kmz {
    /// `wpmz/template.kml`. Pilot 2 and FlightHub 2 import this one.
    pub template: Option<Document>,
    /// `wpmz/waylines.wpml`. The aircraft executes this one (MSDK v5 `startMission`, Cloud API flight tasks).
    pub waylines: Option<Document>,
    /// Files under `wpmz/res/`, as (path below `res/`, bytes).
    pub resources: Vec<(String, Vec<u8>)>,
}

fn file_name_is(path: &str, name: &str) -> bool {
    let p = path.replace('\\', "/");
    let last = p.rsplit('/').next().unwrap_or(&p);
    last.eq_ignore_ascii_case(name)
}

impl Kmz {
    /// Reads a KMZ from bytes.
    pub fn from_bytes(bytes: &[u8]) -> Result<Kmz, Error> {
        Kmz::read(Cursor::new(bytes))
    }

    /// Reads a KMZ. `wpmz/waylines.wpml` and `wpmz/template.kml` are found
    /// by file name, preferring the `wpmz/` folder, so archives repacked
    /// with an extra top folder or without `wpmz/` still open. Either file
    /// may be missing, not both.
    pub fn read<R: Read + Seek>(r: R) -> Result<Kmz, Error> {
        let mut zip = zip::ZipArchive::new(r).map_err(|e| Error::Zip(e.to_string()))?;
        let mut names: Vec<String> = Vec::with_capacity(zip.len());
        for i in 0..zip.len() {
            let f = zip.by_index_raw(i).map_err(|e| Error::Zip(e.to_string()))?;
            let name = f.name().map_err(|e| Error::Zip(e.to_string()))?;
            names.push(name.to_string());
        }
        let pick = |name: &str| -> Option<String> {
            let mut c: Vec<&String> = names.iter().filter(|n| file_name_is(n, name)).collect();
            c.sort_by_key(|n| {
                (
                    !n.replace('\\', "/").ends_with(&format!("wpmz/{name}")),
                    n.len(),
                )
            });
            c.first().map(|s| s.to_string())
        };
        let mut read_text = |path: &str| -> Result<String, Error> {
            let mut f = zip.by_name(path).map_err(|e| Error::Zip(e.to_string()))?;
            let mut buf = Vec::new();
            f.read_to_end(&mut buf)
                .map_err(|e| Error::Zip(e.to_string()))?;
            String::from_utf8(buf).map_err(|_| Error::Xml(format!("{path} is not UTF-8")))
        };
        let waylines = match pick("waylines.wpml") {
            Some(p) => Some(parse_document(&read_text(&p)?).map_err(|e| e.in_file(&p))?),
            None => None,
        };
        let template = match pick("template.kml") {
            Some(p) => Some(parse_document(&read_text(&p)?).map_err(|e| e.in_file(&p))?),
            None => None,
        };
        if waylines.is_none() && template.is_none() {
            return Err(Error::Missing(
                "neither wpmz/waylines.wpml nor wpmz/template.kml is in the archive",
            ));
        }
        let mut resources = Vec::new();
        for n in &names {
            let norm = n.replace('\\', "/");
            let start = if let Some(i) = norm.find(RES_DIR) {
                Some(i + RES_DIR.len())
            } else if norm.starts_with("res/") {
                Some(4)
            } else {
                None
            };
            if let Some(start) = start {
                let rel = &norm[start..];
                if rel.is_empty() || rel.ends_with('/') {
                    continue;
                }
                let mut f = zip.by_name(n).map_err(|e| Error::Zip(e.to_string()))?;
                let mut buf = Vec::new();
                f.read_to_end(&mut buf)
                    .map_err(|e| Error::Zip(e.to_string()))?;
                resources.push((rel.to_string(), buf));
            }
        }
        Ok(Kmz {
            template,
            waylines,
            resources,
        })
    }

    /// Writes the KMZ: `wpmz/template.kml` (when present), then
    /// `wpmz/waylines.wpml`, then `wpmz/res/`, deflated, the same layout
    /// as the KMZ files DJI ships with MSDK v5.
    pub fn write<W: Write + Seek>(&self, w: W) -> Result<(), Error> {
        let mut zip = zip::ZipWriter::new(w);
        let opts =
            SimpleFileOptions::default().compression_method(zip::CompressionMethod::Deflated);
        let z = |e: zip::result::ZipError| Error::Zip(e.to_string());
        let io = |e: std::io::Error| Error::Zip(e.to_string());
        if let Some(t) = &self.template {
            zip.start_file(TEMPLATE_PATH, opts).map_err(z)?;
            zip.write_all(write_document(t, FileKind::Template).as_bytes())
                .map_err(io)?;
        }
        if let Some(wl) = &self.waylines {
            zip.start_file(WAYLINES_PATH, opts).map_err(z)?;
            zip.write_all(write_document(wl, FileKind::Waylines).as_bytes())
                .map_err(io)?;
        }
        for (name, bytes) in &self.resources {
            if name.contains("..") || name.starts_with('/') {
                return Err(Error::Structure(format!("bad resource path {name}")));
            }
            zip.start_file(format!("{RES_DIR}{name}"), opts)
                .map_err(z)?;
            zip.write_all(bytes).map_err(io)?;
        }
        zip.finish().map_err(z)?;
        Ok(())
    }

    /// The KMZ as bytes.
    pub fn to_bytes(&self) -> Result<Vec<u8>, Error> {
        let mut c = Cursor::new(Vec::new());
        self.write(&mut c)?;
        Ok(c.into_inner())
    }

    /// Builds a KMZ from an execution file, generating the matching
    /// `template.kml` so the same file imports into Pilot 2 / FlightHub 2
    /// and runs from MSDK. See [`template_from_waylines`].
    pub fn from_waylines(waylines: Document, geoid_separation: Option<f64>) -> Result<Kmz, Error> {
        let mut template = template_from_waylines(&waylines, geoid_separation)?;
        // Pilot 2 exports always carry these; stamp the file the same way.
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_millis() as u64)
            .ok();
        template.create_time = now;
        template.update_time = now;
        Ok(Kmz {
            template: Some(template),
            waylines: Some(waylines),
            resources: Vec::new(),
        })
    }
}

/// Generates a waypoint `template.kml` that describes the same flight as a
/// `waylines.wpml`, one template per `templateId`.
///
/// Heights are where this goes wrong in practice, so they are mapped
/// explicitly:
///
/// * `relativeToStartPoint` waylines get a `relativeToStartPoint` template;
///   `height` and `ellipsoidHeight` both carry the relative height, as in
///   DJI's own samples.
/// * `WGS84` waylines carry ellipsoid heights, but a template has no
///   ellipsoid mode: its absolute mode is `EGM96` (above mean sea level).
///   Converting needs the geoid separation N at the site (ellipsoid minus
///   EGM96, metres): `height = executeHeight - N`. Without it this returns
///   [`Error::Height`] rather than writing ellipsoid heights as sea level
///   heights, which is off by N (tens of metres in most places).
/// * `realTimeFollowSurface` has no waypoint template counterpart and is
///   refused.
pub fn template_from_waylines(
    waylines: &Document,
    geoid_separation: Option<f64>,
) -> Result<Document, Error> {
    let mut out = Document {
        namespace: waylines.namespace.clone(),
        mission_config: waylines.mission_config.clone(),
        other_namespaces: waylines.other_namespaces.clone(),
        ..Default::default()
    };
    let mut seen = Vec::new();
    for (i, wl) in waylines.folders.iter().enumerate() {
        let tid = wl.template_id.unwrap_or(i as u32);
        if seen.contains(&tid) {
            continue;
        }
        seen.push(tid);
        let mode = wl.execute_height_mode.clone().ok_or_else(|| {
            Error::Height(format!(
                "wayline {} has no executeHeightMode, so its heights have no datum",
                wl.wayline_id.unwrap_or(i as u32)
            ))
        })?;
        let (height_mode, to_height): (HeightMode, Box<dyn Fn(f64) -> f64>) = match mode {
            ExecuteHeightMode::RelativeToStartPoint => {
                (HeightMode::RelativeToStartPoint, Box::new(|h| h))
            }
            ExecuteHeightMode::Wgs84 => {
                let n = geoid_separation.ok_or_else(|| {
                    Error::Height(
                        "waylines use WGS84 ellipsoid heights; template.kml stores EGM96 \
                         (sea level) heights. Pass the geoid separation N at the site \
                         (ellipsoid minus EGM96, metres) to convert"
                            .into(),
                    )
                })?;
                (HeightMode::Egm96, Box::new(move |h| h - n))
            }
            other => {
                return Err(Error::Height(format!(
                    "executeHeightMode {other} has no waypoint template equivalent"
                )))
            }
        };
        let mut t = Folder {
            template_type: Some(TemplateType::Waypoint),
            template_id: Some(tid),
            coordinate_sys: Some(CoordinateSysParam {
                coordinate_mode: Some("WGS84".into()),
                height_mode: Some(height_mode),
                positioning_type: Some("GPS".into()),
                extra: Vec::new(),
            }),
            auto_flight_speed: wl.auto_flight_speed,
            global_height: wl
                .placemarks
                .first()
                .and_then(|p| p.execute_height)
                .map(&to_height),
            cali_flight_enable: Some(false),
            gimbal_pitch_mode: Some("manual".into()),
            global_heading: Some(HeadingParam {
                mode: Some(HeadingMode::FollowWayline),
                ..Default::default()
            }),
            global_turn_mode: Some(TurnMode::ToPointAndStopWithDiscontinuityCurvature),
            global_use_straight_line: Some(true),
            // startActionGroup is a waylines.wpml element only (DJI WPML
            // reference); the aircraft runs it from the waylines file.
            start_action_group: None,
            ..Default::default()
        };
        for p in &wl.placemarks {
            let eh = p.execute_height.ok_or_else(|| {
                Error::Height(format!("waypoint {} has no executeHeight", p.index))
            })?;
            t.placemarks.push(Placemark {
                lon: p.lon,
                lat: p.lat,
                coord_alt: None,
                index: p.index,
                ellipsoid_height: Some(eh),
                height: Some(to_height(eh)),
                use_global_height: Some(false),
                use_global_speed: Some(p.waypoint_speed.is_none()),
                waypoint_speed: p.waypoint_speed,
                use_global_heading_param: Some(p.heading.is_none()),
                heading: p.heading.clone(),
                use_global_turn_param: Some(p.turn.is_none()),
                turn: p.turn.clone(),
                use_straight_line: p.use_straight_line,
                action_groups: p.action_groups.clone(),
                ..Default::default()
            });
        }
        out.folders.push(t);
    }
    Ok(out)
}
