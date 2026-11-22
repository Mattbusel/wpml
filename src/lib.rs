//! DJI WPML waypoint missions: the `.kmz` files that DJI Pilot 2,
//! FlightHub 2, the Cloud API and MSDK v5 read and write.
//!
//! A WPML KMZ is a zip with two XML files and a resource folder:
//!
//! * `wpmz/template.kml`, the editable plan that Pilot 2 and FlightHub 2 import;
//! * `wpmz/waylines.wpml`, the execution file the aircraft flies (MSDK v5
//!   and Cloud API flight tasks run this one);
//! * `wpmz/res/`, resources such as reference photos.
//!
//! [`Kmz::from_bytes`] opens one into typed [`Document`]s, keeping every
//! element it does not model, and [`Kmz::to_bytes`] writes it back.
//! [`Kmz::from_waylines`] writes a matching `template.kml` next to a
//! `waylines.wpml` so one file both imports into Pilot 2 and runs from MSDK.
//!
//! ```
//! # fn main() -> Result<(), wpml::Error> {
//! // The waypoint sample KMZ that DJI ships with MSDK v5 (MIT).
//! let bytes = include_bytes!("../tests/data/msdk_waypointsample.kmz");
//! let kmz = wpml::Kmz::from_bytes(bytes)?;
//! let wl = kmz.waylines.as_ref().unwrap();
//! let folder = &wl.folders[0];
//! assert_eq!(folder.execute_height_mode, Some(wpml::ExecuteHeightMode::RelativeToStartPoint));
//! assert_eq!(folder.placemarks.len(), 4);
//! assert_eq!(folder.placemarks[0].execute_height, Some(100.0));
//! # Ok(()) }
//! ```
//!
//! With the `mavmission` feature, [`bridge`] converts an ArduPilot or PX4
//! mission (QGroundControl `.plan`, Mission Planner `.waypoints`) to WPML,
//! with the height datum mapped explicitly or refused.
#![forbid(unsafe_code)]
#![warn(missing_docs)]

mod kmz;
mod model;
mod parse;
mod write;
pub mod xml;

#[cfg(feature = "mavmission")]
pub mod bridge;

pub use kmz::{template_from_waylines, Kmz, RES_DIR, TEMPLATE_PATH, WAYLINES_PATH};
pub use model::*;
pub use parse::parse_document;
pub use write::write_document;

/// WPML namespace written on new documents. Documents read from a file keep
/// the namespace they had. 1.0.6 is the newest version in DJI's published
/// samples (MSDK v5 sample code); the Cloud API reference shows 1.0.2.
pub const DEFAULT_NAMESPACE: &str = "http://www.dji.com/wpmz/1.0.6";

/// Errors from reading, writing or converting WPML.
#[derive(Debug, Clone, PartialEq)]
pub enum Error {
    /// The XML is not well formed.
    Xml(String),
    /// An element value that does not parse (a number, a bool, coordinates).
    Value {
        /// Normalized element name, such as `wpml:executeHeight`.
        element: String,
        /// The text found.
        value: String,
    },
    /// A required element is missing or in the wrong place.
    Structure(String),
    /// The zip container could not be read or written.
    Zip(String),
    /// A required file is missing from the archive.
    Missing(&'static str),
    /// Heights cannot be mapped between datums without more information.
    Height(String),
    /// A mission item cannot be converted without changing the flight.
    Convert {
        /// Mission item sequence number.
        seq: u16,
        /// Why.
        reason: String,
    },
    /// An error inside one file of the archive.
    InFile {
        /// Path inside the KMZ.
        file: String,
        /// The error.
        error: Box<Error>,
    },
}

impl Error {
    pub(crate) fn in_file(self, file: &str) -> Error {
        Error::InFile {
            file: file.to_string(),
            error: Box::new(self),
        }
    }
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::Xml(m) => write!(f, "XML: {m}"),
            Error::Value { element, value } => write!(f, "{element}: bad value {value:?}"),
            Error::Structure(m) => write!(f, "{m}"),
            Error::Zip(m) => write!(f, "KMZ: {m}"),
            Error::Missing(m) => write!(f, "{m}"),
            Error::Height(m) => write!(f, "height: {m}"),
            Error::Convert { seq, reason } => write!(f, "seq {seq}: {reason}"),
            Error::InFile { file, error } => write!(f, "{file}: {error}"),
        }
    }
}

impl std::error::Error for Error {}
