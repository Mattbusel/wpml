# Test data sources

| File | Source | License |
|---|---|---|
| `msdk_waypointsample.kmz` | [dji-sdk/Mobile-SDK-Android-V5](https://github.com/dji-sdk/Mobile-SDK-Android-V5), `SampleCode-V5/android-sdk-v5-sample/src/main/assets/waypointsample.kmz` (branch `dev-sdk-main`) | MIT, Copyright (c) 2022 DJI (`LICENSE-MIT-dji-msdk-v5-samples`) |
| `msdk_flight_task_demo.kmz` | same repository, `SampleCode-V5/open-pilot-flighttask-sample/server/data/FLIGHT-TASK-DEMO-001.kmz` | MIT, Copyright (c) 2022 DJI |
| `qgc_MissionPlanner.waypoints.txt` | [mavlink/qgroundcontrol](https://github.com/mavlink/qgroundcontrol), `test/MissionManager/MissionPlanner.waypoints.txt` | Apache-2.0 (QGroundControl is dual licensed Apache-2.0 / GPL-3.0) |
| `qgc_ArduPilotSITL-ArduCopter.plan` | same repository, `test/plans/ArduPilotSITL-ArduCopter.plan` | Apache-2.0 |

The `waylines.wpml` text in `tests/fixtures.rs` is the sample from the DJI
Cloud API WPML reference
([waylines.wpml page](https://developer.dji.com/doc/cloud-api-tutorial/en/api-reference/dji-wpml/waylines-wpml.html),
source in [dji-sdk/Cloud-API-Doc](https://github.com/dji-sdk/Cloud-API-Doc)),
with its `longitude,latitude` placeholders filled in so it parses.

Only `msdk_waypointsample.kmz` (used by the crate-level doc example) and its
license file ship in the published crate.
