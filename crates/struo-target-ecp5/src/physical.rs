//! Tool-independent physical feedback and nextpnr report ingestion.

use std::collections::BTreeMap;

use serde::Deserialize;

/// Placed location of a technology-mapped cell.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PhysicalLocation {
    /// Device-grid X coordinate.
    pub x: i32,
    /// Device-grid Y coordinate.
    pub y: i32,
}

/// One routed timing endpoint reported by the physical implementation tool.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PhysicalTimingEndpoint {
    /// Stable mapped cell name.
    pub cell: String,
    /// Cell input port reached by the net.
    pub port: String,
    /// Routed path delay to this endpoint in picoseconds.
    pub delay_ps: u32,
    /// Timing budget assigned to this endpoint in picoseconds.
    pub budget_ps: u32,
}

/// Routed timing observations for one mapped net.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PhysicalNetTiming {
    /// Stable mapped driver cell name.
    pub driver: String,
    /// Physical-tool net name, retained for diagnostics.
    pub net: String,
    /// Routed endpoints of the net.
    pub endpoints: Vec<PhysicalTimingEndpoint>,
}

/// Physical observations returned to synthesis after a deterministic draft run.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct PhysicalFeedback {
    placements: BTreeMap<String, PhysicalLocation>,
    bels: BTreeMap<String, String>,
    net_timings: Vec<PhysicalNetTiming>,
    clock_fmax_khz: Vec<(u32, u32)>,
}

impl PhysicalFeedback {
    /// Reads nextpnr's detailed timing report and post-placement JSON into the
    /// tool-independent feedback representation.
    ///
    /// # Errors
    ///
    /// Returns an error if either JSON document is malformed.
    pub fn from_nextpnr_json(
        report_json: &str,
        placed_json: &str,
    ) -> Result<Self, serde_json::Error> {
        let report: NextpnrReport = serde_json::from_str(report_json)?;
        let placed: NextpnrDesign = serde_json::from_str(placed_json)?;
        let placed_cells = placed
            .modules
            .into_values()
            .flat_map(|module| module.cells)
            .filter_map(|(name, cell)| {
                let bel = cell.attributes.get("NEXTPNR_BEL")?;
                Some((name, bel.clone(), parse_bel_location(bel)?))
            })
            .collect::<Vec<_>>();
        let placements = placed_cells
            .iter()
            .map(|(name, _, location)| (name.clone(), *location))
            .collect();
        let bels = placed_cells
            .into_iter()
            .map(|(name, bel, _)| (name, bel))
            .collect();
        let clock_fmax_khz = report
            .fmax
            .values()
            .map(|clock| {
                (
                    megahertz_to_kilohertz(clock.achieved),
                    megahertz_to_kilohertz(clock.constraint),
                )
            })
            .collect();
        let net_timings = report
            .detailed_net_timings
            .into_iter()
            .map(|timing| PhysicalNetTiming {
                driver: timing.driver,
                net: timing.net,
                endpoints: timing
                    .endpoints
                    .into_iter()
                    .map(|endpoint| PhysicalTimingEndpoint {
                        cell: endpoint.cell,
                        port: endpoint.port,
                        delay_ps: nanoseconds_to_picoseconds(endpoint.delay),
                        budget_ps: nanoseconds_to_picoseconds(endpoint.budget),
                    })
                    .collect(),
            })
            .collect();
        Ok(Self {
            placements,
            bels,
            net_timings,
            clock_fmax_khz,
        })
    }

    /// Returns the placed location of a stable mapped cell name.
    #[must_use]
    pub fn location(&self, cell: &str) -> Option<PhysicalLocation> {
        self.placements.get(cell).copied()
    }

    /// Returns the exact draft BEL assigned to a stable mapped cell name.
    #[must_use]
    pub fn bel(&self, cell: &str) -> Option<&str> {
        self.bels.get(cell).map(String::as_str)
    }

    /// Returns routed net timing observations from the draft run.
    #[must_use]
    pub fn net_timings(&self) -> &[PhysicalNetTiming] {
        &self.net_timings
    }

    /// Returns true when every reported clock is within `percent` of its
    /// target. Local physical rewrites are deliberately restricted to this
    /// near-closure region.
    #[must_use]
    pub fn is_near_timing_closure(&self, percent: u32) -> bool {
        !self.clock_fmax_khz.is_empty()
            && self.clock_fmax_khz.iter().all(|(achieved, target)| {
                *target > 0 && u64::from(*achieved) * 100 >= u64::from(*target) * u64::from(percent)
            })
    }
}

fn parse_bel_location(bel: &str) -> Option<PhysicalLocation> {
    let mut components = bel.split('/');
    let x = components.next()?.strip_prefix('X')?.parse().ok()?;
    let y = components.next()?.strip_prefix('Y')?.parse().ok()?;
    Some(PhysicalLocation { x, y })
}

fn nanoseconds_to_picoseconds(value: f64) -> u32 {
    scaled_thousand_to_u32(value)
}

fn megahertz_to_kilohertz(value: f64) -> u32 {
    scaled_thousand_to_u32(value)
}

#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn scaled_thousand_to_u32(value: f64) -> u32 {
    if !value.is_finite() || value <= 0.0 {
        return 0;
    }
    (value * 1_000.0).round().clamp(0.0, f64::from(u32::MAX)) as u32
}

#[derive(Deserialize)]
struct NextpnrReport {
    #[serde(default)]
    detailed_net_timings: Vec<NextpnrNetTiming>,
    #[serde(default)]
    fmax: BTreeMap<String, NextpnrFmax>,
}

#[derive(Deserialize)]
struct NextpnrFmax {
    achieved: f64,
    constraint: f64,
}

#[derive(Deserialize)]
struct NextpnrNetTiming {
    driver: String,
    net: String,
    #[serde(default)]
    endpoints: Vec<NextpnrTimingEndpoint>,
}

#[derive(Deserialize)]
struct NextpnrTimingEndpoint {
    cell: String,
    port: String,
    delay: f64,
    budget: f64,
}

#[derive(Deserialize)]
struct NextpnrDesign {
    modules: BTreeMap<String, NextpnrModule>,
}

#[derive(Deserialize)]
struct NextpnrModule {
    cells: BTreeMap<String, NextpnrCell>,
}

#[derive(Deserialize)]
struct NextpnrCell {
    #[serde(default)]
    attributes: BTreeMap<String, String>,
}

#[cfg(test)]
mod tests {
    use super::{PhysicalFeedback, PhysicalLocation};

    #[test]
    fn reads_nextpnr_placement_and_detailed_timing() {
        let report = r#"{
            "detailed_net_timings": [{
                "driver": "enable_lut",
                "net": "$enable",
                "endpoints": [{
                    "budget": 3.125,
                    "cell": "value_ff",
                    "delay": 2.375,
                    "event": "posedge clk",
                    "port": "CE"
                }],
                "event": "posedge clk",
                "port": "F"
            }],
            "fmax": {"clk": {"achieved": 317.5, "constraint": 320.0}}
        }"#;
        let placed = r#"{
            "modules": {"top": {"cells": {
                "enable_lut": {"attributes": {
                    "NEXTPNR_BEL": "X12/Y7/SLICEA.K0"
                }},
                "value_ff": {"attributes": {
                    "NEXTPNR_BEL": "X14/Y8/SLICEB.FF1"
                }}
            }}}
        }"#;

        let feedback = PhysicalFeedback::from_nextpnr_json(report, placed).unwrap();

        assert_eq!(
            feedback.location("value_ff"),
            Some(PhysicalLocation { x: 14, y: 8 })
        );
        assert_eq!(feedback.bel("value_ff"), Some("X14/Y8/SLICEB.FF1"));
        assert_eq!(feedback.net_timings()[0].endpoints[0].delay_ps, 2_375);
        assert_eq!(feedback.net_timings()[0].endpoints[0].budget_ps, 3_125);
        assert!(feedback.is_near_timing_closure(98));
    }
}
