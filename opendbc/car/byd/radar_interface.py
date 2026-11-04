"""BYD Song Plus DM-i radar interface: b7 continuous lead + MRR track slots + HUD-bar fallback + 67ms slot table.

Four sources (docs/byd-radar-010x.md ②/⑧/⑨, docs/byd-lane-change.md §8.5,
docs/byd-radar-slot67-2026-10-18.md):

  1. 0x351-0x354 RADAR_TRACK_35x on bus 1 - MRR target slots, ~25 Hz, layout
     taken verbatim from the mypilot community byd_tangdm_radar.dbc. On the
     Song these slots are logged as IsValid=0 placeholder frames across all
     415 archive segments - the path auto-lights the moment a real target
     stream becomes reachable (radar private CAN tap; no duty state on the
     car's wired buses fills the slots).

  2. 0x109/0x10A on bus 1 - the continuous fused lead distance (b7, 25 Hz)
     with the presence nibble gate (b3[7:5]==7, ④f), PINNED u=0.25 m/count.
     dRel = b7*u; vRel = median-filtered dRel diff over a 0.4 s window
     (radar_dvrel_quality: steady noise ~0 m/s, group delay 0.24 s, closing
     tracking r=0.56-0.90 on 13 stop-to-lead events). This is the PRIMARY
     synthesized source for radarState; the scale is runtime-tunable via the
     BydRadarDistScale raw param (radarcan.py injection, RadarInterface
     .radar_dist_scale). b7>=246 is the loss-edge band (④a), treated invalid;
     any invalid frame resets the diff history so vRel never spans a gap.

  3. 0x32D ACC_HUD_ADAS on bus 2 - the stock radar's fused single-lead
     summary (HasLead + LeadingDistance gap bars). Used only when neither
     source above produced a point. The bars are TIME-GAP bands (tau
     {0.71,1.02,1.4} s at u=0.25), so RADAR_HUD_BARS_TO_M is shape-true at
     one speed only; the point is published with measured=False so this
     fork's radard never consumes it (see _update_hud) - position-only
     fallback, matching the source project's intent (docs/byd-radar-010x.md ⑩).

  4. bus1 67ms multi-target slot table (blocks B/C/D, radar_multitarget.py) -
     the ARS410-class multi-target broadcast (distance PINNED 1/32 m; lateral
     and attribute fields HYPOTHESIS-isolated until the phase-1 probe 定案).
     Independent of the priority chain above: its points coexist with the
     b7 lead (namespaced track ids 201+) for multi-target context, cut-in
     capture and leadOne/leadTwo candidates. BydRadarSlotTable raw param
     gates the layer (0=off, 1=display default measured=False, 2=control) -
     radarcan.py injects it like BydRadarDistScale; level 0 reproduces the
     pre-slot-table behavior byte-for-byte (rollback spec: 零残留).

Vendor branch sets, extracted from the decompiled _create_radar_can_parser
bytecode (co_names, 2026-10-11): {BYD_TANG_GAS, BYD_TANG_DM} parse (849,852)
@25Hz on CanBus.BYD_AUX=bus 1 - this IS the tangdm slot protocol; the Song is
in NEITHER vendor branch (no parser). We parse all four slots ourselves with
the community layout. RelSpeed caveat: community dbc labels it km/s (wrong);
we apply KPH_TO_MS. TODO(belabor): verify against the first real IsValid=1.

Status: LIVE for alpha-longitudinal verification (2026-10-19, ported
2026-10-20): interface.py flips CP.radarUnavailable=False. In stock-ACC mode
radarState is not consumed by any longitudinal path, so the flag flip is
behavior-neutral there; with AlphaLongitudinalEnabled the synthesized lead
feeds the longitudinal MPC through this fork's radard (carrot/radar_motion:
a radar point becomes leadOne only through the vision association matcher -
dPath/distance/velocity sanity with a farther-than-vision veto and a
"strongly visible" vision fallback - so a bad-quality lead is dropped, not
trusted; HUD-bar points never enter at all (measured=False)).
"""
from collections import deque
from statistics import median

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.byd.radar_multitarget import MultitargetDecoder, TRACK_ID_BASE as SLOT_TRACK_ID_BASE
from opendbc.car.byd.values import DBC, RADAR_DIST_SCALE_DEFAULT, RADAR_HUD_BARS_TO_M
from opendbc.car.interfaces import RadarInterfaceBase

ACC_HUD_ADAS_MSG = 813  # 0x32D, bus 2
TRACK_MSGS = [849, 850, 851, 852]  # 0x351-0x354, bus 1 (MRR slot block)
DIST_MSGS = [265, 266]  # 0x109 RADAR_DIST_109 / 0x10A RADAR_PRESENCE_10A, bus 1
TRACK_FREQ = 25
TRACK_BUS = 1
DIST_FREQ = 25
HUD_BUS = 2
# track-slot point ids are namespaced away from the HUD fallback id so
# switching sources mid-drive never aliases a KF-tracked point; the b7 lead
# IS the HUD lead (same fused target), so it shares HUD_TRACK_ID
TRACK_ID_BASE = 101
# slot-table (67ms multi-target) points live at 201+ (radar_multitarget.py) -
# strictly above the tang-slot block so the tang cleanup below never touches
# them and neither namespace can alias the other
HUD_TRACK_ID = 1

# BydRadarSlotTable raw param (radarcan.py injection, like BydRadarDistScale):
#   0 = off (b7-only, byte-identical to the pre-slot-table behavior)
#   1 = display level (default): slot points enter RadarData.points with
#       measured=False -> liveTracks for display/telemetry; the radard
#       snapshot drops measured=False wholesale, so control candidates are
#       unchanged (consume spec: 默认仅显示)
#   2 = control level (second-stage enable, V2 road test): quality-gated slot
#       points are measured=True and become leadOne/leadTwo candidates
SLOT_TABLE_LEVEL_MAX = 2
SLOT_TABLE_DEFAULT = 1

# ④f/④a lead gate + diff window (radar_dvrel_quality.py quantification)
PRESENCE_VISIBLE = 7
B7_LO = 1
B7_HI = 245          # >=246 is the loss-edge band, not a trustworthy gap
VREL_DIFF_FRAMES = 10  # 0.4 s @ 25 Hz: steady noise ~0, delay 0.24 s


def _create_track_can_parser(CP):
  return CANParser(DBC[CP.carFingerprint][Bus.radar], [(addr, TRACK_FREQ) for addr in TRACK_MSGS], TRACK_BUS)


def _create_hud_can_parser(CP):
  return CANParser(DBC[CP.carFingerprint][Bus.radar], [(ACC_HUD_ADAS_MSG, 50)], HUD_BUS)


def _create_dist_can_parser(CP):
  return CANParser(DBC[CP.carFingerprint][Bus.radar], [(addr, DIST_FREQ) for addr in DIST_MSGS], TRACK_BUS)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.rcp_tracks = None if CP.radarUnavailable else _create_track_can_parser(CP)
    self.rcp_hud = None if CP.radarUnavailable else _create_hud_can_parser(CP)
    self.rcp_dist = None if CP.radarUnavailable else _create_dist_can_parser(CP)
    self.updated_messages: set[int] = set()
    # meters-per-count scale, injected by radarcan.py from the BydRadarDistScale
    # raw param (default pinned 0.25); counts paths never read this
    self.radar_dist_scale = RADAR_DIST_SCALE_DEFAULT
    self._lead_hist: deque[int] = deque(maxlen=VREL_DIFF_FRAMES + 1)  # b7 counts
    self._lead_visible = False
    # 67ms multi-target slot table (blocks B/C/D): pure-python assembler, no
    # DBC entries until the phase-1 定案 lands (design D3). Level injected by
    # radarcan.py from the BydRadarSlotTable raw param; default here keeps the
    # constructor param-free for the 38-platform truth-table probes.
    self.slot_table_level = SLOT_TABLE_DEFAULT
    self._slot_decoder = MultitargetDecoder()
    self._slot_pending: list[dict] = []

  def _update_tracks(self) -> bool:
    """Refresh points from valid MRR slots. True if any slot is valid."""
    any_valid = False
    live_ids = set()
    for i, addr in enumerate(TRACK_MSGS):
      t = self.rcp_tracks.vl[addr]
      # vendor _update: track_id == 0 is the empty-slot sentinel
      if not t["IsValid_tang"] or t["TrackID_tang"] == 0:
        continue
      any_valid = True
      track_id = TRACK_ID_BASE + i
      live_ids.add(track_id)
      if track_id not in self.pts:
        self.pts[track_id] = structs.RadarData.RadarPoint()
        self.pts[track_id].trackId = track_id
      pt = self.pts[track_id]
      pt.dRel = t["LongDist_tang"]
      # vendor clips LAT_DIST to +-5 m; TODO(sign): confirm left/right convention
      # against the first real frame (passed as-is, same choice as the community)
      pt.yRel = max(-5.0, min(5.0, t["LatDist_tang"]))
      # vendor Tang branch: vRel = REL_SPEED * KPH_TO_MS (bytecode-confirmed)
      pt.vRel = t["RelSpeed_tang"] * CV.KPH_TO_MS
      # NOTE(port): aRel/yvRel/measured are deprecated-group members in this
      # fork's RadarPoint (unused by its radard); the vision-association
      # matcher's dist/vel sanity gates are the effective lead filter here
    # slots that dropped or went invalid this cycle are stale (community _update
    # keeps zero-filled invalid points; we drop them - radarState only wants
    # targets). live_ids is empty when nothing is valid, so this also wipes the
    # slot block the moment the last target leaves. Scoped below 201: the
    # slot-table namespace (SLOT_TRACK_ID_BASE) cleans up independently.
    for track_id in list(self.pts.keys()):
      if TRACK_ID_BASE <= track_id < SLOT_TRACK_ID_BASE and track_id not in live_ids:
        del self.pts[track_id]
    return any_valid

  def _update_slot_points(self) -> None:
    """Fold finished slot-table cycles into self.pts (layer 4). Points persist
    across ticks while the decoder's track stays paired (misses == 0); a track
    that missed its last cycle wipes immediately (tang-slot semantics).
    Display level keeps measured=False - the radard snapshot drops
    measured=False wholesale (consume spec: 默认仅显示). Control level (2)
    publishes a point only with a diffable vRel: a NaN vRel coerced through
    radard's _finite() would read as vLead=vEgo (the _update_hud hazard), so
    no-history points stay display-only. yRel is PINNED (2026-11-02, trusted
    band 8-60 m); other unpinned fields (attributes) stay NaN-isolated."""
    for o in self._slot_pending:
      if not o['valid'] or o['d'] is None:
        continue
      tid = o['track_id']
      if tid not in self.pts:
        self.pts[tid] = structs.RadarData.RadarPoint()
        self.pts[tid].trackId = tid
      pt = self.pts[tid]
      pt.dRel = o['d']
      # yRel PINNED 2026-11-02 (byd-radar-y-wire-port 2.7): C-F0 lateral code,
      # trusted band 8-60 m (Y_BAND_*); outside it stays NaN-isolated
      y = o.get('y_rel')
      pt.yRel = y if y is not None else float('nan')
      has_v = o['v_rel'] is not None
      pt.vRel = o['v_rel'] if has_v else float('nan')  # 0.47 m/s quantized
      # NOTE(port): aRel/yvRel/measured are deprecated-group members in this
      # fork's RadarPoint. The measured=False display discipline is enforced
      # structurally instead: a NaN-vRel point can never pass this fork's
      # radard vision-association vel_sane gate, so display-level points stay
      # telemetry-only by construction; control-level (2) publishes the same
      # finite-vRel points either level would.
    live = {t.track_id for t in self._slot_decoder.tracks if t.misses == 0}
    for tid in list(self.pts.keys()):
      if tid >= SLOT_TRACK_ID_BASE and tid not in live:
        del self.pts[tid]
    self._slot_pending = []

  def _update_lead_b7(self) -> bool:
    """Continuous fused lead from 0x109 b7 + 0x10A presence. True if visible."""
    vl = self.rcp_dist.vl[265]
    presence = int(self.rcp_dist.vl[266]["Presence_10A"])
    b7 = int(vl["MSG_109_B7"])
    visible = presence == PRESENCE_VISIBLE and B7_LO <= b7 <= B7_HI
    if not visible:
      # reset the diff history: vRel must never span a target gap/loss edge
      self._lead_hist.clear()
      self._lead_visible = False
      return False
    if not self._lead_visible:
      # rising edge: fresh target, no pre-gap history
      self._lead_hist.clear()
    self._lead_visible = True

    if HUD_TRACK_ID not in self.pts:
      self.pts[HUD_TRACK_ID] = structs.RadarData.RadarPoint()
      self.pts[HUD_TRACK_ID].trackId = HUD_TRACK_ID
    pt = self.pts[HUD_TRACK_ID]
    d_rel = b7 * self.radar_dist_scale
    pt.dRel = d_rel
    pt.yRel = 0.0  # ⑦: no lateral field on the reachable buses
    # median-filtered diff over the last 0.4 s of raw per-frame diffs
    self._lead_hist.append(b7)
    if len(self._lead_hist) >= 2:
      hist = list(self._lead_hist)
      dt = 1.0 / DIST_FREQ
      diffs = [(b - a) * self.radar_dist_scale / dt for a, b in zip(hist[:-1], hist[1:], strict=True)]
      pt.vRel = median(diffs)
    else:
      pt.vRel = 0.0  # single fresh frame: closing unknown, start at rest
    return True

  def _update_hud(self) -> None:
    # carrot port deviation (this change's design.md D2): the source project
    # guarantees this position-only fallback is never selected by radard via
    # vRel=NaN + its vel_sane matcher. This fork's radard (carrot/radar_motion)
    # coerces every NaN through _finite() to 0.0, which would turn the fake
    # velocity into a vLead=vEgo "moving lead" that CAN associate with a real
    # stopped visual lead - the one path that could corrupt longitudinal. The
    # measured flag is the first gate of snapshot_live_radar_points (dropped
    # wholesale), so measured=False reproduces the original "reference only,
    # never consumed" intent exactly. vRel stays NaN for the display consumer.
    vl = self.rcp_hud.vl[ACC_HUD_ADAS_MSG]
    if vl["HasLead"] and vl["LeadingDistance"] in RADAR_HUD_BARS_TO_M:
      if HUD_TRACK_ID not in self.pts:
        self.pts[HUD_TRACK_ID] = structs.RadarData.RadarPoint()
        self.pts[HUD_TRACK_ID].trackId = HUD_TRACK_ID
      pt = self.pts[HUD_TRACK_ID]
      pt.dRel = RADAR_HUD_BARS_TO_M[int(vl["LeadingDistance"])]
      pt.yRel = 0.0
      # vRel stays NaN: this fork's radard never associates a NaN-vRel point
      # with vision (vel_sane gate), reproducing the measured=False
      # "reference only, never consumed" intent without the deprecated flag
      pt.vRel = float('nan')
    else:
      self.pts.pop(HUD_TRACK_ID, None)

  def update(self, can_packets):
    if self.rcp_hud is None:
      return super().update(can_packets)

    # the slot-table assembler sees every packet regardless of the watch-gate
    # below: its 67ms cycles close independent of the 0x10x/HUD cadence, and
    # finished observations wait in _slot_pending for the next RadarData
    if self.slot_table_level > 0:
      self._slot_pending.extend(self._slot_decoder.update(can_packets))

    self.updated_messages.update(self.rcp_tracks.update(can_packets))
    self.updated_messages.update(self.rcp_hud.update(can_packets))
    self.updated_messages.update(self.rcp_dist.update(can_packets))
    watch = set(TRACK_MSGS) | set(DIST_MSGS) | {ACC_HUD_ADAS_MSG}
    if not (self.updated_messages & watch):
      return None
    self.updated_messages.clear()

    ret = structs.RadarData()
    # only the HUD parser is guaranteed present on the car's wired buses;
    # the track/dist blocks can be legitimately silent (e.g. radar powered
    # down on some trims) so their timeout must not raise canError on its own
    if not self.rcp_hud.can_valid:
      ret.errors.canError = True
      return ret

    if not self._update_tracks():
      if not self._update_lead_b7():
        self._update_hud()
    self._update_slot_points()

    ret.points = list(self.pts.values())
    return ret
