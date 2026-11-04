"""BYD T8 forward-radar gap monitor: 0x109 b7 as the upstream continuous signal
for lane-change gating (docs/byd-radar-010x.md §落地建议 T8 + §④e/§④f design
stats; docs/byd-lane-change.md §8.4).

What the archive says (2026-10-18 ④):
  - b7 crosses the old/new plateau midpoint BEFORE the HUD gear flips in
    309/309 informative transitions, and at flip time the distance has only
    travelled 37-59% of the plateau gap -> a boundary-cross detector on b7
    leads the gear (and any HUD-side reaction) by ~0.2 s to >8 s.
  - the loss path is SYNCHRONOUS (b7==0 within 0.02 s of HasLead dropping),
    so "target gone" carries no lead time and this monitor never uses
    disappearance as a clear signal.
  - a single static threshold misclassifies 29-60% of frames inside constant
    holds -> detection must be boundary-cross with hysteresis + time debounce.
  - presence/visibility filter (④f): presence==7 AND b7>0; b7>=246 is the
    range-edge band (④a: 246 is the continuous max, 247 a loss-edge transient
    code) and is treated as NOT visible.

u-free by construction: everything runs in r = b7 / vEgo [counts per m/s]
against plateau medians pooled from the 415-segment archive - no meters, no
assumed unit (feet vs 0.25 stays a road-test open item, §④b).

Boundaries: B[3]/B[4] are the midpoints of the pooled plateau medians
{2: 2.92, 3: 4.79, 4: 6.61} (④e pre-table); B[2] has no lower neighbour in
the archive (bar 1 unobserved as a follow class) so it is extrapolated with
the same half-step. cls < 2 or an untracked-but-visible target (the --142
mechanism) uses the tightest band B[2].
"""
from collections import deque

PLATEAU_R = {2: 2.92, 3: 4.79, 4: 6.61}          # counts/(m/s), archive-pooled
_HALF_34 = (PLATEAU_R[4] - PLATEAU_R[3]) / 2.0    # 0.91
_HALF_23 = (PLATEAU_R[3] - PLATEAU_R[2]) / 2.0    # 0.935
BAND_BOUNDARY_DOWN = {2: PLATEAU_R[2] - _HALF_23,   # 1.985
                      3: PLATEAU_R[2] + _HALF_23,   # 3.855
                      4: PLATEAU_R[3] + _HALF_34}   # 5.70

# tuning fixed by tools/byd/radar_t8_replay.py on the 415-segment archive
# (2026-10-18 night): steady-hold would-block 8.3% of visible hold seconds
# (vs ④e's static-threshold 29-60% false-cross), closing-flip pre-warning
# 33% (4->3) / 23% (3->2) with median lead 4.8 s, 22 alert episodes/h.
SET_MARGIN = 0.05    # set while r < boundary*(1-SET_MARGIN) ...
SET_TIME = 0.5       # ... for this many seconds (debounce; ④e false-cross guard)
REL_MARGIN = -0.02   # release once r > boundary*(1-REL_MARGIN) i.e. slightly above
REL_TIME = 1.0       # ... held this long (hysteresis against band-edge chatter)
SLOPE_DROP = 0.2     # r must also be descending: r(now) < r(now-1s) - this
                     # (== relative closing of slope*vEgo m/s: 2 m/s @10, 5.4 @27)
SLOPE_WINDOW = 1.0
V_EGO_MIN = 3.0      # m/s - below this r explodes (stop platforms are not alerts)
B7_LO = 1            # ④f: b7>0 required (b7==0 = no-target sentinel)
B7_HI = 245          # ④a: >=246 is the range-edge band, not a trustworthy gap
PRESENCE_VISIBLE = 7
NEW_HOLD = 10.0      # radar capture leads the HUD gear by >=10 s (④e) - keep the
                     # "new target" flag this long after the visible rising edge
STALE_TIME = 0.5     # 0x109 silent this long -> not fresh, treat as invisible

# --- lane discrimination (byd-radar-multitarget-port stage 3, design D5) -----
# Relative lateral only (v1): the slot table's yRel decides own vs adjacent
# band; no absolute installation-angle calibration exists. Values are
# PLACEHOLDER priors - task 3.5 re-calibrates them from V2 road telemetry and
# replaces these numbers (thresholds stay road-test-tunable, no recompile of
# semantics).
LANE_UNKNOWN = 0     # yRel invalid (slot table off / fields 未定案 / NaN)
LANE_OWN = 1         # |yRel| inside the own-lane band
LANE_ADJACENT = 2    # beyond the band + hysteresis margin
LANE_WIDTH_M = 1.75  # own-lane half-width PLACEHOLDER (±1.75 m prior, 3.5)
LANE_HYST_M = 0.25   # band-exit margin: leave own-band past width+HYST (chatter guard)

# y supply pairing gate (byd-radar-y-wire-port 2.5 定门 2026-11-02, design D3):
# the slot row feeding y_rel is the one whose distance best matches the b7 lead
# (|d_row - b7*0.25| <= gate, nearest wins). 415-segment measurement: on real
# ACC-following segments <=1.5 m pairs 75% of cycles (<=3 m: 94%); across the
# whole archive presence-valid cycles pair 2.2% - the lead is often not
# slot-tracked, and no-pair degrades to LANE_UNKNOWN by design. 1.5 m ~ 6x the
# b7 quantization step and absorbs the F0-to-step closing slop.
GAP_Y_PAIR_GATE_M = 1.5
GAP_Y_FRESH_S = 0.15  # row latch older than this does not pair (F0 cadence 67 ms)


class RadarGapMonitor:
  def __init__(self):
    self.t = 0.0
    self.closing = False
    self.new_target = False
    self.visible = False
    self.lane = LANE_UNKNOWN        # classification of the current target
    self.degraded = 0               # closing episodes with no lane data (spec 计数)
    self._below_t = 0.0
    self._above_t = 0.0
    self._invisible_t = 0.0
    self._stale_t = 0.0
    self._edge_t = None       # last visible rising edge
    self._hist = deque()      # (t, r) for the descending-slope check

  @staticmethod
  def _boundary(cls: int) -> float:
    if cls >= 4:
      return BAND_BOUNDARY_DOWN[4]
    if cls == 3:
      return BAND_BOUNDARY_DOWN[3]
    return BAND_BOUNDARY_DOWN[2]

  def _classify_lane(self, y_rel, lane_width: float) -> int:
    """Own/adjacent classification with a hysteresis exit margin. Invalid yRel
    never guesses (spec: 禁止在无横向数据时猜测车道归属) -> UNKNOWN."""
    if y_rel is None or y_rel != y_rel:   # None / NaN -> degrade
      return LANE_UNKNOWN
    if self.lane == LANE_OWN:
      return LANE_OWN if abs(y_rel) <= lane_width + LANE_HYST_M else LANE_ADJACENT
    return LANE_OWN if abs(y_rel) <= lane_width else LANE_ADJACENT

  def update(self, dt: float, b7: int, presence: int, hud_cls: int,
             v_ego: float, frame_fresh: bool, y_rel=None,
             lane_width: float = LANE_WIDTH_M) -> None:
    self.t += dt

    was_closing = self.closing
    self.lane = self._classify_lane(y_rel, lane_width)

    if not frame_fresh:
      self._stale_t += dt
    else:
      self._stale_t = 0.0
    fresh = self._stale_t < STALE_TIME

    visible = bool(fresh and presence == PRESENCE_VISIBLE and
                   B7_LO <= b7 <= B7_HI)
    if visible and not self.visible:
      # rising edge only after a confirmed invisible period (④f: a flickering
      # presence nibble must not stamp every frame as "new")
      if self._invisible_t >= 0.5:
        self._edge_t = self.t
      self.new_target = False
    elif not visible:
      self._invisible_t += dt
    else:
      self._invisible_t = 0.0
    if visible and self.new_target and self._edge_t is not None and \
       (self.t - self._edge_t) > NEW_HOLD:
      self.new_target = False
    self.visible = visible
    if visible and self._edge_t is not None and (self.t - self._edge_t) <= NEW_HOLD:
      self.new_target = True

    r = None
    if visible and v_ego > V_EGO_MIN:
      r = b7 / v_ego
      self._hist.append((self.t, r))
      while self._hist and self.t - self._hist[0][0] > SLOPE_WINDOW + dt:
        self._hist.popleft()

    b = self._boundary(hud_cls)
    set_lim = b * (1.0 - SET_MARGIN)
    rel_lim = b * (1.0 - REL_MARGIN)

    below = r is not None and r < set_lim
    above = r is None or r > rel_lim   # no usable gap -> drift to release
    self._below_t = self._below_t + dt if below else 0.0
    self._above_t = self._above_t + dt if above else 0.0

    if not self.closing:
      old = self._hist[0][1] if self._hist else None
      descending = r is not None and old is not None and r < old - SLOPE_DROP
      if below and descending and self._below_t >= SET_TIME:
        self.closing = True
    else:
      if self._above_t >= REL_TIME or not visible:
        self.closing = False

    # 降级 telemetry (spec): a closing episode that fired without lane data -
    # the consumer treats it with the pre-lane conservative semantics
    if self.closing and not was_closing and self.lane == LANE_UNKNOWN:
      self.degraded += 1
