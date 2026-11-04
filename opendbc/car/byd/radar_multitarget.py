"""BYD Song Plus bus1 67 ms multi-target slot-table decoder (blocks B/C/D).

Structure per docs/byd-radar-slot67-2026-10-18.md (415-segment reverse):
  - 89 addresses broadcast the whole table every 67.16 ms (14.9 Hz); empty
    slots are transmitted as all-zero placeholder frames.
  - b0 = SAE J1850 CRC-8 (poly 0x1D, init 0xFF, no reflection, xorout 0xFF)
    over b1..b7 - verified 100% on the 415-segment archive and re-verified on
    the probe carrier (tools/byd/radar_probe_run.py CRC self-check).
  - b1 = global cycle counter, step 0x11, identical on every address of a
    cycle; it is the cycle-boundary key (no cross-cycle mixing).
  - Block C (0x380-0x397): 6 objects x 4 frames, stride 4. F0 carries the
    distance x11 = bits 16..26 of (b3&0xF)<<8|b2, decoded by the engine as
    d = 0.062319*x - 1.671 (PINNED 2026-10-31: probe ladder + vendor
    rd_spec_table g1/sig0 + R^2=1.0000 regression on the engine output; the
    1/32 slope-8 model is retired - its anchor was scene-dependent). F0 also
    carries the lateral code (bits 35..45) - PINNED 2026-11-02, see Y_* below.
  - Block B (0x340-0x34A minus 342/345/348): 4 objects x 2 frames, near-field
    coarse table, F0 distance = b3 at 1 dm (sparse 13-window fit).
  - Block D (0x3C8-0x3E3): 7 objects x 4 frames - semantics OPEN. The
    2026-10-31 probe campaign (tools/byd/radar_probe_run.py, device harness)
    found velocity-graded deltas on D bits 16-24 (vr/va, scale ~0.01439 m/s
    per LSB at bit 16) - HYPOTHESIS until the three supervisions (S1 vision
    cross-check, S2 turn compensation, S3 slot consistency) and the vendor
    rd_spec_table double-source agree; see the phase-1 RE document.

Decoding discipline (spec byd-radar-multitarget-decode):
  - Only pinned fields feed output points: C distance (2026-10-31) and C
    lateral code -> yRel (2026-11-02). Every other unpinned field stays
    NaN-isolated: the LAYOUT table below is intentionally empty until its
    定案 lands (HYPOTHESIS entries must never reach a point).
  - vRel comes from the in-slot distance differential. One count per 67 ms
    cycle is 0.062319/0.06716 = 0.93 m/s - the quantization floor that makes
    this a LOW-QUALITY source (D4: b7 stays the primary vRel; slot points
    are lateral context / secondary targets / cut-in capture).
  - Empty slots (all-zero payload) never produce points; range-edge counts
    (x==0xFFF for C, x==0xFF for B) are the ④a-style loss band -> invalid.
  - Track identity is slot algebra with distance-gated pairing: a target that
    moves between object rows keeps its trackId while |Δd| stays plausible;
    when pairing fails the old id dies and a new one is born (no reuse).
"""
from collections import Counter, deque

# --- transport layer (pinned) ------------------------------------------------
CRC_POLY = 0x1D
CRC_INIT = 0xFF
CRC_XOROUT = 0xFF
COUNTER_STEP = 0x11          # b1 global cycle counter step
CYCLE_PERIOD_S = 0.06716     # 14.9 Hz table scan

# --- block layout (pinned structure; fields per docs/byd-radar-slot67) -------
BLOCK_B_ADDRS = tuple(a for a in range(0x340, 0x34B) if a not in (0x342, 0x345, 0x348))
BLOCK_C_ADDRS = tuple(range(0x380, 0x398))
BLOCK_D_ADDRS = tuple(range(0x3C8, 0x3E4))
HEADER_ADDR = 0x301
SLOT_ADDRS = frozenset(BLOCK_B_ADDRS + BLOCK_C_ADDRS + BLOCK_D_ADDRS)
BLOCK_C_OBJ_FRAMES = 4       # F0..F3 per object row
BLOCK_D_OBJ_FRAMES = 4
BLOCK_B_OBJ_FRAMES = 2
BLOCK_C_OBJECTS = len(BLOCK_C_ADDRS) // BLOCK_C_OBJ_FRAMES    # 6
BLOCK_D_OBJECTS = len(BLOCK_D_ADDRS) // BLOCK_D_OBJ_FRAMES    # 7
BLOCK_B_OBJECTS = len(BLOCK_B_ADDRS) // BLOCK_B_OBJ_FRAMES    # 4

# distance decode for block C - PINNED 2026-10-31 (triple-sourced, task 1.3):
#   d = C_DIST_SCALE * x11 + C_DIST_OFFSET, x11 = bits 16..26 of the F0 frame
#   (b2 = low byte, b3 low nibble = high 4 bits; bit 27 is a state/overflow
#   bit the engine treats as kill). Sources: (a) probe delta ladder
#   0.062319*2^k over bits 16-26 (tools/byd/radar_probe_run.py campaign),
#   (b) vendor rd_spec_table g1/sig0 (start 16, len 11, scale 0.062319,
#   offset -1.671) - exact match, (c) OLS of the engine's own slot-0 output
#   against the wire x over 221 rows: d = 0.062319*x - 1.67, R^2 = 1.0000.
#   The older 1/32 m model (docs/byd-radar-slot67-2026-10-18.md) leaned on a
#   constant x-vs-b7 slope that the 2026-10-31 re-measure shows to be
#   scene-dependent (5-10 across windows) - retired as the unit anchor; the
#   physical meters truth gets its final seal from the V2 stop-gap check.
C_DIST_SCALE = 0.062319      # m per count (engine decode)
C_DIST_OFFSET = -1.671       # m
C_DIST_MIN_M = 0.0           # decode floor (x11 below ~27 decodes negative)
C_DIST_MAX_M = 125.9         # 0.062319*2047 - 1.671
B_DIST_SCALE = 0.1           # b3, 8 bit, range 25.5 m near-field coarse

# lateral y decode - PINNED 2026-11-02 (byd-radar-y-wire-port 2.7 定案).
#   code = bits 35..45 of the C-F0 frame, 11-bit offset binary, zero at 1024:
#     code11 = ((b4 >> 3) | (b5 << 5)) & 0x7FF;  code = code11 - 1024
#   y = (base + Y_D_OFF) * Y_K * (code - Y_C0),  base = x11 * C_DIST_SCALE.
#   Sources: (a) vendor rd_spec_table g1/sig1 (start 35, len 11, scale 0.000501,
#   offset -0.505509 = -K*1009); (b) engine-label verification 427/427 records,
#   |Δy| median 11.8 mm with code and d exact (tools/byd/radar_y_wire_decode.py
#   vs radar_y_stream_labels.py verbatim vendor run); (c) vision aggregate gate
#   (design D4 per-track medians, vendor envelope 8-60 m): holdout median
#   |Δy| = 0.135 m, bias -0.006 m - gate 0.4/0.15 passed at 1/3 of it.
#   S1's affine kill of bits35-45 was a functional-form error: the code is
#   distance-modulated (y ∝ d·code), invisible to any affine test. Defaults
#   (c0, d_off) ARE the verdict constants - no installation re-fit is burned in
#   (the vendor learns the same two parameters online; road test re-checks).
Y_CODE_ZERO = 1024
Y_K = 0.00050091
Y_C0 = -15.0
Y_D_OFF = -1.671
Y_BAND_MIN_M = 8.0           # trusted band = vendor calibration envelope
Y_BAND_MAX_M = 60.0          # outside -> yRel None (NaN isolation semantics)
# C-F0 carrier addresses (rows 0-5); the gap-side y supply (carstate) parses
# these six messages as raw twins - factor-1, u-free discipline
SLOT_Y_ADDRS = tuple(range(0x380, 0x398, 4))

# track identity (radar_interface maps these onto RadarPoint.trackId; the
# 201+ base is namespaced away from the tang slots (101+) and HUD fallback (1))
TRACK_ID_BASE = 201

# pairing / lifetime tuning (cycle = 67 ms)
PAIR_MAX_JUMP_M = 4.0        # > 2 m/cycle at a 30 m/s close; generous v1 gate
TRACK_MISS_CYCLES = 8        # ~0.54 s without a paired observation -> die
VREL_DIFF_MIN_CYCLES = 2

# vRel quality: one count of x over one cycle (0.4665 m/s at C's 1/32 m scale)
VREL_QUANT_FLOOR_MPS = C_DIST_SCALE / CYCLE_PERIOD_S   # 0.47 m/s


def j1850_crc8(payload: bytes) -> int:
  """SAE J1850 CRC-8 over the given bytes (b1..b7 of a slot-table frame)."""
  crc = CRC_INIT
  for b in payload:
    crc ^= b
    for _ in range(8):
      crc = ((crc << 1) ^ CRC_POLY) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
  return crc ^ CRC_XOROUT


class _Track:
  __slots__ = ('track_id', 'd', 'cycle', 'misses', 'hist')

  def __init__(self, track_id: int, d: float, cycle: int):
    self.track_id = track_id
    self.d = d
    self.cycle = cycle
    self.misses = 0
    self.hist: deque[tuple[int, float]] = deque(maxlen=VREL_DIFF_MIN_CYCLES)


class MultitargetDecoder:
  """Cycle-assembling decoder for the bus1 slot table. Feed it the raw CAN
  packets each radar-interface step; it returns finished-cycle observations.
  Pure Python by design (D3): the cross-frame bit assembly has no single-frame
  DBC expression, and the unpinned fields must stay out of the DBC until
  定案. opendbc stays params-free - level gating lives in radar_interface."""

  def __init__(self):
    self.cycle_b1: int | None = None
    self.cycle_n = -1           # monotonic cycle number (timing/pairing key)
    self.buf: dict[tuple[str, int], dict[int, bytes]] = {}
    self.tracks: list[_Track] = []
    self.next_seq = 0           # track-id sequence, never reused
    self.stats: Counter = Counter()

  # --- transport ------------------------------------------------------------
  @staticmethod
  def _crc_ok(dat: bytes) -> bool:
    return len(dat) == 8 and dat[0] == j1850_crc8(dat[1:8])

  # --- cycle lifecycle ------------------------------------------------------
  def _start_cycle(self, b1: int) -> None:
    self.cycle_b1 = b1
    self.cycle_n += 1
    self.buf = {}

  def _finish_cycle(self) -> list[dict]:
    """Assemble complete objects of the finished cycle into observations."""
    out = []
    obs = []
    for (block, obj), frames in self.buf.items():
      n_frames = {'C': BLOCK_C_OBJ_FRAMES, 'D': BLOCK_D_OBJ_FRAMES,
                  'B': BLOCK_B_OBJ_FRAMES}[block]
      if len(frames) < n_frames:
        self.stats[f'{block}_incomplete'] += 1
        continue
      f0 = frames[0]
      if block == 'C':
        raw = ((f0[3] & 0xF) << 8) | f0[2]
        x11 = raw & 0x7FF                  # 11-bit distance; bit27 = state
        d = x11 * C_DIST_SCALE + C_DIST_OFFSET
        empty = raw == 0
        edge = (raw & 0x800) != 0 or not (C_DIST_MIN_M <= d <= C_DIST_MAX_M)
        d = d if not empty and not edge else None
        # lateral code (same F0 frame, bits 35..45): y_rel only inside the
        # trusted band - outside it the vision gate has no support (NaN stays)
        y_rel = None
        y_code = None
        if d is not None and Y_BAND_MIN_M <= d <= Y_BAND_MAX_M:
          code11 = ((f0[4] >> 3) | (f0[5] << 5)) & 0x7FF
          code = code11 - Y_CODE_ZERO
          y_code = code
          y_rel = (x11 * C_DIST_SCALE + Y_D_OFF) * Y_K * (code - Y_C0)
      elif block == 'B':
        raw = f0[3]
        empty = raw == 0
        edge = raw >= 0xFF
        d = raw * B_DIST_SCALE if not empty and not edge else None
      else:
        # block D: distance semantics OPEN - assemble/validate only
        self.stats['D_assembled'] += 1
        continue
      if empty:
        self.stats[f'{block}_empty'] += 1
        continue
      if edge:
        # ④a-style range-edge band: untrustworthy -> dropped outright (the
        # spec allows measured=False or discard; v1 discards)
        self.stats[f'{block}_edge'] += 1
        continue
      obs.append({'block': block, 'obj': obj, 'd': d, 'valid': True,
                  'y_rel': y_rel if block == 'C' else None,
                  'y_code': y_code if block == 'C' else None})

    # distance-gated pairing across object rows (slot algebra, spec scenario:
    # same target may move rows between cycles; continuity is distance-based)
    unmatched = list(self.tracks)
    for o in obs:
      if not o['valid']:
        continue
      best, best_dj = None, PAIR_MAX_JUMP_M
      for t in unmatched:
        dj = abs(o['d'] - t.d)
        if dj <= best_dj:
          best, best_dj = t, dj
      if best is not None:
        unmatched.remove(best)
        v_rel = None
        if best.hist and best.hist[-1][0] == self.cycle_n - 1:
          d_prev, t_prev = best.hist[-1][1], best.cycle
          v_rel = (o['d'] - d_prev) / (CYCLE_PERIOD_S * (self.cycle_n - t_prev))
        best.d = o['d']
        best.cycle = self.cycle_n
        best.misses = 0
        best.hist.append((self.cycle_n, o['d']))
        o['track_id'] = best.track_id
        o['v_rel'] = v_rel
      else:
        self.next_seq += 1
        t = _Track(TRACK_ID_BASE + self.next_seq, o['d'], self.cycle_n)
        t.hist.append((self.cycle_n, o['d']))
        self.tracks.append(t)
        unmatched.append(t)
        o['track_id'] = t.track_id
        o['v_rel'] = None
        self.stats['track_new'] += 1

    for t in unmatched:
      t.misses += 1
    self.tracks = [t for t in self.tracks if t.misses <= TRACK_MISS_CYCLES]
    self.stats['track_deaths'] += sum(1 for t in unmatched
                                      if t.misses > TRACK_MISS_CYCLES)

    self.stats['cycles'] += 1
    for o in obs:
      if o['valid']:
        out.append(o)
    return out

  # --- main entry -----------------------------------------------------------
  def update(self, can_packets) -> list[dict]:
    """can_packets: the RadarInterface update() payload - list of
    (t, [CanData(address, dat, src), ...]). Returns observations of every
    cycle completed during this call (0 or 1 in practice)."""
    done: list[dict] = []
    for _, frames in can_packets:
      for address, dat, src in frames:
        if src != 1 or address not in SLOT_ADDRS:
          continue
        if not self._crc_ok(dat):
          self.stats['crc_errors'] += 1
          continue
        b1 = dat[1]
        if self.cycle_b1 is None:
          self._start_cycle(b1)
        elif b1 != self.cycle_b1:
          if (b1 - self.cycle_b1) % 256 != COUNTER_STEP:
            # counter break: the in-flight cycle dies unannounced (spec)
            self.stats['counter_breaks'] += 1
            self.buf = {}
          else:
            done.extend(self._finish_cycle())
          self._start_cycle(b1)
        block = ('B' if address in BLOCK_B_ADDRS else
                 'C' if address in BLOCK_C_ADDRS else 'D')
        obj, frame_idx = divmod(
          address - {'B': 0x340, 'C': 0x380, 'D': 0x3C8}[block],
          {'B': 3, 'C': 4, 'D': 4}[block])
        self.buf.setdefault((block, obj), {})[frame_idx] = dat
    return done
