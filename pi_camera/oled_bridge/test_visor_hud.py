#!/usr/bin/env python3

import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

try:
    import numpy as np
except ModuleNotFoundError:
    np = None

import visor_hud
from visor_hud import (
    GuidedScanController,
    IR_MODE_CALIBRATION_NEEDED,
    IR_MODE_LOCK,
    IR_MODE_SCAN,
    IR_MODE_UNAVAILABLE,
    VisorHud,
    TargetHistory,
    TargetSighting,
    classify_ir_target,
    find_ir_target_vector,
    normalize_ir_strength,
    parse_line,
    parse_oled_status,
)


class _FakeRgbImage:
    shape = (480, 800, 3)


class _WritableThermalImage:
    shape = (480, 800, 3)
    dtype = "uint8"

    def __init__(self):
        self.assigned = None

    def __setitem__(self, key, value):
        self.assigned = (key, value)


class _FakeCv2:
    COLOR_BGR2GRAY = 7
    COLORMAP_INFERNO = 13

    def __init__(self):
        self.calls = []
        self.grey = object()
        self.balanced = object()
        self.thermal = object()

    def cvtColor(self, image, conversion):
        self.calls.append(("grey", image, conversion))
        return self.grey

    def equalizeHist(self, image):
        self.calls.append(("balance", image))
        return self.balanced

    def applyColorMap(self, image, palette):
        self.calls.append(("palette", image, palette))
        return self.thermal


def capture_rgb_commands(hud: VisorHud):
    """Render without numpy and return the HUD text/line draw commands."""

    texts = []
    lines = []
    hud._blend_rect = lambda *args: None
    hud._fill_rect = lambda *args: None
    hud._draw_text = lambda image, text, x, y, color, scale: texts.append(
        (text, color)
    )
    hud._draw_line = lambda image, start, end, color, width: lines.append(
        (start, end, color, width)
    )
    hud.draw_rgb(_FakeRgbImage())
    return texts, lines


def capture_rgb_details(hud: VisorHud):
    """Capture Mac HUD primitives without requiring a NumPy frame."""

    details = {"texts": [], "lines": [], "fills": [], "blends": []}
    hud._blend_rect = lambda image, rect, color, alpha: details["blends"].append(
        (rect, tuple(color), alpha)
    )
    hud._fill_rect = lambda image, rect, color: details["fills"].append(
        (rect, tuple(color))
    )
    hud._draw_text = lambda image, text, x, y, color, scale: details["texts"].append(
        (text, x, y, tuple(color), scale)
    )
    hud._draw_line = lambda image, start, end, color, width: details["lines"].append(
        (start, end, tuple(color), width)
    )
    hud.draw_rgb(_FakeRgbImage())
    return details


def feed_confirmed_fire(hud: VisorHud, line: str = "FLAME:4095") -> None:
    """Feed the consecutive strong readings required for a real fire lock."""

    for _ in range(visor_hud.FIRE_CONFIRM_SAMPLES):
        hud.feed_line(line)


class ParsingTests(unittest.TestCase):
    def test_guided_scan_starts_left_and_reverses_after_overshoot(self) -> None:
        scan = GuidedScanController()
        first = scan.update(100, fresh=True, calibrated=True, now=0.0)
        self.assertEqual((first.state, first.instruction), ("search", "left"))
        self.assertFalse(first.measured_bearing)

        for index, strength in enumerate((900, 1800, 1900), start=1):
            guidance = scan.update(
                strength, fresh=True, calibrated=True, now=float(index)
            )
        self.assertEqual(guidance.instruction, "left")
        for index in range(4, 9):
            guidance = scan.update(0, fresh=True, calibrated=True, now=float(index))
        self.assertEqual(guidance.instruction, "right")
        self.assertFalse(guidance.measured_bearing)

    def test_guided_scan_jitter_does_not_flip_direction(self) -> None:
        scan = GuidedScanController()
        for index, strength in enumerate((500, 650, 610, 670, 625, 680)):
            guidance = scan.update(
                strength, fresh=True, calibrated=True, now=index * 0.2
            )
        self.assertEqual(guidance.instruction, "left")

    def test_guided_scan_lock_hysteresis_and_one_shot_entry(self) -> None:
        scan = GuidedScanController()
        scan.update(0, fresh=True, calibrated=True, now=0.0)
        locked = scan.update(4095, fresh=True, calibrated=True, now=0.1)
        # EMA needs more than one high sample to cross the lock threshold.
        locked = scan.update(4095, fresh=True, calibrated=True, now=0.2)
        locked = scan.update(4095, fresh=True, calibrated=True, now=0.3)
        locked = scan.update(4095, fresh=True, calibrated=True, now=0.4)
        self.assertEqual((locked.state, locked.instruction), ("lock", "ahead"))
        self.assertTrue(locked.lock_entered)
        held = scan.update(1800, fresh=True, calibrated=True, now=0.5)
        self.assertEqual(held.state, "lock")
        self.assertFalse(held.lock_entered)

    def test_guided_scan_flat_signal_alternates_and_stale_resets(self) -> None:
        scan = GuidedScanController()
        scan.update(50, fresh=True, calibrated=True, now=0.0)
        right = scan.update(50, fresh=True, calibrated=True, now=5.1)
        self.assertEqual(right.instruction, "right")
        unavailable = scan.update(None, fresh=False, calibrated=True, now=5.2)
        self.assertEqual((unavailable.state, unavailable.instruction), ("unavailable", "none"))

    def test_guided_search_starts_only_after_a_credible_signal(self) -> None:
        weak = VisorHud(sensor_port=None, udp_port=18922, ir_calibrated=True)
        weak.feed_line("FLAME:400")
        weak_text, weak_lines = capture_rgb_commands(weak)
        self.assertIn("NO FIRE DETECTED", [text for text, _ in weak_text])
        self.assertFalse(any("GUIDED SEARCH" in text for text, _ in weak_text))
        self.assertFalse(any(width == 10 for _, _, _, width in weak_lines))

        meaningful = VisorHud(sensor_port=None, udp_port=18923, ir_calibrated=True)
        meaningful_line = f"FLAME:{visor_hud.GUIDED_SCAN_MIN_PEAK + 100:.0f}"
        meaningful.feed_line(meaningful_line)
        meaningful.feed_line(meaningful_line)
        hot_text, hot_lines = capture_rgb_commands(meaningful)
        self.assertTrue(any("GUIDED SEARCH - TURN LEFT" in text for text, _ in hot_text))
        self.assertFalse(any("DEG" in text for text, _ in hot_text if "GUIDED" in text))
        self.assertTrue(any(color == (74, 75, 226) and width == 10 for _, _, color, width in hot_lines))

    def test_one_strong_spike_never_starts_turn_guidance(self) -> None:
        hud = VisorHud(sensor_port=None, udp_port=18956, ir_calibrated=True)
        hud.feed_line("FLAME:0")
        hud.feed_line("FLAME:4095")

        for _ in range(2):
            labels = [command[0] for command in capture_rgb_details(hud)["texts"]]
            self.assertIn("NO FIRE DETECTED", labels)
            self.assertFalse(any("GUIDED SEARCH" in label for label in labels))
            self.assertFalse(any("TURN LEFT" in label or "TURN RIGHT" in label for label in labels))
            self.assertFalse(hud.snapshot().guided_scan.signal_confirmed)
            self.assertEqual(hud.draw_oled(bytes(1024)), bytes(1024))
            hud.feed_line("FLAME:0")

    def test_confirmed_guidance_releases_after_sustained_low_signal(self) -> None:
        hud = VisorHud(sensor_port=None, udp_port=18958, ir_calibrated=True)
        hud.feed_line("FLAME:1300")
        hud.feed_line("FLAME:1300")
        self.assertTrue(hud.snapshot().guided_scan.signal_confirmed)
        self.assertNotEqual(hud.draw_oled(bytes(1024)), bytes(1024))

        for _ in range(visor_hud.GUIDED_SCAN_RELEASE_SAMPLES + 4):
            hud.feed_line("FLAME:0")

        snapshot = hud.snapshot()
        self.assertFalse(snapshot.guided_scan.signal_confirmed)
        labels = [command[0] for command in capture_rgb_details(hud)["texts"]]
        self.assertIn("NO FIRE DETECTED", labels)
        self.assertFalse(any("GUIDED SEARCH" in label for label in labels))
        self.assertFalse(
            any("TURN LEFT" in label or "TURN RIGHT" in label for label in labels)
        )
        self.assertEqual(hud.draw_oled(bytes(1024)), bytes(1024))

    def test_measured_heading_vector_precedes_signal_only_search(self) -> None:
        hud = VisorHud(sensor_port=None, udp_port=18924, ir_calibrated=True)
        feed_confirmed_fire(hud, "HEADING:105,FLAME:4095")
        hud.feed_line("HEADING:75,FLAME:500")
        texts, lines = capture_rgb_commands(hud)
        self.assertIn("TURN RIGHT 30 DEG", [text for text, _ in texts])
        self.assertFalse(any("GUIDED SEARCH" in text for text, _ in texts))
        self.assertTrue(any(color == (74, 75, 226) and width == 10 for _, _, color, width in lines))

    def test_target_history_persists_latest_sighting(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "targets.sqlite3")
            history = TargetHistory(path)
            sighting = TargetSighting(
                seen_at_utc="2026-11-09T02:45:00.000Z",
                ir_raw=147.5,
                ir_strength=4095.0,
                heading=92.0,
                bearing=95.0,
            )
            history.record(sighting)
            self.assertEqual(history.count(), 1)
            self.assertEqual(TargetHistory(path).latest(), sighting)

    def test_target_history_logs_lock_transitions_not_frames(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "targets.sqlite3")
            hud = VisorHud(
                sensor_port=None,
                udp_port=18920,
                ir_ambient_raw=4095,
                ir_target_raw=147.5,
                target_db_path=path,
                target_log_cooldown_s=0,
            )
            feed_confirmed_fire(hud, "HEADING:90,FLAME:147.5")
            self.assertEqual(TargetHistory(path).count(), 1)
            first = hud.snapshot().last_target
            self.assertIsNotNone(first)
            self.assertEqual(first.heading, 90.0)
            self.assertEqual(first.bearing, 95.0)

            hud.feed_line("HEADING:90,FLAME:4095")
            feed_confirmed_fire(hud, "HEADING:90,FLAME:147.5")
            self.assertEqual(TargetHistory(path).count(), 2)

    def test_target_history_rate_limits_repeated_reacquisition(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "targets.sqlite3")
            hud = VisorHud(
                sensor_port=None,
                udp_port=18921,
                ir_ambient_raw=4095,
                ir_target_raw=147.5,
                target_db_path=path,
                target_log_cooldown_s=60,
            )
            feed_confirmed_fire(hud, "FLAME:147.5")
            hud.feed_line("FLAME:4095")
            feed_confirmed_fire(hud, "FLAME:147.5")
            self.assertEqual(TargetHistory(path).count(), 1)
            latest = TargetHistory(path).latest()
            self.assertIsNone(latest.heading)
            self.assertIsNone(latest.bearing)

    def test_new_confirmed_fire_replaces_stronger_previous_session_bearing(self) -> None:
        hud = VisorHud(sensor_port=None, udp_port=18957, ir_calibrated=True)
        feed_confirmed_fire(hud, "HEADING:105,FLAME:4095")
        hud.feed_line("HEADING:105,FLAME:0")

        # This distinct source is weaker than the first but is a new confirmed
        # lock, so its bearing must replace the previous session marker.
        feed_confirmed_fire(hud, "HEADING:45,FLAME:3000")
        snapshot = hud.snapshot()
        self.assertTrue(snapshot.fire_confirmed)
        self.assertEqual(snapshot.session_fire_bearing, 45.0)
        self.assertEqual(snapshot.session_fire_strength, 3000.0)

    def test_stored_bearing_is_display_only_and_never_drives_guidance(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "targets.sqlite3")
            history = TargetHistory(path)
            history.record(
                TargetSighting(
                    seen_at_utc="2026-11-09T02:45:00.000Z",
                    ir_raw=800.0,
                    ir_strength=3200.0,
                    heading=350.0,
                    bearing=355.0,
                )
            )
            hud = VisorHud(
                sensor_port=None,
                udp_port=18925,
                ir_calibrated=True,
                target_db_path=path,
            )
            self.assertEqual(hud.snapshot().guided_scan.instruction, "none")
            hud.feed_line("FLAME:400")
            self.assertEqual(hud.snapshot().guided_scan.instruction, "left")
            texts, _ = capture_rgb_commands(hud)
            labels = [text for text, _ in texts]
            self.assertIn(
                "LAST FIRE 02:45:00 UTC - REL BEARING 355 DEG", labels
            )
            self.assertIn("NO FIRE DETECTED", labels)
            self.assertFalse(any("GUIDED SEARCH" in text for text in labels))

    def test_last_target_without_heading_is_labeled_location_unknown(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "targets.sqlite3")
            TargetHistory(path).record(
                TargetSighting(
                    seen_at_utc="2026-11-09T02:46:00.000Z",
                    ir_raw=100.0,
                    ir_strength=4095.0,
                    heading=None,
                    bearing=None,
                )
            )
            hud = VisorHud(
                sensor_port=None,
                udp_port=18926,
                ir_calibrated=True,
                target_db_path=path,
            )
            texts, _ = capture_rgb_commands(hud)
            self.assertIn(
                "LAST FIRE 02:46:00 UTC - AHEAD - LOCATION UNKNOWN",
                [text for text, _ in texts],
            )

    def test_heading_tagged_ir_peak_reports_turn_degrees(self) -> None:
        slices = [-1.0] * 36
        slices[10] = 3200.0  # 105 degree slice center.

        right = find_ir_target_vector(75.0, slices, 500.0)
        self.assertIsNotNone(right)
        self.assertEqual(right.direction, "right")
        self.assertEqual(right.turn_degrees, 30)
        self.assertEqual(right.bearing, 105.0)

        ahead = find_ir_target_vector(95.0, slices, 3200.0)
        self.assertIsNotNone(ahead)
        self.assertEqual(ahead.direction, "ahead")

    def test_ir_turn_guidance_requires_heading_and_hot_scan_peak(self) -> None:
        weak = [-1.0] * 36
        weak[10] = visor_hud.FLAME_HOT - 1.0
        self.assertIsNone(find_ir_target_vector(75.0, weak, 100.0))
        self.assertIsNone(find_ir_target_vector(None, [3200.0] * 36, 3200.0))
        self.assertIsNone(find_ir_target_vector(75.0, [3200.0] * 36, None))

    def test_ir_turn_guidance_wraps_left_across_north(self) -> None:
        slices = [-1.0] * 36
        slices[35] = 3000.0  # 355 degree slice center.
        vector = find_ir_target_vector(15.0, slices, 500.0)
        self.assertIsNotNone(vector)
        self.assertEqual(vector.direction, "left")
        self.assertEqual(vector.turn_degrees, 20)

    def test_ir_scan_lock_state_is_shared_and_calibration_gated(self) -> None:
        self.assertEqual(
            classify_ir_target(
                fresh=False, calibrated=True, raw=None, strength=None
            ),
            IR_MODE_UNAVAILABLE,
        )
        self.assertEqual(
            classify_ir_target(
                fresh=True, calibrated=False, raw=4095, strength=None
            ),
            IR_MODE_CALIBRATION_NEEDED,
        )
        self.assertEqual(
            classify_ir_target(
                fresh=True,
                calibrated=True,
                raw=3900,
                strength=visor_hud.FLAME_HOT - 0.1,
            ),
            IR_MODE_SCAN,
        )
        self.assertEqual(
            classify_ir_target(
                fresh=True,
                calibrated=True,
                raw=500,
                strength=visor_hud.FLAME_HOT,
            ),
            IR_MODE_LOCK,
        )

    def test_two_point_ir_calibration_infers_both_polarities(self) -> None:
        self.assertAlmostEqual(normalize_ir_strength(3000, 4000, 2000), 2047.5)
        self.assertAlmostEqual(normalize_ir_strength(2000, 1000, 3000), 2047.5)
        self.assertEqual(normalize_ir_strength(4095, 4000, 2000), 0.0)
        self.assertEqual(normalize_ir_strength(0, 4000, 2000), 4095.0)

    def test_ir_calibration_rejects_no_response(self) -> None:
        with self.assertRaises(ValueError):
            normalize_ir_strength(4095, 4095, 4095)
        with self.assertRaises(ValueError):
            VisorHud(
                sensor_port=None,
                udp_port=18905,
                ir_ambient_raw=4095,
                ir_target_raw=4080,
            )

    def test_sensor_line(self) -> None:
        self.assertEqual(
            parse_line("HEADING:90,AIR:23.1,FLAME:820,GAS:1100"),
            {"HEADING": 90.0, "AIR": 23.1, "FLAME": 820.0, "GAS": 1100.0},
        )

    def test_bad_values_are_ignored(self) -> None:
        self.assertEqual(
            parse_line("HEADING:ERR,AIR:24.5,JUNK,BOOT:1"), {"AIR": 24.5}
        )

    def test_nonfinite_and_out_of_range_values_are_ignored(self) -> None:
        self.assertEqual(
            parse_line("HEADING:nan,AIR:inf,FLAME:-1,GAS:5000"), {}
        )
        self.assertEqual(parse_line("HEADING:360"), {"HEADING": 0.0})

    def test_receiver_status_line(self) -> None:
        self.assertEqual(
            parse_oled_status(
                "VSRSTAT:OK=405,SHOWN=404,CRC=1,LEN=2,PARTIAL=3,LAST=77"
            ),
            {
                "accepted_frames": 405,
                "displayed_frames": 404,
                "crc_errors": 1,
                "length_errors": 2,
                "packet_timeouts": 3,
                "last_sequence": 77,
            },
        )

    def test_timeout_status_is_backward_compatible(self) -> None:
        self.assertEqual(
            parse_oled_status(
                "Camera timeout (ok=12 shown=11 crc=0 len=0 partial=1 last=9)."
            )["displayed_frames"],
            11,
        )
        self.assertEqual(parse_oled_status("boot ok=12 shown=11"), {})

class RenderingTests(unittest.TestCase):
    def test_mac_thermal_look_uses_inferno_palette(self) -> None:
        fake_cv = _FakeCv2()
        image = _WritableThermalImage()
        with patch.object(visor_hud, "cv2", fake_cv):
            self.assertTrue(VisorHud._apply_thermal_look(image))

        self.assertEqual(
            [call[0] for call in fake_cv.calls],
            ["grey", "balance", "palette"],
        )
        self.assertIs(image.assigned[1], fake_cv.thermal)

    def test_mac_thermal_look_has_safe_module_fallback(self) -> None:
        image = _WritableThermalImage()
        with patch.object(visor_hud, "cv2", None):
            self.assertFalse(VisorHud._apply_thermal_look(image))
        self.assertIsNone(image.assigned)

    def test_mac_thermal_look_runs_before_hud_overlay(self) -> None:
        hud = VisorHud(sensor_port=None, udp_port=18942, ir_calibrated=True)
        hud.feed_line("FLAME:100")
        events = []
        hud._apply_thermal_look = lambda image: events.append("thermal") or True
        hud._blend_rect = lambda *args: events.append("overlay")
        hud._fill_rect = lambda *args: events.append("overlay")
        hud._draw_line = lambda *args: events.append("overlay")
        hud._draw_text = lambda *args: events.append("overlay")
        hud.draw_rgb(_FakeRgbImage())
        self.assertEqual(events[0], "thermal")
        self.assertIn("overlay", events[1:])

    def test_oled_renderer_never_invokes_mac_thermal_stage(self) -> None:
        hud = VisorHud(sensor_port=None, udp_port=18941, ir_calibrated=True)
        hud.feed_line("FLAME:400")
        hud._apply_thermal_look = lambda image: self.fail(
            "OLED must not invoke Mac thermal rendering"
        )
        self.assertEqual(len(hud.draw_oled(bytes(1024))), 1024)

    @staticmethod
    def _legacy_strip_fills(details):
        """Return the 800x480 heading-strip sector rectangles only."""

        return [
            (rect, color)
            for rect, color in details["fills"]
            if rect[1] == 22 and rect[3] == 46
        ]

    def test_mac_heading_strip_maps_scanned_strength_blue_to_red(self) -> None:
        hud = VisorHud(
            sensor_port=None,
            udp_port=18930,
            ir_calibrated=True,
        )
        hud.feed_line("HEADING:90,FLAME:0")
        feed_confirmed_fire(hud, "HEADING:100,FLAME:4095")
        details = capture_rgb_details(hud)
        strip_colors = [color for _, color in self._legacy_strip_fills(details)]

        self.assertIn(VisorHud._heat_color(0.0), strip_colors)
        self.assertTrue(
            any(color[2] > 210 and color[0] < 100 for color in strip_colors)
        )
        labels = [command[0] for command in details["texts"]]
        self.assertFalse(any("IR HEAT PROXY" in label for label in labels))

    def test_mac_heading_strip_scrolls_and_fades_previous_fire_peak(self) -> None:
        hud = VisorHud(
            sensor_port=None,
            udp_port=18931,
            ir_calibrated=True,
        )
        feed_confirmed_fire(hud, "HEADING:90,FLAME:4095")
        original_peak = hud.slices[9]

        # Five seconds of decay leaves a visible but fainter remembered peak.
        # Turning 50 degrees moves that peak left while the newly scanned
        # low-signal sector appears blue just right of the centre marker.
        hud.last_fade -= 5.0
        hud.feed_line("HEADING:140,FLAME:0")
        details = capture_rgb_details(hud)
        self.assertGreater(hud.slices[9], 0.0)
        self.assertLess(hud.slices[9], original_peak)

        fills = self._legacy_strip_fills(details)
        cold = VisorHud._heat_color(0.0)
        gray = (100, 100, 100)
        self.assertTrue(
            any((rect[0] + rect[2]) // 2 > 400 and color == cold for rect, color in fills)
        )
        remembered = [
            color
            for rect, color in fills
            if (rect[0] + rect[2]) // 2 < 200 and color not in (cold, gray)
        ]
        self.assertTrue(remembered)
        self.assertNotIn(VisorHud._heat_color(4095.0), remembered)

    def test_mac_no_heading_does_not_show_scalar_hot_cold_fallback(self) -> None:
        hud = VisorHud(
            sensor_port=None,
            udp_port=18932,
            ir_ambient_raw=4095.0,
            ir_target_raw=147.5,
        )
        hud.feed_line("FLAME:2500")
        labels = [command[0] for command in capture_rgb_details(hud)["texts"]]
        self.assertFalse(any("IR HEAT PROXY" in label for label in labels))
        self.assertNotIn("COLD", labels)
        self.assertNotIn("HOT", labels)

    def test_mac_live_fire_signal_draws_only_a_local_heat_patch(self) -> None:
        hud = VisorHud(
            sensor_port=None,
            udp_port=18935,
            ir_ambient_raw=4095.0,
            ir_target_raw=147.5,
        )
        feed_confirmed_fire(hud, "FLAME:147.5")
        details = capture_rgb_details(hud)
        thermal = [
            (rect, color, alpha)
            for rect, color, alpha in details["blends"]
            if color != (0, 0, 0)
        ]
        self.assertTrue(thermal)
        # At forward lock the synthetic heat stays around the camera centre,
        # never becoming a whole-screen color wash.
        self.assertTrue(
            all(300 <= rect[0] < rect[2] <= 500 for rect, _, _ in thermal)
        )
        self.assertTrue(
            all(145 <= rect[1] < rect[3] <= 305 for rect, _, _ in thermal)
        )
        self.assertTrue(all(alpha < 0.35 for _, _, alpha in thermal))
        labels = [command[0] for command in details["texts"]]
        self.assertIn("FIRE", labels)
        self.assertNotIn("PREVIOUS FIRE DETECTED HERE", labels)

    def test_fire_lock_requires_three_consecutive_strong_samples(self) -> None:
        hud = VisorHud(sensor_port=None, udp_port=18953, ir_calibrated=True)

        hud.feed_line("FLAME:4095")
        hud.feed_line("FLAME:4095")
        self.assertFalse(hud.snapshot().fire_confirmed)
        self.assertIsNone(hud.snapshot().last_target)
        labels = [command[0] for command in capture_rgb_details(hud)["texts"]]
        self.assertNotIn("FIRE", labels)

        # A low sample breaks the sequence; two more highs still do not lock.
        hud.feed_line("FLAME:0")
        hud.feed_line("FLAME:4095")
        hud.feed_line("FLAME:4095")
        self.assertFalse(hud.snapshot().fire_confirmed)
        self.assertIsNone(hud.snapshot().last_target)

        hud.feed_line("FLAME:4095")
        snapshot = hud.snapshot()
        self.assertTrue(snapshot.fire_confirmed)
        self.assertIsNotNone(snapshot.last_target)
        labels = [command[0] for command in capture_rgb_details(hud)["texts"]]
        self.assertIn("FIRE", labels)

    def test_live_fire_label_wins_then_becomes_previous_at_same_bearing(self) -> None:
        hud = VisorHud(sensor_port=None, udp_port=18954, ir_calibrated=True)
        feed_confirmed_fire(hud, "HEADING:105,FLAME:4095")

        # Hysteresis keeps this confirmed source live even below the entry
        # threshold; it must still be FIRE, not a previous marker.
        hud.feed_line("HEADING:105,FLAME:2500")
        live_labels = [
            command[0] for command in capture_rgb_details(hud)["texts"]
        ]
        self.assertIn("FIRE", live_labels)
        self.assertNotIn("PREVIOUS FIRE DETECTED HERE", live_labels)

        hud.feed_line("HEADING:105,FLAME:0")
        previous_labels = [
            command[0] for command in capture_rgb_details(hud)["texts"]
        ]
        self.assertNotIn("FIRE", previous_labels)
        self.assertIn("PREVIOUS FIRE DETECTED HERE", previous_labels)

    def test_live_fire_and_different_previous_bearing_are_both_labeled(self) -> None:
        hud = VisorHud(sensor_port=None, udp_port=18955, ir_calibrated=True)
        feed_confirmed_fire(hud, "HEADING:105,FLAME:4095")

        # Keep the confirmed lock through hysteresis while looking 30 degrees
        # away. The current forward source and the prior bearing are distinct.
        hud.feed_line("HEADING:75,FLAME:2500")
        details = capture_rgb_details(hud)
        labels = [command[0] for command in details["texts"]]
        self.assertIn("FIRE", labels)
        self.assertIn("PREVIOUS FIRE DETECTED HERE", labels)
        previous_blobs = [
            rect
            for rect, color, _ in details["blends"]
            if color == (100, 190, 255)
        ]
        self.assertTrue(previous_blobs)
        self.assertTrue(
            all((rect[0] + rect[2]) / 2 > 500 for rect in previous_blobs)
        )

    def test_mac_remembered_fire_patch_moves_with_bearing_and_fades(self) -> None:
        hud = VisorHud(sensor_port=None, udp_port=18936, ir_calibrated=True)
        feed_confirmed_fire(hud, "HEADING:105,FLAME:4095")
        live = capture_rgb_details(hud)
        live_alpha = max(
            alpha
            for _, color, alpha in live["blends"]
            if color != (0, 0, 0)
        )

        # Turn 30 degrees left of the remembered source and lose the live
        # signal. The faint patch should remain to the right of centre.
        hud.feed_line("HEADING:75,FLAME:0")
        remembered = capture_rgb_details(hud)
        thermal = [
            (rect, alpha)
            for rect, color, alpha in remembered["blends"]
            if color != (0, 0, 0)
        ]
        self.assertTrue(thermal)
        self.assertTrue(all((rect[0] + rect[2]) / 2 > 500 for rect, _ in thermal))
        self.assertLess(max(alpha for _, alpha in thermal), live_alpha)
        labels = [command[0] for command in remembered["texts"]]
        self.assertIn("PREVIOUS FIRE DETECTED HERE", labels)
        self.assertNotIn("FIRE", labels)

    def test_mac_light_orange_fire_memory_survives_strip_fade(self) -> None:
        hud = VisorHud(sensor_port=None, udp_port=18940, ir_calibrated=True)
        feed_confirmed_fire(hud, "HEADING:105,FLAME:4095")
        hud.feed_line("HEADING:45,FLAME:0")

        # Erase the temporary scan strip, then look back at the acquired world
        # bearing. Session memory must remain independently as light orange.
        hud.slices = [-1.0] * len(hud.slices)
        hud.feed_line("HEADING:105,FLAME:0")
        details = capture_rgb_details(hud)
        orange = [
            (rect, alpha)
            for rect, color, alpha in details["blends"]
            if color == (100, 190, 255)
        ]
        self.assertTrue(orange)
        self.assertTrue(all((rect[0] + rect[2]) / 2 == 400 for rect, _ in orange))
        self.assertLessEqual(max(alpha for _, alpha in orange), 0.22)
        labels = [command[0] for command in details["texts"]]
        self.assertIn("PREVIOUS FIRE DETECTED HERE", labels)
        self.assertNotIn("FIRE", labels)

    def test_persistent_database_target_never_becomes_session_orange_patch(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "targets.sqlite3")
            TargetHistory(path).record(
                TargetSighting(
                    seen_at_utc="2026-11-09T03:20:00.000Z",
                    ir_raw=147.5,
                    ir_strength=4095.0,
                    heading=105.0,
                    bearing=105.0,
                )
            )
            hud = VisorHud(
                sensor_port=None,
                udp_port=18943,
                ir_calibrated=True,
                target_db_path=path,
            )
            hud.feed_line("HEADING:105,FLAME:0")
            details = capture_rgb_details(hud)
            self.assertFalse(
                any(
                    color == (100, 190, 255)
                    for _, color, _ in details["blends"]
                )
            )

    def test_mac_does_not_place_off_camera_fire_patch(self) -> None:
        hud = VisorHud(sensor_port=None, udp_port=18939, ir_calibrated=True)
        feed_confirmed_fire(hud, "HEADING:105,FLAME:4095")

        # The remembered peak is now 60 degrees to the right, beyond the
        # camera's roughly 62-degree horizontal field of view. Navigation may
        # point right, but the video must not pretend the source is in-frame.
        hud.feed_line("HEADING:45,FLAME:0")
        details = capture_rgb_details(hud)
        self.assertFalse(
            any(color != (0, 0, 0) for _, color, _ in details["blends"])
        )

    def test_mac_weak_signal_does_not_fake_a_hot_area(self) -> None:
        hud = VisorHud(sensor_port=None, udp_port=18937, ir_calibrated=True)
        hud.feed_line("FLAME:100")
        details = capture_rgb_details(hud)
        self.assertFalse(
            any(color != (0, 0, 0) for _, color, _ in details["blends"])
        )

    def test_mac_fresh_calibrated_low_signal_says_no_fire(self) -> None:
        hud = VisorHud(sensor_port=None, udp_port=18944, ir_calibrated=True)
        hud.feed_line("FLAME:599")
        labels = [command[0] for command in capture_rgb_details(hud)["texts"]]

        self.assertIn("NO FIRE DETECTED", labels)
        self.assertFalse(any("GUIDED SEARCH" in label for label in labels))
        self.assertFalse(any("TURN LEFT" in label or "TURN RIGHT" in label for label in labels))

        camera = bytes((index * 37) & 0xFF for index in range(1024))
        self.assertEqual(hud.draw_oled(camera), camera)

    def test_no_fire_with_heading_has_no_mac_or_oled_turn_arrow(self) -> None:
        """Regression for the live heading-strip view shown by the wearer."""

        hud = VisorHud(sensor_port=None, udp_port=18952, ir_calibrated=True)
        hud.feed_line("HEADING:120,FLAME:599")
        details = capture_rgb_details(hud)
        labels = [command[0] for command in details["texts"]]

        self.assertIn("NO FIRE DETECTED", labels)
        self.assertFalse(any("GUIDED SEARCH" in label for label in labels))
        self.assertFalse(any("TURN LEFT" in label or "TURN RIGHT" in label for label in labels))
        self.assertFalse(any(line[3] == 10 for line in details["lines"]))

        camera = bytes((index * 53) & 0xFF for index in range(1024))
        self.assertEqual(hud.draw_oled(camera), camera)

    def test_mac_no_fire_uses_strict_minimum_peak_boundary(self) -> None:
        below = VisorHud(sensor_port=None, udp_port=18945, ir_calibrated=True)
        below.feed_line(
            f"FLAME:{visor_hud.GUIDED_SCAN_MIN_PEAK - 1:.0f}"
        )
        at_threshold = VisorHud(
            sensor_port=None, udp_port=18946, ir_calibrated=True
        )
        at_threshold.feed_line(
            f"FLAME:{visor_hud.GUIDED_SCAN_MIN_PEAK:.0f}"
        )
        at_threshold.feed_line(
            f"FLAME:{visor_hud.GUIDED_SCAN_MIN_PEAK:.0f}"
        )

        below_labels = [
            command[0] for command in capture_rgb_details(below)["texts"]
        ]
        threshold_labels = [
            command[0]
            for command in capture_rgb_details(at_threshold)["texts"]
        ]
        self.assertIn("NO FIRE DETECTED", below_labels)
        self.assertNotIn("NO FIRE DETECTED", threshold_labels)
        self.assertIn(
            f"SCAN RAW {visor_hud.GUIDED_SCAN_MIN_PEAK:.0f}",
            threshold_labels,
        )
        self.assertEqual(below.draw_oled(bytes(1024)), bytes(1024))
        self.assertNotEqual(at_threshold.draw_oled(bytes(1024)), bytes(1024))

    def test_mac_recent_credible_peak_prevents_no_fire_wording(self) -> None:
        hud = VisorHud(sensor_port=None, udp_port=18951, ir_calibrated=True)
        hud.feed_line("FLAME:4095")
        hud.feed_line("FLAME:4095")
        hud.feed_line("FLAME:100")
        snapshot = hud.snapshot()
        labels = [command[0] for command in capture_rgb_details(hud)["texts"]]

        self.assertGreaterEqual(
            snapshot.guided_scan.peak_strength, visor_hud.GUIDED_SCAN_MIN_PEAK
        )
        self.assertNotIn("NO FIRE DETECTED", labels)

    def test_mac_no_fire_requires_a_fresh_calibrated_sensor_sample(self) -> None:
        unavailable = VisorHud(
            sensor_port=None, udp_port=18947, ir_calibrated=True
        )
        stale = VisorHud(sensor_port=None, udp_port=18948, ir_calibrated=True)
        stale.feed_line("HEADING:90,FLAME:100")
        stale.field_updated["flame"] = 0.0
        uncalibrated = VisorHud(sensor_port=None, udp_port=18949)
        uncalibrated.feed_line("FLAME:100")

        for hud in (unavailable, stale, uncalibrated):
            labels = [
                command[0] for command in capture_rgb_details(hud)["texts"]
            ]
            self.assertNotIn("NO FIRE DETECTED", labels)

    def test_mac_saved_fire_history_alone_never_claims_no_fire(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "targets.sqlite3")
            TargetHistory(path).record(
                TargetSighting(
                    seen_at_utc="2026-11-09T03:25:00.000Z",
                    ir_raw=4095.0,
                    ir_strength=4095.0,
                    heading=90.0,
                    bearing=95.0,
                )
            )
            hud = VisorHud(
                sensor_port=None,
                udp_port=18950,
                ir_calibrated=True,
                target_db_path=path,
            )
            labels = [
                command[0] for command in capture_rgb_details(hud)["texts"]
            ]

            self.assertNotIn("NO FIRE DETECTED", labels)
            self.assertTrue(any(label.startswith("LAST FIRE") for label in labels))

    def test_mac_uses_fire_wording_for_wearer_facing_target_text(self) -> None:
        hud = VisorHud(sensor_port=None, udp_port=18938, ir_calibrated=True)
        feed_confirmed_fire(hud)
        labels = [command[0] for command in capture_rgb_details(hud)["texts"]]
        self.assertIn("FIRE TARGET LOCK - SIG 4095", labels)
        self.assertIn("FIRE TARGET", labels)
        self.assertNotIn("NO FIRE DETECTED", labels)
        self.assertFalse(any(label.startswith("IR ") for label in labels))

    def test_mac_sensor_cards_remain_visible_with_partial_hardware(self) -> None:
        hud = VisorHud(
            sensor_port=None,
            udp_port=18934,
            ir_ambient_raw=4095.0,
            ir_target_raw=147.5,
        )
        hud.feed_line("FLAME:2500,AIR:23.1,GAS:1100")
        labels = [command[0] for command in capture_rgb_details(hud)["texts"]]
        for title in ("FIRE TARGET", "HEADING", "AIR TEMP", "GAS"):
            self.assertIn(title, labels)
        self.assertIn("YAW --", labels)
        self.assertIn("23.1 C", labels)
        self.assertIn("1100", labels)
        self.assertIn("WARMUP", labels)

    def test_oled_link_evidence_is_kept_separate_from_sensor_data(self) -> None:
        hud = VisorHud(sensor_port=None, udp_port=18891)
        hud.feed_line("READY VSR1: receiver online")
        hud.feed_line(
            "VSRSTAT:OK=120,SHOWN=119,CRC=0,LEN=0,PARTIAL=0,LAST=118"
        )
        link = hud.oled_link_snapshot()
        self.assertTrue(link.ready)
        self.assertTrue(link.fresh)
        self.assertEqual(link.accepted_frames, 120)
        self.assertEqual(link.displayed_frames, 119)
        self.assertEqual(link.last_sequence, 118)
        self.assertFalse(hud.snapshot().fresh)

    def test_real_values_reach_both_renderers(self) -> None:
        hud = VisorHud(
            sensor_port=None, udp_port=18892, ir_calibrated=True
        )
        hud.feed_line("HEADING:30,AIR:23.1,FLAME:2600,GAS:1100")
        hud.feed_line("HEADING:30,AIR:23.1,FLAME:2600,GAS:1100")
        hud.feed_line("HEADING:90,FLAME:400")
        snapshot = hud.snapshot()
        self.assertTrue(snapshot.fresh)
        self.assertEqual(snapshot.heading, 90.0)

        oled = hud.draw_oled(bytes(1024))
        self.assertEqual(len(oled), 1024)
        self.assertNotEqual(oled, bytes(1024))

        if np is not None:
            frame = np.zeros((480, 800, 3), dtype=np.uint8)
            hud.draw_rgb(frame)
            self.assertGreater(int(frame.sum()), 0)

    def test_ir_only_line_from_oled_s3_is_live(self) -> None:
        hud = VisorHud(
            sensor_port=None, udp_port=18894, ir_calibrated=True
        )
        hud.feed_line("FLAME:1732")
        hud.feed_line("FLAME:1732")
        snapshot = hud.snapshot()
        self.assertTrue(snapshot.fresh)
        self.assertEqual(snapshot.flame, 1732.0)
        self.assertIsNone(snapshot.heading)

        # A weak calibrated single-channel reading starts the guided search.
        oled = hud.draw_oled(bytes(1024))
        no_sensor = VisorHud(sensor_port=None, udp_port=18901).draw_oled(bytes(1024))
        self.assertNotEqual(oled, no_sensor)

        if np is not None:
            frame = np.zeros((480, 800, 3), dtype=np.uint8)
            hud.draw_rgb(frame)
            self.assertGreater(int(frame[:80].sum()), 0)

    def test_strong_forward_ir_gets_target_lock_without_heading(self) -> None:
        locked = VisorHud(
            sensor_port=None, udp_port=18902, ir_calibrated=True
        )
        feed_confirmed_fire(locked)
        scanning = VisorHud(
            sensor_port=None, udp_port=18903, ir_calibrated=True
        )
        scanning.feed_line("FLAME:400")

        locked_oled = locked.draw_oled(bytes(1024))
        scanning_oled = scanning.draw_oled(bytes(1024))
        # Lock needs no turn cue, and a clearly low reading must not invent a
        # search direction. Both leave processed vision unobstructed.
        self.assertEqual(locked_oled, bytes(1024))
        self.assertEqual(scanning_oled, bytes(1024))

        if np is not None:
            locked_frame = np.zeros((480, 800, 3), dtype=np.uint8)
            scanning_frame = np.zeros((480, 800, 3), dtype=np.uint8)
            locked.draw_rgb(locked_frame)
            scanning.draw_rgb(scanning_frame)
            self.assertGreater(
                int(locked_frame[155:175, 285:320].sum()),
                int(scanning_frame[155:175, 285:320].sum()),
            )

    def test_saturated_uncalibrated_ir_never_claims_target_lock(self) -> None:
        uncalibrated = VisorHud(sensor_port=None, udp_port=18904)
        uncalibrated.feed_line("FLAME:4095")
        rendered = uncalibrated.draw_oled(bytes(1024))
        # Before polarity is known, the OLED must not invent any guidance or
        # add calibration text over the processed camera image.
        self.assertEqual(rendered, bytes(1024))

        if np is not None:
            frame = np.zeros((480, 800, 3), dtype=np.uint8)
            # This assertion isolates target overlays. The production Mac
            # renderer intentionally applies a full-frame thermal palette.
            with patch.object(
                uncalibrated, "_apply_thermal_look", return_value=False
            ):
                uncalibrated.draw_rgb(frame)
            self.assertEqual(int(frame[155:175, 285:320].sum()), 0)

    def test_active_low_calibration_produces_strength_and_heat_samples(self) -> None:
        hud = VisorHud(
            sensor_port=None,
            udp_port=18906,
            ir_ambient_raw=4000,
            ir_target_raw=1000,
        )
        hud.feed_line("HEADING:90,FLAME:2500")
        snapshot = hud.snapshot()
        self.assertTrue(snapshot.ir_calibrated)
        self.assertEqual(snapshot.ir_raw, 2500.0)
        self.assertAlmostEqual(snapshot.ir_strength, 2047.5)
        self.assertAlmostEqual(snapshot.slices[9], 2047.5, delta=1.0)

    def test_flame_does_not_reuse_stale_heading(self) -> None:
        hud = VisorHud(sensor_port=None, udp_port=18895, ir_calibrated=True)
        hud.feed_line("HEADING:90,FLAME:800")
        self.assertAlmostEqual(hud.snapshot().slices[9], 800.0, delta=1.0)

        hud.field_updated["heading"] = 0.0
        hud.feed_line("FLAME:3000")
        snapshot = hud.snapshot()
        self.assertIsNone(snapshot.heading)
        self.assertLess(snapshot.slices[9], 3000.0)

    def test_no_sensor_state_preserves_camera_without_text(self) -> None:
        hud = VisorHud(sensor_port=None, udp_port=18893)
        camera = bytes((index * 37) & 0xFF for index in range(1024))
        self.assertEqual(hud.draw_oled(camera), camera)

    def test_oled_excludes_air_and_gas_detail(self) -> None:
        camera = bytes((index * 37) & 0xFF for index in range(1024))
        baseline = VisorHud(
            sensor_port=None, udp_port=18896, ir_calibrated=True
        )
        baseline.feed_line("FLAME:400")
        expected = baseline.draw_oled(camera)

        with_details = VisorHud(
            sensor_port=None, udp_port=18897, ir_calibrated=True
        )
        with_details.feed_line("FLAME:400,AIR:24,GAS:1120")
        self.assertEqual(with_details.draw_oled(camera), expected)

    def test_oled_unavailable_retains_every_processed_camera_pixel(self) -> None:
        hud = VisorHud(sensor_port=None, udp_port=18898)
        camera = bytes((index * 37) & 0xFF for index in range(1024))
        self.assertEqual(hud.draw_oled(camera), camera)

    def test_oled_guided_search_is_exactly_one_arrow_without_text(self) -> None:
        hud = VisorHud(
            sensor_port=None, udp_port=18900, ir_calibrated=True
        )
        guidance_line = f"FLAME:{visor_hud.GUIDED_SCAN_MIN_PEAK + 100:.0f}"
        hud.feed_line(guidance_line)
        hud.feed_line(guidance_line)
        expected = bytearray(1024)
        VisorHud._oled_direction_arrow(expected, left=True)
        self.assertEqual(hud.draw_oled(bytes(1024)), bytes(expected))

        # A side arrow and its black halo do not replace camera pixels with
        # labels in the top or bottom rows.
        camera = bytes((index * 37) & 0xFF for index in range(1024))
        rendered = hud.draw_oled(camera)
        self.assertEqual(rendered[:128], camera[:128])
        self.assertEqual(rendered[7 * 128 :], camera[7 * 128 :])

        # The turn cue stays at the edge; the processed camera remains the
        # dominant OLED image rather than becoming a full-screen arrow.
        changed = set()
        for y in range(64):
            for x in range(128):
                mask = 1 << (y & 7)
                index = (y >> 3) * 128 + x
                if (rendered[index] & mask) != (camera[index] & mask):
                    changed.add((x, y))
        self.assertTrue(changed)
        self.assertTrue(
            all(x <= 33 and 16 <= y <= 48 for x, y in changed)
        )

    def test_oled_measured_vector_selects_only_the_correct_side_arrow(self) -> None:
        left = VisorHud(sensor_port=None, udp_port=18927, ir_calibrated=True)
        feed_confirmed_fire(left, "HEADING:35,FLAME:4095")
        left.feed_line("HEADING:95,FLAME:400")
        expected_left = bytearray(1024)
        VisorHud._oled_direction_arrow(expected_left, left=True)
        self.assertEqual(left.draw_oled(bytes(1024)), bytes(expected_left))

        right = VisorHud(sensor_port=None, udp_port=18928, ir_calibrated=True)
        feed_confirmed_fire(right, "HEADING:155,FLAME:4095")
        right.feed_line("HEADING:95,FLAME:400")
        expected_right = bytearray(1024)
        VisorHud._oled_direction_arrow(expected_right, left=False)
        self.assertEqual(right.draw_oled(bytes(1024)), bytes(expected_right))
        self.assertNotEqual(expected_left, expected_right)

        def lit_pixels(payload):
            return {
                (x, y)
                for y in range(64)
                for x in range(128)
                if payload[(y >> 3) * 128 + x] & (1 << (y & 7))
            }

        left_pixels = lit_pixels(expected_left)
        right_pixels = lit_pixels(expected_right)
        self.assertLessEqual(max(x for x, _ in left_pixels), 32)
        self.assertGreaterEqual(min(x for x, _ in right_pixels), 95)
        self.assertEqual(
            right_pixels, {(127 - x, y) for x, y in left_pixels}
        )

    def test_oled_lock_leaves_processed_vision_unobstructed(self) -> None:
        hud = VisorHud(
            sensor_port=None, udp_port=18929, ir_calibrated=True
        )
        feed_confirmed_fire(hud)
        camera = bytes((index * 37) & 0xFF for index in range(1024))
        self.assertEqual(hud.draw_oled(camera), camera)

    @unittest.skipIf(np is None, "numpy is unavailable")
    def test_mac_scan_state_keeps_central_camera_clear(self) -> None:
        hud = VisorHud(
            sensor_port=None,
            udp_port=18907,
            ir_calibrated=True,
        )
        hud.feed_line("FLAME:400")
        frame = np.full((480, 800, 3), 31, dtype=np.uint8)
        before = frame[120:330, 120:680].copy()
        # Check only the scan-state HUD geometry; the thermal transform is
        # deliberately full-frame and is covered by dedicated tests above.
        with patch.object(hud, "_apply_thermal_look", return_value=False):
            hud.draw_rgb(frame)
        self.assertTrue(np.array_equal(frame[120:330, 120:680], before))
        self.assertNotEqual(int(frame[400:470].sum()), int(np.full((70, 800, 3), 31).sum()))


if __name__ == "__main__":
    unittest.main()
