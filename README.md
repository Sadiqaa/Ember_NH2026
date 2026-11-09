# Ember — NATHACKS 2026 firefighter visor prototype

One Raspberry Pi camera capture drives both displays:

- a sharp, full-frame false-colour thermal-style camera HUD on the Mac with
  dedicated fire, heading, air, and gas cards plus the heading-relative
  blue-to-orange/red scan strip;
- a centre-cropped, high-contrast 128x64 processed camera view on the
  monochrome OLED with compact left/right edge cues.

Detailed sensor values belong to the Mac interface. The OLED stays glanceable:
its main display is the processed camera stand-in, with one compact left or
right arrow at the edge only while a turn is needed. It contains no text or
secondary symbols. At target lock or when trustworthy guidance is unavailable,
the camera image is left unobstructed rather than fabricated.

At ambient signal, the Mac explicitly displays NO FIRE DETECTED and neither
display shows a turn arrow. Guidance begins only after two consecutive
calibrated readings reach the search threshold. A fire needs three consecutive
strong readings before it is labeled or saved, which rejects brief reflections
and sensor spikes. Sustained low signal clears guidance again.

## Connect

1. Connect the Pi's USB-C gadget cable to the Mac.
2. Connect the Freenove ESP32-S3 to a Pi USB-A port using the S3's **right
   USB-C socket under BOOT/IO0**. The left native USB socket is not used.
3. Connect the separate IMU/MQ-7/air-temperature ESP directly to the Mac by
   USB-C. The launcher recognizes its verified USB identity automatically.
4. Leave the Mac connected to its normal Wi-Fi. The launchers use only the
   Pi's fixed USB address 10.12.194.1 and do not change the default route.

See [HARDWARE_WIRING.md](HARDWARE_WIRING.md) for the OLED and sensor pin map.

## Run

Double-click Launch_Firefighter_Visor.command for the firefighter's Mac view.
The Pi sends the matching monochrome view to the OLED automatically.

From Terminal, the equivalent commands are:

sh
git clone https://github.com/Sadiqaa/Ember_NH2026.git
cd Ember_NH2026
./Launch_Firefighter_Visor.command


Press f in the firefighter window to enter or leave full screen, and q to
close it. If the Pi camera service restarts, Firefighter View pauses briefly
and reconnects in the same window.

The OLED is physically monochrome and its processed camera image is a visual
stand-in, not a thermal sensor. On the Mac, OpenCV contrast enhancement and the
INFERNO palette turn the full visible-light feed into a polished thermal-style
false-colour view; the crisp labels and detailed sensor cards are drawn after
that effect. The top strip records calibrated IR
strength in 10-degree heading sectors: newly scanned low-signal areas are blue,
alignment grows orange/red, and an earlier source remains as a gradually fading
mark while the strip scrolls with head movement. This is directional IR signal,
not a temperature measurement. This prototype is not certified life-safety gear.

When a confirmed fire signal falls within the camera's horizontal field of
view, the Mac adds a brighter localized patch labeled FIRE. The most recent
confirmed source in the current IMU/service session is remembered at its
world-relative bearing as a fixed light-orange patch labeled
PREVIOUS FIRE DETECTED HERE. It disappears when that bearing leaves the
camera view and reappears in the correct place when the wearer looks back. If
the source is still active at that location, the current FIRE label takes
precedence. This session marker deliberately resets with the service because
relative-yaw zero can change after an IMU restart. Off-camera sources use
arrows only, and the OLED remains text-free and unchanged by either colour
effect.

## IR-emitter target demo

The glasses-mounted GPIO10 receiver and a separate IR emitter stand in for a
fire detector and fire source. The wire protocol still calls the raw field
FLAME for compatibility, but wearer-facing screens consistently say FIRE.

The installed calibration records emitter-off/covered and emitter-on/aimed
readings and accounts for active-high versus active-low modules. This receiver
is calibrated active-low with ambient 4095.0 and an aimed target median of
147.5; those endpoints are installed in the Pi service.
One forward-facing receiver can provide signal strength and a centre lock. A
measured left/right bearing additionally requires the heading sensor or two
spatially separated IR receivers.

Without a heading sensor, the visor now provides a guided-search fallback. It
commands a left or right sweep, watches the filtered IR signal, reverses after
the signal peak is passed, and removes the turn cue at lock. These arrows are
search instructions rather than a measured bearing, so they do not display
degrees. On the Mac they change from amber to red as the signal becomes
meaningful; the OLED shows one matching direction arrow over its processed
camera view. Guidance requires two consecutive normalized readings of at least
1200; a confirmed FIRE requires three consecutive readings of at least
2800. Eight sustained low readings (about 0.8 seconds) release search
guidance and return the displays to the no-fire state.

The separate Mac-attached ESP now supplies IMU yaw, air temperature, and MQ-7
readings automatically. Keep it still for about 1.5 seconds whenever its USB
connection opens, then slowly sweep the wearer past the emitter once. The
program bins the glasses IR strength into 10-degree heading sectors and displays
TURN LEFT/RIGHT N DEG on the Mac while the OLED keeps only the matching small
edge arrow. The number is relative yaw, not an absolute compass bearing.

Each new target lock is also saved in the Pi's SQLite history at
/home/nathacks/camera-oled/data/target_history.sqlite3. The Mac HUD displays
the most recent time, signal, and relative bearing when available. With no
heading or GPS, the stored location is explicitly marked unknown; history is
never reused as a current navigation bearing after a restart.
