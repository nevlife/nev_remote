# nev_teleop_bot — agent guide

ROS 2 Humble vehicle-side teleop bridge. Encodes video, publishes telemetry to Zenoh, executes remote commands. C++ video bridges, Python net/telemetry bridge.

## Code map

- `nev_teleop_bot/scripts/net_bridge.py` — Python ROS 2 ↔ Zenoh bridge. Telemetry, control, ping/pong, **video_ctl forward**.
- `nev_teleop_bot/include/video_bridge_base.hpp` — common GStreamer pipeline + Zenoh publisher + `force_keyframe()` + `/video/ctl` subscriber. h264 and h265 share this.
- `nev_teleop_bot/src/video_bridge_h264.cpp`, `_h265.cpp` — codec-specific entry points. Active target is h265.
- `config/teleop_topics.yaml` — declares dynamic vehicle topics bridged to `nev/robot/{id}/vehicle/{name}`.

## Video transport (branch `feat/video-rtp-multivehicle`)

Two pipeline variants gated by ROS 2 param `rtp_mode` (default `false`).

`rtp_mode=false` (legacy): `... ! nvh265enc name=enc ! h265parse ! video/x-h265,stream-format=byte-stream,alignment=au ! appsink`. Each Zenoh sample = 1 access unit. Header `[ts:f64][enc_ms:f32][NAL]`.

`rtp_mode=true`: `... ! nvh265enc name=enc ! h265parse config-interval=1 ! video/x-h265,stream-format=byte-stream,alignment=au ! rtph265pay pt=96 config-interval=1 aggregate-mode=zero-latency mtu=1200 ! application/x-rtp,media=video,encoding-name=H265,clock-rate=90000,payload=96 ! appsink emit-signals=true sync=false drop=false max-buffers=8`. Each Zenoh sample = 1 RTP packet. Same 12 B header. AU boundary detected via `GST_BUFFER_FLAG_MARKER`. `enc_ms` is computed on the AU's first packet, replicated to followers (`au_first_`, `last_au_enc_ms_` members).

UDP is permanently blocked on the deployment network. RTP here is a wire format only; transport stays Zenoh TCP. Do not propose WebRTC, SRT, or RTSP-over-UDP.

## Signaling channel

Direction: client → server → bot.

Wire: `nev/gcs/{vehicle_id}/video_ctl` (Zenoh, reliable, INTERACTIVE_HIGH). JSON body `{"type":"pli"|"keyframe_req"|"bitrate","kbps":<u32>}`.

Bot path:
- `net_bridge.py` declares the Zenoh subscriber, republishes the JSON string as `std_msgs/String` on ROS 2 topic `/video/ctl`.
- `video_bridge_base` subscribes `/video/ctl`, parses JSON with a minimal `json_field()` helper (no nlohmann dep).
  - `pli` / `keyframe_req` → `force_keyframe()` with 200 ms debounce against `last_keyframe_mono_`.
  - `bitrate` → `g_object_set(encoder_, "bitrate", N, NULL)`. See compatibility note below.

`force_keyframe()` injects a downstream force-key-unit event:
```cpp
GstEvent* ev = gst_video_event_new_downstream_force_key_unit(
    GST_CLOCK_TIME_NONE, GST_CLOCK_TIME_NONE, GST_CLOCK_TIME_NONE,
    TRUE, key_unit_count_++);
gst_element_send_event(encoder_, ev);
```
`encoder_` is captured at pipeline init via `gst_bin_get_by_name(pipeline_, "enc")`. Both hw_accel branches now name the encoder element `enc`.

## Build dependencies

- ROS 2 Humble, Ubuntu 22.04.
- zenoh-c 1.8.0.
- pkg-config: `gstreamer-1.0`, `gstreamer-app-1.0`, `gstreamer-video-1.0`. The third is required for `gst_video_event_new_downstream_force_key_unit` linkage. `CMakeLists.txt` wires it explicitly.
- Plugins: `gst-plugins-base/good` (rtph265pay), `gst-plugins-bad` (nvh265enc).

## Known compatibility risks (GStreamer 1.20 on Humble)

- `rtph265pay aggregate-mode=zero-latency` may be missing in 1.20. Verify with `gst-inspect-1.0 rtph265pay | grep -A8 aggregate-mode`. Fallback is to drop the property; default aggregation still works, just less coalescing.
- `nvh265enc` runtime bitrate change. 1.20's NVENC plugin may ignore live `g_object_set` or only honor it on next IDR. Verify the `CONTROLLABLE` flag with `gst-inspect-1.0 nvh265enc | grep -B1 -A3 bitrate`. PR 5 (bitrate adaptation) needs to call `force_keyframe()` after each bitrate change as a safety, or restart the encoder element.

## Pending work (PRs)

- PR 4 — client-side automatic PLI emission on jitterbuffer `on-lost-packet`. Bot side already complete.
- PR 5 — closed-loop bitrate adaptation: client measures `recv_kbps` / `loss_ratio`, sends `bitrate` over signaling at 1 Hz; bot updates encoder. Will likely add a `force_keyframe()` after each bitrate change to work around the 1.20 NVENC quirk.
