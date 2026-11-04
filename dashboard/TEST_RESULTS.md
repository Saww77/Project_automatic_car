# Dashboard telemetry verification

## Passed

- 21 Python protocol/state/HTTP tests (`python3 -m unittest discover -s tests -p 'test_*.py' -v`). Covers existing binary commands and local HTTP checks, every telemetry subscription/decoder, field bounds and padding, independent freshness, retained/malformed sample rejection, reconnect invalidation, fault labels and bounded history.
- JavaScript syntax check (`node --check web/app.js`).
- Real Mosquitto 2.0.21 plus Chromium 155 / Playwright integration, using paho-mqtt 2.1.0 and a simulated Pico protocol fixture.
- CLI startup/address prompt, no automatic START, actual HTTP-to-broker binary START/STOP, ACK display, rapid START/STOP ordering, all five telemetry topics through MQTT/HTTP/browser, RPM trend rendering, missing-data placeholders, independent stale indicators, motor/vehicle faults, mobile width, heartbeat expiry and broker loss.
- Browser integration reported no JavaScript errors. All test-started broker/server/browser processes were shut down.

Browser results:

```text
PASS CLI: empty address reprompted; valid IP starts local HTTP dashboard; no startup command
PASS browser: controls, heartbeat, all five telemetry cards and missing-data placeholders render
PASS browser: retained status does not enable START; connected STOP remains available
PASS end-to-end: button -> HTTP -> real Mosquitto -> binary 01/00, QoS 0, non-retained -> fixture ACK -> browser
PASS browser: rapid START then STOP preserves command order
PASS end-to-end telemetry: all five binary topics decoded over real broker, HTTP and browser
PASS browser: mobile layout has no horizontal overflow
PASS browser: independent telemetry expiry and motor/mission fault rendering
PASS browser: stale heartbeat disables START and keeps STOP available
PASS browser: broker loss disables controls; no auto-command; no JavaScript errors
```

## Limits

The protocol fixture is software, not the physical Pico. Actual car Wi-Fi, broker delivery from the Pico, sensors, motor movement and track completion were not tested. Terrain/obstacle test values demonstrate decoding only; they do not imply those producers are implemented in the current firmware. No firmware changes were needed for this dashboard update. No credentials, virtual environments or bytecode caches are included in the distributable.
