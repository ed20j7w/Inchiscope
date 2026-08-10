# SROM files go here

Put the 6D sensor's virtual SROM `.rom` file in this directory (e.g.
`distal_sensor.rom`), then point `aurora_tracker_node`'s `sensor_srom_path`
parameter at it in `../params.yaml`, e.g.:

```yaml
aurora_tracker_node:
  ros__parameters:
    sensor_srom_path: <repo root>/src/inchiscope_bringup/config/sroms/distal_sensor.rom
```

`.rom` files themselves are gitignored (they're tied to your specific
physical sensor coil's characterisation, not source) -- this directory
exists just so there's an obvious place to put it.

The reference tool's SROM is on its own physical chip and is auto-detected
by the tracker on connect; it needs no `.rom` file here at all.
