# Safe Z Lift

The `[safe_z_lift]` module implements a safety mechanism for Z-axis lifting operations, particularly useful during
homing and probing sequences.

When the printer is at or near Z-max, operations that require a Z hop (like `safe_z_home` or
`dockable_probe` deployment) can crash into the axis' mechanical limits. The `safe_z_lift` module allows the printer to
perform these lifting moves while monitoring a specific endstop (Z-max endstop). If the endstop is triggered during the
lift, the movement stops immediately, but the operation proceeds without error.

## Configuration

To enable this feature, add a `[safe_z_lift]` section to your `printer.cfg` file.

```ini
[safe_z_lift]
max_endstop_pin: ^PA1
#   The pin connected to the Z-max endstop. This parameter is required.
#   See the "mcu" section for details on pin configuration.
```

## Integration

Once configured, the following modules will automatically utilize the safe lift mechanism:

* **[safe_z_home]**: The initial Z-hop before XY movement will be converted to a safe lift.
* **[dockable_probe]**: Z-hops during probe attachment and detachment will be converted to safe lifts.

No additional configuration is required in `[safe_z_home]` or `[dockable_probe]` to enable this behavior; they simply
detect the presence of the `safe_z_lift` configuration section.

## G-Code Commands

### SAFE_Z_LIFT

`SAFE_Z_LIFT DIST=<distance> SPEED=<speed>`

Performs a Z-axis lift of the specified distance at the specified speed (in mm/sec). May be used even if the printer is
not homed.
Movement stops if either the distance has been covered or the endstop is triggered.
