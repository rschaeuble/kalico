# Safe move
#
# Allows safe moves of an axis even when unhomed, as long as an endstop is available in the respective direction.
# This is used by modules like safe_z_home and dockable_probe during their Z hops, and be used via a Gcode
# command as well.


class SafeMove:
    """Execute safe single-axis moves using directional endstops."""

    def __init__(self, config):
        self.printer = config.get_printer()
        self.homing = self.printer.load_object(config, "homing")
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "SAFE_MOVE", self.cmd_SAFE_MOVE, desc=self.cmd_SAFE_MOVE_help
        )

    def move(self, toolhead, axis, dist, speed, allow_unsafe=False):
        """Move on one axis, stopping early if protected endstops trigger."""
        axis_lower = axis.lower()
        if axis_lower not in "xyz":
            raise self.printer.command_error(
                "SAFE_MOVE: AXIS must be X, Y, or Z"
            )
        axis_idx = "xyz".index(axis_lower)
        kin = toolhead.get_kinematics()
        try:
            rail = kin.rails[axis_idx]
        except IndexError as exc:
            raise self.printer.command_error(
                "SAFE_MOVE: Axis %s is not available" % axis.upper()
            ) from exc

        if dist == 0.0:
            return
        positive = dist > 0.0
        endstops = rail.get_endstops_for_direction(positive)
        rail_min, rail_max = rail.get_range()

        reactor = self.printer.get_reactor()
        curtime = reactor.monotonic()
        homed_axes = kin.get_status(curtime)[
            "homed_axes"
        ]  # TODO: is curtime the correct time to use?

        was_unhomed = False
        position = toolhead.get_position()
        if axis_lower not in homed_axes:
            was_unhomed = True
            # Assume a safe coordinate so the requested move stays in range.
            position[axis_idx] = rail_max if dist < 0.0 else rail_min
            toolhead.set_position(position, homing_axes=[axis_idx])
            position = toolhead.get_position()

        target_pos = list(position)
        target_pos[axis_idx] = position[axis_idx] + dist
        # Clamp moves for homed axes to avoid out-of-range errors.
        target_pos[axis_idx] = max(
            rail_min, min(rail_max, target_pos[axis_idx])
        )
        if target_pos[axis_idx] == position[axis_idx]:
            return

        try:
            if endstops:
                self.homing.manual_home(
                    toolhead,
                    endstops,
                    target_pos,
                    speed,
                    triggered=True,
                    check_triggered=False,
                )
            elif allow_unsafe:
                move_cmd = [None, None, None, None]
                move_cmd[axis_idx] = target_pos[axis_idx]
                toolhead.manual_move(move_cmd, speed)
            else:
                raise self.printer.command_error(
                    "SAFE_MOVE: No endstop protects axis %s in the %s direction"
                    % (
                        axis.upper(),
                        "positive" if positive else "negative",
                    )
                )
        finally:
            if was_unhomed:
                kin.clear_homing_state([axis_idx])

    cmd_SAFE_MOVE_help = "Perform a safe axis move"

    def cmd_SAFE_MOVE(self, gcmd):
        axis = gcmd.get("AXIS", None)
        if axis is None:
            raise gcmd.error("AXIS must be specified")

        dist = gcmd.get_float("DIST")
        if dist == 0.0:
            return
        speed = gcmd.get_float("SPEED", above=0.0)

        toolhead = self.printer.lookup_object("toolhead")
        try:
            self.move(toolhead, axis, dist, speed)
        except self.printer.command_error as err:
            raise gcmd.error(str(err))


def load_config(config):
    return SafeMove(config)
