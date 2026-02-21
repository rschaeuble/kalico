# Prevents Z-axis crashes during blind lifts by monitoring a safety endstop at the axis maximum.
#
# This file may be distributed under the terms of the GNU GPLv3 license.


class SafeZLift:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.endstop = None
        self.z_steppers_registered = False

        # Configuration
        z_hop_pin = config.get("max_endstop_pin")

        # Setup Endstop
        ppins = self.printer.lookup_object("pins")
        self.endstop = ppins.setup_pin("endstop", z_hop_pin)

        # Register with query_endstops
        query_endstops = self.printer.load_object(config, "query_endstops")
        query_endstops.register_endstop(self.endstop, "safe_z")

        # Register G-Code command
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "SAFE_Z_LIFT", self.cmd_SAFE_Z_LIFT, desc=self.cmd_SAFE_Z_LIFT_help
        )

    def move_to_safe_z(self, toolhead, lift_dist, speed, force_unhomed=False):
        """
        Performs a Z-lift move that stops if the safety endstop is triggered.
        lift_dist: The distance to lift the Z axis.
        If force_unhomed is True, the current position is assumed to be Z=0
        and the homing state is cleared after the move.
        """
        # Lookup homing object
        homing = self.printer.lookup_object("homing")

        # Ensure Z-axis steppers are registered with the auxiliary endstop.
        if not self.z_steppers_registered:
            homing.register_axis_steppers(toolhead, self.endstop, "z")
            self.z_steppers_registered = True

        # Handle unhomed state logic
        was_unhomed = False
        if force_unhomed:
            curtime = self.printer.get_reactor().monotonic()
            kin_status = toolhead.get_kinematics().get_status(curtime)
            if "z" not in kin_status["homed_axes"]:
                was_unhomed = True
                curpos = toolhead.get_position()
                curpos[2] = 0.0
                toolhead.set_position(curpos, homing_axes=[2])

        # Calculate target position
        cur_z = toolhead.get_position()[2]
        target_pos = toolhead.get_position()
        target_pos[2] = cur_z + lift_dist

        # Perform the move
        try:
            homing.manual_home(
                toolhead,
                [(self.endstop, "safe_z")],
                target_pos,
                speed,
                triggered=True,
                check_triggered=False,
            )
        finally:
            if was_unhomed:
                toolhead.get_kinematics().clear_homing_state([2])

    cmd_SAFE_Z_LIFT_help = "Perform a safe Z lift move"

    def cmd_SAFE_Z_LIFT(self, gcmd):
        dist = gcmd.get_float("DIST", above=0.0)
        speed = gcmd.get_float("SPEED", above=0.0)
        toolhead = self.printer.lookup_object("toolhead")

        # Check if Z is already homed
        curtime = self.printer.get_reactor().monotonic()
        kin_status = toolhead.get_kinematics().get_status(curtime)

        self.move_to_safe_z(
            toolhead,
            dist,
            speed,
            force_unhomed="z" not in kin_status["homed_axes"],
        )


def load_config(config):
    return SafeZLift(config)
