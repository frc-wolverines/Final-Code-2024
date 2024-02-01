package frc.robot.systems.swerve;

public class ModuleInterface {
    public final int drive_motor_can_id;
    public final int pivot_motor_can_id;
    public final int can_coder_can_id;
    public final boolean reverse_drive_motor;

    public ModuleInterface(
        int drive_id,
        int pivot_id,
        int can_coder_id,
        boolean reverse_drive
    ) {
        this.drive_motor_can_id = drive_id;
        this.pivot_motor_can_id = pivot_id;
        this.can_coder_can_id = can_coder_id;
        this.reverse_drive_motor = reverse_drive;
    }
}
