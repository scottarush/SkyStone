package org.firstinspires.ftc.teamcode.drivetrain;

/**
 * Interface implemented by listeners from the Drivetrain for DriveSession status.
 */
public interface IDriveSessionStatusListener {
    /** Drive session completed successfully. **/
    void driveComplete();
    /** Drive session failed. **/
    void driveByEncoderTimeoutFailure();
}
