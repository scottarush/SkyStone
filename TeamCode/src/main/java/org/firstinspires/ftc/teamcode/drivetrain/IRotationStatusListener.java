package org.firstinspires.ftc.teamcode.drivetrain;

/**
 * Interface implemented by listeners ot Drivetrain for completion of a rotation event.
 */
public interface IRotationStatusListener {

    /**
     * called when rotation successfully completed via the IMU
     * @param angle actual angle that was reached
     */
    void rotationComplete(int angle);

    /**
     * Called when rotation did not complete within the supplied timeout.
     * @param angle angle that was reached at timeout
     */
    void rotationTimeout(int angle);
}
