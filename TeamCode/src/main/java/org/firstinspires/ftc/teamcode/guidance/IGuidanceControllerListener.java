package org.firstinspires.ftc.teamcode.guidance;

public interface IGuidanceControllerListener {
    /**
     * Sets a straight line command to the motors either forward or backward
     * @param power -1.0..1.0 backward to forward
     */
    void setStraightCommand(double power);

    /**
     * Sets a composite steering command to the motors with both steering and power input
     * @param steering 0 = straight ahead, -1.0 max left, +1.0 max right
     * @param power -1.0..1.0 backward to forward
     */
    void setSteeringCommand(double steering,double power);
    /**
     * sets the motors to rotation mode with a rotation gain onlky
     * @param rotation 0 = stop, >0..+1.0 turn to right
     *                 >0..-1.0 turn to left
     */
    void setRotationCommand(double rotation);

    /**
     * called when a setTargetHeading triggered rotation is complete.
     */
    void rotationComplete();

    /**
     * Called when a path follow is complete.
     */
    void pathFollowComplete();
}
