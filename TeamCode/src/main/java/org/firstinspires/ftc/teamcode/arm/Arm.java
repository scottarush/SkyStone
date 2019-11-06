package org.firstinspires.ftc.teamcode.arm;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

/**
 * This is the base class for a robot arm.  Subclasses implement specific mechanics of an arm.
 */
public abstract class Arm {

    protected OpMode opMode;

    protected HardwareMap hwMap = null;
    /**
     * Base class constructor
     * @param opMode to be used for telemetry in subclasses.
     */
    public Arm(OpMode opMode){
        this.opMode = opMode;
    }

    /**
     * Initialization method
     * @param ahwMap
     * @throws Exception
     */
    public abstract void init(HardwareMap ahwMap) throws Exception;

     /**
     * Sets the arm to an absolute position in degrees. 0 degree reference is arm specific.
     */
    public abstract void gotoAngle(double deltaDegrees);
    /**
     * moves the arm by a delta angle from current position. ignored in manual mode
     */
    public abstract void moveDeltaAngle(double degrees);


    /**
     * Starts moving the arm continously until stop is called at the last provided
     * power value.
     * @return false if the arm motor wasn't init correctly or the arm was initialized in angle mode
     */
    public abstract boolean moveArm(boolean up);

    /**
     * @return true if arm was init'ed with angle mode
     */
    public abstract boolean isAngleMode();
    /**
     * @return true if arm is moving, false if stopped.
     *
     */
    public abstract boolean isArmMoving();


    /**
     * Retrieves the current arm position.
     * @return current position of arm in degrees with 0 being the position of the arm at startup.
     */
    public abstract double getCurrentAngle();

    /**
     * Stops the arm at the current position
     */
    public abstract void stop();
}
