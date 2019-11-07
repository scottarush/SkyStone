package org.firstinspires.ftc.teamcode.drivetrain;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;

public abstract class Drivetrain {

    /**
     * Number of encoder counts of each rotation of the shaft.
     **/
    public static final double ENCODER_COUNTS_PER_ROTATION = 1120;

    /* local OpMode members. */
    public HardwareMap hwMap = null;

    public abstract void init(HardwareMap ahwMap) throws Exception;
    public abstract void setPower(double lf, double rf, double lr, double rr);
    public abstract void stop();
    public abstract void setMotorModes(DcMotor.RunMode mode);

     /** OpMode in order to access telemetry from subclasses. **/
    protected OpMode opMode;

    public Drivetrain(OpMode opMode){
        this.opMode = opMode;
    }

    /**
     * starts a drive by encoder session.  If robot is moving, it will be stopped.
     *
     * @param speed   speed to move
     * @param xdist   distance in x inches to move
     * @param ydist   distance in y inches to move
     * @param timeout timeout in seconds to abort if move not completed.
     *
     * @return true if session started, false on error.
     */
     public boolean startDriveByEncoderSession(double speed, double xdist, double ydist, double timeout) {
        return false;
    }
    /**
     * continues a drive by encoder session.  If robot is moving, it will be stopped.
     * @return true if the session is still active.  false if session complete.
     */
    public boolean continueDriveByEncoder() {
        return false;
    }
    /**
     * @return true if the last encoder movement was successful, false if it timed out.
     **/
    public boolean driveByEncoderSuccess() {
        return false;
    }
    /**
     * @return true if the drivetrain is moving.  false if stopped.
     */
    public abstract boolean isMoving();

    /**
         *  Utility function to handle motor initialization.  init must have been called
         *  with a non-null hwMap or exception will be thrown.
         *
         */
    protected DcMotor tryMapMotor(String motorName) throws Exception {
        DcMotor motor = null;
        try {
            motor = hwMap.get(DcMotor.class, motorName);
        }
        catch(Exception e){
            // Throw an exception for the caller to catch so we can debug.
            throw new Exception ("Cannot map motor: "+motorName);
        }
        return motor;
    }

}
