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
     * Utility function moves the robot a delta x and y.  Base class function does
     * nothing.
     *
     * @param speed   speed to move
     * @param xdist   distance in x inches to move
     * @param ydist   distance in y inches to move
     * @param timeout timeout in seconds to abort if move not completed.
     */
    public void driveByEncoder(double speed, double xdist, double ydist, double timeout){

    }

        /**
         *  Utility function to handle motor initialization.  init must have been called
         *  with a non-null hwMap or exception will be thrown.
         *
         */
    protected DcMotor tryMapMotor(String motorName){
        DcMotor motor = null;
        try {
            if (hwMap == null){
                throw new Exception("tryMapMotor called with null hwMap. Must call init first.");
            }
            motor = hwMap.get(DcMotor.class, motorName);
        }
        catch(Exception e){
            //e.printStackTrace();
            opMode.telemetry.addData("Motor Init Failed: "+e.getMessage(), motorName);
            opMode.telemetry.update();
        }
        return motor;
    }

}
