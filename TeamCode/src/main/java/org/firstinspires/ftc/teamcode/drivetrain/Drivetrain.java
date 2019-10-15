package org.firstinspires.ftc.teamcode.drivetrain;

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
    abstract DcMotor tryMapMotor(String motorName);
}
