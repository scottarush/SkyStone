package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class GrabberBotMecanumDrive extends BaseMecanumDrive {


    public GrabberBotMecanumDrive(OpMode opMode){
        super(opMode);
    }

    /* Initialize standard Hardware interfaces.
     * NOTE:  This class throws Exception on any hardware init error so be sure to catch and
     * report to Telemetry in your initialization. */
    public void init(HardwareMap ahwMap) throws Exception {
        // Save reference to Hardware map
        mHWMap = ahwMap;

        // Define and Initialize Motors
        String motorInitError = "";
        try {
            lfMotor = tryMapMotor("lf");
        }
        catch (Exception e){
            motorInitError += "lf,";
        }
        try {
            rfMotor = tryMapMotor("rf");
        }
        catch(Exception e){
            motorInitError += "rf,";
        }
        try {
            lrMotor = tryMapMotor("lr");
            lrMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        catch(Exception e){
            motorInitError += "lr,";
        }
        try {
            rrMotor = tryMapMotor("rr");
            rrMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        catch(Exception e){
            motorInitError += "rr,";
        }
        // Initialize the IMU with the superclass initialization
        try{
            super.init(ahwMap);
        }
        catch(Exception e){
            // This exception can't actually happen but change in the future to catch this somehow
            motorInitError += "IMU init";
        }

        // Set all motors to zero power
        setPower(0, 0, 0, 0);

        // Set all motors to run with encoders.
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        if (motorInitError.length() > 0){
            throw new Exception("Motor init errs: '"+motorInitError+"'");
        }

    }


}
