package org.firstinspires.ftc.teamcode.arm;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FourBarArm extends Arm {

    private DcMotor armMotor = null;

    public static final String ARM_MOTOR_NAME = "arm_motor";

    public FourBarArm(OpMode opMode){
        super(opMode);
    }

    // Constants
    public static final double ARM_GEAR_REDUCTION_RATIO = Math.pow(125d/30d,2d);
    public static final double HEX_MOTOR_COUNTS_PER_REVOLUTION = 545d;  // This should be 288d but we are off by a factor of 2
    public static final double COUNTS_PER_ARM_360_ROTATION = ARM_GEAR_REDUCTION_RATIO * HEX_MOTOR_COUNTS_PER_REVOLUTION;

    public static final double COUNTS_PER_DEGREE = COUNTS_PER_ARM_360_ROTATION / 360;

    public static final double MAX_ANGLE = 90;
    public static final double MIN_ANGLE = -90;

    private double mCurrentAngle = 0.0;

    private double mMotorSpeed = 0.0;

    @Override
    public void init(HardwareMap ahwMap) throws Exception {
        hwMap = ahwMap;
        try {
            if (hwMap == null){
                throw new Exception("tryMapArmMotor called with null hwMap. Must call init first.");
            }
            armMotor = hwMap.get(DcMotor.class, ARM_MOTOR_NAME);
        }
        catch(Exception e){
            //e.printStackTrace();
            opMode.telemetry.addData("Motor Init Failed: "+e.getMessage(), ARM_MOTOR_NAME);
            opMode.telemetry.update();
            return;
        }
        // now set the motor mode

        // TODO Need to find a way to set the arm to the MIN_ANGLE reference point at startup
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Reset the arm to the MIN_ANGLE position
     */
    public void resetPosition() {
        setAngle(MIN_ANGLE);
     }

    @Override
    public void stop() {
        armMotor.setPower(0);
        mCurrentAngle = armMotor.getCurrentPosition() / COUNTS_PER_DEGREE;
    }

    @Override
    public void setAngle(double targetAngle) {
        if (targetAngle > MAX_ANGLE)
            targetAngle = MAX_ANGLE;
        if (targetAngle < MIN_ANGLE)
            targetAngle = MIN_ANGLE;
        // Compute the angle to move in degrees
        double deltaDegrees = targetAngle - mCurrentAngle;

        // Now get the number of counts to move
        int counts = (int)Math.round(deltaDegrees * COUNTS_PER_DEGREE);

       // And move the arm
        armMotor.setPower(mMotorSpeed);
        armMotor.setTargetPosition(counts);
        mCurrentAngle = getCurrentAngle();
    }

    @Override
    public double getCurrentAngle() {
        return armMotor.getCurrentPosition() / COUNTS_PER_DEGREE;
    }

    @Override
    public void setSpeed(double speed) {
        mMotorSpeed = speed;
    }

}
