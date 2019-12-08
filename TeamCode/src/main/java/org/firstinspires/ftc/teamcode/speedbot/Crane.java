package org.firstinspires.ftc.teamcode.speedbot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This class encapsulates the Crane and grabbing hand on the speed bot.
 */
public class Crane {

    private OpMode mOpMode = null;

    private DcMotor mCraneMotor = null;

    public static final String CRANE_MOTOR_NAME = "crane";
    public static final String HAND_SERVO_NAME = "handservo";
    
    public static final double HAND_CLOSE_POSITION = 0.0d;
    public static final double HAND_OPEN_POSITION = 1.0d;

    private Servo mHandServo = null;

    private ElapsedTime mRampTimer = new ElapsedTime();

   // private DigitalChannel mLimitSwitch = null;

    // Constants

    public static final double RAISE_MAX_POWER = 0.5d;
    public static final double RAISE_START_POWER = 0.1d;
    public static final double RAISE_RAMP_UP_TIME = 1.0d;
    public static final double RAISE_POWER_SLOPE_PER_SEC = (RAISE_MAX_POWER - RAISE_START_POWER)/ RAISE_RAMP_UP_TIME;
    public static final double LOWER_MAX_POWER = 0.5d;
    public static final double LOWER_RAMP_UP_TIME = 0.5d;
    public static final double LOWER_START_POWER = 0.1d;
    public static final double LOWER_POWER_SLOPE_PER_SEC = (LOWER_MAX_POWER - LOWER_START_POWER)/ LOWER_RAMP_UP_TIME;

    /**
     * Constructs the Crane
     * @param opMode
     */
    public Crane(OpMode opMode) {
        mOpMode = opMode;
    }


    public void init(HardwareMap ahwMap) throws Exception {
        String initErrString = "";
        try {
            mCraneMotor = ahwMap.get(DcMotor.class, CRANE_MOTOR_NAME);
        } catch (Exception e) {
            initErrString += "Crane motor err,";
        }
        try {
            mHandServo = ahwMap.get(Servo.class, HAND_SERVO_NAME);
        } catch (Exception e) {
            initErrString += "Claw servo err";
        }
        if (initErrString.length() > 0){
            throw new Exception(initErrString);
        }
    }

    public void stop() {
        if (mCraneMotor != null){
            mCraneMotor.setPower(0d);
        }
    }

    /**
     * raises the crane continuously
     */
    public void raise(){
        if (mCraneMotor == null){
            return;
        }
        double abspower = 0d;
        double power = mCraneMotor.getPower();
        if (power == 0d){
            // We are starting a raise
            mRampTimer.reset();
            abspower = RAISE_START_POWER;
         }
        else if (power < 0d) {
            // We are lowering and reversing to a raise
            stop();
            mRampTimer.reset();
            abspower = RAISE_START_POWER;
        }
        if (mRampTimer.time() < RAISE_MAX_POWER){
            abspower = abspower + (RAISE_POWER_SLOPE_PER_SEC * mRampTimer.time());
        }
        if (abspower > RAISE_MAX_POWER) {
            abspower = RAISE_MAX_POWER;
        }
        mCraneMotor.setPower(power);
    }
    /**
     * lowers the crane continuously
     */
    public void lower(){
        if (mCraneMotor == null){
            return;
        }
        double abspower = 0d;
        double power = mCraneMotor.getPower();
        if (power == 0d){
            // We are starting a lower
            mRampTimer.reset();
            abspower = LOWER_START_POWER;
        }
        else if (power > 0d) {
            // We are raising so reverse to a lower
            stop();
            mRampTimer.reset();
            abspower = LOWER_START_POWER;
        }
        if (mRampTimer.time() < LOWER_MAX_POWER){
            abspower = abspower + (LOWER_POWER_SLOPE_PER_SEC * mRampTimer.time());
        }
        if (abspower > LOWER_MAX_POWER) {
            abspower = LOWER_MAX_POWER;
        }
        // Negate power to motor for lowering
        mCraneMotor.setPower(-power);

    }

     /**
     * closes the hand
     */
    public void closeHand(){
        if (mHandServo == null){
            return;
        }

        mHandServo.setPosition(HAND_CLOSE_POSITION);
    }
    /**
     * opens the hand
     */
    public void openHand(){
        if (mHandServo == null){
            return;
        }
        mHandServo.setPosition(HAND_OPEN_POSITION);
    }
}