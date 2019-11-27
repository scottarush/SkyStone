package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.arm.FourBarArm;
import org.firstinspires.ftc.teamcode.drivetrain.BaseMecanumDrive;

/**
 * This is the Mecanum bot with a grabber arm.
 * HUB2 layout:
 * -------------------------------------------------
 * Port0:  left grabber motor  "lgrabber"
 * Port1:  right grabber motor "rgrabber"
 * Port2:  arm motor           "arm_motor"
 *
 * HUB1 Layout:
 * -------------------------------------------------
 * Port0:  left front wheel hex motor "lf"
 * Port1:  right front wheel hex motor "rf"
 * Port2:  left rear wheel hex motor "lr"
 * Port3:  right rear wheel hex motor "rr"
 *
 * Dig0:  arm limit switch "armlimitsw"
 *
 * Servo0:  Front hook servo "hookservo"
 * Servo1:  Arm claw servo "claw"
 */
public class MecanumGrabberBot extends Robot {

    private Grabber grabber;

    private FourBarArm arm = null;

    private Hook hook = null;

    private SideHook sideHook= null;

    private boolean mManualArmMode = false;

    public MecanumGrabberBot(OpMode opMode, boolean manualArmMode){
        super(DriveTrainStyle.GRABBER_BOT_MECANUM_DRIVE,opMode);
        mManualArmMode = manualArmMode;
    }

    public BaseMecanumDrive getDrivetrain(){
        return (BaseMecanumDrive) super.getDrivetrain();
    }

    /**
     * Returns the grabber.
     */
    public Grabber getGrabber(){
        return grabber;
    }
    /**
     * Override base class function to initialize the rest of the
     * bot.
     * @throws Exception
     */
    public void init() throws Exception {
        String initErrString = "";
        try {
            super.init();
        }
        catch (Exception e){
            initErrString += e.getMessage();
        }
        // base drivetrain init must have been OK so init the rest of the bot.
        try{
            grabber = new Grabber(opMode);
            grabber.init(opMode.hardwareMap);
        }
        catch (Exception e) {
            initErrString += e.getMessage();
        }
        // Initialize the arm to manual mode
        try{
            arm = new FourBarArm(opMode, mManualArmMode);
            arm.init(opMode.hardwareMap);
            arm.setClaw(false);
        }
        catch(Exception e){
            initErrString += e.getMessage();
        }
        // Initialize the front hook
        try{
            hook = new Hook(opMode);
            hook.init(opMode.hardwareMap);
        }
        catch(Exception e){
            initErrString += e.getMessage();
        }
        // Initialize the side hook
        try{
            sideHook = new SideHook(opMode);
            sideHook.init(opMode.hardwareMap);
        }
        catch(Exception e){
            initErrString += e.getMessage();
        }
        if (initErrString.length() > 0) {
            throw new Exception("Robot init errs: " + initErrString);
        }

    }

    public FourBarArm getArm(){
        return arm;
    }

    public Hook getHook(){
        return hook;
    }

    public SideHook getSideHook() { return sideHook; }
}
