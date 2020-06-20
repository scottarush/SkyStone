package org.firstinspires.ftc.teamcode.grabberbot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drivetrain.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.GrabberBotMecanumDrive;

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
public class MecanumGrabberBot  {

    private Grabber grabber;

    private FourBarArm arm = null;

    private Hook hook = null;

    private SideHook sideHook= null;

    private boolean mManualArmMode = true;
    protected OpMode mOpMode;

    private GrabberBotMecanumDrive mDrivetrain = null;


    public MecanumGrabberBot(OpMode opMode, boolean enableIMU){
        mOpMode = opMode;

        mDrivetrain = new GrabberBotMecanumDrive(mOpMode);

    }

    /**
     * Returns the BaseMecanumDrive for this bot
     */
    public BaseMecanumDrive getDrivetrain(){
        return mDrivetrain;
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
        try{
           mDrivetrain.init(mOpMode.hardwareMap);
        }
        catch(Exception e){
            initErrString += e.getMessage();
        }
        try{
            grabber = new Grabber(mOpMode);
            grabber.init(mOpMode.hardwareMap);
        }
        catch (Exception e) {
            initErrString += e.getMessage();
        }
        // Initialize the arm to manual mode
        try{
            arm = new FourBarArm(mOpMode, mManualArmMode);
            arm.init(mOpMode.hardwareMap);
            arm.setClaw(false);
        }
        catch(Exception e){
            initErrString += e.getMessage();
        }
        // Initialize the front hook
        try{
            hook = new Hook(mOpMode);
            hook.init(mOpMode.hardwareMap);
        }
        catch(Exception e){
            initErrString += e.getMessage();
        }
        // Initialize the side hook
        try{
            sideHook = new SideHook(mOpMode);
            sideHook.init(mOpMode.hardwareMap);
        }
        catch(Exception e) {
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
