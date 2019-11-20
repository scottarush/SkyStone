package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MecanumGrabberBot;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.GamepadMenu;
import org.firstinspires.ftc.teamcode.util.VuforiaCommon;

//@Autonomous(name="AutonomousMode", group="Robot")
//@Disabled
public class SkystoneAutonomousOpMode {

 //   private MecanumGrabberBot robot = new MecanumGrabberBot(this, Robot.DriveTrainStyle.MECANUM_HEX_BOT,true);
    private MecanumGrabberBot robot = null;

    private static final boolean RUN_VUFORIA_NAVIGATION = false;

    /** team options. **/
    private static final String[] TEAM_OPTIONS = new String[]{"BLUE","RED"};
    public static final int BLUE_TEAM = 0;
    public static final int RED_TEAM = 1;
    private boolean mBlueAlliance = true;

    /** control mode. **/
    private static final String[] CONTROL_MODE_OPTIONS = new String[]{"CLOSED-LOOP ENCODER","OPEN-LOOP TIME","CLOSE-LOOP VUFORIA"};
    public static final int ENCODER_CONTROL = 0;
    public static final int OPEN_LOOP_TIME = 1;
    public static final int CLOSED_LOOP_VUFORIA = 2;
    private int mControlMode = OPEN_LOOP_TIME;

    /** confirmation **/
    private static final String[] SETUP_COMPLETE_OPTIONS = new String[]{"YES"};
    private boolean setupComplete = false;

    /** list of options. **/
    private static final int TEAM_MENU_INDEX =0;
    private static final int CONTROL_MODE_MENU_INDEX = 1;
    private static final int SETUP_COMPLETE_MENU_INDEX = 2;
    private GamepadMenu menus[];
    private int currentMenu = 0;

    public static final double DRIVE_BY_ENCODER_POWER = 1.0d;

    private AutonomousController autoController = null;

    private Gamepad mSetupGamepad = null;

    private double mLastInitLoopTime = 0d;
    private static final double MIN_INIT_LOOP_TIME = 0.150d;

    private OpMode mOpmode = null;
    /**
     * Reference to VuforiaCommon utilities
     */
    VuforiaCommon mVuforia = null;

    public SkystoneAutonomousOpMode(OpMode opMode, boolean isBlueTeam){
        mOpmode = opMode;
        mBlueAlliance = isBlueTeam;
     }

    public void init() {
/**        mSetupGamepad = gamepad1;
        menus = new GamepadMenu[SETUP_COMPLETE_MENU_INDEX+1];
        menus[TEAM_MENU_INDEX] = new GamepadMenu(this,mSetupGamepad,"Select Team:",TEAM_OPTIONS);
        menus[CONTROL_MODE_MENU_INDEX] = new GamepadMenu(this,mSetupGamepad,"Select Control Mode:", CONTROL_MODE_OPTIONS);
        menus[SETUP_COMPLETE_MENU_INDEX] = new GamepadMenu(this,mSetupGamepad,"Setup Complete? ",SETUP_COMPLETE_OPTIONS);
**/
        String initErrs = "";
        try {
            robot = new MecanumGrabberBot(mOpmode, Robot.DriveTrainStyle.MECANUM_HEX_BOT,true);
            robot.init();
            robot.getArm().setClaw(false);
        }
        catch(Exception e){
            initErrs += e.getMessage();
        }
        // Initialize Vuforia
        mVuforia = new VuforiaCommon(mOpmode);
        try{
            mVuforia.initVuforia();
            // Init the vuforia navigation if enabled
            if (RUN_VUFORIA_NAVIGATION) {
                 // No exception so start Vuforia navigation
                mVuforia.startVuforiaNavigation();
            }
        }
        catch(Exception e){
            initErrs += e.getMessage();
        }
        // Initialize the controller
        autoController = new AutonomousController(mOpmode,robot,mVuforia, mBlueAlliance,mControlMode);

        if (initErrs.length() == 0){
            mOpmode.telemetry.addData("Status:","Robot init complete");
            mOpmode.telemetry.update();
        }
        else{
            mOpmode.telemetry.addData("Init errors:",initErrs);
            mOpmode.telemetry.update();
        }


    }


    public void loop() {
         if (!autoController.isAutonomousComplete()){
            autoController.doOpmode();
        }


        // Update our location
        VuforiaCommon.VuforiaLocation location = mVuforia.getVuforiaNavLocation();
        if (location.valid){
            mOpmode.telemetry.addData("Found location","x="+location.x+",y="+location.y+", heading="+location.heading);
        }

    }

/**
    public static void main(String[] args) {
        SkystoneAutonomousOpMode mode = new SkystoneAutonomousOpMode();
        mOpmode.mode.gamepad1 = new Gamepad();
        mOpmode.mode.telemetry = new TelemetryImpl(mode);
        mode.init_loop();
    }
**/

}
