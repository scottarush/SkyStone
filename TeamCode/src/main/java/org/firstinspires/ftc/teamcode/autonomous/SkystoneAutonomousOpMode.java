package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.teamcode.MecanumGrabberBot;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.VuforiaCommon;

@Autonomous(name="AutonomousMode", group="Robot")
//@Disabled
public class SkystoneAutonomousOpMode extends OpMode {

 //   private MecanumGrabberBot robot = new MecanumGrabberBot(this, Robot.DriveTrainStyle.MECANUM_HEX_BOT,true);
    private MecanumGrabberBot robot = new MecanumGrabberBot(this, Robot.DriveTrainStyle.MECANUM_REV_DEV_BOT,true);

    /** team options. **/
    private static final String[] TEAM_OPTIONS = new String[]{"BLUE","RED"};
    public static final int BLUE_TEAM = 0;
    public static final int RED_TEAM = 1;
    private int team = BLUE_TEAM;

    /** control mode. **/
    private static final String[] CONTROL_MODE_OPTIONS = new String[]{"CLOSED-LOOP ENCODER","OPEN-LOOP TIME","CLOSE-LOOP VUFORIA"};
    public static final int ENCODER_CONTROL = 0;
    public static final int OPEN_LOOP_TIME = 1;
    public static final int CLOSED_LOOP_VUFORIA = 2;
    private int mControlMode = ENCODER_CONTROL;

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

    private DragFoundationController dfController = null;

    private Gamepad mSetupGamepad = null;

    private double mLastInitLoopTime = 0d;
    private static final double MIN_INIT_LOOP_TIME = 0.150d;

    /**
     * Reference to VuforiaCommon utilities
     */
    VuforiaCommon mVuforia = null;

    public SkystoneAutonomousOpMode(){
    }

    @Override
    public void init() {
        mSetupGamepad = gamepad1;
        menus = new GamepadMenu[SETUP_COMPLETE_MENU_INDEX+1];
        menus[TEAM_MENU_INDEX] = new GamepadMenu(this,mSetupGamepad,"Select Team:",TEAM_OPTIONS);
        menus[CONTROL_MODE_MENU_INDEX] = new GamepadMenu(this,mSetupGamepad,"Select Control Mode:", CONTROL_MODE_OPTIONS);
        menus[SETUP_COMPLETE_MENU_INDEX] = new GamepadMenu(this,mSetupGamepad,"Setup Complete? ",SETUP_COMPLETE_OPTIONS);

        String initErrs = "";
        try {
            robot.init();
        }
        catch(Exception e){
            initErrs += e.getMessage();
        }
        // Initialize Vuforia
        mVuforia = new VuforiaCommon(hardwareMap);
        try{
            mVuforia.initVuforiaNavigation();
            // No exception so start Vuforia navigation
            mVuforia.startVuforiaNavigation();
        }
        catch(Exception e){
            initErrs += e.getMessage();
        }

        if (initErrs.length() == 0){
            telemetry.addData("Status:","Robot init complete");
            telemetry.update();
        }
        else{
            telemetry.addData("Init errors:",initErrs);
            telemetry.update();
        }


    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        if ((getRuntime()-mLastInitLoopTime) < MIN_INIT_LOOP_TIME){
            return;
        }
        mLastInitLoopTime = getRuntime();
        if (setupComplete){
            return;
        }
        if (mSetupGamepad.dpad_right){
            currentMenu++;
            if (currentMenu >= menus.length){
                currentMenu = 0;
            }
        }
        if (menus[currentMenu].doOptionMenu()){
            int option = menus[currentMenu].getSelectedOption();
            // This is a confirm.  Update the setting
            switch(currentMenu){
                case TEAM_MENU_INDEX:
                    team = option;
                    break;
                case CONTROL_MODE_MENU_INDEX:
                    mControlMode = option;
                    break;
                case SETUP_COMPLETE_MENU_INDEX:
                    setupComplete = true;
                    String s = doSetupCompleteInit();
                    telemetry.addData("Ready to Start:",s);
                    break;

            }
            currentMenu++;
            if (currentMenu >= menus.length){
                currentMenu = 0;
            }
        }

    }

    /**
     * called once after setup is complete to initalize with the selected parameters
     */
    private String doSetupCompleteInit(){
        if (team == BLUE_TEAM){
            dfController = new DragFoundationController(this,robot,mVuforia,true,mControlMode);
        }
        else {
            dfController = new DragFoundationController(this,robot,mVuforia,false,mControlMode);
        }
        return "Team="+TEAM_OPTIONS[team]+" Mode="+CONTROL_MODE_OPTIONS[mControlMode];
    }


    @Override
    public void loop() {
        if (dfController == null){
            // This is a restart
            doSetupCompleteInit();
        }
        if (!dfController.isDragFoundationComplete()){
            dfController.doOpmode();
        }


        // Update our location
        VuforiaCommon.VuforiaLocation location = mVuforia.getVuforiaNavLocation();
        if (location.valid){
            telemetry.addData("Found location","x="+location.x+",y="+location.y+", heading="+location.heading);
        }

    }


    public static void main(String[] args) {
        SkystoneAutonomousOpMode mode = new SkystoneAutonomousOpMode();
        mode.gamepad1 = new Gamepad();
        mode.telemetry = new TelemetryImpl(mode);
        mode.init_loop();
    }


}
