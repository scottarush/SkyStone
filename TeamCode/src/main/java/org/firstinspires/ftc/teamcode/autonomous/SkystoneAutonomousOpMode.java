package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.teamcode.MecanumGrabberBot;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.VuforiaCommon;

@Autonomous(name="AutonomousMode", group="Robot")
//@Disabled
public class SkystoneAutonomousOpMode extends OpMode {

    private MecanumGrabberBot robot = new MecanumGrabberBot(this,true);

    /** team options. **/
    private static final String[] TEAM_OPTIONS = new String[]{"BLUE","RED"};
    public static final int BLUE_TEAM = 0;
    public static final int RED_TEAM = 1;
    private int team = BLUE_TEAM;

    /** confirmation **/
    private static final String[] SETUP_COMPLETE_OPTIONS = new String[]{"NO","YES"};
    public static final int COMPLETE_NO = 0;
    public static final int COMPLETE_YES = 1;
    private boolean setupComplete = false;

    /** list of options. **/
    private static final int TEAM_MENU_INDEX =0;
    private static final int SETUP_COMPLETE_MENU_INDEX = 1;
    private GamepadMenu menus[];
    private int currentMenu = 0;

    public static final double DRIVE_BY_ENCODER_POWER = 1.0d;

    private DragFoundationController dfController = null;

    private Gamepad mSetupGamepad = null;

    /**
     * Reference to VuforiaCommon utilities
     */
    VuforiaCommon vuforia = null;

    public SkystoneAutonomousOpMode(){
        mSetupGamepad = gamepad1;
        menus = new GamepadMenu[2];
        menus[TEAM_MENU_INDEX] = new GamepadMenu(this,mSetupGamepad,"Select Team:",TEAM_OPTIONS);
        menus[SETUP_COMPLETE_MENU_INDEX] = new GamepadMenu(this,mSetupGamepad,"Setup Complete?:",SETUP_COMPLETE_OPTIONS);
    }

    @Override
    public void init() {
        String initErrs = "";
        try {
            robot.init();
        }
        catch(Exception e){
            initErrs += e.getMessage();
        }
        // Initialize Vuforia
        vuforia = new VuforiaCommon(hardwareMap);
        try{
            vuforia.initVuforiaNavigation();
            // No exception so start Vuforia navigation
            vuforia.startVuforiaNavigation();
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
        if (setupComplete){
            return;
        }
        if (mSetupGamepad.dpad_right){
            currentMenu++;
            if (currentMenu > menus.length){
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
                case SETUP_COMPLETE_MENU_INDEX:
                    if (option == COMPLETE_YES){
                        setupComplete = true;
                        String s = doSetupCompleteInit();
                        telemetry.addData("Status:","Setup complete:"+s);
                    }
                    break;

            }
        }

    }

    /**
     * called once after setup is complete to initalize with the selected parameters
     */
    private String doSetupCompleteInit(){
        String rets = "";
        if (team == BLUE_TEAM){
            rets = doBlueTeamInit();
        }
        else {
            rets = doRedTeamInit();
        }
        return rets;
    }

    private String doRedTeamInit(){
        dfController = new DragFoundationController(this,robot,false);
        return "Red team setup complete";
    }
    private String doBlueTeamInit(){
        dfController = new DragFoundationController(this,robot,true);
        return "Blue team setup complete";

    }

    @Override
    public void loop() {
        if (!dfController.isDragFoundationComplete()){
            dfController.doOpmode();
        }


        // Update our location
        VuforiaCommon.VuforiaLocation location = vuforia.getVuforiaNavLocation();
        if (location.valid){
            telemetry.addData("Found location","x="+location.x+",y="+location.y+", heading="+location.heading);
        }

    }

/**
    public static void main(String[] args) {
        SkystoneAutonomousOpMode mode = new SkystoneAutonomousOpMode();
        mode.gamepad1 = new Gamepad();
        mode.telemetry = new TelemetryImpl(mode);
        mode.init_loop();;
    }
**/

}
