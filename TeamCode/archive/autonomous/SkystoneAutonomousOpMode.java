package org.firstinspires.ftc.teamcode.archive.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.VuforiaCommon;

@Autonomous(name="AutonomousMode", group="Robot")
@Disabled
public class SkystoneAutonomousOpMode extends OpMode {

    private SkystoneFieldGraph mFieldGraph;

    private Robot mRobot = new Robot(Robot.DriveTrainStyle.MECANUM_HEX_BOT, this);

    /** team options. **/
    private static final String[] TEAM_OPTIONS = new String[]{"BLUE","RED"};
    public static final int BLUE_TEAM = 0;
    public static final int RED_TEAM = 1;
    private int mTeam = BLUE_TEAM;

    /** selected route. **/
    private static final String[] ROUTE_OPTIONS = new String[]{"WHITE","BLACK"};
    public static final int WHITE_ROUTE = 0;
    public static final int BLACK_ROUTE = 1;
    private int mSelectedRouteOptionIndex = WHITE_ROUTE;

    private Route mSelectedRoute = null;

    /** confirmation **/
    private static final String[] SETUP_COMPLETE_OPTIONS = new String[]{"NO","YES"};
    public static final int COMPLETE_NO = 0;
    public static final int COMPLETE_YES = 1;
    private boolean mSetupComplete = false;

    /** list of options. **/
    private static final int TEAM_MENU_INDEX =0;
    private static final int ROUTE_MENU_INDEX = 1;
    private static final int SETUP_COMPLETE_MENU_INDEX = 2;
    private GamepadMenu mMenus[];
    private int mCurrentMenu = 0;

    private boolean mRouteComplete = false;
    private boolean mDriveByEncoderActive = false;
    public static final double DRIVE_BY_ENCODER_POWER = 1.0d;

    /**
     * Reference to VuforiaTensorFlowObjectDetector utilities
     */
    VuforiaCommon mVuforia = null;

    public SkystoneAutonomousOpMode(){

        // Create the field graph
        mFieldGraph = new SkystoneFieldGraph();

        mMenus = new GamepadMenu[3];
        mMenus[TEAM_MENU_INDEX] = new GamepadMenu(this,"Team:",TEAM_OPTIONS);
        mMenus[ROUTE_MENU_INDEX] = new GamepadMenu(this,"Route:",ROUTE_OPTIONS);
        mMenus[SETUP_COMPLETE_MENU_INDEX] = new GamepadMenu(this,"Route:",SETUP_COMPLETE_OPTIONS);
    }

    @Override
    public void init() {
        String initErrs = "";
        try {
            mRobot.init();
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
        if (mSetupComplete){
            return;
        }
        if (gamepad1.dpad_right){
            mCurrentMenu++;
            if (mCurrentMenu > mMenus.length){
                mCurrentMenu = 0;
            }
        }
        if (mMenus[mCurrentMenu].doOptionMenu()){
            int option = mMenus[mCurrentMenu].getSelectedOption();
            // This is a confirm.  Update the setting
            switch(mCurrentMenu){
                case TEAM_MENU_INDEX:
                    mTeam = option;
                    break;
                case ROUTE_MENU_INDEX:
                    mSelectedRouteOptionIndex = option;
                    break;
                case SETUP_COMPLETE_MENU_INDEX:
                    if (option == COMPLETE_YES){
                        mSetupComplete = true;
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
        if (mTeam == BLUE_TEAM){
            rets = doBlueTeamInit();
        }
        else {
            rets = doRedTeamInit();
        }
        return rets;
     }

    private String doRedTeamInit(){
        return "Error:  Red team init is TBD";
    }
    private String doBlueTeamInit(){
        String rets = "";
        switch(mSelectedRouteOptionIndex){
            case WHITE_ROUTE:
                // Add the route
                mSelectedRoute = addBlueWhiteRoute();
                // And place the robot on the field
                if (!mFieldGraph.setSkystoneFieldPosition(-60,-36,mSelectedRoute)){
                    rets += "Error setting robot position on BlueWhite route.";
                }
                rets +=  "Blue team White route set.";
                break;
        }
        return rets;

    }

    @Override
    public void loop() {
        if (mRouteComplete)
            return;
        if (mRobot.getDrivetrain().isMoving()){
            // Robot is moving so wait until it stops.
            return;
        }

        if (mDriveByEncoderActive){
            mDriveByEncoderActive = mRobot.getDrivetrain().continueDriveByEncoder();
            if (!mDriveByEncoderActive){
                // This was the end of the session. check if it timed out or not
                if (!mRobot.getDrivetrain().driveByEncoderSuccess()){
                    telemetry.addData("Error:","Drive by encoder timed out");
                    telemetry.update();
                }
                return;
            }
        }
        // Otherwise check if an encoder session is active
        Maneuver maneuver = mFieldGraph.getNextManeuver();
        if (maneuver == null){
            if (!mRouteComplete){
                mRouteComplete = true;
                telemetry.addData("Status",mSelectedRoute.getName()+" complete");
                telemetry.update();
            }
            return;
        }

        // Update our location
        VuforiaCommon.VuforiaLocation location = mVuforia.getVuforiaNavLocation();
        if (location.valid){
            telemetry.addData("Found location","x="+location.x+",y="+location.y+", heading="+location.heading);
        }

        if (maneuver instanceof MovementManeuver){
            MovementManeuver movement = (MovementManeuver)maneuver;
            mDriveByEncoderActive = mRobot.getDrivetrain().startDriveByEncoder(DRIVE_BY_ENCODER_POWER,movement.xDelta,movement.yDelta,3);
        }
    }
    /**
     *  Adds the BlueWhite route to the graph
     */
    public Route addBlueWhiteRoute(){
        Route r = new Route("BlueWhite",mFieldGraph);
        r.setStartTile(7);
        r.addRouteTransition(1,null);
        r.addRouteTransition(2,null);
        r.addRouteTransition(1,null);
        r.addRouteTransition(7,null);
        r.addRouteTransition(13,null);
        r.addRouteTransition(19,null);
        r.addRouteTransition(25,null);
        r.addRouteTransition(31,null);
        r.addRouteTransition(32,null);
        r.addRouteTransition(31,null);
        r.addRouteTransition(25,null);
        r.addRouteTransition(19,null);
        return r;
    }

}
