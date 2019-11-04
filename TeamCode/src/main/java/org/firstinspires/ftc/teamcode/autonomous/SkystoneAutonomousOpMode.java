package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.autonomous.options.GamepadMenu;

@Autonomous(name="AutonomousMode", group="Robot")
@Disabled
public class SkystoneAutonomousOpMode extends OpMode {

    private SkystoneFieldGraph mFieldGraph;

    private Robot mRobot = new Robot(Robot.DriveTrainStyle.MECANUM, this);


    /** team options. **/
    private static final String[] TEAM_OPTIONS = new String[]{"BLUE","RED"};
    public static final int BLUE_TEAM = 0;
    public static final int RED_TEAM = 1;
    private int mTeam = BLUE_TEAM;

    /** selected route. **/
    private static final String[] ROUTE_OPTIONS = new String[]{"WHITE","BLACK"};
    public static final int WHITE_ROUTE = 0;
    public static final int BLACK_ROUTE = 1;
    private int mRoute = WHITE_ROUTE;

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
        boolean noerrs = true;
        try {
            mRobot.init();
        }
        catch(Exception e){
            noerrs = false;
            telemetry.addData("Init errors:",e.getMessage());
            telemetry.update();
        }
        if (noerrs){
            telemetry.addData("Status:","Robot init complete");
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
                    mRoute = option;
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
        return "";
    }

    @Override
    public void loop() {

    }
    /**
     *  Adds the BlueWhite route to the graph
     */
    public void addBlueWhiteRoute(){
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
    }

}
