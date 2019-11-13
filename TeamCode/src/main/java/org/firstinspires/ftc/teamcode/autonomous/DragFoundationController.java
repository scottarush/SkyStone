package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hook;
import org.firstinspires.ftc.teamcode.MecanumGrabberBot;
import org.firstinspires.ftc.teamcode.arm.FourBarArm;
import org.firstinspires.ftc.teamcode.drivetrain.IDriveSessionStatusListener;
import org.firstinspires.ftc.teamcode.drivetrain.IRotationStatusListener;
import org.firstinspires.ftc.teamcode.util.VuforiaCommon;
import org.firstinspires.ftc.teamcode.util.OneShotTimer;

import java.util.ArrayList;
import java.util.Iterator;

public class DragFoundationController {

    private static boolean TELEMETRY_STATE_LOGGING_ENABLED = true;

    private DragFoundationContext dragsm = null;

    private OpMode opMode = null;

    private boolean blueTeam = false;

    private MecanumGrabberBot robot = null;

    private ArrayList<OneShotTimer> mStateTimers = new ArrayList<>();

    private OneShotTimer mHookTimer = new OneShotTimer(1000,new HookTimeout());

    private int mControlMode = 0;

    private VuforiaCommon mVuforia = null;

    public DragFoundationController(final OpMode opMode, MecanumGrabberBot robot, VuforiaCommon vuforia, boolean blueTeam, int controlMode){
        dragsm = new DragFoundationContext(this);
        this.opMode = opMode;
        this.blueTeam = blueTeam;
        this.robot = robot;
        mControlMode = controlMode;
        mVuforia = vuforia;
        // Add all the timers to the state timers
        mStateTimers.add(mHookTimer);

        // Add listeners to drivetrain for callbacks in order to translate into state machine events
        robot.getDrivetrain().addDriveSessionStatusListener(new IDriveSessionStatusListener() {
            @Override
            public void driveComplete() {
                dragsm.evDriveComplete();
            }

            @Override
            public void driveByEncoderTimeoutFailure() {
                dragsm.evDriveFail();
            }
        });
        robot.getDrivetrain().addRotationStatusListener(new IRotationStatusListener() {
            @Override
            public void rotationComplete() {
                dragsm.evRotationComplete();
            }
        });
    }

    /**
     * Called from the OpMode loop until the state machine reaches Success
     */
     public void doOpmode(){
        // If we haven't started then kick if off.
        if (dragsm.getState() == DragFoundationContext.DragFoundation.Idle){
            dragsm.evStart();
        }

        // Check all the timers to trigger any timeout events
        checkTimers();
        // Call doService method on drivetrain to support drive and rotation controls
         robot.getDrivetrain().doService();
        // If a reset of the arm is active, then service it by calling it from here.
         if (robot.getArm().isResetToRetractInProgress()){
             int status = robot.getArm().resetToRetractPosition();
             if (status != FourBarArm.RESET_RETRACT_IN_PROGRESS){
                 // Assume same event even if we fail to retract.
                 dragsm.evArmRetracted();
             }
         }
    }

    public boolean isDragFoundationComplete(){
        return (dragsm.getState() == DragFoundationContext.DragFoundation.Success);
    }

    /**
     *
     * @param xdist
     * @param ydist
     */
    private void startDriveByEncoder(double speed, double xdist, double ydist, int timeoutms){
        if (robot.getDrivetrain().isMoving()){
            return;  // Already driving, this is an error, ignore
        }
        robot.getDrivetrain().driveByEncoder(speed,xdist,ydist,timeoutms);
    }

    /**
     * Called from state machine to drive toward the foundation
     */
    public void driveToFoundation(){
        switch(mControlMode){
            case SkystoneAutonomousOpMode.ENCODER_CONTROL:
                startDriveByEncoder(1.0d, 36.0d,0d,3000);
                break;
            case SkystoneAutonomousOpMode.OPEN_LOOP_TIME:
                robot.getDrivetrain().driveByTime(12d);
                break;
            case SkystoneAutonomousOpMode.CLOSED_LOOP_VUFORIA:
                // TODO
                break;
        }
    }
    /**
     * Called from state machine to drive toward the foundation
     */
    public void dragFoundation(){
        switch(mControlMode){
            case SkystoneAutonomousOpMode.ENCODER_CONTROL:
                startDriveByEncoder(1.0d, -36.0d,0d,3000);
                break;
            case SkystoneAutonomousOpMode.OPEN_LOOP_TIME:
                robot.getDrivetrain().driveByTime(-12d);
                break;
            case SkystoneAutonomousOpMode.CLOSED_LOOP_VUFORIA:
                // TODO
                break;
        }
    }
    /**
     * Called from state machine to drive toward quarry
     */
    public void driveToQuarry(){
        switch(mControlMode){
            case SkystoneAutonomousOpMode.ENCODER_CONTROL:
                startDriveByEncoder(1.0d, 36.0d,0d,3000);
                break;
            case SkystoneAutonomousOpMode.OPEN_LOOP_TIME:
                robot.getDrivetrain().driveByTime(36d);
                break;
            case SkystoneAutonomousOpMode.CLOSED_LOOP_VUFORIA:
                // TODO
                break;
        }
    }

    /**
     * Called from state machine to lower the arm until it hits the limit switch.
     */
    public void retractArm(){
        // Start the retract.  OpMode loop will do the rest
        robot.getArm().resetToRetractPosition();
    }

    /**
     * Called from OpMode when the arm has closed.
     */
    public void armRetracted(){

    }
    /**
     * Called from state machine to open the hook
     */
    public void openHook(){
        robot.getHook().setPosition(Hook.OPEN);
    }
    /**
     * Called from state machine to close the hook
     */
    public void closeHook(){
        robot.getHook().setPosition(Hook.CLOSED);
    }
    /**
     * Rotates the robot toward the quarry.
     */
    public void rotateToQuarry(){
        if (blueTeam){
            // Rotate -90
            robot.getDrivetrain().doRotationByTime(-90);
        }
        else{
            // Red team is +90
            robot.getDrivetrain().doRotationByTime(90);
        }
    }
    /**
     * Called from state machine to stop the robot
     */
    public void stop(){
        robot.getDrivetrain().stop();
    }

    public void startHookTimer(){
       mHookTimer.start();
    }

    private class HookTimeout implements OneShotTimer.IOneShotTimerCallback {
        @Override
        public void timeoutComplete() {
            dragsm.evHookTimeout();
        }
    }

    private void checkTimers(){
        for(Iterator<OneShotTimer> iter = mStateTimers.iterator(); iter.hasNext();){
            OneShotTimer timer = iter.next();
            timer.checkTimer();
        }
    }

    public void setLogMessage(String msg){
        if (TELEMETRY_STATE_LOGGING_ENABLED) {
            opMode.telemetry.addData("Status", msg);
            opMode.telemetry.update();
        }
    }


}

