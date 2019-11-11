package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hook;
import org.firstinspires.ftc.teamcode.MecanumGrabberBot;

import java.util.ArrayList;
import java.util.Iterator;

public class DragFoundationController {

    private DragFoundationContext dragsm = null;

    private OpMode opMode = null;

    private boolean blueTeam = false;

    private MecanumGrabberBot robot = null;

    private ArrayList<StateTimer> mStateTimers = new ArrayList<>();

    private StateTimer mHookTimer = new StateTimer(1.0d,new HookTimeout());

    private boolean mCheckDrivingSuccess = false;

    public DragFoundationController(OpMode opMode, MecanumGrabberBot robot,boolean blueTeam){
        dragsm = new DragFoundationContext(this);
        this.opMode = opMode;
        this.blueTeam = blueTeam;
        this.robot = robot;

    }

     public void doOpmode(){
        // If we haven't started then kick if off.
        if (dragsm.getState() == DragFoundationContext.DragFoundation.Idle){
            dragsm.evStart();
        }

        // Check all the timers to trigger any timeout events
        checkTimers();
        // Check for success or failure of driving if active on last cycle
        if (mCheckDrivingSuccess){
            if (!robot.getDrivetrain().isDriveByEncoderSessionActive()){
                // Session no longer active so check for success
                if (robot.getDrivetrain().driveByEncoderSuccess()){
                    dragsm.evDriveSuccess();
                }
                else{
                    dragsm.evDriveFail();
                }
                // And clear check flag
                mCheckDrivingSuccess = false;
            }
            else{
                // Session is still active so call continue function
                robot.getDrivetrain().continueDriveByEncoder();
            }
        }
        if (robot.getDrivetrain().isRotationInProgress()) {
            if (robot.getDrivetrain().continueRotation()) {
                dragsm.evRotationComplete();
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
    private void startDriveByEncoder(double speed, double xdist, double ydist, double timeout){
        if (robot.getDrivetrain().isMoving()){
            return;  // Already driving, this is an error, ignore
        }
        robot.getDrivetrain().startDriveByEncoder(speed,xdist,ydist,timeout);
        // And set check flag to trigger checking for success events
        mCheckDrivingSuccess = true;
    }

    /**
     * Called from state machine to drive toward the foundation
     */
    public void startDriveToFoundation(){
        startDriveByEncoder(1.0d, 36.0d,0d,3.0d);
     }
    /**
     * Called from state machine to drive toward the foundation
     */
    public void startDragFoundation(){
        startDriveByEncoder(1.0d, -36.0d,0d,3.0d);
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
            robot.getDrivetrain().startRotation(-90d);
        }
        else{
            // Red team is +90
            robot.getDrivetrain().startRotation(90d);
        }
    }
    /**
     * Called from state machine to stop the robot
     */
    public void stop(){
        robot.getDrivetrain().stop();
    }

    public void startHookTimer(){
        mHookTimer.reset();
        mStateTimers.add(mHookTimer);
    }

    private class HookTimeout implements StateTimer.IStateTimerCallback{
        @Override
        public void timeoutComplete() {
            mStateTimers.remove(mHookTimer);
            dragsm.evHookTimeout();
        }
    }

    private void checkTimers(){
        for(Iterator<StateTimer>iter=mStateTimers.iterator();iter.hasNext();){
            StateTimer timer = iter.next();
            timer.checkTimer();
        }
    }

    public void setLogMessage(String msg){
        opMode.telemetry.addData("Status",msg);
        opMode.telemetry.update();
    }
}
