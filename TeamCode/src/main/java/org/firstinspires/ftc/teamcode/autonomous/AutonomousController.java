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

public class AutonomousController {

    private static boolean TELEMETRY_STATE_LOGGING_ENABLED = true;

    private AutonomousStateMachineContext mAsm = null;

    private OpMode opMode = null;

    private boolean blueTeam = false;

    private MecanumGrabberBot robot = null;

    private ArrayList<OneShotTimer> mStateTimers = new ArrayList<>();

    private OneShotTimer mHookTimer = new OneShotTimer(1000,new HookTimeout());

    private int mControlMode = 0;

    private VuforiaCommon mVuforia = null;

    public AutonomousController(final OpMode opMode, MecanumGrabberBot robot, VuforiaCommon vuforia, boolean blueTeam, int controlMode){
        mAsm = new AutonomousStateMachineContext(this);
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
                mAsm.evDriveComplete();
            }

            @Override
            public void driveByEncoderTimeoutFailure() {
                // mAsm.evDriveFail()
            }
        });
        robot.getDrivetrain().addRotationStatusListener(new IRotationStatusListener() {
            @Override
            public void rotationComplete() {
                mAsm.evRotationComplete();
            }
        });
    }

    /**
     * Called from the OpMode loop until the state machine reaches Success
     */
     public void doOpmode(){
        // If we haven't started then kick if off.
        if (mAsm.getState() == AutonomousStateMachineContext.AutonomousStateMachine.Idle){
            mAsm.evStart();
        }

        // Check all the timers to trigger any timeout events
        checkTimers();
        // Call doService method on drivetrain to support drive and rotation controls
         robot.getDrivetrain().doService();
        // If a reset of the arm is active, then service it by calling it from here.
         if (robot.getArm().isResetToRetractInProgress()){
             if (!robot.getArm().resetToRetractPosition()){
                 // Assume same event even if we fail to retract.
      //           mAsm.evArmRetracted();
             }
         }
    }

    public boolean isDragFoundationComplete(){
        return (mAsm.getState() == AutonomousStateMachineContext.AutonomousStateMachine.Success);
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
        robot.getDrivetrain().encoderDrive(speed,xdist,ydist,timeoutms);
    }

    /**
     *
     */
    public void linearDrive(double distance){
        switch(mControlMode){
            case SkystoneAutonomousOpMode.ENCODER_CONTROL:
                startDriveByEncoder(1.0d, distance,0d,3000);
                break;
            case SkystoneAutonomousOpMode.OPEN_LOOP_TIME:
                robot.getDrivetrain().doLinearTimedDrive(distance);
                break;
            case SkystoneAutonomousOpMode.CLOSED_LOOP_VUFORIA:
                // TODO
                break;
        }

    }

    /**
     * strafe drive
     */
    public void strafeDrive(double distance){
        robot.getDrivetrain().doVectorTimedDrive(distance,0d);
    }
    /**
     * Called from state machine to drive toward the foundation
     */
    public void dragFoundation(){
        switch(mControlMode){
            case SkystoneAutonomousOpMode.ENCODER_CONTROL:
                startDriveByEncoder(1.0d, -12.0d,0d,3000);
                break;
            case SkystoneAutonomousOpMode.OPEN_LOOP_TIME:
                robot.getDrivetrain().doLinearTimedDrive(-12d);
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
     * rotation by degrees.  + is cw
     */
    public void rotate(int degrees){
        robot.getDrivetrain().doTimedRotation(degrees);
    }

    /**
     * returns true for blue, false for red.
     */
    public boolean isBlueTeam(){
        return blueTeam;
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
            mAsm.evHookTimeout();
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

