package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Hook;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drivetrain.IDriveSessionStatusListener;
import org.firstinspires.ftc.teamcode.drivetrain.IRotationStatusListener;
import org.firstinspires.ftc.teamcode.util.OneShotTimer;

import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import statemap.FSMContext;
import statemap.State;

public class AutonomousController {
    public static final int ENCODER_CONTROL = 0;
    public static final int OPEN_LOOP_TIME = 1;
    public static final int CLOSED_LOOP_VUFORIA = 2;

    private static boolean TELEMETRY_STATE_LOGGING_ENABLED = true;

    private AutonomousStateMachineContext mAsm = null;

    private OpMode opMode = null;

    private boolean mBlueAlliance = false;

    private Robot robot = null;

    private Recognition mSkystoneRecognition = null;

    private Recognition mStoneRecognition = null;

    private ArrayList<OneShotTimer> mStateTimers = new ArrayList<>();

    private OneShotTimer mHookTimer = new OneShotTimer(1000, new OneShotTimer.IOneShotTimerCallback() {
        @Override
        public void timeoutComplete() {
            mAsm.evHookTimeout();
        }
    });

    private OneShotTimer mGrabberTimer = new OneShotTimer(5000, new OneShotTimer.IOneShotTimerCallback() {
        @Override
        public void timeoutComplete() {
            robot.getGrabber().stop();
        }
    });

    private int mControlMode;

    private VuforiaTargetLocator mVuforia = null;

    public AutonomousController(final OpMode opMode, Robot robot, VuforiaTargetLocator vuforia, boolean blueAlliance, int controlMode){
        mAsm = new AutonomousStateMachineContext(this);
        this.opMode = opMode;
        this.mBlueAlliance = blueAlliance;
        this.robot = robot;
        mControlMode = controlMode;
        mVuforia = vuforia;
        // Add all the timers to the state timers so that they get service each loop
        mStateTimers.add(mHookTimer);
        mStateTimers.add(mGrabberTimer);

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

        // And add a listener to the state machine to send the state transitions to telemtry
        mAsm.addStateChangeListener(new PropertyChangeListener() {
            @Override
            public void propertyChange(PropertyChangeEvent event) {
                FSMContext fsm = (FSMContext) event.getSource();
                String propertyName = event.getPropertyName();
                State previousStatus = (State) event.getOldValue();
                State newState = (State) event.getNewValue();
                if (opMode != null){
                    opMode.telemetry.addData("Transition:",previousStatus.getName()+" to "+newState.getName());
                    opMode.telemetry.update();
                }
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

        // Service all the timers in order to trigger any timeout callbacks/events
        serviceTimers();
        // Call doLoop method on drivetrain to support drive and rotation controls
         robot.getDrivetrain().doLoop();
        // If a reset of the arm is active, then service it by calling it from here.
         if (robot.getArm().isResetToRetractInProgress()){
             if (!robot.getArm().resetToRetractPosition()){
                 // Assume same event even if we fail to retract.
      //           mAsm.evArmRetracted();
             }
         }
    }

    public boolean isAutonomousComplete(){
        return (mAsm.getState() == AutonomousStateMachineContext.AutonomousStateMachine.Complete);
    }


    private void startDriveByEncoder(double speed, double linearDistance, int timeoutms){
        if (robot.getDrivetrain().isMoving()){
            return;  // Already driving, this is an error, ignore
        }
        robot.getDrivetrain().driveEncoder(speed,linearDistance,timeoutms);
    }

    /**
     * start grabber turns the grab on forward or reverse
     */
    public void startGrabber(boolean intake, int timeoutms){
        if (intake){
            robot.getGrabber().moveGrabber(false,false, 1.0,1.0);
        }
        else{
            // Spit it out
            robot.getGrabber().moveGrabber(true,true, 0.0,0.0);

        }
        mGrabberTimer.setTimeout(timeoutms);
        mGrabberTimer.start();
    }

    public void stopGrabber(){
        robot.getGrabber().stop();
    }
    /**
     *
     */
    public void linearDrive(double distance){
        switch(mControlMode){
            case ENCODER_CONTROL:
                startDriveByEncoder(1.0d, distance,3000);
                break;
            case OPEN_LOOP_TIME:
                robot.getDrivetrain().driveLinearTime(distance,1.0d);
                break;
            case CLOSED_LOOP_VUFORIA:
                // TODO
                break;
        }

    }
    /**
     * strafe drives to a skystone.   A skystone must be in view or immediately triggers the
     * evDriveComplete event.
     */
    public void strafeDriveToSkystone() {
        if (mSkystoneRecognition == null){
            mAsm.evDriveComplete();
        }
        strafeDrive(getLateralDistanceToStone());
    }
    /**
     * strafe drives to a stone.   A stone must be in view or immediately triggers the
     * evDriveComplete event.
     */
    public void strafeDriveToStone() {
        if (mStoneRecognition == null){
            mAsm.evDriveComplete();
        }
        strafeDrive(getLateralDistanceToStone());
    }

        /**
         * strafe drive
         */
    public void strafeDrive(double distance){
        robot.getDrivetrain().doVectorTimedDrive(distance,0d);
    }

    /**
     * Reads the camera and looks for a skystone.
     * Triggers evStoneFound if a stone is found.
     *          evSkystoneFound if a Skystone is found.
     *          evNoStone if no stone currently in view
     */
    public void checkForStones(){
        List<Recognition> list = mVuforia.getRecognitions();
        if (list == null){
            mAsm.evNoStoneFound();
            return;
        }
           for (Iterator<Recognition> riter = list.iterator(); riter.hasNext(); ) {
                Recognition rec = riter.next();
                if (rec.getLabel().equalsIgnoreCase(VuforiaTargetLocator.SKYSTONE_TFOD_LABEL)) {
                    mSkystoneRecognition = rec;
                   mAsm.evSkystoneFound();
                    return;
                } else if (rec.getLabel().equalsIgnoreCase(VuforiaTargetLocator.STONE_TFOD_LABEL)) {
                    mStoneRecognition = rec;
                    mAsm.evStoneFound();
                    return;
                }
            }
        mSkystoneRecognition = null;
        mStoneRecognition = null;
        mAsm.evNoStoneFound();
    }

    /**
     * @return the lateral distance to move to align to a stone in front of the robt.  + is to robots left, - to right
     * or 0 if no stone or skystone in view.
      */
    public double getLateralDistanceToStone(){
        if (mSkystoneRecognition == null){
            return 0d;
        }
        // TODO finish this
        mSkystoneRecognition.estimateAngleToObject(AngleUnit.DEGREES);
        return 0d;
    }
    /**
     * Called from state machine to drive toward the foundation
     */
    public void dragFoundation(){
        switch(mControlMode){
            case ENCODER_CONTROL:
                startDriveByEncoder(1.0d, -12.0d,3000);
                break;
            case OPEN_LOOP_TIME:
                robot.getDrivetrain().driveLinearTime(-12d,1.0d);
                break;
            case CLOSED_LOOP_VUFORIA:
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
     * rotation by degrees.  + is ccw
     */
    public void rotate(int degrees){
        robot.getDrivetrain().rotate(degrees);
    }

    /**
     * returns true for blue, false for red.
     */
    public boolean isBlueAlliance(){
        return mBlueAlliance;
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

    private void serviceTimers(){
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

