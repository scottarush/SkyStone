package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Hook;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drivetrain.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.drivetrain.IDriveSessionStatusListener;
import org.firstinspires.ftc.teamcode.drivetrain.IRotationStatusListener;
import org.firstinspires.ftc.teamcode.util.OneShotTimer;

import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

import statemap.FSMContext;
import statemap.State;

public class AutonomousController {
    public static final int ENCODER_CONTROL = 0;
    public static final int OPEN_LOOP_TIME = 1;
    private int mDriveMode = ENCODER_CONTROL;

    private static boolean TELEMETRY_STATE_LOGGING_ENABLED = true;

    private AutonomousStateMachineContext mAsm = null;

    private OpMode opMode = null;

    private boolean mBlueAlliance = false;

    private Robot robot = null;

    private BaseMecanumDrive mecanumDrive = null;

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


    private VuforiaTargetLocator mVuforia = null;
    /***
     * Needed to queue events into the state machine
     */
    private static HashMap<String, Method> mTransition_map;
    private LinkedList<String> mTransition_queue;

    public AutonomousController(final OpMode opMode, Robot robot, VuforiaTargetLocator vuforia, boolean blueAlliance){
        mAsm = new AutonomousStateMachineContext(this);
        this.opMode = opMode;
        this.mBlueAlliance = blueAlliance;
        this.robot = robot;
        mVuforia = vuforia;

        buildTransitionTable();

        // Add all the timers to the state timers so that they get service each loop
        mStateTimers.add(mHookTimer);
        mStateTimers.add(mGrabberTimer);

        mecanumDrive = (BaseMecanumDrive)robot.getDrivetrain();

        // Add listeners to drivetrain for callbacks in order to translate into state machine events
        robot.getDrivetrain().addDriveSessionStatusListener(new IDriveSessionStatusListener() {
            @Override
            public void driveComplete() {
                opMode.telemetry.addData("driveComplete Called","");
                opMode.telemetry.update();
                transition("evDriveComplete");
            }

            @Override
            public void driveByEncoderTimeoutFailure() {
                transition("evDriveFail");
            }
        });
        robot.getDrivetrain().addRotationStatusListener(new IRotationStatusListener() {
            @Override
            public void rotationComplete() {
                transition("evRotationComplete");
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
                    opMode.telemetry.addData("Current State: ",newState.getName());
                    opMode.telemetry.update();
                }
            }
        });
    }

    /**
     * helper method to build the transition table so that we can trigger events from
     * within state machine handlers.
     */
    private void buildTransitionTable(){
        // Initialize the transition table for use in queueing events
        mTransition_map = new HashMap<>();
        try
        {
            Class context = AutonomousStateMachineContext.class;
            Method[] transitions = context.getDeclaredMethods();
            String name;
            int i;

            for (i = 0; i < transitions.length; ++i)
            {
                name = transitions[i].getName();

                // Ignore the getState and getOwner methods.
                if (name.compareTo("getState") != 0 &&
                        name.compareTo("getOwner") != 0)
                {
                    mTransition_map.put(name, transitions[i]);
                }
            }
        }
        catch (Exception ex)
        {}

        mTransition_queue = new LinkedList<>();
    }

    /**
     * Transition method needed to make transition calls within the Controller
     * @param trans_name
     */
    private synchronized void transition(String trans_name)
    {
        // Add the transition to the queue.
        mTransition_queue.add(trans_name);

        // Only if a transition is not in progress should a
        // transition be issued.
        if (mAsm.isInTransition() == false)
        {
            String name;
            Method transition;
            Object[] args = new Object[0];

            while (mTransition_queue.isEmpty() == false)
            {
                name = (String) mTransition_queue.remove(0);
                transition = (Method) mTransition_map.get(name);
                try
                {
                    transition.invoke(mAsm, args);
                }
                catch (Exception ex)
                {
                    String msg = ex.getMessage();
                    if (msg == null)
                        msg = "exception in state machine";
                    Log.e(getClass().getSimpleName(),msg);
                }
            }
        }

        return;
    }



    /**
     * Called from the OpMode loop until the state machine reaches Success
     */
     public void doOpmode(){
        // If we haven't started then kick if off.
        if (mAsm.getState() == AutonomousStateMachineContext.AutonomousStateMachine.Idle){
            transition("evStart");
        }

        // Service all the timers in order to trigger any timeout callbacks/events
        serviceTimers();
        // Call loop method on drivetrain to support drive and rotation controls
         robot.getDrivetrain().loop();
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
        switch(mDriveMode){
            case ENCODER_CONTROL:
                startDriveByEncoder(1.0d, distance,3000);
                break;
            case OPEN_LOOP_TIME:
                robot.getDrivetrain().driveLinearTime(distance,1.0d);
                break;
        }

    }

    /**
     * Reads the camera and looks for a skystone.
     * Triggers evStoneFound if a stone is found.
     *          evSkystoneFound if a Skystone is found.
     *          evNoStone if no stone currently in view
     */
    public void checkStoneRecognition(){
        List<Recognition> list = mVuforia.getRecognitions();
        if (list == null){
            transition("evNoStoneFound");
            return;
        }
           for (Iterator<Recognition> riter = list.iterator(); riter.hasNext(); ) {
                Recognition rec = riter.next();
                if (rec.getLabel().equalsIgnoreCase(VuforiaTargetLocator.SKYSTONE_TFOD_LABEL)) {
                    mSkystoneRecognition = rec;
                    transition("evSkystoneFound");
                    return;
                } else if (rec.getLabel().equalsIgnoreCase(VuforiaTargetLocator.STONE_TFOD_LABEL)) {
                    mStoneRecognition = rec;
                    transition("evStoneFound");
                    return;
                }
            }
        mSkystoneRecognition = null;
        mStoneRecognition = null;
        transition("evNoStoneFound");
    }

    /**
     * Strafes sideways to a Skystone based only on recognition.
     * triggers:
     * evDriveComplete when the drive is finished.
     * evNoStone if no stone is in view.
     * @param linearDistance forward distance from expected skystone
     * @param timeoutms timeout in ms
     */
    public void strafeToSkystone(double linearDistance,int timeoutms){
        if (mSkystoneRecognition == null){
            transition("evNoStoneFound");
        }
        double angle = mSkystoneRecognition.estimateAngleToObject(AngleUnit.RADIANS);
        double strafeDistance = linearDistance /Math.tan(angle);
        mecanumDrive.strafeEncoder(1.0d,strafeDistance,timeoutms);
    }

    /**
     * @return the lateral distance to move to align to a stone in front of the robt.  + is to robots left, - to right
     * or 0 if no stone or skystone in view.
      */
    public double getLateralDistanceToStone(){
        if (mSkystoneRecognition == null){
            return 0d;
        }
        mSkystoneRecognition.estimateAngleToObject(AngleUnit.DEGREES);
        return 0d;
    }
    /**
     * Called from state machine to drive toward the foundation
     */
    public void dragFoundation(){
        switch(mDriveMode){
            case ENCODER_CONTROL:
                startDriveByEncoder(1.0d, -12.0d,3000);
                break;
            case OPEN_LOOP_TIME:
                robot.getDrivetrain().driveLinearTime(-12d,1.0d);
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

