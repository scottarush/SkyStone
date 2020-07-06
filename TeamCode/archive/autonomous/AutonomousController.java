package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drivetrain.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.speedbot.Crane;
import org.firstinspires.ftc.teamcode.speedbot.CraneSpeedBot;
import org.firstinspires.ftc.teamcode.speedbot.ICraneMovementStatusListener;
import org.firstinspires.ftc.teamcode.speedbot.BaseSpeedBot;
import org.firstinspires.ftc.teamcode.util.OneShotTimer;

import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;

import statemap.FSMContext;
import statemap.State;

public class AutonomousController implements ICraneMovementStatusListener {
    public static final int SEQUENCE_DRAG_FOUNDATION = 0;
    public static final int SEQUENCE_GET_STONE = 1;
    public static final int SEQUENCE_ROTATE_TO_BRIDGE_PARK = 2;
    public static final int SEQUENCE_PIBOTS_BRIDGE_PARK = 3;
    public static final int SEQUENCE_TECHNOLOGIC_BRIDGE_PARK = 4;

    private static boolean TELEMETRY_STATE_LOGGING_ENABLED = true;

    private SpeedBotAutoStateMachineContext mSpeedBotAutoSM = null;

    private OpMode opMode = null;

    private boolean mBlueAlliance = false;

    /**
     * Max rotation power.
     */
    private static final double MAX_ROTATION_POWER = 0.75d;

    /**
     * rotation timeout. same value used for all transitions
     */
    private static final int ROTATION_TIMEOUTMS = 1000;
    /**
     * This reference will be non-null when using the speed bot
     */
    private BaseSpeedBot mSpeedBot = null;

    /**
     * Common reference used for the MecanumDrive on either bot
     */
    private BaseMecanumDrive mMecanumDrive = null;

    private ArrayList<OneShotTimer> mStateTimers = new ArrayList<>();

    /**
     * Common timer used to open or close the hooks on either bot
     */
    private OneShotTimer mTimer = new OneShotTimer(1000, new OneShotTimer.IOneShotTimerCallback() {
        @Override
        public void timeoutComplete() {
            transition("evTimeout");
        }
    });


    /***
     * Needed to queue events into the state machine
     */
    private static HashMap<String, Method> mTransition_map;
    private LinkedList<String> mTransition_queue;

    private int mLastRotationAngle = 0;
    private double mLastDriveDistance = 0d;
    private int mSequence = SEQUENCE_DRAG_FOUNDATION;

    /**
     * Constructor
     * @param opMode
     * @param speedBot
     * @param blueAlliance
     */
    public AutonomousController(final OpMode opMode,
                                BaseSpeedBot speedBot,
                                boolean blueAlliance,
                                int sequence) {
        mSpeedBotAutoSM = new SpeedBotAutoStateMachineContext(this);
        this.opMode = opMode;
        this.mBlueAlliance = blueAlliance;
        mSpeedBot = speedBot;
         mSequence = sequence;

        mMecanumDrive = speedBot.getDrivetrain();

        if (speedBot instanceof CraneSpeedBot) {
            ((CraneSpeedBot)mSpeedBot).getCrane().addCraneMovementStatusListener(this);
        }

        // Add timers to be checked
        mStateTimers.add(mTimer);

        // Put the hand in retract position for regulation since we inspected in that position
        retractHand();
        // Now do common initializations
        init();
    }


    /**
     * Does initializations common to both robots.
     */
    private void init(){
        buildTransitionTable();

//        // Add listeners to drivetrain for callbacks in order to translate into state machine events
//        mMecanumDrive.addDriveSessionStatusListener(new IDriveSessionStatusListener() {
//            @Override
//            public void driveComplete(double distance) {
//                mLastDriveDistance = distance;
//                transition("evDriveComplete");
//            }
//
//            @Override
//            public void driveByEncoderTimeoutFailure(double distance) {
//                mLastDriveDistance = distance;
//                transition("evDriveTimeout");
//            }
//        });
//        mMecanumDrive.addRotationStatusListener(new IRotationStatusListener() {
//            @Override
//            public void rotationComplete(int angle) {
//                mLastRotationAngle = angle;
//                transition("evRotationComplete");
//            }
//
//            @Override
//            public void rotationTimeout(int angle) {
//                mLastRotationAngle = angle;
//                transition("evRotationTimeout");
//            }
//        });
//

        // And add a listener to the state machine to send the state transitions to telemtry
        mSpeedBotAutoSM.addStateChangeListener(new PropertyChangeListener() {
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
            Class context = mSpeedBotAutoSM.getClass();
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

    @Override
    public void moveComplete() {
        transition("evCraneMoveComplete");
    }

    @Override
    public void moveTimeoutFailure() {
        transition("evCraneMoveTimeoutFailure");

    }

    /**
     * Raises or lowers the crane.
     */
    public void moveCrane(double height){
        if (mSpeedBot instanceof CraneSpeedBot){
            ((CraneSpeedBot)mSpeedBot).getCrane().moveByEncoder(1.0d,height);
        }
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
        if (mSpeedBotAutoSM.isInTransition() == false)
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
                    transition.invoke(mSpeedBotAutoSM, args);
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
    public void loop(){
        // If we haven't started then kick if off.
        boolean triggerStart = false;
        if (mSpeedBotAutoSM != null){
            if (mSpeedBotAutoSM.getState() == SpeedBotAutoStateMachineContext.SpeedBotAutoStateMachine.Idle){
                triggerStart = true;
            }
        }
        // Start event depends upon the selected sequence
        if (triggerStart){
            switch(mSequence) {
                case SEQUENCE_DRAG_FOUNDATION:
                    transition("evStartDragFoundation");
                    break;
                case SEQUENCE_GET_STONE:
                    transition("evStartDriveToStones");
                    break;
                case SEQUENCE_PIBOTS_BRIDGE_PARK:
                    transition("evStartPiGearsBridgePark");
                    break;
                case SEQUENCE_TECHNOLOGIC_BRIDGE_PARK:
                    transition("evStartTechnoLogicsBridgePark");
                    break;
            }
        }

        // Service all the timers in order to trigger any timeout callbacks/events
        serviceTimers();
        // Call loop method on drivetrain to support drive and rotation controls
        mMecanumDrive.loop();
    }

     /**
     * Reads the color sensor to check if we have found a skystone in
     * front of us
     */
    public void checkStoneRecognition(){
 /**       List<Recognition> list = mVuforia.getRecognitions();
        if (list == null){
            transition("evNoStoneFound");
            return;
        }
        // Try to find a Skystone first
        for (Iterator<Recognition> riter = list.iterator(); riter.hasNext(); ) {
            Recognition rec = riter.next();
            if (rec.getLabel().equalsIgnoreCase(VuforiaTargetLocator.SKYSTONE_TFOD_LABEL)) {
                mSkystoneRecognition = rec;
                mStoneRecognition = null;
                transition("evSkystoneFound");
                return;
            }
        }
        // If control to here, then try to find a Stone
        for (Iterator<Recognition> riter = list.iterator(); riter.hasNext(); ) {
            Recognition rec = riter.next();
            if (rec.getLabel().equalsIgnoreCase(VuforiaTargetLocator.STONE_TFOD_LABEL)) {
                mStoneRecognition = rec;
                mSkystoneRecognition = null;
                transition("evStoneFound");
                return;
            }
        }

        mSkystoneRecognition = null;
        mStoneRecognition = null;
        transition("evNoStoneFound");
  **/
    }


    /**
     * Called from state machine to open the hook(s) on either supported bot
     */
    public void openHook(){
        if (mSpeedBot != null){
            mSpeedBot.getFrontHooks().openHooks();
        }
    }
    /**
     * Called from state machine to close the hook(s) on either supported bot
     */
    public void closeHook(){
        if (mSpeedBot != null){
            mSpeedBot.getFrontHooks().closeHooks();
        }
    }

    /**
     * opens the hand on the speed bot.
     */
    public void openHand(){
        if (mSpeedBot != null){
            if (mSpeedBot instanceof CraneSpeedBot) {
                ((CraneSpeedBot)mSpeedBot).getCrane().setHandPosition(Crane.HAND_OPEN);
            }
        }
    }
    /**
     * opens the hand on the speed bot.
     */
    public void closeHand(){
        if (mSpeedBot != null){
            if (mSpeedBot instanceof CraneSpeedBot) {
                ((CraneSpeedBot)mSpeedBot).getCrane().setHandPosition(Crane.HAND_CLOSED);
            }
        }
    }
    /**
     * retracts the hand on the speed bot.
     */
    public void retractHand(){
        if (mSpeedBot != null){
            if (mSpeedBot instanceof CraneSpeedBot) {
                ((CraneSpeedBot)mSpeedBot).getCrane().setHandPosition(Crane.HAND_RETRACTED);
            }
        }
    }




    /**
     * rotation by degrees.  + is ccw
     */
    public void rotate(int degrees){
//        mMecanumDrive.rotate(degrees, MAX_ROTATION_POWER,ROTATION_TIMEOUTMS);
    }

    /**
     * @return IMU angle reached by last rotate
     */
    public int getLastRotationAngle(){
        return mLastRotationAngle;
    }
    /**
     * @return distance from last encoder drive
     */
    public double getLastDriveDistance(){
        return mLastDriveDistance;
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
        mMecanumDrive.stop();
        if (mSpeedBot instanceof CraneSpeedBot) {
            ((CraneSpeedBot) mSpeedBot).getCrane().stop();
        }
    }

    /**
     * Starts a timer that will fire the evTimeout event
     * @param timeoutms
     */
    public void startTimer(int timeoutms){
       mTimer.setTimeout(timeoutms);
       mTimer.start();
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

