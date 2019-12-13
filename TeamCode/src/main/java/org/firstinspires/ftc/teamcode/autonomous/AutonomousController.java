package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.grabberbot.Hook;
import org.firstinspires.ftc.teamcode.drivetrain.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.drivetrain.IDriveSessionStatusListener;
import org.firstinspires.ftc.teamcode.drivetrain.IRotationStatusListener;
import org.firstinspires.ftc.teamcode.grabberbot.MecanumGrabberBot;
import org.firstinspires.ftc.teamcode.speedbot.Crane;
import org.firstinspires.ftc.teamcode.speedbot.ICraneMovementStatusListener;
import org.firstinspires.ftc.teamcode.speedbot.SpeedBot;
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

public class AutonomousController implements ICraneMovementStatusListener {
    public static final int SEQUENCE_DRAG_FOUNDATION = 0;
    public static final int SEQUENCE_GET_STONE = 1;

    private static boolean TELEMETRY_STATE_LOGGING_ENABLED = true;

    private GrabberBotAutoStateMachineContext mGrabberBotAutoSM = null;
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
    private SpeedBot mSpeedBot = null;
    /**
     * This reference will be non-null when using the grabber bot
     */
    private MecanumGrabberBot mGrabberBot = null;

    /**
     * Common reference used for the MecanumDrive on either bot
     */
    private BaseMecanumDrive mMecanumDrive = null;

    private Recognition mSkystoneRecognition = null;

    private Recognition mStoneRecognition = null;

    private ArrayList<OneShotTimer> mStateTimers = new ArrayList<>();

    /**
     * Common timer used to open or close the hooks on either bot
     */
    private OneShotTimer mHookTimer = new OneShotTimer(1000, new OneShotTimer.IOneShotTimerCallback() {
        @Override
        public void timeoutComplete() {
            transition("evHookTimeout");
        }
    });

    /**
     * Timmer for grabber only on grabber bot
     */
    private OneShotTimer mGrabberTimer = new OneShotTimer(5000, new OneShotTimer.IOneShotTimerCallback() {
        @Override
        public void timeoutComplete() {
            if (mGrabberBot != null)
                mGrabberBot.getGrabber().stop();

        }
    });

    /**
     * Timmer for hand only on speed bot
     */
    private OneShotTimer mHandTimer = new OneShotTimer(1000, new OneShotTimer.IOneShotTimerCallback() {
        @Override
        public void timeoutComplete() {
            transition("evHandTimeout");
        }
    });

    /**
     * 3 second timer to lower the crane
     */
    private OneShotTimer mCraneTimer = new OneShotTimer(2000, new OneShotTimer.IOneShotTimerCallback() {
        @Override
        public void timeoutComplete() {
            transition("evCraneTimeout");
        }
    });



    private VuforiaTargetLocator mVuforia = null;
    /***
     * Needed to queue events into the state machine
     */
    private static HashMap<String, Method> mTransition_map;
    private LinkedList<String> mTransition_queue;

    private int mLastRotationAngle = 0;
    private double mLastDriveDistance = 0d;
    private int mSequence = SEQUENCE_DRAG_FOUNDATION;

    /**
     * Constructor for use with SpeedBot
     * @param opMode
     * @param speedBot
     * @param vuforia
     * @param blueAlliance
     */
    public AutonomousController(final OpMode opMode,
                                SpeedBot speedBot,
                                VuforiaTargetLocator vuforia,
                                boolean blueAlliance,
                                int sequence) {
        mSpeedBotAutoSM = new SpeedBotAutoStateMachineContext(this);
        this.opMode = opMode;
        this.mBlueAlliance = blueAlliance;
        mSpeedBot = speedBot;
        mVuforia = vuforia;
        mSequence = sequence;

        mMecanumDrive = speedBot.getDrivetrain();

        mSpeedBot.getCrane().addCraneMovementStatusListener(this);

        // Add timers to be checked
        mStateTimers.add(mHandTimer);
        mStateTimers.add(mHookTimer);
        mStateTimers.add(mCraneTimer);

        // Now do common initializations
        init();
    }

    /**
     * Constructor for use with MecanumGrabberBot
     * @param opMode
     * @param grabberBot
     * @param vuforia
     * @param blueAlliance
     */
    public AutonomousController(final OpMode opMode, MecanumGrabberBot grabberBot, VuforiaTargetLocator vuforia, boolean blueAlliance,int sequence) {
        mGrabberBotAutoSM = new GrabberBotAutoStateMachineContext(this);
        this.opMode = opMode;
        this.mBlueAlliance = blueAlliance;
        mGrabberBot = grabberBot;
        mVuforia = vuforia;
        mSequence = sequence;

        mMecanumDrive = mGrabberBot.getDrivetrain();

        // Add all the timers to the state timers so that they get service each loop
        mStateTimers.add(mHookTimer);
        mStateTimers.add(mGrabberTimer);

        // Now do common initializations
        init();
    }

    /**
     * Does initializations common to both robots.
     */
    private void init(){
        buildTransitionTable();

        // Add listeners to drivetrain for callbacks in order to translate into state machine events
        mMecanumDrive.addDriveSessionStatusListener(new IDriveSessionStatusListener() {
            @Override
            public void driveComplete(double distance) {
                mLastDriveDistance = distance;
                transition("evDriveComplete");
            }

            @Override
            public void driveByEncoderTimeoutFailure(double distance) {
                mLastDriveDistance = distance;
                transition("evDriveTimeout");
            }
        });
        mMecanumDrive.addRotationStatusListener(new IRotationStatusListener() {
            @Override
            public void rotationComplete(int angle) {
                mLastRotationAngle = angle;
                transition("evRotationComplete");
            }

            @Override
            public void rotationTimeout(int angle) {
                mLastRotationAngle = angle;
                transition("evRotationTimeout");
            }
        });


        // And add a listener to the state machine to send the state transitions to telemtry
        getFSMContext().addStateChangeListener(new PropertyChangeListener() {
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
     * helper function returns the valid FSMContext based on which robot was initialized
     */
    private FSMContext getFSMContext(){
        FSMContext context = null;
        if (mGrabberBotAutoSM != null){
            context = mGrabberBotAutoSM;
        }
        else if (mSpeedBotAutoSM != null){
            context = mSpeedBotAutoSM;
        }
        return context;
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
            Class context = getFSMContext().getClass();
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
        if (mSpeedBot != null){
            mSpeedBot.getCrane().moveByEncoder(1.0d,height);
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
        if (getFSMContext().isInTransition() == false)
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
                    transition.invoke(getFSMContext(), args);
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
        if (mGrabberBotAutoSM != null){
            if (mGrabberBotAutoSM.getState() == GrabberBotAutoStateMachineContext.GrabberBotAutoStateMachine.Idle) {
                triggerStart = true;
            }
        }
        else if (mSpeedBotAutoSM != null){
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
            }
        }

        // Service all the timers in order to trigger any timeout callbacks/events
        serviceTimers();
        // Call loop method on drivetrain to support drive and rotation controls
        mMecanumDrive.loop();
        // Call loop on Crane to service automatic movement
        mSpeedBot.getCrane().loop();
    }


    /**
     * start grabber turns the grab on forward or reverse.  does nothing if not using GrabberBot
     */
    public void startGrabber(boolean intake, int timeoutms){
        if (mGrabberBot == null)
            return;
        if (intake){
            mGrabberBot.getGrabber().moveGrabber(false,false, 1.0,1.0);
        }
        else{
            // Spit it out
            mGrabberBot.getGrabber().moveGrabber(true,true, 0.0,0.0);

        }
        mGrabberTimer.setTimeout(timeoutms);
        mGrabberTimer.start();
    }


    public void stopGrabber(){
        if (mGrabberBot == null)
            return;
        mGrabberBot.getGrabber().stop();
    }

    /**
     * This is a hack to get us through the bug blocking competition.
     * Just does a short drive with a 500 ms timeout.
     */
    public void linearDriveBugUnblocker(){
        mMecanumDrive.driveEncoder(1.0d,0.5d,500);
    }
    /**
     * drives forward at full speed
     */
    public void linearDrive(double distance){
         mMecanumDrive.driveEncoder(1.0d,distance,2000);
    }

    /**
     * drives forward at 1/2 speed
     */
    public void linearDriveSlow(double distance){
        mMecanumDrive.driveEncoder(0.5d,distance,2000);
    }

    /**
     * Strafes either right + or left -
     */
    public void strafeDrive(double distance){
        mMecanumDrive.strafeEncoder(1.0d,distance,2000);
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
    }

    /**
     * Strafes sideways to a Skystone based only on recognition.
     * triggers:
     * evDriveComplete when the drive is finished.
     * @param linearDistance  straight y distance to candidate skystone
     * @param timeoutms timeout in ms
     */
    public void strafeToSkystone(double linearDistance,int timeoutms){
        if (mSkystoneRecognition == null){
            transition("evDriveComplete");
        }
        double angle = mSkystoneRecognition.estimateAngleToObject(AngleUnit.RADIANS);
        double strafeDistance = linearDistance /Math.tan(angle);
        mMecanumDrive.strafeEncoder(1.0d,strafeDistance,timeoutms);
    }

    /**
     * Strafes sideways to a Skystone based only on recognition.
     * triggers:
     * evDriveComplete when the drive is finished.
     * @param linearDistance straight y distance to candidate stone
     * @param timeoutms timeout in ms
     */
    public void strafeToStone(double linearDistance,int timeoutms){
        if (mStoneRecognition == null){
            transition("evDriveComplete");
        }
        double angle = mStoneRecognition.estimateAngleToObject(AngleUnit.RADIANS);
        double strafeDistance = linearDistance /Math.tan(angle);
        mMecanumDrive.strafeEncoder(1.0d,strafeDistance,timeoutms);
    }

    /**
     * Called from state machine to open the hook(s) on either supported bot
     */
    public void openHook(){
        if (mGrabberBot != null) {
            mGrabberBot.getHook().setPosition(Hook.OPEN);
        }
        if (mSpeedBot != null){
            mSpeedBot.getFrontHooks().openHooks();
        }
    }
    /**
     * Called from state machine to close the hook(s) on either supported bot
     */
    public void closeHook(){
        if (mGrabberBot != null) {
            mGrabberBot.getHook().setPosition(Hook.CLOSED);
        }
        if (mSpeedBot != null){
            mSpeedBot.getFrontHooks().closeHooks();
        }
    }

    /**
     * opens the hand on the speed bot.
     */
    public void openHand(){
        if (mSpeedBot != null){
            mSpeedBot.getCrane().setHandPosition(Crane.HAND_OPEN);
        }
    }
    /**
     * opens the hand on the speed bot.
     */
    public void closeHand(){
        if (mSpeedBot != null){
            mSpeedBot.getCrane().setHandPosition(Crane.HAND_CLOSED);
        }
    }
    /**
     * retracts the hand on the speed bot.
     */
    public void retractHand(){
        if (mSpeedBot != null){
            mSpeedBot.getCrane().setHandPosition(Crane.HAND_RETRACTED);
        }
    }




    /**
     * rotation by degrees.  + is ccw
     */
    public void rotate(int degrees){
        mMecanumDrive.rotate(degrees, MAX_ROTATION_POWER,ROTATION_TIMEOUTMS);
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
        mSpeedBot.getCrane().stop();
    }

    public void startHookTimer(){
       mHookTimer.start();
    }
    public void startHandTimer(){
        mHandTimer.start();
    }
    public void startCraneTimer(){
        mCraneTimer.start();
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

