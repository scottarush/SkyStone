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

    private static boolean TELEMETRY_STATE_LOGGING_ENABLED = true;

    private GrabberBotAutoStateMachineContext mGrabberBotAutoSM = null;
    private SpeedBotAutoStateMachineContext mSpeedBotAutoSM = null;

    private OpMode opMode = null;

    private boolean mBlueAlliance = false;

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
            if (mSpeedBot != null){
                transition("evHandTimeout");
            }

        }
    });


    private VuforiaTargetLocator mVuforia = null;
    /***
     * Needed to queue events into the state machine
     */
    private static HashMap<String, Method> mTransition_map;
    private LinkedList<String> mTransition_queue;

    /**
     * Constructor for use with SpeedBot
     * @param opMode
     * @param speedBot
     * @param vuforia
     * @param blueAlliance
     */
    public AutonomousController(final OpMode opMode, SpeedBot speedBot, VuforiaTargetLocator vuforia, boolean blueAlliance) {
        mSpeedBotAutoSM = new SpeedBotAutoStateMachineContext(this);
        this.opMode = opMode;
        this.mBlueAlliance = blueAlliance;
        mSpeedBot = speedBot;
        mVuforia = vuforia;

        mMecanumDrive = speedBot.getDrivetrain();

        mSpeedBot.getCrane().addCraneMovementStatusListener(this);
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
    public AutonomousController(final OpMode opMode, MecanumGrabberBot grabberBot, VuforiaTargetLocator vuforia, boolean blueAlliance) {
        mGrabberBotAutoSM = new GrabberBotAutoStateMachineContext(this);
        this.opMode = opMode;
        this.mBlueAlliance = blueAlliance;
        mGrabberBot = grabberBot;
        mVuforia = vuforia;

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
        mMecanumDrive.addRotationStatusListener(new IRotationStatusListener() {
            @Override
            public void rotationComplete() {
                transition("evRotationComplete");
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
         if (mGrabberBotAutoSM != null){
             if (mGrabberBotAutoSM.getState() == GrabberBotAutoStateMachineContext.GrabberBotAutoStateMachine.Idle){
                 transition("evStart");
             }
         }
         else if (mSpeedBotAutoSM != null){
             if (mSpeedBotAutoSM.getState() == SpeedBotAutoStateMachineContext.SpeedBotAutoStateMachine.Idle){
                 transition("evStart");
             }
         }

        // Service all the timers in order to trigger any timeout callbacks/events
        serviceTimers();
        // Call loop method on drivetrain to support drive and rotation controls
        mMecanumDrive.loop();
        // Call loop on Crane to service automatic movement
         mSpeedBot.getCrane().loop();
     }

    public boolean isAutonomousComplete(){
         return false;
 /*       if (mGrabberBotAutoSM != null){
            if (mGrabberBotAutoSM.getState() == GrabberBotAutoStateMachineContext.GrabberBotAutoStateMachine.){
                transition("evStart");
            }
        }
        else if (mSpeedBotAutoSM != null){
            if (mSpeedBotAutoSM.getState() == SpeedBotAutoStateMachineContext.SpeedBotAutoStateMachine.Idle){
                transition("evStart");
            }
        }
**/
    }


    private void startDriveByEncoder(double speed, double linearDistance, int timeoutms){
        if (mMecanumDrive.isMoving()){
            return;  // Already driving, this is an error, ignore
        }
        mMecanumDrive.driveEncoder(speed,linearDistance,timeoutms);
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
     *
     */
    public void linearDrive(double distance){
        startDriveByEncoder(1.0d, distance,3000);
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
            mSpeedBot.getCrane().openHand();
        }
    }
    /**
     * opens the hand on the speed bot.
     */
    public void closeHand(){
        if (mSpeedBot != null){
            mSpeedBot.getCrane().closeHand();
        }
    }


    /**
     * rotation by degrees.  + is ccw
     */
    public void rotate(int degrees){
        mMecanumDrive.rotate(degrees);
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

