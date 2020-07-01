package org.firstinspires.ftc.teamcode.filter.devstatemachines;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drivetrain.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.drivetrain.IDriveSessionStatusListener;
import org.firstinspires.ftc.teamcode.drivetrain.IRotationStatusListener;
import org.firstinspires.ftc.teamcode.speedbot.BaseSpeedBot;
import org.firstinspires.ftc.teamcode.speedbot.CraneSpeedBot;
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

public class FilterDevController {
    private static boolean TELEMETRY_STATE_LOGGING_ENABLED = true;

    private FilterDevStateMachineContext mStateMachineContext = null;

    private OpMode opMode = null;

    /**
     * Max rotation power.
     */
    private static final double MAX_ROTATION_POWER = 0.75d;

    /**
     * rotation timeout. same value used for all transitions
     */
    private static final int ROTATION_TIMEOUTMS = 1000;

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

    /**
     * Constructor
     * @param opMode
     * @param speedBot
     */
    public FilterDevController(final OpMode opMode,
                               BaseSpeedBot speedBot) {
        mStateMachineContext = new FilterDevStateMachineContext(this);
        this.opMode = opMode;
        mSpeedBot = speedBot;

        mMecanumDrive = speedBot.getDrivetrain();

        // Add timers to be checked
        mStateTimers.add(mTimer);

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
        mStateMachineContext.addStateChangeListener(new PropertyChangeListener() {
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
            Class context = mStateMachineContext.getClass();
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
        if (mStateMachineContext.isInTransition() == false)
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
                    transition.invoke(mStateMachineContext, args);
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
        if (mStateMachineContext != null){
            if (mStateMachineContext.getState() == FilterDevStateMachineContext.FilterDevStateMachine.Idle){
                triggerStart = true;
            }
        }
        // Start event depends upon the selected sequence
        if (triggerStart){
            transition("");
          }

        // Service all the timers in order to trigger any timeout callbacks/events
        serviceTimers();
        // Call loop method on drivetrain to support drive and rotation controls
        mMecanumDrive.loop();
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
     * rotation by degrees.  + is ccw
     */
    public void rotate(int degrees){
        mMecanumDrive.rotate(degrees, MAX_ROTATION_POWER,ROTATION_TIMEOUTMS);
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

