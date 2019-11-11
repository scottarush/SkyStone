package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.util.ElapsedTime;

public class StateTimer {
    private double mTimeout;
    private IStateTimerCallback callback;
    private ElapsedTime mElapsedTime = new ElapsedTime();

    public StateTimer(double timeout,IStateTimerCallback callback){
        this.callback = callback;
        mTimeout = timeout;
        mElapsedTime = new ElapsedTime();
        mElapsedTime.reset();
    }

    public interface IStateTimerCallback {
        void timeoutComplete();
    }

    /**
     * Resets the timer to allow reuse.
     */
    public void reset(){
        mElapsedTime.reset();
    }

    public void checkTimer(){
        if (mElapsedTime.time() >= mTimeout){
            callback.timeoutComplete();
        }
    }
}
