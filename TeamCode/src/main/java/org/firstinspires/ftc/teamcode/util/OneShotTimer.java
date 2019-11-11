package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * One-shot timer that triggers a IOneShotTimerCallback after the timeout from .start occurs.
 */
public class OneShotTimer {
    private double mTimeout;
    private IOneShotTimerCallback callback;
    private ElapsedTime mElapsedTime = null;

    private boolean mIsEnabled = false;

    public OneShotTimer(double timeout, IOneShotTimerCallback callback){
        this.callback = callback;
        mTimeout = timeout;
        mElapsedTime = new ElapsedTime();
    }

    public interface IOneShotTimerCallback {
        void timeoutComplete();
    }

    /**
     * changes the timeout to the supplied value
     */
    public void setTimeout(double timeout){
        mTimeout = timeout;

    }
    /**
     * Returns true if running, false if expired
     */
    public boolean isRunning(){
        if (mIsEnabled){
            return mElapsedTime.time() < mTimeout;
        }
        return false;
    }
    /**
     * Re-enables and restarts the timer.
     */
    public void start(){
        mElapsedTime.reset();
        mIsEnabled = true;
    }

     public void checkTimer(){
        if (mIsEnabled) {
            if (mElapsedTime.time() >= mTimeout){
                // Disable for next time and make the callback
                mIsEnabled = false;
                callback.timeoutComplete();
            }
        }
    }

  /**  public static void main(String[] args){
        OneShotTimer timer = new OneShotTimer(0.700d, new IOneShotTimerCallback() {

            @Override
            public void timeoutComplete() {
                System.out.println("complete");
            }
        });
        timer.start();
        System.out.println("Starting");
        while(timer.isRunning()) {
            timer.checkTimer();
        }

    }
   **/
}
