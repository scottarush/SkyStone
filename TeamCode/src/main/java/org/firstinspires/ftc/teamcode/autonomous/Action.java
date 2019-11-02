package org.firstinspires.ftc.teamcode.autonomous;

/**
 * Class encapsulating an Action within a route.
 */
public abstract class Action {

    public Action(){

    }

    /**
     *
     * @return true if the action was started, false if it can not be executed.
     */
    public abstract boolean startAction();

    /**
     *
     * @return true if action has finished, false if not.
     */
    public boolean isActionComplete(){
        return true;
    }
}
