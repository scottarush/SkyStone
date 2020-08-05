package org.firstinspires.ftc.teamcode.guidance;

public interface IGuidanceControllerStatusListener {

    /**
     * called when a setTargetHeading triggered rotation is complete.
     */
    void rotationComplete();

    /**
     * Called when a path follow is complete.
     */
    void pathFollowComplete();
}
