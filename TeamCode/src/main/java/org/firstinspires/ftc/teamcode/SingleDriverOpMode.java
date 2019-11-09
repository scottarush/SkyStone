/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This is the class to be used for the single driver competition mode.
 * Layout is:
 * Tank drive on both joysticks.
 * Grabber on both sets of bumpers and triggers.
 * Arm up down on pad.
 */
@TeleOp(name="SingleDriverOpMode", group="Robot")
//@Disabled
public class SingleDriverOpMode extends OpMode{

    private MecanumGrabberBot robot  = null;

    public static final double DELTA_ARM_ANGLE_STEP = 5.0d;

    public ElapsedTime mRetractButtonHoldTimer = new ElapsedTime();

     private boolean mAButtonPressed = false;

    private static final double RETRACT_BUTTON_HOLD_TIME = 1.0d;
    private boolean mRetractButtonPressed = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot = new MecanumGrabberBot(this, true);
        /*
         * Initialize the robot.  Be sure to catch exception and dump out as
         * the exception string will have details of what didn't initialize and
         * we can figure out what went wrong.
         */
        String statusMsg = "Success";
        try{
            robot.init();
        }
        catch(Exception e){
            statusMsg = "InitErrors:"+e.getMessage();
        }
        telemetry.addData("Status",statusMsg);
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        double xleft = gamepad1.left_stick_x;
        double yleft = -gamepad1.left_stick_y;
        double xright = gamepad1.right_stick_x;
        double yright = -gamepad1.right_stick_y;

        // the speeds with the new gamepad inputs
        robot.getDrivetrain().setTankDriveJoystickInput(xleft,yleft,xright,yright);

        boolean leftBumper = gamepad1.left_bumper || gamepad2.right_bumper;
        boolean rightBumper = gamepad1.right_bumper || gamepad2.right_bumper;

        double leftTrigger = gamepad1.left_trigger;
        if (Math.abs(leftTrigger) < 0.05d){
            leftTrigger = gamepad2.left_trigger;
        }
        double rightTrigger = gamepad1.right_trigger;
        if (Math.abs(rightTrigger) < 0.05d){
            rightTrigger = gamepad2.right_trigger;
        }

        // Read the bumpers and triggers for the grabber.
         robot.getGrabber().moveGrabber(leftBumper,rightBumper,leftTrigger,rightTrigger);

        // Do arm handling
        // If a reset is in progress then call teh service routine in the arm
        Gamepad armPad = gamepad2;
        if (robot.getArm().isResetToRetractInProgress()){
            int retcode = robot.getArm().resetToRetractPosition();
        }
        else{
            if (robot.getArm().isAngleMode()) {
                // Read gamepad and move delta degrees per press
                if (armPad.dpad_up) {
                    if (!robot.getArm().isArmMoving()) {
                        // Arm has stopped.  Move it another 5 degrees.
                        robot.getArm().moveDeltaAngle(DELTA_ARM_ANGLE_STEP);
                    }
                } else if (armPad.dpad_down) {
                    if (!robot.getArm().isArmMoving()) {
                        // Arm has stopped.  Move it another 5 degrees.
                        robot.getArm().moveDeltaAngle(-DELTA_ARM_ANGLE_STEP);
                    }
                }
            } else {
                // Manual arm mode read the d-pad up and down for the arm moti
                if (armPad.dpad_up) {
                    // While dpad-up pressed move the arm in up direction
                    robot.getArm().moveArm(true);
                } else if (armPad.dpad_down) {
                    robot.getArm().moveArm(false);
                } else {
                    robot.getArm().stop();
                }
            }
        }
        // Do the hook
        processHookPosition();
         // Do the claw
        if (gamepad1.x){
            // Open the clase
            robot.getArm().setClaw(true);
        }
        else if (gamepad1.b){
            robot.getArm().setClaw(false);
        }

    }

    /**
     * Processes the hook buttons y and a
     */
    private void processHookPosition(){
        boolean yButtonState = (gamepad2.y || gamepad1.y);
        if (yButtonState){
            // If the robot isn't retracted then check for a push and hold
            if (robot.getHook().getPosition() == Hook.OPEN){
                if (!mRetractButtonPressed) {
                    // This is an initial press of the y button so start the timer
                    mRetractButtonHoldTimer.reset();
                    mRetractButtonPressed = true;
                }
                else if (mRetractButtonHoldTimer.time() > RETRACT_BUTTON_HOLD_TIME){
                    robot.getHook().setPosition(Hook.RETRACTED);
                }
            }
            else if (robot.getHook().getPosition() == Hook.CLOSED){
                robot.getHook().setPosition(Hook.OPEN);
            }
        }
        else{
            // y button released.  Just clear the press flag
            mRetractButtonPressed = false;
        }
        // Now do a button
        boolean aButtonState = (gamepad1.a || gamepad2.a);
        if (aButtonState){
             if (robot.getHook().getPosition() == Hook.RETRACTED){
                 if (!mAButtonPressed) {
                     // This is an initial press of the a button so go to Open
                     robot.getHook().setPosition(Hook.OPEN);
                     // And set the flag to debounce
                     mAButtonPressed = true;
                 }
            }
            else if (robot.getHook().getPosition() == Hook.OPEN){
                if (!mAButtonPressed) {
                    robot.getHook().setPosition(Hook.CLOSED);
                    mAButtonPressed = true;
                }
            }
        }
        else{
            // Clear the flag for the next press
            mAButtonPressed = false;
        }

    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.getDrivetrain().stop();
    }


}
