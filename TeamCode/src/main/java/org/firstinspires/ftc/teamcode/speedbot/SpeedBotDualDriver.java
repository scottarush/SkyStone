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

package org.firstinspires.ftc.teamcode.speedbot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This is the class to be used for the teleop dual driver mode with theSpeedBot
 * Control layout is:
 * 1. Tank drive on both joysticks.  Gamepad 1 is primary but if both joysticks are idle,
 *    then Gamepad2 joysticks work otherwise ignored.
 * 2. left bumper on either controller closes hand
 * 3. right bumper on either controller closes hand
 * 4. left trigger on either controller lowers crane
 * 5. right trigger on either controller raises crane.
 * 6. y button on either controller closes both front hooks
 * 7. a button on either controller opens both front hooks
 */
@TeleOp(name="SpeedBotDualDriver", group="Robot")
//@Disabled
public class SpeedBotDualDriver extends OpMode{

    private SpeedBot robot  = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot = new SpeedBot(this,false);
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

        // Allow gamepad 2 if gamepad1 all zeros
        if ((xleft == 0d) && (yleft == 0d) && (xright == 0d) && (yright == 0d)){
            xleft = gamepad2.left_stick_x;
            yleft = -gamepad2.left_stick_y;
            xright = gamepad2.right_stick_x;
            yright = -gamepad2.right_stick_y;

        }
        robot.getDrivetrain().setTankDriveJoystickInput(xleft,yleft,xright,yright);

        // Use the bumpers to open and close the hand on the crane
        boolean leftBumper = gamepad1.left_bumper || gamepad2.left_bumper;
        boolean rightBumper = gamepad1.right_bumper || gamepad2.right_bumper;
        if (leftBumper) {
            if (!rightBumper) {
                robot.getCrane().closeHand();
            }
        }
        if (rightBumper){
            if (!leftBumper){
                robot.getCrane().openHand();
            }
        }

        // Use the triggers to raise and lower the crane.  Allow either controller to
        // raise and lower, but if both triggers are press just stop the crane motor
        double triggerThreshold = 0.05d;
        double leftTrigger = gamepad1.left_trigger;
        if (Math.abs(leftTrigger) < triggerThreshold){
            leftTrigger = gamepad2.left_trigger;
        }
        double rightTrigger = gamepad1.right_trigger;
        if (Math.abs(rightTrigger) < triggerThreshold){
            rightTrigger = gamepad2.right_trigger;
        }
        if (rightTrigger > triggerThreshold){
            if (leftTrigger < triggerThreshold){
                robot.getCrane().raiseManual(Math.abs(rightTrigger));
            }
            else{
                robot.getCrane().stop();  // both triggers pressed
            }
        }
        else if (leftTrigger > triggerThreshold){
            if (rightTrigger < triggerThreshold){
                robot.getCrane().lowerManual(Math.abs(leftTrigger));
            }
            else {
                robot.getCrane().stop();  // both triggers pressed
            }
        }
        else{
            // Neither trigger so stop the crane
            robot.getCrane().stop();
        }

        // Use the Y and A buttons to close and open the front hooks
        boolean y = gamepad1.y || gamepad2.y;
        boolean a = gamepad1.a || gamepad2.a;
        if (y) {
            robot.getFrontHooks().closeHooks();
        }
        else if (a){
            robot.getFrontHooks().openHooks();
        }

//        // Process the autoramp Crane inputs on either controller's dpad Up or Down
//        boolean dpadUp = gamepad1.dpad_up || gamepad1.dpad_up;
//        boolean dpadDown = gamepad2.dpad_down || gamepad2.dpad_down;
//        if (dpadUp){
//            robot.getCrane().raiseManualRamp();
//        }
//        else if (dpadDown){
//            robot.getCrane().lowerManualRamp();
//        }
//        else {
//            robot.getCrane().stop();
//        }
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.getDrivetrain().stop();
    }


}
