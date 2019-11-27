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

package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.FrameDevelopmentBot;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.MecanumGrabberBot;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.autonomous.VuforiaSkystoneLocator;
import org.firstinspires.ftc.teamcode.drivetrain.BaseMecanumDrive;

import java.util.List;


/**
This class implements the equations that Marcus derived on October 3.
  */

@TeleOp(name="TestFindStone", group="Robot")
//@Disabled
public class TestFindStone extends OpMode{

    private Robot mRobot = null;

    private BaseMecanumDrive mecanumDrive = null;

    private long lastTime = 0;
    VuforiaSkystoneLocator mVuforiaLocator = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Have to extend stuck detect because the IMU can take a long time to initialize
        msStuckDetectInit = 30000;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        String initErrs = "";
        try {
            if (Globals.USE_DEV_FRAME_BOT){
                mRobot = new FrameDevelopmentBot(this);
            }
            else {
                mRobot = new MecanumGrabberBot(this,true);
            }
            // Get utility variable
            mecanumDrive = (BaseMecanumDrive)mRobot.getDrivetrain();
            mRobot.init();
        }
        catch(Exception e){
            initErrs += e.getMessage();
         }
        // Initialize Vuforia
         mVuforiaLocator = new VuforiaSkystoneLocator();
        try{
            mVuforiaLocator.init(this);
        }
        catch(Exception e){
            initErrs += e.getMessage();
        }
        if (initErrs.length() > 0){
            telemetry.addData("Robot Init Error",initErrs);
            telemetry.update();
        }
        mVuforiaLocator.activate();

        lastTime = System.currentTimeMillis();
        // Send telemetry message to signify drivetrain waiting;
        telemetry.addData("Say", "Init Complete");
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

        long currentTime = System.currentTimeMillis();
        long delta = currentTime-lastTime;
        mRobot.getDrivetrain().doLoop();

        double yoffset = 0d;
        double xoffset = 0d;
        VuforiaSkystoneLocator.VuforiaPosition position = mVuforiaLocator.getStoneLocation();
        if (position != null){
             telemetry.addData("Skystone x,y,O,H=","%.1f, %.1f, %.1f, %.0f",position.x,position.y,position.orientation,position.heading);
             yoffset = position.y;
             xoffset = position.x;
         }
//        List<Recognition>recList = mVuforiaLocator.getRecognitions();
//        if (!recList.isEmpty()) {
//            Recognition recog = recList.get(0);
//            if (recog.getLabel().equalsIgnoreCase(VuforiaSkystoneLocator.SKYSTONE_TFOD_LABEL)){
//                VuforiaSkystoneLocator.VuforiaPosition location = mVuforiaLocator.getStoneLocation();
//                if (location != null){
//                 }
//                else{
//                    telemetry.addData("Recognize Skystone but no location","");
//                }
//            }
//        }
//        if (gamepad1.b){
//            mRobot.getDrivetrain().driveEncoder(1.0d,12.0d,3000);
//        }
//        if (gamepad1.x){
//            ((BaseMecanumDrive)mRobot.getDrivetrain()).strafeEncoder(1.0d,12.0d,3000);
//        }
        if (gamepad1.x){
            mecanumDrive.strafeEncoder(1.0d,xoffset,5000);
        }
        if (gamepad1.y){
            mecanumDrive.driveEncoder(1.0d,yoffset,2000);
        }
        if (gamepad1.b){
            mecanumDrive.strafeEncoder(1.0d,12d,10000);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        mRobot.getDrivetrain().stop();
    }


 }