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
import org.firstinspires.ftc.teamcode.speedbot.SpeedBot;
import org.firstinspires.ftc.teamcode.grabberbot.MecanumGrabberBot;
import org.firstinspires.ftc.teamcode.autonomous.TargetPosition;
import org.firstinspires.ftc.teamcode.autonomous.VuforiaTargetLocator;
import org.firstinspires.ftc.teamcode.drivetrain.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.drivetrain.IDriveSessionStatusListener;
import org.firstinspires.ftc.teamcode.drivetrain.IRotationStatusListener;

import java.util.Iterator;
import java.util.List;


/**
This class tests using the Vuforia system to locate a stone.
  */

@TeleOp(name="TestFindStone", group="Robot")
//@Disabled
public class TestFindStone extends OpMode{

    private MecanumGrabberBot mGrabberBot = null;
    private SpeedBot mSpeedBot = null;

    private BaseMecanumDrive mMecanumDrive = null;

    private long lastTime = 0;
    VuforiaTargetLocator mVuforiaLocator = null;

    private static final boolean ENABLE_VUFORIA_TELEMETRY = false;

    private static final boolean USE_GRABBER_BOT = true;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Have to extend stuck detect because the IMU can take a long time to initialize
        msStuckDetectInit = 40000;

        /* Initialize the hardware variables.
         * The initIMU() method of the hardware class does all the work here
         */
        String initErrs = "";
        try {
            if (USE_GRABBER_BOT){
                mGrabberBot = new MecanumGrabberBot(this, true);
                mGrabberBot.init();
                mMecanumDrive = mGrabberBot.getDrivetrain();
            }
            else{
                mSpeedBot = new SpeedBot(this, true);
                mSpeedBot.init();
                mMecanumDrive = mSpeedBot.getDrivetrain();
            }
        }
        catch(Exception e){
            initErrs += e.getMessage();
         }
        // Initialize Vuforia
         mVuforiaLocator = new VuforiaTargetLocator();
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
        if (mVuforiaLocator.mInitialized){
            mVuforiaLocator.activate();
        }
        mMecanumDrive.addDriveSessionStatusListener(new IDriveSessionStatusListener() {
            @Override
            public void driveComplete() {
                telemetry.addData("driveComplete Called","");
                telemetry.update();
            }

            @Override
            public void driveByEncoderTimeoutFailure() {
                telemetry.addData("driveComplete Called","");
                telemetry.update();
            }
        });
        mMecanumDrive.addRotationStatusListener(new IRotationStatusListener() {
            @Override
            public void rotationComplete() {
                telemetry.addData("rotationComplete Called","");
                telemetry.update();
            }
        });

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
        mMecanumDrive.loop();

        double yoffset = 0d;
        double xoffset = 0d;
        List<TargetPosition> targets = mVuforiaLocator.getTargets();
        if (ENABLE_VUFORIA_TELEMETRY)
            telemetry.addData("","Numtargets="+targets.size());
        if (targets.size() > 0) {
            for (Iterator<TargetPosition> iter = targets.iterator(); iter.hasNext(); ) {
                TargetPosition position = iter.next();
                if (ENABLE_VUFORIA_TELEMETRY)
                    telemetry.addData(position.targetLabel+" x,y,O,H=", "%.1f, %.1f, %.1f, %.0f", position.x, position.y, position.orientation, position.heading);
                if (position.targetLabel.equalsIgnoreCase(VuforiaTargetLocator.TARGET_SKYSTONE)) {
                    if (Math.abs(position.y) > Math.abs(yoffset)) {
                        yoffset = position.y;
                        xoffset = position.x;
                    }
                }
            }
        }
        List<Recognition> recognitions = mVuforiaLocator.getRecognitions();
        if (ENABLE_VUFORIA_TELEMETRY)
            telemetry.addData("NumRecognitions="+recognitions.size(),"");
        if (recognitions.size() > 0){
            for(Iterator<Recognition>iter=recognitions.iterator();iter.hasNext();){
                Recognition recog = iter.next();
                if (ENABLE_VUFORIA_TELEMETRY)
                    telemetry.addData("Recognition: "+recog.getLabel()+" angle=","%.0f",recog.estimateAngleToObject(AngleUnit.DEGREES));
            }
        }
        if (ENABLE_VUFORIA_TELEMETRY)
            telemetry.update();
//        List<Recognition>recList = mVuforiaLocator.getRecognitions();
//        if (!recList.isEmpty()) {
//            Recognition recog = recList.get(0);
//            if (recog.getLabel().equalsIgnoreCase(VuforiaTargetLocator.SKYSTONE_TFOD_LABEL)){
//                VuforiaTargetLocator.TargetPosition location = mVuforiaLocator.getTargets();
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
        if (gamepad1.a){
            mMecanumDrive.rotate(90);
        }
        if (gamepad1.x){
            mMecanumDrive.strafeEncoder(1.0d,xoffset,5000);
        }
        if (gamepad1.y){
 //           mMecanumDrive.driveEncoder(1.0d,yoffset,2000);
            mMecanumDrive.driveEncoder(1.0d,12d,2000);
        }
        if (gamepad1.b){
            mMecanumDrive.strafeEncoder(1.0d,12d,10000);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        mMecanumDrive.stop();
    }


 }