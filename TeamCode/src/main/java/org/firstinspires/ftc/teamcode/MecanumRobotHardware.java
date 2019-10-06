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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigationWebcam;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.concurrent.CountDownLatch;

/**
 * This is NOT an opmode.
 *
 * This class define's specific hardware for our team's Mecanum robot.  It was created starting
 * from the HardwareRobot class within the samples directory.
 *
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot"
 * in the samples for help in creating usage.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left front drive motor:        "left_front_drive"
 * Motor channel:  Right front drive motor:  "right_front_drive"
 * Motor channel:  Left rear drive motor:    "left_rear_drive"
 * Motor channel:  Right rear drive motor:   "right_rear_drive"
 *
 */
public class MecanumRobotHardware
{
    /* Public OpMode members. */
    public DcMotor lfMotor = null;
    public DcMotor rfMotor = null;
    public DcMotor lrMotor = null;
    public DcMotor rrMotor = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    OpenGLMatrix lastLocation = null;

    /** Number of encoder counts of each rotation of the shaft.**/
    public static final double ENCODER_COUNTS_PER_ROTATION = 1120;

    /** Wheel circumference in inches **/
    public static final double MECANUM_WHEEL_CIRCUMFERENCE = 12.1211;

    /** Number of counts per inch of direct wheel movement.  **/
    public static final int COUNTS_PER_INCH = (int)Math.round(ENCODER_COUNTS_PER_ROTATION / MECANUM_WHEEL_CIRCUMFERENCE);
    /** Timer for timouts**/
     private ElapsedTime timeoutTimer = new ElapsedTime();
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName;

    /** activity flag set true whenever a utility method is running **/
    private boolean isRunning = false;

    /* Constructor */
    public MecanumRobotHardware(){

    }

    /* Initialize standard Hardware interfaces.
    * NOTE:  This class throws Exception on any hardware init error so be sure to catch and
    * report to Telemttry in your initialization. */
    public void init(HardwareMap ahwMap) throws Exception {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        lfMotor = hwMap.get(DcMotor.class, "left_front_drive");
        lfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rfMotor = hwMap.get(DcMotor.class, "right_front_drive");
        lrMotor = hwMap.get(DcMotor.class, "left_rear_drive");
        lrMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rrMotor = hwMap.get(DcMotor.class, "right_rear_drive");

        // Set all motors to zero power
        setDriveMotorPower(0,0,0,0);

        // Reset and Set all motors to run with encoders.
        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        // TODO Define and initialize ALL installed servos once the actuators are defined by the build team

    }

    /**
     * Helper function to set power to the wheel drive motors
     * @param lf left front motor power
     * @param rf right front motor power
     * @param lr left rear motor power
     * @param rr right rear motor power
     */
    public void setDriveMotorPower(double lf, double rf, double lr, double rr){
        lfMotor.setPower(lf);
        rfMotor.setPower(rf);
        lrMotor.setPower(lr);
        rrMotor.setPower(rr);
    }

    /**
     * helper function to stop all motors on the robot.
     */
    public void stopAll(){
        // Clear isRunning flag
        isRunning = false;

        setDriveMotorPower(0,0,0,0);
     }

    private void initVuforia(){
        /*
         * Retrieve the camera we are to use.
         */
        webcamName = hwMap.get(WebcamName.class, "Webcam 1");

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /** Set the vuforia license key **/
        parameters.vuforiaLicenseKey = "AcKS+1f/////AAABmSfE4zB+90Wek6WzDi8g9Kw5Y7UUvCIs/xewLbEyh1FnM9TKbT+OXm1jp/q0e0G+b3EfoikZfLj1W+tXrZ4vrSJKyIuX/dhgfNzqJiRjnhiM9EWGVpKKQRYaK5Vr6Tp/UUif1/0/g15dgu/Gy4CvEoTUG3BeGGyDZDy9DlyoJImjnf1C0IBTb1kRz5oTW+lyx4AEeuG2a6egQVGU61IbESGMTXKnQxfj9ccnbZdHLHV62WowIoMJJtXDO4jfLcnGmPEr3v60y9ZPzzYifER84G+ulCUxe0ssoxIzRLNyC9FcHuJ11qvk9yGj8rbKclJjhCE4zHjJO7/3wS0/EEWy+iLg32J0IVrPGipUX/Pxn2Z/";


        /**
         * We also indicate which camera on the RC we wish to use. For pedagogical purposes,
         * we use the same logic as in {@link ConceptVuforiaNavigationWebcam}.
         */
        parameters.cameraName = webcamName;
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
    }

    /**
     * This function moves the robot a delta x and y distance.
     * @param opmode OpMode of the caller.  Needed to be able to send Telemetry
     * @param speed speed to move
     * @param xdist distance in x inches to move
     * @param ydist distance in y inches to move
     * @param timeout timeout in seconds to abort if move not completed.
     */
    public void driveByEncoder(OpMode opmode, double speed, double xdist, double ydist, double timeout){

        // Compute the number of encoder counts for each wheel to move the requested distanc
        int lfDeltaCounts = (int)Math.round((xdist+ydist) * COUNTS_PER_INCH);
        int rfDeltaCounts = (int) Math.round((ydist-xdist) * COUNTS_PER_INCH);
        int lrDeltaCounts = (int)Math.round((ydist-xdist) * COUNTS_PER_INCH);
        int rrDeltaCounts = (int)Math.round((xdist+ydist) * COUNTS_PER_INCH);

        // Set target counts for each motor to the above
        lfMotor.setTargetPosition(lfDeltaCounts+lfMotor.getCurrentPosition());
        rfMotor.setTargetPosition(rfDeltaCounts+rfMotor.getCurrentPosition());
        lrMotor.setTargetPosition(lrDeltaCounts+lrMotor.getCurrentPosition());
        rrMotor.setTargetPosition(rrDeltaCounts+rrMotor.getCurrentPosition());

        // Set mode to run to position
        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

         // Reset timer and set motor power
        timeoutTimer.reset();
        double aspeed = Math.abs(speed);
        if (aspeed > 1.0)
            aspeed = 1.0;
        setDriveMotorPower(aspeed,aspeed,aspeed,aspeed);

        // Set isRunning to true
        isRunning = true;
         while(isRunning &&(timeoutTimer.seconds() < timeout) &&
                (lfMotor.isBusy() || rfMotor.isBusy() || lrMotor.isBusy() || rrMotor.isBusy())){

             opmode.telemetry.addData("TargetPositions","lf:%7d rf:%7d lr:%7d rr:%7d",
                     lfMotor.getTargetPosition(),
                     rfMotor.getTargetPosition(),
                     lrMotor.getTargetPosition(),
                     rrMotor.getTargetPosition());
             opmode.telemetry.addData("Positions:",  "lf:%7d rf:%7d lr:%7d rr:%7d",
                     lfMotor.getCurrentPosition(),
                     rfMotor.getCurrentPosition(),
                     lrMotor.getCurrentPosition(),
                     rrMotor.getCurrentPosition());
             opmode.telemetry.update();
        }
        // Stop all
        stopAll();
    }

    /**
     * Helper function sets all motor modes to the same mode
     * @param mode
     */
    public void setMotorModes(DcMotor.RunMode mode){
        lfMotor.setMode(mode);
        rfMotor.setMode(mode);
        lrMotor.setMode(mode);
        rrMotor.setMode(mode);
    }

    /**
     * TODO Utility function to handle motor initialization.
     */
}