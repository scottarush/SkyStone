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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

/**
 * This class contains common methods usable by multiple opmodes.
 *
 * Opmodes use this class by creating an instance and passing a reference to the
 * robot in the constructor
 *
 */
public class UtilityMethods {

    private OpenGLMatrix lastLocation = null;


    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // Distance of camera in front of robot-center
    final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // Height of camera above ground
    final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line


    private MecanumDrive robot;

    /** reference to vuforia base.  **/
    private VuforiaCommon vuforiaCommon;

    /* Constructor */
    public UtilityMethods(MecanumDrive theRobot) {
        robot = theRobot;
        vuforiaCommon = new VuforiaCommon(robot.hwMap);
     }

     /**
     * This function moves the robot a delta x and y distance.
     *
     * @param opmode  OpMode of the caller.  Needed to be able to send Telemetry
     * @param speed   speed to move
     * @param xdist   distance in x inches to move
     * @param ydist   distance in y inches to move
     * @param timeout timeout in seconds to abort if move not completed.
     */
    public void driveByEncoder(LinearOpMode opmode, double speed, double xdist, double ydist, double timeout) {

        // Compute the number of encoder counts for each wheel to move the requested distanc
        int lfDeltaCounts = (int) Math.round((xdist + ydist) * MecanumDrive.COUNTS_PER_INCH);
        int rfDeltaCounts = (int) Math.round((ydist - xdist) * MecanumDrive.COUNTS_PER_INCH);
        int lrDeltaCounts = (int) Math.round((ydist - xdist) * MecanumDrive.COUNTS_PER_INCH);
        int rrDeltaCounts = (int) Math.round((xdist + ydist) * MecanumDrive.COUNTS_PER_INCH);

        // Set target counts for each motor to the above
        robot.lfMotor.setTargetPosition(lfDeltaCounts + robot.lfMotor.getCurrentPosition());
        robot.rfMotor.setTargetPosition(rfDeltaCounts + robot.rfMotor.getCurrentPosition());
        robot.lrMotor.setTargetPosition(lrDeltaCounts + robot.lrMotor.getCurrentPosition());
        robot.rrMotor.setTargetPosition(rrDeltaCounts + robot.rrMotor.getCurrentPosition());

        // Set mode to run to position
        robot.setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        // Reset timer and set motor power
        ElapsedTime drivetimeout = new ElapsedTime();
        drivetimeout.reset();
        double aspeed = Math.abs(speed);
        if (aspeed > 1.0)
            aspeed = 1.0;
        robot.setPower(aspeed, aspeed, aspeed, aspeed);

        while (opmode.opModeIsActive() && (drivetimeout.seconds() < timeout) &&
                (robot.lfMotor.isBusy() || robot.rfMotor.isBusy() || robot.lrMotor.isBusy() || robot.rrMotor.isBusy())) {

            opmode.telemetry.addData("TargetPositions", "lf:%7d rf:%7d lr:%7d rr:%7d",
                    robot.lfMotor.getTargetPosition(),
                    robot.rfMotor.getTargetPosition(),
                    robot.lrMotor.getTargetPosition(),
                    robot.rrMotor.getTargetPosition());
            opmode.telemetry.addData("Positions:", "lf:%7d rf:%7d lr:%7d rr:%7d",
                    robot.lfMotor.getCurrentPosition(),
                    robot.rfMotor.getCurrentPosition(),
                    robot.lrMotor.getCurrentPosition(),
                    robot.rrMotor.getCurrentPosition());
            opmode.telemetry.update();
        }
        // Stop all
        robot.stop();
    }

    public static final int NO_STONES_FOUND = 0;
    public static final int FOUND_SKYSTONE = 1;
    public static final int FOUND_STONE = 2;
    /**
     * helper function to find a Stone using the Vurforia TensorFlow by strafing in the x direction
     * @param opmode the calling opmode needed for Telemetry
     * @param searchRight set true to search right, false to search left
     * @param onlySkystone keep searching until a SkyStone is found.
     * @param timeout timeout to give up if no stone found
     * @return NO_STONES_FOUND, FOUND_SKYSTONE, or FOUND_STONE
     */
    public int findStone(LinearOpMode opmode, boolean searchRight, boolean onlySkystone,double timeout){
         // Initialize the tensor flow engine - if not already init'ed
        vuforiaCommon.initTensorFlowObjectDetection();
        /**
         * Activate TensorFlow Object Detection
         **/
        if (vuforiaCommon.tensorFlowObjDetector != null) {
            vuforiaCommon.tensorFlowObjDetector.activate();
        }

        ElapsedTime findTimer = new ElapsedTime();
         // Set return code assuming we don't find a stone
        int retcode = NO_STONES_FOUND;
         // Set 1/4 power
        double power = 0.25;
        if (searchRight) {
            // Strafe to right
            robot.setPower(power, -power, -power, power);
        }
        else{
            // Strafe to left
            robot.setPower(-power, power, power, -power);
        }

        boolean keepSearching = true;
       while (opmode.opModeIsActive() && (findTimer.seconds() < timeout) &&
               (keepSearching == true)) {
           // Now check for recognitions
           List<Recognition> updatedRecognitions = vuforiaCommon.tensorFlowObjDetector.getUpdatedRecognitions();
           if (updatedRecognitions != null) {
               opmode.telemetry.addData("# Object Detected", updatedRecognitions.size());
               opmode.telemetry.update();
               // step through the list of recognitions and display boundary info.
               int i = 0;
               for (Recognition recognition : updatedRecognitions) {
                   opmode.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                   opmode.telemetry.update();
                   if (recognition.getLabel().compareToIgnoreCase("Stone") == 0) {
                       retcode = FOUND_STONE;
                       // Check if we have to keep searching for a Skystone.
                       if (onlySkystone == false){
                           keepSearching = false;
                       }
                       break;  // Break out to return
                   } else if (recognition.getLabel().compareToIgnoreCase("Skystone") == 0) {
                       retcode = FOUND_SKYSTONE;
                       keepSearching = false;
                       break; // Break out to return
                   }
               }
           }
       }
        // Stop the motors
        robot.stop();
        // Shut down the engine before returning
        if (vuforiaCommon.tensorFlowObjDetector != null) {
            vuforiaCommon.tensorFlowObjDetector.shutdown();
        }

        return retcode;
    }
    /**
     * helper function to move robot in front of a stone using Vuforia Navigation.
     * The robot must be sitting in front of a stone assumed to be found using
     * findStone method.
     * @param opmode the calling opmode needed for Telemetry
     * @param timeout timeout to give up if no stone found
     * @return true if move was successfully completed. false if Skystone went out of view
     */
    public boolean moveRobotToStone(LinearOpMode opmode, double timeout){
        // Initialize VuforiaNavigation
         vuforiaCommon.initVuforiaNavigation();

        ElapsedTime findTimer = new ElapsedTime();
        // Set return code assuming we have an error locating to the skysteon
        boolean retcode = false;
        // Set 1/4 power
        double power = 0.25;


        /** Assume stone was visible on entry. **/
        boolean stoneVisible = true;

        OpenGLMatrix lastLocation = null;

        // Assume that the skyStone is somewhere in front of the robot so reset the stoneTarget
        vuforiaCommon.stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, 0, 0)));

        // Activate the targets
        vuforiaCommon.targetsSkyStone.activate();

        while (opmode.opModeIsActive() && (findTimer.seconds() < timeout) &&
                (stoneVisible == true)) {
            stoneVisible = false;
            for (VuforiaTrackable trackable : vuforiaCommon.allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    opmode.telemetry.addData("Visible Target", trackable.getName());
                    opmode.telemetry.update();
                    if (trackable.getName().compareToIgnoreCase(vuforiaCommon.stoneTarget.getName())==0){
                        stoneVisible = true;
                    }

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (stoneVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                opmode.telemetry.addData("Stone Pos:", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                opmode.telemetry.addData("Stone Rot", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else {
                opmode.telemetry.addData("Lost Stone", "No stone found.");
            }
            opmode.telemetry.update();
        }

        // Disable Tracking when we are done;
        vuforiaCommon.targetsSkyStone.deactivate();
        // Stop the motors
        robot.stop();

        return retcode;
    }

 }