/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * Copied from the VuforiaSkyStoneNavigationWebcam sample.
 **/
public class VuforiaTargetLocator {

    public static final String TARGET_STONE = "Stone Target";
    // Vuforia doesn't direclty track a skystone.  Have to use the tensor flow detector in com
    // combination to set this label.
    public static final String TARGET_SKYSTONE = "Skystone Target";
    public static final String TARGET_BLUE_REAR_BRIDGE = "Blue Rear Bridge";
    public static final String TARGET_RED_REAR_BRIDGE = "Red Rear Bridge";
    public static final String TARGET_BLUE_FRONT_BRIDGE = "Blue Front Bridge";
    public static final String TARGET_RED_FRONT_BRIDGE = "Red Front Bridge";
    public static final String TARGET_RED_PERIMETER_1 = "Red Perimeter 1";
    public static final String TARGET_RED_PERIMETER_2 = "Red Perimeter 2";
    public static final String TARGET_FRONT_PERIMETER_1 = "Front Perimeter 1";
    public static final String TARGET_FRONT_PERIMETER_2 = "Front Perimeter 2";
    public static final String TARGET_BLUE_PERIMETER_1 = "Blue Perimeter 1";
    public static final String TARGET_BLUE_PERIMETER_2 = "Blue Perimeter 2";
    public static final String TARGET_REAR_PERIMETER_1 = "Rear Perimeter 1";
    public static final String TARGET_REAR_PERIMETER_2 = "Rear Perimeter 2";

    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private static final String VUFORIA_KEY = "AcKS+1f/////AAABmSfE4zB+90Wek6WzDi8g9Kw5Y7UUvCIs/xewLbEyh1FnM9TKbT+OXm1jp/q0e0G+b3EfoikZfLj1W+tXrZ4vrSJKyIuX/dhgfNzqJiRjnhiM9EWGVpKKQRYaK5Vr6Tp/UUif1/0/g15dgu/Gy4CvEoTUG3BeGGyDZDy9DlyoJImjnf1C0IBTb1kRz5oTW+lyx4AEeuG2a6egQVGU61IbESGMTXKnQxfj9ccnbZdHLHV62WowIoMJJtXDO4jfLcnGmPEr3v60y9ZPzzYifER84G+ulCUxe0ssoxIzRLNyC9FcHuJ11qvk9yGj8rbKclJjhCE4zHjJO7/3wS0/EEWy+iLg32J0IVrPGipUX/Pxn2Z/";


    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

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
    private static final float quadField = 36 * mmPerInch;

    // Class Members
    
    private OpenGLMatrix targetLocation = null;
    private VuforiaLocalizer vuforia = null;

    private VuforiaTrackables targetsSkyStone;

    private List<VuforiaTrackable> allTrackables;
    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;
    private TFObjectDetector tfod = null;

    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;

    public static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    public static final String STONE_TFOD_LABEL = "Stone";
    public static final String SKYSTONE_TFOD_LABEL = "Skystone";

    private OpMode mOpMode = null;

    public boolean mInitialized = false;

    public VuforiaTargetLocator(){

    }
    public void init(OpMode opMode) {
        mOpMode = opMode;
        /*
         * Retrieve the camera we are to use.
         */
        webcamName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", mOpMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName(TARGET_STONE);
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName(TARGET_BLUE_REAR_BRIDGE);
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName(TARGET_RED_REAR_BRIDGE);
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName(TARGET_RED_FRONT_BRIDGE);
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName(TARGET_BLUE_FRONT_BRIDGE);
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName(TARGET_RED_PERIMETER_1);
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName(TARGET_RED_PERIMETER_2);
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName(TARGET_FRONT_PERIMETER_1);
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName(TARGET_FRONT_PERIMETER_2);
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName(TARGET_BLUE_PERIMETER_1);
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName(TARGET_BLUE_PERIMETER_2);
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName(TARGET_REAR_PERIMETER_1);
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName(TARGET_REAR_PERIMETER_2);

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }


        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // Now initialize the tensor flow object detector
        int tfodMonitorViewId = mOpMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", mOpMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, STONE_TFOD_LABEL, SKYSTONE_TFOD_LABEL);

        mInitialized = true;
    }


    /**
     * Called to check for a recognition.  Object labels are defined in RECOGNITION_OBJECT_XXX string constansts
     * @return list of Recognition objects detected or null if no objects detected or error in initialization
     */
    public List<Recognition> getRecognitions(){
        if (tfod == null){
            return new ArrayList<Recognition>();
        }
        // Return latest recognitions
        return tfod.getRecognitions();
    }


    public void activate(){
        if (!mInitialized)
            return;
        targetsSkyStone.activate();
        tfod.activate();
    }

    /**
     * Returns the position of all objects in view of the camera.  Coord system is:
     * +y from the camera starting at 0
     * +x to the right of the camera
     * +z = up
     * orientation = the face of the stone
     * The heading has to be manually added using
     * @return
     */
    public List<TargetPosition> getTargets() {

        ArrayList<TargetPosition>targets = new ArrayList<>();
        if (!mInitialized)
            return targets;

        // check all the trackable targets to see which one (if any) is visible.
        VuforiaTrackable targetTrackable = null;
        // Get any recognitions fronm the tfod and use it to set the heading as well as determine a Skystone
        // vs. a stone from VuforiaTrackable
        List<Recognition>recList = getRecognitions();

        for (VuforiaTrackable trackable : allTrackables) {
            VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener)trackable.getListener();
            // Check if the trackable is visible and if so, pull its data and add to the return list.
            if (listener.isVisible()) {
                //                  telemetry.addData("Visible Target", trackable.getName());
                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    targetLocation = robotLocationTransform;
                }
                // express position (translation) of robot in inches.
                VectorF translation = targetLocation.getTranslation();
                Orientation rotation = Orientation.getOrientation(targetLocation, EXTRINSIC, XYZ, DEGREES);

                // Create the position object but swap the coordinates to align with the TargetPosition object coordiante system
                double x = translation.get(0)/mmPerInch;
                double y = translation.get(1)/mmPerInch;
                double z = translation.get(2)/mmPerInch;
                double heading = rotation.thirdAngle;
                TargetPosition position = new TargetPosition(trackable.getName(),y,-x,z, rotation.thirdAngle,0);
                //                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                targets.add(position);

                if (recList.size() > 0){
                    for(Iterator<Recognition>iter=recList.iterator();iter.hasNext();){
                        Recognition recog = iter.next();
                        if (recog.getLabel().equalsIgnoreCase(VuforiaTargetLocator.SKYSTONE_TFOD_LABEL)) {
                            if (position.targetLabel.equalsIgnoreCase(TARGET_STONE)){
                                // Assume it is the same one and set it as a Skystone.
                                position.heading = recog.estimateAngleToObject(AngleUnit.DEGREES);
                                position.headingValid = true;
                                position.targetLabel = TARGET_SKYSTONE;
                            }
                        }
                        else if (recog.getLabel().equalsIgnoreCase(VuforiaTargetLocator.STONE_TFOD_LABEL)){
                            if (position.targetLabel.equalsIgnoreCase(TARGET_STONE)){
                                // Assume it is the same one and leave it as a stone
                                position.heading = recog.estimateAngleToObject(AngleUnit.DEGREES);
                                position.headingValid = true;
                                position.targetLabel = TARGET_STONE;
                            }

                        }
                    }
                }
            }
        }
        return targets;
     }
}

