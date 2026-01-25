package org.firstinspires.ftc.teamcode;/* Copyright (c) 2021 FIRST. All rights reserved.
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



import android.hardware.Sensor;

import androidx.lifecycle.GenericLifecycleObserver;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.LED;
import java.util.List;
/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Declan Is An: TeleOp Mode", group="Linear OpMode")
public class DeclanIsAnOpMode extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private static final boolean USE_WEBCAM = true;
    LED status_led;
    boolean slowMode = false;
    boolean lastBumperState = false;

    boolean intakeActive = false;
    boolean lastIntakeState = false;

    private DistanceSensor frontDistanceSensor;

    private boolean hasFoundMotifTag = false;
    
    private String motifPattern = "UNKNOWN";
    private AprilTagProcessor aprilTag; // april tag processer deal thing, thanks ftc
    private VisionPortal visionPortal; // literally just a camera streamer

    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor launch_motor = null;

    private DcMotor intake_a = null;
    private DcMotor intake_b = null;
    private DcMotor intake_motor = null;
    private boolean lastOutakeState = false;
    private boolean outakeActive = false;
    private boolean lastRunningState = false;
    private boolean runningForceActive = true;
    private boolean injectorActive = false;

    private double targetLaunchSpeed = 0.7;
    private boolean isRightGateOpen = false;
    private Servo rightGateServo = null;

    private double intakeSpeed = 1;

    private boolean engineerSpeedTakeover = false;

    @Override
    public void runOpMode() {


        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right");

        intake_a = hardwareMap.get(DcMotor.class, "i_1");
        intake_b = hardwareMap.get(DcMotor.class, "i_2");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        intake_motor = hardwareMap.get(DcMotor.class, "intake");
        launch_motor = hardwareMap.get(DcMotor.class, "outake");

        launch_motor.setDirection(DcMotor.Direction.FORWARD);
        intake_motor.setDirection(DcMotor.Direction.REVERSE);

        intake_a.setDirection(DcMotor.Direction.REVERSE);
        intake_b.setDirection(DcMotor.Direction.REVERSE);

        rightGateServo = hardwareMap.get(Servo.class, "stage2");


        //initAprilTag();

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        rightGateServo.setPosition(1);
        // 0.3

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            intake_a.setPower(1);
            intake_b.setPower(1);
            //telemetry.addData("Motif", motifPattern);

            TeleOp();


            /*if (hasFoundMotifTag == false){
                telemetry.addData("AI Status", "Attempting to find motif... Elapsed: " + runtime.toString());
 n                 attemptFetchMotif();

            }else{
                visionPortal.stopStreaming();
            }*/
            telemetry.addData("Outake Active: ", outakeActive ? "TRUE" : "FALSE");
            telemetry.addData("Outake Running Force: ", runningForceActive ? "TRUE" : "FALSE");
            telemetry.addData("Intake Active: ", intakeActive ? "TRUE" : "FALSE");
            telemetry.addData("Engineer Speed Control: ", engineerSpeedTakeover ? "TRUE" : "FALSE");
            telemetry.addData("Current Launch Speed Target: ", targetLaunchSpeed);
            telemetry.update();

        }


    }

    static double q_v1 = 0.000000242231;

    private double GetLauncherSpeedQuadratic(double x){
        return q_v1*(x*x)-0.00034377*x+0.719796;
    }

    private void TeleOp() { // Remove if using for auto
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;


        boolean bumperPressed = gamepad1.left_stick_button;
        boolean intakePressed = gamepad1.a;
        boolean outakePressed = gamepad1.x;
        boolean runningPressed = gamepad1.dpad_down;
        if (bumperPressed && !lastBumperState) {
            slowMode = !slowMode;
        }

        if (runningPressed && !lastRunningState){
            runningForceActive=!runningForceActive;
        }

        if (intakePressed && !lastIntakeState){
            intakeActive = !intakeActive;


        }


        if (outakePressed && !lastOutakeState){
            outakeActive = !outakeActive;


        }

        intake_motor.setPower(gamepad1.right_trigger);
        if (!outakeActive){
            launch_motor.setPower(gamepad1.left_trigger);
        }

        if (gamepad2.dpadLeftWasPressed()){
            targetLaunchSpeed-=0.025;
        }
        if (gamepad2.dpadRightWasPressed()){
            targetLaunchSpeed+=0.025;
        }

        if (gamepad2.aWasPressed()){
            engineerSpeedTakeover = !engineerSpeedTakeover;
        }

        if (gamepad1.yWasPressed()){
            if (isRightGateOpen){
                rightGateServo.setPosition(1);
                isRightGateOpen=false;
            }else{
                rightGateServo.setPosition(0.3);
                isRightGateOpen=true;
            }
        }

        if (gamepad2.triangleWasPressed()){

        }

        if (outakeActive){
            /*if (!engineerSpeedTakeover){
                double dist = frontDistanceSensor.getDistance(DistanceUnit.MM);
                if (dist>1500){
                    targetLaunchSpeed=0.7;
                }else{
                    targetLaunchSpeed=GetLauncherSpeedQuadratic(dist);
                }
            }*/

            targetLaunchSpeed=1;
        }


        if (targetLaunchSpeed>1) {
            targetLaunchSpeed = 1;
        } else if (targetLaunchSpeed < 0.6) {
            targetLaunchSpeed=0.6;
        }

        lastBumperState = bumperPressed;
        lastIntakeState = intakePressed;
        lastOutakeState = outakePressed;

        // Apply speed multiplier
        double speedMultiplier = slowMode ? 0.25 : 1.0;

        intake_motor.setPower(intakeActive ? 1.0 : 0.0);
        launch_motor.setPower(outakeActive ? targetLaunchSpeed : (runningForceActive ? 0.5 : 0.0));

        double frontLeftPower  = (axial + lateral + yaw) * speedMultiplier;
        double frontRightPower = (axial - lateral - yaw) * speedMultiplier;
        double backLeftPower   = (axial - lateral + yaw) * speedMultiplier;
        double backRightPower  = (axial + lateral - yaw) * speedMultiplier;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));


        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);

    }


    private void attemptFetchMotif(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.metadata.name.startsWith("Obelisk")){
                    telemetry.addLine("Obelisk tag detected!");
                    motifPattern = detection.metadata.name.split("_")[1];
                    hasFoundMotifTag = true;
                }
            }else{
                telemetry.addData("MOTIF ERROR", "Unable to read april tag! Clean lense!");
            }
        }
    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    } }


