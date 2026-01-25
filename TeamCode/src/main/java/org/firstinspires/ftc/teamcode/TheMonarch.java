package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.hardware.Sensor;

import androidx.lifecycle.GenericLifecycleObserver;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.LED;
import java.util.List;
@Configurable
public class TheMonarch {
    boolean slow_mode = false;
    DistanceSensor distance_sensor;
    DcMotor front_left_drive = null;
    DcMotor back_left_drive = null;
    DcMotor front_right_drive = null;
    DcMotor back_right_drive = null;

    DcMotorEx launch_motor = null;
    DcMotor intake_motor = null;
    DcMotor injector_motor = null;

    Servo right_door = null;
    Servo left_door = null;
    Servo chamber = null;

    VisionPortal front_camera;

    public static double speedMultiplier = 1.0;

    public static double launch_motor_target = 6000;
    public static double launch_motor_base = 0;

    public static double left_gate_open_angle = 0.65;
    public static double left_gate_closed_angle = 0.3;
    public static double right_gate_open_angle = 0.68;
    public static double right_gate_closed_angle = 1;

    ColorSensor chamber_color;

    AprilTagProcessor april_tag;

    void Init(HardwareMap hardwareMap){


        // -- MOTORS --
        // hw map
        front_left_drive = hardwareMap.get(DcMotor.class, "front_left");
        back_left_drive = hardwareMap.get(DcMotor.class, "back_left");
        front_right_drive = hardwareMap.get(DcMotor.class, "front_right");
        back_right_drive = hardwareMap.get(DcMotor.class, "back_right");
        injector_motor = hardwareMap.get(DcMotor.class, "i_2");

        intake_motor = hardwareMap.get(DcMotor.class, "intake");
        launch_motor = hardwareMap.get(DcMotorEx.class, "outake");

        chamber = hardwareMap.get(Servo.class, "chamber");

        // directions
        front_left_drive.setDirection(DcMotor.Direction.REVERSE);
        back_left_drive.setDirection(DcMotor.Direction.REVERSE);
        front_right_drive.setDirection(DcMotor.Direction.FORWARD);
        back_right_drive.setDirection(DcMotor.Direction.FORWARD);

        launch_motor.setDirection(DcMotor.Direction.FORWARD);
        intake_motor.setDirection(DcMotor.Direction.REVERSE);

        injector_motor.setDirection(DcMotor.Direction.REVERSE);

        // -- SERVOS --
        right_door = hardwareMap.get(Servo.class, "r_gate"); // should probably rename this
        left_door = hardwareMap.get(Servo.class, "l_gate");

        // -- SENSORS --

        april_tag = AprilTagProcessor.easyCreateWithDefaults();
        front_camera.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), april_tag
        );

        chamber_color = hardwareMap.get(ColorSensor.class, "c_col");
        chamber_color.enableLed(true);


    }

    void Reset(){
        ToggleRightDoor(false);
        ToggleLeftDoor(false);
        ToggleChamber(false);
    }

    void ToggleRightDoor(boolean active){
        if (!active){
            right_door.setPosition(right_gate_closed_angle);
        }else{
            right_door.setPosition(right_gate_open_angle);
        }

    }

    void ToggleChamber(boolean active){
        if (active){
            chamber.setPosition(0);
        }else{
            chamber.setPosition(1);
        }
    }


    void ToggleLeftDoor(boolean active){
        if (!active){
            left_door.setPosition(left_gate_closed_angle);
        }else{
            left_door.setPosition(left_gate_open_angle);
        }

    }

    void Chamber(){

        if (launch_motor.getVelocity(AngleUnit.DEGREES)/6>4000){
            // TODO
        }
        // otherwise we shouldn't chamber because we'll miss
    }

    void TeleOpDrive(double axial, double lateral, double yaw){
        double max;
        double frontLeftPower  = (axial + lateral + yaw) * speedMultiplier;
        double frontRightPower = (axial - lateral - yaw) * speedMultiplier;
        double backLeftPower   = (axial - lateral + yaw) * speedMultiplier;
        double backRightPower  = (axial + lateral - yaw) * speedMultiplier;

        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));


        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        front_left_drive.setPower(frontLeftPower);
        front_right_drive.setPower(frontRightPower);
        back_left_drive.setPower(backLeftPower);
        back_right_drive.setPower(backRightPower);
    }

    void ToggleLaunchMotor(boolean active){
        // x*6 to convert RPM to Deg/Sec
        if (active){
            launch_motor.setVelocity(launch_motor_target*6, AngleUnit.DEGREES);
        }else{
            launch_motor.setVelocity(launch_motor_base*6, AngleUnit.DEGREES);
        }
    }

    void ToggleIntake(boolean active){
        if (active){
            intake_motor.setPower(1);
        }else{
            intake_motor.setPower(0);
        }
    }

    void ToggleInjectorBar(boolean active){
        if (active){
            injector_motor.setPower(0.4 );
        }else{
            injector_motor.setPower(0);
        }
    }

    List<AprilTagDetection> FetchAprilTagDetections(){
        return april_tag.getDetections();
    }

    void CameraDebugData(Telemetry telemetry){
        List<AprilTagDetection> currentDetections = april_tag.getDetections();
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


}
