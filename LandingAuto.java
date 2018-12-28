package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.regex.Matcher;


@Autonomous(name="LandinAuto", group="LinearOpmode")

public class LandingAuto extends LinearOpMode {

    Servo servo;

    DcMotor leftDrive = null;
    DcMotor rightDrive = null;

    double ServoPosition = 0.0;
    DcMotor armJoint = null;

    double servoPosition = 0.0;
    double power = -0.5;
    double armjpower = 0.5;

    private Matcher                                                                                                                                 runtime;

    @Override
    public void runOpMode() {


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        servo = (Servo) hardwareMap.get("servo");
        servo.setPosition(servoPosition);

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        servo.setPosition(0.0);


        armJoint = (DcMotor) hardwareMap.get("armJoint");


        waitForStart();
        runtime.reset();

        armJoint.setPower(armjpower);

        sleep(1000);

        servoPosition = 1.0;
        servo.setPosition(servoPosition);

        sleep(1000);

        armJoint.setPower(-armjpower);

        sleep(1000);

        servoPosition = 0.0;
        servo.setPosition(servoPosition);

        //Backwards
        leftDrive.setPower(power);
        rightDrive.setPower(power);

        sleep(1500);
        //Right turn
        leftDrive.setPower(power);
        rightDrive.setPower(-power);

        sleep(1000);
        //Forward
        leftDrive.setPower(power);
        rightDrive.setPower(power);

        sleep(2500);
    }
}