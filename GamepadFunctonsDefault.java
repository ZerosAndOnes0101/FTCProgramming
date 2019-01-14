package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "GamepadFunctionsDefault", group = "LinearOpmode")

public class GamepadFunctonsDefault extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor latch;
    private DcMotor armJoint;
    private Servo boxJoint;
    private Servo sweeper;
    boolean toggleready = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        //leftDrive = front left motor
        //rightDrive = right front motor
        //leftBackDrive = back left motor
        //rightBackDrive = back right motor
        //armJoint = arm motor
        //sweeper = sweeper servo
        //latch = rack and pinion motor
        //boxJoint = box moving servo

        leftDrive = hardwareMap.get(DcMotor.class, "FL");
        rightDrive = hardwareMap.get(DcMotor.class, "FR");
        armJoint = hardwareMap.get(DcMotor.class, "lift");
        leftBackDrive = hardwareMap.get(DcMotor.class, "BL");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BR");
        latch = hardwareMap.get(DcMotor.class, "hook");
        boxJoint = hardwareMap.get(Servo.class, "boxJoint");
        sweeper = hardwareMap.get(Servo.class, "intake");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            double motorPower = gamepad1.left_stick_y;


            latch.setPower(gamepad2.left_stick_y);

            if (gamepad1.a == false) {

                toggleready = true;
            }

            if (gamepad1.a && toggleready) {

                toggleready = false;
                if(motorPower == 1.0){
                    motorPower = .5;
                }

            }
            else if (motorPower == .5) {
                motorPower = 1.0;
            }
            if (gamepad1.left_bumper) {
                leftDrive.setPower(-motorPower);
                leftBackDrive.setPower(-motorPower);
                rightDrive.setPower(-motorPower);
                rightBackDrive.setPower(-motorPower);
            }
            if (gamepad1.right_bumper) {
                leftDrive.setPower(motorPower);
                leftBackDrive.setPower(motorPower);
                rightDrive.setPower(motorPower);
                rightBackDrive.setPower(motorPower);
            }

            if (gamepad2.left_bumper) {
                armJoint.setPower(1);
            } else {
                armJoint.setPower(0);
            }


            if (gamepad2.right_bumper) {
               armJoint.setPower(0);
            } else {
                armJoint.setPower(1);
            }

            if (gamepad1.x) {
                sweeper.setPosition(1);

            } else {
                sweeper.setPosition(0);

            }
            if (gamepad1.y) {
                sweeper.setPosition(0);

            } else {
                sweeper.setPosition(1);

                if (gamepad1.a) {
                    boxJoint.setPosition(1);
                } else {
                    boxJoint.setPosition(0);
                }

                if (gamepad1.b) {
                    armJoint.setPower(0);
                } else {
                    armJoint.setPower(1);
                }

                if (gamepad1.dpad_up) {
                    leftBackDrive.setPower(motorPower);
                    rightDrive.setPower(-motorPower);

                } else if (gamepad1.dpad_right) {

                    rightBackDrive.setPower(-motorPower);
                    leftDrive.setPower(motorPower);

                } else if (gamepad1.dpad_down) {
                    leftBackDrive.setPower(-motorPower);
                    rightDrive.setPower(motorPower);

                } else if (gamepad1.dpad_left) {

                    rightBackDrive.setPower(motorPower);
                    leftDrive.setPower(-motorPower);

                } else {
                    leftBackDrive.setPower(-motorPower);
                    rightBackDrive.setPower(motorPower);
                    leftDrive.setPower(-motorPower);
                    rightDrive.setPower(motorPower);

                }
            }
        }
    }
}
