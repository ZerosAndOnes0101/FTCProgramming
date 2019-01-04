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

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armJoint = hardwareMap.get(DcMotor.class, "armJoint");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        latch = hardwareMap.get(DcMotor.class, "latch");
        boxJoint = hardwareMap.get(Servo.class, "boxJoint");
        sweeper = hardwareMap.get(Servo.class, "sweeper");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            latch.setPower(gamepad2.left_stick_y);

            if (gamepad2.x) {
                sweeper.setPosition(180);

            } else {
                sweeper.setPosition(0);

            }
            if (gamepad2.y) {
                sweeper.setPosition(0);

            } else {
                sweeper.setPosition(180);

                if (gamepad2.a) {
                    armJoint.setPower(.5);
                } else {
                    armJoint.setPower(0.0);
                }

                if (gamepad2.b) {
                    armJoint.setPower(-.5);
                } else {
                    armJoint.setPower(0.0);
                }


                if (gamepad2.left_bumper) {
                    boxJoint.setPosition(180);
                } else {
                    boxJoint.setPosition(0);
                }


                if (gamepad2.right_bumper) {
                    boxJoint.setPosition(0);
                } else {
                    boxJoint.setPosition(180);
                }


                if (gamepad1.right_bumper) {
                    leftBackDrive.setPower(-1);
                    rightBackDrive.setPower(-1);
                    leftDrive.setPower(1);
                    rightDrive.setPower(1);

                } else if (gamepad1.left_bumper) {
                    leftBackDrive.setPower(1);
                    rightBackDrive.setPower(1);
                    leftDrive.setPower(-1);
                    rightDrive.setPower(-1);


                } else {
                    leftBackDrive.setPower(-gamepad1.left_stick_y);
                    rightBackDrive.setPower(-gamepad1.left_stick_y);
                    leftDrive.setPower(-gamepad1.left_stick_y);
                    rightDrive.setPower(-gamepad1.left_stick_y);

                }
            }
        }
    }
}
