package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "GamepadFunctionsDefault", group = "LinearOpmode")

public class GamepadFunctonsDefault extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor latch = null;
    private DcMotor armJoint = null;
    private CRServo boxJoint;
    private CRServo sweeper;
    private CRServo armExtend;
    private boolean isAOn = false;
    private double slowModeValue = 1.0;
    private double leftPower;
    private double rightPower;
    private double rightBackPower;
    private double leftBackPower;
    private double armJointPower;
    private double latchPower;

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
        boxJoint = hardwareMap.get(CRServo.class, "boxJoint");
        sweeper = hardwareMap.get(CRServo.class, "intake");
        armExtend = hardwareMap.get(CRServo.class, "aExt");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();


            if (gamepad1.a && !isAOn) {

                if (slowModeValue == 1.0) slowModeValue = 2;
                else slowModeValue = 1.0;
                isAOn = true;

            } else if (!gamepad1.a) {
                isAOn = false;
            }


            //Calculations for motor power
            double r = Math.hypot(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rotate = -gamepad1.right_stick_x;
            final double frontLeft = -r * Math.cos(robotAngle) + rotate;
            final double frontRight = r * Math.sin(robotAngle) + rotate;
            final double rearLeft = -r * Math.sin(robotAngle) + rotate;
            final double rearRight = r * Math.cos(robotAngle) + rotate;

            if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0) {
                if (gamepad1.left_stick_x > 0.1 || gamepad1.left_stick_x < 0.1) {
                    //Motor Power Sets
                    leftDrive.setPower(frontLeft);
                    rightDrive.setPower(frontRight);
                    leftBackDrive.setPower(rearLeft);
                    rightBackDrive.setPower(rearRight);
                } else {
                    //Motor Power Sets
                    leftDrive.setPower(frontLeft / slowModeValue);
                    rightDrive.setPower(frontRight / slowModeValue);
                    leftBackDrive.setPower(rearLeft / slowModeValue);
                    rightBackDrive.setPower(rearRight / slowModeValue);
                }
            } else {
                leftDrive.setPower(0.005);
                rightDrive.setPower(0.005);
                leftBackDrive.setPower(0.005);
                rightBackDrive.setPower(0.005);

            }

            armJoint.setPower(gamepad2.right_stick_y * .5);
            boxJoint.setPower(gamepad2.left_stick_y);
            sweeper.setPower(gamepad2.left_stick_y);

            if (gamepad2.left_bumper) {
                latch.setPower(1);
            } else {

                sweeper.setPower(0);

            }

            if (gamepad2.right_bumper) {

                latch.setPower(-1);

            } else {

                sweeper.setPower(0);

            }





        }

    }

}

