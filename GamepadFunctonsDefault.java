package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "GamepadFunctionsDefault", group = "LinearOpmode")

public class GamepadFunctonsDefault extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor latch;
    private DcMotor armJoint;
    private CRServo boxJoint;
    private CRServo sweeper;
    private CRServo armExtend;
    private boolean isAOn = false;
    private double slowModeValue = 1.0;
    private double leftPower;
    private double rightPower;
    private double rightBackPower;
    private double leftBackPower;






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
        boolean changed = false;
        boolean on = false;
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();


        if (gamepad1.a && !isAOn) {

            if(slowModeValue == 1.0)
                slowModeValue = .5;
            else
                slowModeValue = 1.0;
                        isAOn = true;

        }
        else if (!gamepad1.a) {
            isAOn = false;
        }




        double drive = gamepad1.left_stick_y * slowModeValue;
        double turn = -gamepad1.right_stick_x;
        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);
        leftBackPower = Range.clip(drive + turn, -1.0, 1.0);
        rightBackPower = Range.clip(drive - turn, -1.0, 1.0);







            latch.setPower(gamepad2.left_stick_y);


            if (gamepad1.right_bumper) {
                leftDrive.setPower(-1);
                leftBackDrive.setPower(-1);
                rightDrive.setPower(-1);
                rightBackDrive.setPower(-1);
            }
            if (gamepad1.left_bumper) {
                leftDrive.setPower(1);
                leftBackDrive.setPower(1);
                rightDrive.setPower(1);
                rightBackDrive.setPower(1);
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


            if (gamepad2.b) {

                armExtend.setPower(1);
            }
            else {
                armExtend.setPower(0);
            }


        }
        if (gamepad2.y) {
            sweeper.setPower(1);
        }
        else {
            sweeper.setPower(0);
        }



        if (gamepad2.a) {
            boxJoint.setPower(1);
        }

        else {
            boxJoint.setPower(0);
        }




        if (gamepad1.dpad_up) {

            leftBackDrive.setPower(1);
            rightDrive.setPower(-1);

        } else if (gamepad1.dpad_right) {

            rightBackDrive.setPower(-1);
            leftDrive.setPower(1);

        } else if (gamepad1.dpad_down) {
            leftBackDrive.setPower(-1);
            rightDrive.setPower(1);

        } else if (gamepad1.dpad_left) {

            rightBackDrive.setPower(1);
            leftDrive.setPower(-1);

        } else {
            leftBackDrive.setPower(-leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            leftDrive.setPower(-leftPower);
            rightDrive.setPower(rightPower);

        }
    }
}