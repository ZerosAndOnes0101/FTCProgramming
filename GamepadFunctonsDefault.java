package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;




@TeleOp(name="GamepadFunctionsDefault", group="Linear Opmode")
//@Disabled
public class GamepadFunctonsDefault extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotor leftDrive = null;
    //private DcMotor rightDrive = null;

    DcMotor leftBackDrive;
    DcMotor rightBackDrive;
    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor latch;
    Servo sweeper;
    DcMotor armJoint;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // "left_drive = leftMotor
        // "right_drive = rightMotor
        // "armJoint" = tower
        // "left_hand" = box
        // "sweeper" = sweeper

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armJoint = hardwareMap.get(DcMotor.class, "armJoint");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        sweeper = hardwareMap.get(Servo.class, "sweeper");
        latch = hardwareMap.get(DcMotor.class, "latch");


        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            leftBackDrive.setPower(gamepad1.left_stick_y);
            rightBackDrive.setPower(gamepad1.left_stick_y);
            leftDrive.setPower(gamepad1.left_stick_y);
            rightDrive.setPower(gamepad1.left_stick_y);

            if (gamepad2.a) {
                latch.setPower(1.0);
            } else {
                latch.setPower(0.0);
            }

            if (gamepad2.b) {
                latch.setPower(-1.0);
            } else {
                latch.setPower(0.0);
            }

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

            if (gamepad2.right_bumper) {
                sweeper.setPosition(180);
            }

            if (gamepad2.left_bumper) {
                sweeper.setPosition(-180);
            }


        }
    }
}




