package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="Gamepad", group="Linear Opmode")
//@Disabled
public class PranavGamepad extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotor leftDrive = null;
    //private DcMotor rightDrive = null;

    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor armJoint;
    Servo servo;
    DcMotor sweeper;


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
        armJoint=hardwareMap.get(DcMotor.class, "armJoint");
        servo=hardwareMap.get(Servo.class, "left_hand");
        sweeper=hardwareMap.get(DcMotor.class, "sweeper");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        armJoint.setDirection(DcMotor.Direction.FORWARD);
        sweeper.setDirection(DcMotor.Direction.FORWARD);


        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Servo Position",servo.getPosition() );
            telemetry.update();

            leftDrive.setPower(gamepad1.left_stick_y);
            rightDrive.setPower(gamepad1.left_stick_y);
            armJoint.setPower(gamepad2.left_stick_y);

            if (gamepad1.x) {
                leftDrive.setPower(1.5);
                rightDrive.setPower(0);
            }
            else {
                leftDrive.setPower(0.0);
                rightDrive.setPower(0.0);
            }

            if (gamepad1.b) {
                leftDrive.setPower(0);
                rightDrive.setPower(1.5);
            }
            else {
                leftDrive.setPower(0.0);
                rightDrive.setPower(0.0);
            }
            if (gamepad2.y) {
                sweeper.setPower(1.5);
            }else {
                sweeper.setPower(0);
            }
            if (gamepad2.x){
                servo.setPosition(0);
            } else if ( gamepad2.b) {
                servo.setPosition(180);
            }
        }
    }
}



