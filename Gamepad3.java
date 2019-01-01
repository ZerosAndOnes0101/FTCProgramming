package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name="Gamepad3", group="Linear Opmode")
//@Disabled
public class Gamepad3 extends LinearOpMode {

   private ElapsedTime runtime = new ElapsedTime();


   public DcMotor leftFrontDrive ;
   public DcMotor rightFrontDrive ;
   public DcMotor leftBackDrive;
   public DcMotor rightBackDrive;
   public DcMotor armJoint;
   public DcMotor Latch;
   public DcMotor sweeper;

   @Override
   public void runOpMode() {
       telemetry.addData("Status", "Initialized");
       telemetry.update();

       Latch = hardwareMap.get(DcMotor.class, "latch");
       leftFrontDrive = hardwareMap.get(DcMotor.class, "left_drive");
       rightFrontDrive = hardwareMap.get(DcMotor.class, "right_drive");
       armJoint = hardwareMap.get(DcMotor.class, "armJoint");
       rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
       leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
       sweeper = hardwareMap.get(DcMotor.class, "sweeper");







       rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
       rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

       waitForStart();
       runtime.reset();

       while (opModeIsActive()) {
           double leftPower;
           double rightPower;


           leftBackDrive.setPower(gamepad1.left_stick_y);
           rightBackDrive.setPower(gamepad1.left_stick_y);
           leftFrontDrive.setPower(gamepad1.left_stick_y);
           rightFrontDrive.setPower(gamepad1.left_stick_y);




           if (gamepad1.left_bumper) {
               Latch.setPower(1);
           }

           if (gamepad1.right_bumper) {
               Latch.setPower(-1);
           }

           if (gamepad1.x) {
               sweeper.setPower(1);
           } else {
               sweeper.setPower(0);
           }

           if (gamepad1.y) {
               armJoint.setPower(0.5);
           }

           if (gamepad1.b) {
               sweeper.setPower(-1);
           } else {
               sweeper.setPower(0);
           }

           if (gamepad1.a) {
               armJoint.setPower(-0.5);
           }


       }


   }
}



