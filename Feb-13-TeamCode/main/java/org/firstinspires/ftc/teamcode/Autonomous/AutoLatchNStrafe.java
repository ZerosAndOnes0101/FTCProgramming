package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.DbgLog;

@Autonomous(name = "AutoLatchNStrafe", group = "Autonomous")
public class AutoLatchNStrafe extends Nav_Routines {

    boolean goldfound = false;
    // int leftrightcenter = 1; // 1 = left,  2 = right, 3= center
    double distancetraveledtodepot = 0;
    int leftcenterright = 1;
    double current_heading;

    double new_heading;

    @Override
    public void runOpMode() throws InterruptedException {
        Nav_Init();
        winchhookdown();
        current_heading = currentheadingreading();
        goRight();

        new_heading = currentheadingreading();
        telemetry.addData("Before Forward", "Running at %2f", new_heading);
        telemetry.update();


        new_heading = currentheadingreading();
        telemetry.addData("Call Forward", "Running at %2f", new_heading);
        telemetry.update();

        leftFront.setPower(-.3);
        rightFront.setPower(-.3);
        leftRear.setPower(-.3);
        rightRear.setPower(-.3);
        sleep(900);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        leftFront.setPower(.3);
        rightFront.setPower(-.3);
        leftRear.setPower(.3);
        rightRear.setPower(-.3);
        sleep(350);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);




        go_sideways(270, 0, 1, 7);

        goldfound = checktfod();
        if (!goldfound) {
            go_sideways(90, 0, 1, 23);

            goldfound = checktfod();
            if (!goldfound) {
               // go_sideways(90, 0, 1 , 17);
                leftFront.setPower(.3);
                rightFront.setPower(-.3);
                leftRear.setPower(.3);
                rightRear.setPower(-.3);
                sleep(800);
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftRear.setPower(0);
                rightRear.setPower(0);
            }
        }


        deactivateTfod();
    }
}