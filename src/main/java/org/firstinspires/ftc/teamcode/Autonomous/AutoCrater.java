package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoCrater", group = "Autonomous")
public class AutoCrater extends Nav_Routines {

    boolean goldfound = false;
    // int leftrightcenter = 1; // 1 = left,  2 = right, 3= center
    double distancetraveledtodepot = 0;
    int leftcenterright = 1;
    double current_heading;

    double new_heading;
    double power=1.00;



    @Override
    public void runOpMode() throws InterruptedException {
        Nav_Init();
        winchhookdown();

        power=getBatteyVolatge();

        telemetry.addData("BatteryPowerVoltage", "Running at %2f", power);
        telemetry.update();

        strafeRight(power);

        new_heading = currentheadingreading();
        telemetry.addData("Before Forward", "Running at %2f", new_heading);
        telemetry.update();

        leftFront.setPower(-.3*power);
        rightFront.setPower(-.3*power);
        leftRear.setPower(-.3*power);
        rightRear.setPower(-.3*power);
        sleep(900);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        leftFront.setPower(.3*power);
        rightFront.setPower(-.3*power);
        leftRear.setPower(.3*power);
        rightRear.setPower(-.3*power);
        sleep(350);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);


        new_heading = currentheadingreading();
        telemetry.addData("Call Forward", "Running at %2f", new_heading);
        telemetry.update();
        go_sideways(270, 0, 1*power, 7);

        goldfound = checktfod();
        if (!goldfound) {
            go_sideways(90, 0, 1*power, 23);

            goldfound = checktfod();
            if (!goldfound) {
               // go_sideways(90, 0, 1 , 17);
                leftFront.setPower(.3*power);
                rightFront.setPower(-.3*power);
                leftRear.setPower(.3*power);
                rightRear.setPower(-.3*power);
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