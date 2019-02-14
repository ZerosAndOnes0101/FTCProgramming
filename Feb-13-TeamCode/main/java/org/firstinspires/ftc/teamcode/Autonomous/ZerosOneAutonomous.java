package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "ZerosOneAutonomous", group = "Autonomous")
public class ZerosOneAutonomous extends Nav_Routines {

    boolean goldfound = false;
    int leftrightcenter = 1; // 1 = left,  2 = right, 3= center
    double distancetraveledtodepot = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Nav_Init();
        //zno winchup();
        winchdown();
        goldfound = checktfod();
        if (!goldfound) {
            //go_forward(14, 0, -.4, false);
            telemetry.addLine("Go Sideway to right | ")
                    .addData("ang", "270")
                    .addData("power", ".3");

            //jv go_sideways(270, 0, .3, 5); // Confirmed for right side
            go_sideways(90, 0, 1, 40);


            // go_sideways(90, 0, .3, 5); // Confirmed for Center

            goldfound = checktfod();
            if (!goldfound) {
                telemetry.addLine("Assumed on left  | ")
                        .addData("ang", "270")
                        .addData("power", ".3");

                go_sideways(90, 0, 1, 40);

                leftFront.setPower(-.3);
                rightFront.setPower(-.3);
                leftRear.setPower(-.3);
                rightRear.setPower(-.3);
                sleep(100);
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftRear.setPower(0);
                rightRear.setPower(0);

                go_sideways(90, 0, 1, 40);

                telemetry.update();
            } else {
                telemetry.addData("Found", "right");
                leftrightcenter = 2;

            }
        } else {
            telemetry.addData("Found", "Center");
            leftrightcenter = 3;

            telemetry.update();

        }
        hitMineral();
        // zno go_forward(1, 0, .2, false);
        // zno sleep(500);
        //zno go_forward(1, 0, -.2, false);
       // zno  go_sideways(90, 0, .3, 5);  // Knock off the mineral
       // zno  mineralknockservo.setPosition(.5);
       // zno  sleep(500);
        // zno mineralknockservo.setPosition(1);

      // zno   go_forward(1, 0, -.2, false);
        //zno go_sideways(270, 0, .3, 2);  // back out

        deactivateTfod();

       //zno go_forward(14 + (12 * leftcenterright), 0, .35, false);

        /*turn_to_heading(135);
        go_sideways_to_wall(135, .5, 5, true);
        // go to depot
        wallfollow(28, 135, .4, 5, true, false );
        distancetraveledtodepot = wallfollow(14, 135, .4, 5, true, true);

        //drop the marker
        deploymarker2();

        // go to crater
        wallfollow(43 - distancetraveledtodepot, 135, -.4, 5, true, false);
        go_forward(14, 135, -.4, true);*/


    }
}
