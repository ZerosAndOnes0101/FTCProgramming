package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Autonomous.Nav_Routines;


@Autonomous(name="0&1_Autonomous_Period", group="Autonomous")
//@Disabled
public class Autonomous_Period extends Nav_Routines {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        Nav_Init();


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            //Move to Scanning Posit

            go_sideways(270, -90, 1, 20); // back ward

            go_forward(330, 0, 1, false);

            go_sideways(0, 25, .5, 9); // This is working left

            // Scan the image


            //If scan the rover/footprint
            go_sideways(270, -90, 1, 20); // back ward

            go_forward(500, 0, -1, true);

            sleep(20000);

            //If scan the mars/space
            go_sideways(0, 90, 1, 20); // 90

            go_forward(500, 0, -1, true);


            if (1==1) {
                sleep(10000);
            }
        }
    }
}




