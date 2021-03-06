package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOp_14652", group = "TeleOp")
public class TeleOp_14652 extends OpMode {

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;
    DcMotor winchmotor;
    CRServo mil1;
    CRServo mil2;
    DcMotor liftmotor;
    Servo mineralknockservo;
    CRServo mineralintakeservo;
    CRServo mineralboxservo;
    CRServo mineralslidesblockservo;
    DigitalChannel magneticlimitswitch;
    int mil1startticks;
    int mil2startticks;
    int liftmotorstartticks;
    int prevmil1ticks;
    double mil1tickspersec;
    double mineralboxpos = 1;
    boolean autoliftmode = false;
    boolean dropliftmmode = false;
    boolean xispressed = false;
    boolean yispressed = false;
    boolean intakeon = false;
    boolean mineralarmendgame = false;

    ElapsedTime mineralliftmodetimer = new ElapsedTime();
    ElapsedTime mineraldropmodetimer = new ElapsedTime();
    ElapsedTime mineralarmendgametimer = new ElapsedTime();
    ElapsedTime ytimer = new ElapsedTime();
    ElapsedTime xtimer = new ElapsedTime();
    ElapsedTime mil1tickpersectimer = new ElapsedTime();
    ElapsedTime silvermineraldroptimer = new ElapsedTime();

    @Override
    public void init() {
        leftFront = hardwareMap.dcMotor.get("FL");
        rightFront = hardwareMap.dcMotor.get("FR");
        leftRear = hardwareMap.dcMotor.get("BL");
        rightRear = hardwareMap.dcMotor.get("BR");

        winchmotor = hardwareMap.dcMotor.get("hook");


        liftmotor = hardwareMap.dcMotor.get("lift");

        mineralknockservo = hardwareMap.servo.get("mks");
        mineralintakeservo = hardwareMap.crservo.get("intake");

        mineralboxservo = hardwareMap.crservo.get("boxJoint");
        mineralslidesblockservo = hardwareMap.crservo.get("boxJoint");

        mil1 = hardwareMap.crservo.get("aExt");
        mil2 = hardwareMap.crservo.get("aExt");

        //zno magneticlimitswitch = hardwareMap.digitalChannel.get("mls");

        mineralknockservo.setPosition(1);
        // zno mineralslidesblockservo.setPosition(.5);

        mil1startticks = (int) mil1.getController().getServoPosition(1);
        mil2startticks = (int) mil2.getController().getServoPosition(1);
        liftmotorstartticks = liftmotor.getCurrentPosition();

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        winchmotor.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winchmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // zno mil1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // zno mil2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        //Driving
        double leftstickx = 0;
        double leftsticky = 0;
        double rightstickx = 0;
        double wheelpower;
        double stickangleradians;
        double rightX;
        double leftfrontpower;
        double rightfrontpower;
        double leftrearpower;
        double rightrearpower;
        double dpadpower = .2;
        double dpadturningpower = .4;
        double speedmodifier = 1;

        if (gamepad1.right_bumper) {
            speedmodifier = .5;
        }
        if (gamepad1.left_bumper) {
            speedmodifier = 1;
        }

        if (gamepad1.dpad_up) {
            leftsticky = dpadpower;
        } else if (gamepad1.dpad_right) {
            leftstickx = dpadturningpower;
        } else if (gamepad1.dpad_down) {
            leftsticky = -dpadpower;
        } else if (gamepad1.dpad_left) {
            leftstickx = -dpadturningpower;
        } else {
            leftstickx = gamepad1.left_stick_x * speedmodifier;
            leftsticky = -gamepad1.left_stick_y * speedmodifier;
            rightstickx = gamepad1.right_stick_x * speedmodifier;
        }
        if (Math.abs(leftsticky) <= .15) {
            leftsticky = 0;
        }
        wheelpower = Math.hypot(leftstickx, leftsticky);
        stickangleradians = Math.atan2(leftsticky, leftstickx);

        stickangleradians = stickangleradians - Math.PI / 4; //adjust by 45 degrees

        rightX = rightstickx * .5;
        leftfrontpower = wheelpower * Math.cos(stickangleradians) + rightX;
        rightfrontpower = wheelpower * Math.sin(stickangleradians) - rightX;
        leftrearpower = wheelpower * Math.sin(stickangleradians) + rightX;
        rightrearpower = wheelpower * Math.cos(stickangleradians) - rightX;

        leftFront.setPower(leftfrontpower);
        rightFront.setPower(rightfrontpower);
        leftRear.setPower(leftrearpower);
        rightRear.setPower(rightrearpower);

        //Mineral Intake
        if (gamepad2.right_bumper) {
            intakeon = true;
        }
        if (gamepad2.left_bumper) {
            intakeon = false;
        }

        if (intakeon) {
            if (gamepad1.right_trigger == 1) {
                mineralintakeservo.getController().setServoPosition(1,1);
            } else {
                // zno mineralintakeservo.setPower(0);
                mineralintakeservo.getController().setServoPosition(1,0);
            }
        } else {
            if (gamepad1.right_trigger == 1) {
                // zno mineralintakeservo.setPower(1);
                mineralintakeservo.getController().setServoPosition(1,1);
            } else {
                // zno mineralintakeservo.setPower(.5);
                mineralintakeservo.getController().setServoPosition(1,0.5);
            }
        }


        //Mineral Lift System
        int mil1ticks;
        int mil2ticks;
        int liftticks;
        int phase = 0;
        final double servoturnpos = .20; // the pos where the servo starts turning
        final double maxmil1ticks = 501;
        final double droplevelticks = 95;
        final double liftdropticks = 3500;
        final double tickstoturnbox = 405; // box must be fully turned between droplevelticks and droplevelticks + tickstoturnbox
        double boxmil1ticks;
        double minrangedifference;

        mil1ticks = mil1startticks - (int) mil1.getController().getServoPosition(1);
        mil2ticks = mil2startticks - (int) mil2.getController().getServoPosition(1);
        liftticks = liftmotorstartticks - liftmotor.getCurrentPosition();

        if (mil1tickpersectimer.seconds() > .05) {
            mil1tickspersec = (mil1ticks - prevmil1ticks) / mil1tickpersectimer.seconds();
            mil1tickpersectimer.reset();
        } else {
            prevmil1ticks = mil1ticks;
        }


        if (gamepad2.y && ytimer.seconds() > .35) {
            yispressed = !yispressed;
            silvermineraldroptimer.reset();
            mineralboxpos = .4;
            ytimer.reset();
        }

        if (gamepad2.x && xtimer.seconds() > .35) {
            xispressed = !xispressed;
            yispressed = false;
            xtimer.reset();
        }

        if (gamepad1.x) {
            mineralknockservo.setPosition(.8);
        }

        if (gamepad1.b) {
            mineralknockservo.setPosition(1);
        }

        if (yispressed) {
           // mineralboxpos = .43;
            if (silvermineraldroptimer.seconds() > .75) {
                mineralboxpos = .45;
            }
            xispressed = false;
        }else if (xispressed){
            mineralboxpos = .45;
            yispressed = false;
        } else{     // move the mineral box servo to angle that is based on the mineral arm angle (mil1ticks)
            /*
            if(autoliftmode){
                minrangedifference = .55;  // tip the box up a bit more if we're trying to lift
            } else {
                minrangedifference = .36;
            }
            */
            boxmil1ticks = mil1ticks;
            if (boxmil1ticks > droplevelticks + tickstoturnbox) {   // set the max for boxmilticks
                boxmil1ticks = droplevelticks + tickstoturnbox;
            }
            if (boxmil1ticks < droplevelticks) {                     // set the min for boxmilticks
                boxmil1ticks = droplevelticks;
            }
           mineralboxpos = ((boxmil1ticks - droplevelticks) / tickstoturnbox) * (.5 - servoturnpos) + servoturnpos; // makes a range from .2 to .5
            //mineralboxpos = 1 - (((boxmil1ticks - droplevelticks) / tickstoturnbox) * minrangedifference); // makes a range from 1 to .66
        }

        if (mil1ticks > 200) {
            yispressed = false;
            xispressed = false;
        }

        //zno mineralboxservo.setPosition(mineralboxpos);
        mineralboxservo.getController().setServoPosition(1,mineralboxpos);

        if (gamepad2.b && mineralliftmodetimer.seconds() >= .35) {
            autoliftmode = !autoliftmode;
            dropliftmmode = false;
            mineralarmendgame = false;
            mineralliftmodetimer.reset();
        }

        if (gamepad2.a && mineraldropmodetimer.seconds() >= .35) {
            dropliftmmode = !dropliftmmode;
            autoliftmode = false;
            mineralarmendgame = false;
            mineraldropmodetimer.reset();
        }

        if (gamepad2.dpad_up && mineralarmendgametimer.seconds() >= .35) {
            mineralarmendgame = !mineralarmendgame;
            dropliftmmode = false;
            autoliftmode = false;
            mineralarmendgametimer.reset();
        }

        if (autoliftmode || mineralarmendgame) {
            if (mil1ticks > 200) {
                phase = 1;
            } else if (mil1ticks > droplevelticks) {
                phase = 2;
            } else {
                mineralarmendgame = false;
                phase = 3;
            }

            if (phase == 1) {
                if (liftticks > 1000) {
                    liftmotor.setPower(1);
                } else {
                    liftmotor.setPower(0);
                }
                if (mil1ticks > 300) {
                    mil1.setPower(1);
                    mil2.setPower(1);
                } else {
                    mil1.setPower(.65);
                    mil2.setPower(.65);
                }
            }

            if (phase == 2) {
                /*
                if (liftticks > 1000) {
                    liftmotor.setPower(1);
                } else {
                    liftmotor.setPower(0);
                }
                */
                if (!mineralarmendgame) {
                    if (liftticks < liftdropticks) {
                        liftmotor.setPower(-1);
                    } else {
                        liftmotor.setPower(0);
                    }
                }
                if (mil1ticks > 120) {
                    mil1.setPower(.3);
                    mil2.setPower(.3);
                } else{
                    mil1.setPower(.1);
                    mil2.setPower(.1);
                }
            }

            if (phase == 3) {
                if (!mineralarmendgame) {
                    mil1.setPower(0);
                    mil2.setPower(0);
                    if (liftticks < liftdropticks) {
                        liftmotor.setPower(-1);
                    } else {
                        liftmotor.setPower(0);
                        //mil1startticks = (int)(mil1startticks + droplevelticks - mil1ticks);
                        autoliftmode = false;
                    }
                }
            }
        } else if (dropliftmmode) {
            if (mil1ticks < 120) {
                mil1.setPower(-.9);
                mil2.setPower(-.9);
            } else if (mil1ticks < 400) {
                mil1.setPower(-.4);
                mil2.setPower(-.4);
                if (liftticks > 1000) {
                    liftmotor.setPower(1);
                } else {
                    liftmotor.setPower(0);
                }
            } else if (mil1ticks < 440) {
                if (liftticks > 1000) {
                    liftmotor.setPower(1);
                } else {
                    liftmotor.setPower(0);
                }
                if (mil1tickspersec > 1100 && liftticks > 500) {
                    mil1.setPower(0);
                    mil2.setPower(0);
                }else {
                    mil1.setPower(-.2);
                    mil2.setPower(-.2);
                }
            } else {
                mil1.setPower(0);
                mil2.setPower(0);
                dropliftmmode = false;
            }

            yispressed = false;
            xispressed = false;

        } else {
            if (gamepad2.right_stick_y <= 0 && mil1ticks > maxmil1ticks) {
                mil1.setPower(.3);
                mil2.setPower(.3);
            } else {
                mil1.setPower(gamepad2.right_stick_y);
                mil2.setPower(gamepad2.right_stick_y);
            }
            liftmotor.setPower(gamepad2.left_stick_y);
        }

        if (mil1ticks >= 300) {
            // zno mineralslidesblockservo.setPosition(1);
            mineralslidesblockservo.getController().setServoPosition(1,1);
        }


        //Winch Lift
        boolean magnetistouching;

        // zno magnetistouching = !magneticlimitswitch.getState();
        magnetistouching = false;

        if (gamepad2.right_trigger >= .75 && !magnetistouching) {
            winchmotor.setPower(1);
        } else if (gamepad2.left_trigger >= .75) {
            winchmotor.setPower(-1);
        } else {
            winchmotor.setPower(0);
        }

        telemetry.addData("Mil 1 Ticks", mil1ticks);
        telemetry.addData("Mil 2 Ticks", mil2ticks);
        telemetry.addData("Lift Motor Ticks", liftticks);
        telemetry.addData("Box Pos", mineralboxpos);
        telemetry.addData("Ticks per Second", mil1tickspersec);
        telemetry.addData("Prev mil1 Ticks", prevmil1ticks);
        telemetry.addData("Phase", phase);
        telemetry.addData("mil1 Start Ticks", mil1startticks);
        telemetry.update();

    }
}
