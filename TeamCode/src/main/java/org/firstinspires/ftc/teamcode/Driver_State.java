/*
Copyright 2016 FIRST Tech Challenge Team 11497

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

//import com.qualcomm.robotcore.hardware.CRServo;

/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp
public class Driver_State extends LinearOpMode {
    private DcMotor bottomRight; //Right Rear Actual , port0
    private DcMotor topRight; //Right Front Actual, port1
    private DcMotor topLeft; //Left Front Actual, port2
    private DcMotor bottomLeft; //Left Rear Actual, port3
    private DcMotorEx motor5; //for shooter
    private DcMotorEx motor6; //for shooter
    private DcMotor motor7; //angler for shooter
    private DcMotor motor8; //test for intake
    private Servo servo1; //Artifact flicker - name by micah
    private Servo servo2; //ball juggler

    // private DcMotor motor_5; //horizontal arm
    // private DcMotor motor_6; //vertical arm
    /*
    private DcMotor motor_7; //robot lift
    private DcMotor motor_8; //robot lift
    */
    // private CRServo servo_1; //Intake Lift 1=open and .3=closed
    // private CRServo servo_2; //tray .7=flat and .8=down
    /*
    private Servo servo_3; //left speciman finger .9=closed and .7=open
    private Servo servo_4; //right speciman finger .05=closed and .7=open
    private Servo servo_5; //intake wheel 1=stop, .3=in, 0=out
    private Servo servo_6; //specimen rotator
    private Servo servo_7; //servo7, specimen tilter 0 is down and .5 is midway
    */
    @Override
    public void runOpMode() {

        bottomRight = hardwareMap.get(DcMotor.class, "bottomRight");
        topRight = hardwareMap.get(DcMotor.class, "topRight");
        topLeft = hardwareMap.get(DcMotor.class, "topLeft");
        bottomLeft = hardwareMap.get(DcMotor.class, "bottomLeft");
        //ballSorter = hardwareMap.get(DcMotor.class, "ballSorter");
        motor5 = hardwareMap.get(DcMotorEx.class, "motor5");
        motor6 = hardwareMap.get(DcMotorEx.class, "motor6");
        motor7 = hardwareMap.get(DcMotor.class, "motor7");
        motor8 = hardwareMap.get(DcMotor.class, "motor8");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        //motor_5 = hardwareMap.get(DcMotor.class, "motor_5");)

        motor7.setTargetPosition(0);
        motor7.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // motor7.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor7.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor7.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int encoderPosition1 = motor7.getCurrentPosition();

        int shootAngle = 0;
        int DriverSwap = 1;

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //boolean is_hanging = false;
        //boolean jonas = true; //motor_5 encoder function
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        //double tgtPower = 0;

        while (opModeIsActive()) {
            //set wheel directions
            bottomRight.setDirection(DcMotor.Direction.REVERSE); //Right Rear Actual
            topRight.setDirection(DcMotor.Direction.REVERSE); //Right Front Actual //reverse to forward
            topLeft.setDirection(DcMotor.Direction.FORWARD); //Left Front Actual
            bottomLeft.setDirection(DcMotor.Direction.FORWARD); //Left Rear Actual //was forward before motor change
            motor5.setDirection(DcMotor.Direction.REVERSE); //shooting motor
            motor6.setDirection(DcMotor.Direction.FORWARD); //shooting motor
            motor7.setDirection(DcMotor.Direction.FORWARD);
            servo2.setDirection(Servo.Direction.REVERSE);

            waitForStart();
            //sample code from https://gm0.org/en/stable/docs/software/mecanum-drive.html

            //driver 1 the Driver
            double y = gamepad1.left_stick_y;// * DriverSwap; // Remember, this is reversed! //added*-1 in both y n x
            double x = gamepad1.left_stick_x * 1.5; // Counteract imperfect strafing, readd driverswap here if use
            double rx = gamepad1.right_stick_x * -1.5;
            //boolean  = gamepad1.right_bumper;
            boolean rb = gamepad1.left_bumper;

            double speed; //variable to slow robot driving
            if (rb) {
                speed = 0.6;
            } else {
                speed = 1;
            }

            //ball sorter spin!
           /* if (gamepad1.left_trigger > .51) { //if the trigger on the gamepad is pressed, then:
                ballSorter.setPower(-0.3);
            } else {
                ballSorter.setPower(0);
            } */

            //shooting code
                if (gamepad1.right_trigger > .51) {
                    motor5.setPower(0.8);
                    motor6.setPower(0.8);
                } else {
                    motor5.setPower(0);
                    motor6.setPower(0);
                }

                if (gamepad1.left_trigger > .51) {
                    motor5.setPower(.5);
                    motor6.setPower(.5);
                } else {
                    motor5.setPower(0);
                    motor6.setPower(0);
                }

                if (gamepad1.right_bumper) {
                    motor5.setPower(0.65);
                    motor6.setPower(0.65);
                } else {
                    motor5.setPower(0);
                }

                //intake test
                if (gamepad1.right_bumper) {
                motor8.setPower(1);
                } else {
                motor8.setPower(0);
                }

                //Shooter angle code
                //sets shoot angle to 50 when left dpad is pressed
            /*
                if (gamepad1.dpad_left) {
                    motor7.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    shootAngle = 30;
                }
                else if (gamepad1.dpad_right) {
                    motor7.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    shootAngle = 0;
                } */
                if (gamepad1.dpad_up) {
                    motor7.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    shootAngle = 60;
                }
                motor7.setTargetPosition(shootAngle);
                motor7.setPower(1);
                if (gamepad1.dpad_down) {
                    motor7.setPower(0);
                    motor7.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }

                //start of code for the ball whacker
                if (gamepad1.dpad_left) {
                    servo1.setPosition(.83);
                }
                /*
                if (gamepad2.b) {
                    servo1.setPosition(.6);
                }
                if (gamepad2.x) {
                    servo1.setPosition(0.4);
                }
                */
                if (gamepad1.dpad_right) {
                    servo1.setPosition(0.2);
                } //end of code for the ball whacker

                //start of test code for the ball juggler
                if (gamepad1.a) { //start position
                servo2.setPosition(1);
                }
                if (gamepad1.b) {
                servo2.setPosition(0.55); //2nd position
                }
                if (gamepad1.y) {
                servo2.setPosition(0.3);
                }
                if (gamepad1.x) { //last position
                servo2.setPosition(0);
                } //end of the ball juggler test code

//if (gamepad1.left_stick_y > .1 || gamepad1.left_stick_x > .1) {
                topLeft.setPower((y - x + rx) * speed); //front left says the sample...this one is mapped good
                bottomLeft.setPower((y + x + rx) * speed); //back left says the sample
                topRight.setPower((y + x - rx) * speed); //front right says the sample
                bottomRight.setPower((y - x - rx) * speed); //back right says the sample.
//}

//#### Drive 2 - attachment person ###

                /*
                if (this.gamepad2.left_bumper) { //deliver to tray
                    servo_2.setPosition(.68);
                    servo_1.setPosition(0.2);
                } else if (this.gamepad2.right_bumper) { //intake on ground
                    servo_1.setPosition(.8);

                } else if (this.gamepad2.b) { //intake not on ground
                    servo_1.setPosition(.63);

                }
                if (this.gamepad2.right_trigger > .5) {
                    servo_1.setPosition(1);
                }
                */
                telemetry.addData("Motor 1 Power", bottomRight.getPower());
                telemetry.addData("Motor 2 Power", topRight.getPower());
                telemetry.addData("Motor 3 Power", topLeft.getPower());
                telemetry.addData("Motor 4 Power", bottomLeft.getPower());
                telemetry.addData("Motor 5 Power", motor5.getPower());
                telemetry.addData("Motor 6 Power", motor6.getPower());
                telemetry.addData("Status", "Running");
                telemetry.update();
            }
        }
    }
