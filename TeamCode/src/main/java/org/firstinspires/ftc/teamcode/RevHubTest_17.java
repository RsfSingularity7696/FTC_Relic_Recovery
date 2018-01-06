/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

//In this test I programmed everything according to the chart on my phone. I added the new lift motor and Servos for the claws and implemented
//them on the gamepads. However, I did not program anything for the relic - do that in a new version.
@TeleOp(name="RevHubTest 17", group="Pushbot")
public class RevHubTest_17 extends OpMode {
    private SampleOp_EngineModule engine = new SampleOp_EngineModule();
    private ColorSensor color_sensor;
    private DistanceSensor distance_sensor = null;
 //   private DcMotor encoder_motor = null;
    private DcMotor lift_motor_Left = null;
    private DcMotor lift_motor_Right = null;

    private Servo jewelArm = null;
   // private Servo accuator = null;
    private Servo topClaw = null;
    private Servo botClaw = null;

    private double maxSpeed = 1.0d;
    private double moveSpeed = 1.0d;

    private int clawStateBot = 0;
    private int clawStateTop = 0;

    private int liftState = 0;
    private int toggleState = 0;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        engine.Initialize(hardwareMap);

        jewelArm = hardwareMap.servo.get("jewelArm");
      //  accuator = hardwareMap.servo.get("accuator");
        topClaw = hardwareMap.servo.get("topClaw");
        botClaw = hardwareMap.servo.get("botClaw");


        color_sensor = hardwareMap.colorSensor.get("color");
        distance_sensor = hardwareMap.get(DistanceSensor.class, "color");
      //  encoder_motor = hardwareMap.get(DcMotor.class, "encoder");
        //encoder_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift_motor_Left = hardwareMap.get(DcMotor.class, "liftMotor");
        lift_motor_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift_motor_Right = hardwareMap.get(DcMotor.class, "liftMotor2");
        lift_motor_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
     //   jewelArm.setPosition(0.0d);
      //  accuator.setPosition(0.0d);
        topClaw.setPosition(0.0d);
        botClaw.setPosition(0.0d);
        lift_motor_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift_motor_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        /*if (jewelArm.getPosition() <= 0.10d) {
            if (time < 30.0d)
            {
                if (color_sensor.red() > color_sensor.blue()) {
                    engine.SetSpeed(-0.25d);
                }
                else {
                    engine.SetSpeed(0.25d);
                }
            }
        }*/


        double left = clamp(gamepad1.right_stick_y);
        double right = clamp(gamepad1.left_stick_y);

    //Gamepad 1 section
        //For half speed and full speed
        if (gamepad1.right_bumper) {
            engine.SetMaxMotorPower(0.5d);

        } else if (gamepad1.left_bumper) {

            engine.SetMaxMotorPower(1.0d);


        }
        //This controls the mechanum movement staments as well as the joysticks. Please refer to sampleOp_Engine Module
        //For more information and how the joysticks and robot moves.
        if (!gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.dpad_up) {
            engine.SetSpeed(left, right);
        }
        else {
            engine.Move(GetInputs(gamepad1), 1.0d);
        }
    //Gamepad  2 section
//This moves the lift with the 2 lift motors
      /*  if(toggleState == 0){
            if (gamepad2.right_trigger > 0.10d) {
                if(liftState == 0){
                    lift_motor_Left.setPower(1.0d);
                }
                else if(liftState == 1){
                    lift_motor_Right.setPower(1.0d);
                }
            }
            else if (gamepad2.left_trigger > 0.10d) {
                if(liftState == 0){
                    lift_motor_Left.setPower(-1.0d);
                }
                else if(liftState == 1){
                    lift_motor_Right.setPower(-1.0d);
                }

            }
            else {
                lift_motor_Left.setPower(0.1d);
                lift_motor_Right.setPower(0.1d);
            }
            if(gamepad2.a){
                if(liftState == 0){
                    liftState = 1;
                }
                else if(liftState == 1){
                    liftState = 0;
                }

            }

        }
        if (toggleState == 1){
            if (gamepad2.a) {
                lift(0);
            }
            else if (gamepad2.x) {
                lift(700, 0);
            }
            else if (gamepad2.y) {
                lift(700, 1364);
            }

        }
        if(gamepad2.b){
           if (toggleState == 0){
               toggleState = 1;
           }
            if ((toggleState == 1)) {

                toggleState = 0;
            }
            }
*/

           /* if (gamepad2.a) {
                lift(0);
            }
            else if (gamepad2.x) {
                lift(770, 0);
            }
            else if (gamepad2.y) {
                lift(770, 1450);
            }
            */

        if (gamepad2.right_trigger > 0.10d) {
            if(liftState == 0){
                lift_motor_Left.setPower(1.0d);
            }
            else if(liftState == 1){
                lift_motor_Right.setPower(1.0d);
            }
        }
        else if (gamepad2.left_trigger > 0.10d) {
            if(liftState == 0){
                lift_motor_Left.setPower(-1.0d);
            }
            else if(liftState == 1){
                lift_motor_Right.setPower(-1.0d);
            }

        }
        else {
            lift_motor_Left.setPower(0.1d);
            lift_motor_Right.setPower(0.1d);
        }
        if(gamepad2.b){
            if(liftState == 0){
                liftState = 1;
            }
            else if(liftState == 1){
                liftState = 0;
            }

        }


        //ta da!


//Moves the claws(Change to the bumpers)use integer clawStateTop and clawStateBot to check the states between open and close.
        /*if (gamepad2.x) {
            topClaw.setPosition(1.0d);
        } else if (gamepad2.b) {
            topClaw.setPosition(-1.0d);
        }
        */
        //Moves the bottom claw with the right bumper(Below right trigger).
        if(gamepad1.b && botClaw.getPosition() <= 0.10d){
            botClaw.setPosition(1.0d);
        }
        else if(gamepad1.b && botClaw.getPosition() >= 0.90d){
            botClaw.setPosition(-0.3d);
        }

        if(gamepad1.x && topClaw.getPosition() <= 0.10d){
            topClaw.setPosition(1.0d);
        }
        else if(gamepad1.x && topClaw.getPosition() >= 0.90d){
            topClaw.setPosition(-1.0d);
        }




        //if(gamepad2.right_bumper){
        //    lift_motor.setPower(-1.0d);
        //}
        //if(gamepad2.left_bumper){
        //     lift_motor.setPower(1.0d);
    //}

        //Telemetry(The Values you can see on your driver Phone(For testing purposes))
      //  telemetry.addData("Encoders: ", encoder_motor.getCurrentPosition());

        telemetry.addData("Left Lift: ", lift_motor_Left.getCurrentPosition());
        telemetry.addData("Right Lift: ", lift_motor_Right.getCurrentPosition());
        telemetry.addData("Left Stick: ", gamepad1.left_stick_y);
        telemetry.addData("Right Stick: ", gamepad1.right_stick_y);
        telemetry.addData("Left Power: ", left);
        telemetry.addData("Right Power: ", right);
        telemetry.addData("Max Speed: ", maxSpeed);
        telemetry.addData("ClawState Bot: ", clawStateBot);
        telemetry.addData("ClawState Top: ", clawStateTop);


        telemetry.addData("Red: ", color_sensor.red());
        telemetry.addData("Green: ", color_sensor.green());
        telemetry.addData("Blue: ", color_sensor.blue());
        telemetry.addData("Alpha: ", color_sensor.alpha());
        telemetry.addData("Argb: ", color_sensor.argb());

        telemetry.addData("Distance (cm): ", String.format(Locale.US, "%.02f", distance_sensor.getDistance(DistanceUnit.CM)));

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        engine.Stop();
        lift_motor_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_motor_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public SampleOp_States.Dpad GetInputs(Gamepad gamepad) {
        if (gamepad.dpad_down && gamepad.dpad_left) {
            return SampleOp_States.Dpad.DownLeft;
        }
        else if (gamepad.dpad_down && gamepad.dpad_right) {
            return SampleOp_States.Dpad.DownRight;
        }
        else if (gamepad.dpad_down && gamepad.dpad_right) {
            return SampleOp_States.Dpad.DownRight;
        }
        else if (gamepad.dpad_up && gamepad.dpad_left) {
            return SampleOp_States.Dpad.UpLeft;
        }
        else if (gamepad.dpad_up && gamepad.dpad_right) {
            return SampleOp_States.Dpad.UpRight;
        }
        else if (gamepad.dpad_down) {
            return SampleOp_States.Dpad.Down;
        }
        else if (gamepad.dpad_left) {
            return SampleOp_States.Dpad.Left;
        }
        else if (gamepad.dpad_right) {
            return SampleOp_States.Dpad.Right;
        }
        else if (gamepad.dpad_up) {
            return SampleOp_States.Dpad.Up;
        }
        else {
            return SampleOp_States.Dpad.None;
        }
    }

    private double clamp(double value) {
        if (value > maxSpeed) {
            return maxSpeed;
        }
        else if (value < -maxSpeed) {
            return -maxSpeed;
        }
        else {
            return value;
        }
    }

    private void lift(int position) {
        lift(position, position);
    }

    private void lift(int left, int right) {
        lift_motor_Left.setPower(1.0d);
        lift_motor_Left.setTargetPosition(left);

        lift_motor_Right.setPower(1.0d);
        lift_motor_Right.setTargetPosition(right);
    }
}
