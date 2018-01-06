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

//Changed servo position and change half and full speed values
@TeleOp(name="RevHubTest 25", group="Pushbot")
public class RevHubTest_25 extends OpMode {
    private enum LiftType  {
        Manual,
        Relic,
    }

    private LiftType liftMode = LiftType.Manual;
    private int liftPhase = 0;

    private boolean _leftTriggerDown = false;
    private boolean _rightTriggerDown = false;

    private int leftPosition = 0;
    private int rightPosition = 0;

    private int increment = 50;
    private int liftMax = 1500;

    private SampleOp_EngineModule engine = new SampleOp_EngineModule();
    private ColorSensor color_sensor;

    private ColorSensor colorRight = null;
    private ColorSensor colorLeft = null;

    private DistanceSensor distance_sensor = null;
 //   private DcMotor encoder_motor = null;
    private DcMotor lift_motor_Left = null;
    private DcMotor lift_motor_Right = null;
 //   private DcMotor relicArm = null;

    private Servo jewelArm = null;
   // private Servo accuator = null;
    private Servo topClaw = null;
    private Servo botClaw = null;
    private Servo gripLeft = null;
    private Servo gripRight = null;

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

      //  relicArm = hardwareMap.dcMotor.get("relicArm");
      //  relicArm.setDirection(DcMotorSimple.Direction.FORWARD);

        jewelArm = hardwareMap.servo.get("jewelArm");
      //  accuator = hardwareMap.servo.get("accuator");
        topClaw = hardwareMap.servo.get("topClaw");
        topClaw.setDirection(Servo.Direction.FORWARD);

        botClaw = hardwareMap.servo.get("botClaw");
        topClaw.setDirection(Servo.Direction.FORWARD);

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
        jewelArm.setPosition(0.0d);
        topClaw.setPosition(0.0d);
        botClaw.setPosition(0.0d);

        lift_motor_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift_motor_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left = clamp(gamepad1.right_stick_y);
        double right = clamp(gamepad1.left_stick_y);

        //Gamepad 1 section
        //For half speed and full speed
        if (gamepad1.right_bumper) {
            engine.SetMaxMotorPower(0.70d);

        }
        else if (gamepad1.left_bumper) {
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

        if (gamepad2.right_trigger >= 0.10d) {
            lift_motor_Right.setPower(1.0d);
        }
       // else if (gamepad2.left_trigger >= 0.10d) {
         //   lift_motor_Right.setPower(-1.0d);
       // }
        else {
            lift_motor_Right.setPower(0.0d);
        }

        if (gamepad2.right_bumper) {
            lift_motor_Left.setPower(1.0d);
        }
        else if (gamepad2.left_bumper) {
            lift_motor_Left.setPower(-1.0d);
        }
        else {
            lift_motor_Left.setPower(0.0d);
        }


        //Set a lock and Unlock TOGGLE On The Robot

        //if(gamepad2.left_trigger >= 0.10d){
            //lift_motor_Right
        //}

        /*String _color = checkColor();

        if (_color == "LEFT") {

        }
        else if (_color == "RIGHT") {

        }
        else {

        }*/

        if (gamepad2.left_stick_y >= 0.1d) {
            topClaw.setPosition(1.0d);
        }
        else if (gamepad2.left_stick_y <= -0.1d) {
            topClaw.setPosition(0.30d);
        }

        if (gamepad2.right_stick_y >= 0.1d) {
            botClaw.setPosition(1.0d);
        }
        else if (gamepad2.right_stick_y <= -0.1d) {
            botClaw.setPosition(0.4d);
        }

        telemetry.addData("Left: ", lift_motor_Left.getPower());
        telemetry.addData("Right: ", lift_motor_Right.getPower());

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

    private int clamp(int min, int max, int value) {
        return Math.max(min, Math.min(max, value));
    }

    private void lift(int position) {
        lift(position, position);
    }

    private void lift(int left, int right) {
        lift_motor_Left.setPower(1.0d);
        lift_motor_Left.setTargetPosition(clamp(0, liftMax, left));

        lift_motor_Right.setPower(1.0d);
        lift_motor_Right.setTargetPosition(clamp(0, liftMax, right));
    }

    private String checkColor() {
        int left = colorLeft.red() - colorLeft.blue();
        int right = colorRight.red() - colorRight.blue();

        if (left > right) {
            return "LEFT";
        }
        else if (left < right) {
            return "RIGHT";
        }
        else {
            return "NONE";
        }
    }
}
