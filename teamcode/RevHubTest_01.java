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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;


import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@TeleOp(name="RevHub 01", group="Pushbot")
@Disabled
public class RevHubTest_01 extends OpMode {
    private SampleOp_EngineModule engine = new SampleOp_EngineModule();
    private ColorSensor color_sensor;
    private DistanceSensor distance_sensor = null;
    private DcMotor encoder_motor = null;
    private DcMotor lift_motor = null;

    private Servo jewelArm = null;
   // private Servo accuator = null;
    private Servo topClaw = null;
    private Servo bottomClaw = null;

    private double maxSpeed = 1.0d;
    private double moveSpeed = 1.0d;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        engine.Initialize(hardwareMap);

        jewelArm = hardwareMap.servo.get("jewelArm");
      //  accuator = hardwareMap.servo.get("accuator");
        topClaw = hardwareMap.servo.get("topClaw");
        bottomClaw = hardwareMap.servo.get("botClaw");


        color_sensor = hardwareMap.colorSensor.get("color");
        distance_sensor = hardwareMap.get(DistanceSensor.class, "color");
        encoder_motor = hardwareMap.get(DcMotor.class, "encoder");
        encoder_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift_motor = hardwareMap.get(DcMotor.class, "liftMotor");
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
      //  accuator.setPosition(0.0d);
        topClaw.setPosition(0.0d);
        bottomClaw.setPosition(0.0d);
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
        double left = clamp(gamepad1.left_stick_y);
        double right = clamp(gamepad1.right_stick_y);

      //  if (gamepad2.right_trigger > 0.0d) {
        //    lift_motor.setPower(gamepad2.right_trigger);
        //}
          //  if(gamepad2.left_trigger > 0.0d){
            //    lift_motor.setPower(gamepad2.left_trigger);
           // }

        if (gamepad1.right_bumper) {
            topClaw.setPosition(0.0d);
        } else if (gamepad1.left_bumper) {
            {
                engine.SetMaxMotorPower(1.0d);
            }
            engine.SetMaxMotorPower(0.50d);
        } else if (gamepad1.y) {
            engine.SetMaxMotorPower(0.25d);
        } else if (gamepad1.a) {
            engine.SetMaxMotorPower(0.0d);
        } else if (gamepad1.x) {
            bottomClaw.setPosition(1.0d);
        } else if (gamepad1.b) {
            bottomClaw.setPosition(0.0d);
        }

        if (gamepad2.dpad_up) {
            encoder_motor.setPower(0.20d);
        } else if (gamepad2.dpad_down) {
            encoder_motor.setPower(-0.20d);
        } else {
            encoder_motor.setPower(0.0d);
        }

        if (gamepad2.x) {
            topClaw.setPosition(1.0d);
        } else if (gamepad2.b) {
            topClaw.setPosition(-1.0d);
        }

        if(gamepad2.right_bumper){
            lift_motor.setPower(-1.0d);
        }
        if(gamepad2.left_bumper){
             lift_motor.setPower(1.0d);
    }
        if(gamepad2.a){
            lift_motor.setPower(0.2d);
        }
        if (!gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.dpad_up) {
            engine.SetSpeed(left, right);
        }
        else {
            engine.Move(GetInputs(gamepad1), 1.0d);
        }
        telemetry.addData("Encoders: ", encoder_motor.getCurrentPosition());
        telemetry.addData("Left Stick: ", gamepad1.left_stick_y);
        telemetry.addData("Right Stick: ", gamepad1.right_stick_y);
        telemetry.addData("Left Power: ", left);
        telemetry.addData("Right Power: ", right);
        telemetry.addData("Max Speed: ", maxSpeed);

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
}
