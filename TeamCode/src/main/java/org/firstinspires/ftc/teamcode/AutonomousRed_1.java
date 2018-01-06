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

//In this test I programmed everything according to the chart on my phone. Programmed the motors to move with encoders
@TeleOp(name="Autonomous Red", group="Pushbot")
public class AutonomousRed_1 extends OpMode {
    private SampleOp_EngineModule engine = new SampleOp_EngineModule();
    private ColorSensor color_sensor;
    private DistanceSensor distance_sensor = null;
 // private DcMotor encoder_motor = null;
    private DcMotor lift_motor_Left = null;
    private DcMotor lift_motor_Right = null;


    private Servo jewelArm = null;
    //private Servo accuator = null;
    private Servo topClaw = null;
    private Servo botClaw = null;

    private double maxSpeed = 1.0d;
    private double moveSpeed = 1.0d;

    private int stage = 0;

    private double timeOffSet =0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        engine.Initialize(hardwareMap);

        jewelArm = hardwareMap.servo.get("jewelArm");
        //accuator = hardwareMap.servo.get("accuator");
        topClaw = hardwareMap.servo.get("topClaw");
        botClaw = hardwareMap.servo.get("botClaw");


        color_sensor = hardwareMap.colorSensor.get("color");
        distance_sensor = hardwareMap.get(DistanceSensor.class, "color");
        //encoder_motor = hardwareMap.get(DcMotor.class, "encoder");
        //encoder_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift_motor_Left = hardwareMap.get(DcMotor.class, "liftMotor");
        lift_motor_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift_motor_Right = hardwareMap.get(DcMotor.class, "liftMotor2");
        lift_motor_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        engine.SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        topClaw.setPosition(0.0d);
        botClaw.setPosition(0.0d);
        jewelArm.setPosition(0.0d);
        lift_motor_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift_motor_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        engine.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (stage) {
            case 0:
                jewelArm.setPosition(1.0d);

                timeOffSet = time;
                stage++;
                break;
            case 1:
                if (jewelArm.getPosition() >= 0.90d && time > (timeOffSet + 2.0d)) {
                    stage++;
                }
                break;
            case 2:
                if (color_sensor.red() > color_sensor.blue()) {
                    stage = 4;
                }
                else {
                    stage = 3;
                }
                break;
            case 3:
                if (engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontRight).getCurrentPosition() > -250) {
                    engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontRight).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    engine.SetSpeed(-0.10d, -0.10d);
                }
                else {
                    stage = 10;
                }
                break;
            case 4:
                if (engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontRight).getCurrentPosition() < 250) {
                    engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontRight).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    engine.SetSpeed(1.0d, 1.0d);
                    stage++;
                }
                else {
                    stage = 10;
                }
                break;


                   // if (time < 30.0d)
                   // {
                    /*jewelArm.setPosition(1.0d);
                        if (color_sensor.red() >  color_sensor.blue()) {
                            engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontRight).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            engine.SetSpeed(-1.0d,-1.0d);
                        }
                        else {
                            engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontRight).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            engine.SetSpeed(1.0d, 1.0d);
                        }
                    //}*/


                //Turns Right
              /* if (engine.GetPosition(SampleOp_EngineModule.EngineMotor.FrontRight) >= 1000) {
                    engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontRight).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    stage = 1;
                }
                else {
                    engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontRight).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    engine.SetSpeed(-0.50d, 0.50d);
                }
                break;
                */
            /*case 1:
                //Resets Encoders
                /*if (engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontRight).getMode() == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
                    stage = 2;
                }
                break;
                */
            //case 2:
                //Turns Left
                /*if (engine.GetPosition(SampleOp_EngineModule.EngineMotor.FrontRight) <= -1000) {
                    engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontRight).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    stage = 3;
                }
                else {
                    engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontRight).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    engine.SetSpeed(0.50d, -0.50d);
                }
                break;
                */
            default:
                engine.Stop();
                break;
        }

        telemetry.addData("Stage: ", stage);
        telemetry.addData("FrontLeft: ", engine.GetPosition(SampleOp_EngineModule.EngineMotor.FrontLeft));
        telemetry.addData("FrontRight: ", engine.GetPosition(SampleOp_EngineModule.EngineMotor.FrontRight));
        telemetry.addData("BackLeft: ", engine.GetPosition(SampleOp_EngineModule.EngineMotor.BackLeft));
        telemetry.addData("BackRight: ", engine.GetPosition(SampleOp_EngineModule.EngineMotor.BackRight));

        telemetry.addData("Distance (cm): ", String.format(Locale.US, "%.02f", distance_sensor.getDistance(DistanceUnit.CM)));

        telemetry.addData("Red: ", color_sensor.red());
        telemetry.addData("Green: ", color_sensor.green());
        telemetry.addData("Blue: ", color_sensor.blue());
        telemetry.addData("Alpha: ", color_sensor.alpha());
        telemetry.addData("Argb: ", color_sensor.argb());
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
