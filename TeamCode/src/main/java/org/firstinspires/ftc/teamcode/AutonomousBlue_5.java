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
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

//import com.qualcomm.robotcore.hardware.DistanceSensor;

//In this test I programmed everything according to the chart on my phone. Programmed the motors to move with encoders
@TeleOp(name="Autonomous Blue_5", group="Pushbot")
public class AutonomousBlue_5 extends OpMode {
    private SampleOp_EngineModule engine = new SampleOp_EngineModule();
 //   private ColorSensor color_sensor;
   // private DistanceSensor distance_sensor = null;
 // private DcMotor encoder_motor = null;
    private DcMotor lift_motor_Left = null;
    private DcMotor lift_motor_Right = null;

    private ColorSensor colorRight = null;
    private ColorSensor colorLeft = null;

    private Servo jewelArm = null;
    //private Servo accuator = null;
    private Servo topClaw = null;
    private Servo botClaw = null;
    private Servo knocker = null;

    private double maxSpeed = 1.0d;
    private double moveSpeed = 1.0d;

    private int stage = -1;

    private double timeOffSet =0;

    private VuforiaLocalizer vuforia = null;
    private VuforiaTrackables relicTrackables = null;
    private VuforiaTrackable relicTemplate = null;

    private int mechanum = 0;
    private int center = 1150;
    private int left = 1950;
    private int right = 350;

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
        knocker = hardwareMap.servo.get("knocker");
        colorLeft = hardwareMap.colorSensor.get("colorLeft1");
        colorRight = hardwareMap.colorSensor.get("colorRight0");

      //  color_sensor = hardwareMap.colorSensor.get("color");
       // distance_sensor = hardwareMap.get(DistanceSensor.class, "color");
        //encoder_motor = hardwareMap.get(DcMotor.class, "encoder");
        //encoder_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift_motor_Left = hardwareMap.get(DcMotor.class, "liftMotor");
        lift_motor_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift_motor_Right = hardwareMap.get(DcMotor.class, "liftMotor2");
        lift_motor_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        engine.SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AZjwTlP/////AAAAGQ0Nw67MdEzAm0Cuvb+oJGOCSFyqWb6WplQDZyJo9vOTw0iLTPRXEXyjZ/WAN+V4wKPlbTv0pbPad2yBRZ3vyHl4hRafu7wJGLMgzfsTeozU4SzDgl378Q3zGZsJWK+UkfmekmXkzeysQqnkL0rZK+1KNaARZjbR14/OONP2j9AjqOoJa6yMMMN/2jgM5x/Lshv/++vTyHUzJYs7rEHh26FGDtlE9J8FoRXaNPrii2i3j2msG2bTO0AzrlwnK6AMtP92bqRGHnIvU4GQVnMgi2fn/EoxNRLiNktloDfdzv9vRaU3zgpOENxmqzkfdBskZzfy6EYGsEzMQPtJ07brBfDhC0l9w4kvuBi2i5dK7Mbo";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");

        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        relicTrackables.activate();
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
        topClaw.setPosition(1.0d);
        botClaw.setPosition(0.0d);
        jewelArm.setPosition(0.0d);
      //  knocker.setPosition(0.0d);
        lift_motor_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift_motor_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        engine.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mechanum = center;
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "%s visible", vuMark);

            if (vuMark == RelicRecoveryVuMark.CENTER) {
                mechanum = center;
            }
            else if (vuMark == RelicRecoveryVuMark.LEFT) {
                mechanum = left;
            }
            else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                mechanum = right;
            }
        }

        switch (stage) {
            case -1:
                jewelArm.setPosition(0.8d);

                NextStage();
                break;
            case 0:
                if (jewelArm.getPosition() >= 0.7d) {
                    jewelArm.setPosition(1.0d);
                }


                NextStage();
                break;
            case 1:
                if (jewelArm.getPosition() >= 0.90d && time > (timeOffSet + 2.0d)) {
                    NextStage();
                }
                break;
            case 2:
                String _color = checkColor();

                if (_color == "LEFT") {
                    GoToStage(4);
                } else if (_color == "RIGHT") {
                    GoToStage(3);
                } else {
                    GoToStage(5);
                }
                break;
            case 3:
                knocker.setPosition(1.0d);

                if (time > (timeOffSet + 2.0d)) {
                    GoToStage(6);
                }
                break;
            case 4:
                knocker.setPosition(-1.0d);

                if (time > (timeOffSet + 2.0d)) {
                    GoToStage(6);
                }
                break;

            case 5:
                GoToStage(6);
                break;

            case 6:
                jewelArm.setPosition(0.0d);

                if (time > (timeOffSet + 2.0d)) {
                    NextStage();
                }
                break;
            case 7:
                lift(800, 0);


                if (time > (timeOffSet + 2.0d)) {
                    NextStage();
                }
                break;
            case 8:
                if (engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getCurrentPosition() < 1850) {
                    if (engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                        engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }

                    engine.SetSpeed(0.20d, 0.20d);
                } else {
                    NextStage();
                }
                break;
            case 9:
                if (engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getMode() != DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
                    engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    NextStage();
                }
                break;
            case 10:
                if (engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getCurrentPosition() < 3300) {
                    if (engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                        engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }

                    engine.SetSpeed(0.20d, -0.20d);
                }

                else {
                    NextStage();
                }

                break;
            case 11:
                if (engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getMode() != DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
                    engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    NextStage();
                }
                break;
            case 12:
                if (engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getCurrentPosition() < mechanum) {
                    if(engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                        engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }

                    engine.Move(SampleOp_States.Dpad.Right, 0.20d);
                  //  engine.SetSpeed(-0.20d, -0.20d);
                }
                else {
                    NextStage();
                }
                break;
            case 13:
                topClaw.setPosition(0.0d);
                NextStage();
                break;
            case 14:
                if (engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getMode() != DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
                    engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    NextStage();
                }
                break;
            case 15:
                if (engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getCurrentPosition() > -500) {
                    if(engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                        engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }

                      engine.SetSpeed(-0.15d, -0.15d);
                }
                else {
                    NextStage();
                }
            default:
                engine.Stop();
                break;
        }

        telemetry.addData("Stage: ", stage);
        telemetry.addData("FrontLeft: ", engine.GetPosition(SampleOp_EngineModule.EngineMotor.FrontLeft));
        telemetry.addData("FrontRight: ", engine.GetPosition(SampleOp_EngineModule.EngineMotor.FrontRight));
        telemetry.addData("BackLeft: ", engine.GetPosition(SampleOp_EngineModule.EngineMotor.BackLeft));
        telemetry.addData("BackRight: ", engine.GetPosition(SampleOp_EngineModule.EngineMotor.BackRight));

       // telemetry.addData("Distance (cm): ", String.format(Locale.US, "%.02f", distance_sensor.getDistance(DistanceUnit.CM)));

      //  telemetry.addData("Red: ", color_sensor.red());
       // telemetry.addData("Green: ", color_sensor.green());
        //telemetry.addData("Blue: ", color_sensor.blue());
        //telemetry.addData("Alpha: ", color_sensor.alpha());
        //telemetry.addData("Argb: ", color_sensor.argb());
        //telemetry.update();
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

    private void lift(int position) {
        lift(position, position);
    }

    private void lift(int left, int right) {
        lift_motor_Left.setPower(1.0d);
        lift_motor_Left.setTargetPosition(left);

        lift_motor_Right.setPower(1.0d);
        lift_motor_Right.setTargetPosition(right);
    }

    private void GoToStage(int target) {
        timeOffSet = time;
        stage = target;
    }

    private void NextStage() {
        timeOffSet = time;
        stage++;
    }

    private void PreviousStage() {
        timeOffSet = time;
        stage--;
    }

}
