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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

//this test will have provided for thechanges on the lists. Anything that was deleted or commented out in this program can be found in Test 25.
@TeleOp(name="RevHubTest 46", group="Pushbot")
public class RevHubTest_46 extends OpMode {
    private enum LiftType  {
        Manual,
        Relic,
    }

    private boolean _leftTriggerDown = false;
    private boolean _rightTriggerDown = false;

    private boolean    _lockOn = false;



    private int _lockPosition = 0;
    private int liftMax = 1500;

    private double maxSpeed = 1.0d;
    private LiftType liftMode = LiftType.Manual;

    private SampleOp_EngineModule engine = new SampleOp_EngineModule();

    private ColorSensor colorRight = null;
    private ColorSensor colorLeft = null;

    private DcMotor lift_motor_Left = null;
    private DcMotor lift_motor_Right = null;

    private Servo jewelArm = null;
    private Servo topClaw = null;
    private Servo botClaw = null;
    private Servo gripLeft = null;
    private Servo gripRight = null;
    private Servo clawRollerTopLeft = null;
    private Servo clawRollerTopRight = null;

    private int _liftPosition = 0;
    //lift_motor_Right.getCurrentPosition();

    private DcMotor relic_Motor = null;
    private DcMotor scissor_Relic = null;

    private int _scissPosition1 = 310;
    private int _scissPosition2 = 1237;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        engine.Initialize(hardwareMap);

        jewelArm = hardwareMap.servo.get("jewelArm");
        topClaw = hardwareMap.servo.get("topClaw");
        topClaw.setDirection(Servo.Direction.FORWARD);

        botClaw = hardwareMap.servo.get("botClaw");
        botClaw.setDirection(Servo.Direction.FORWARD);



        lift_motor_Left = hardwareMap.get(DcMotor.class, "liftMotor");
        lift_motor_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift_motor_Right = hardwareMap.get(DcMotor.class, "liftMotor2");
        lift_motor_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        colorLeft = hardwareMap.colorSensor.get("colorLeft1");
        colorRight = hardwareMap.colorSensor.get("colorRight0");

        _liftPosition = lift_motor_Right.getCurrentPosition();

        gripLeft = hardwareMap.servo.get("gripLeft");
        gripLeft.setDirection(Servo.Direction.REVERSE);

        gripRight = hardwareMap.servo.get("gripRight");

        relic_Motor = hardwareMap.get(DcMotor.class, "relicMotor");
        relic_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        relic_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        scissor_Relic = hardwareMap.get(DcMotor.class, "scissor");
        scissor_Relic.setDirection(DcMotorSimple.Direction.REVERSE);
        scissor_Relic.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        clawRollerTopRight = hardwareMap.servo.get("clawRollerTopRight");
        clawRollerTopRight.setDirection(Servo.Direction.FORWARD);

        clawRollerTopLeft = hardwareMap.servo.get("clawRollerTopLeft");
        clawRollerTopLeft.setDirection(Servo.Direction.FORWARD);
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
        gripRight.setPosition(0.0d);
        gripLeft.setPosition(0.0d);
        clawRollerTopLeft.setPosition(0.0d);
        clawRollerTopRight.setPosition(0.0d);

        _lockOn = false;

        lift_motor_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift_motor_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        relic_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        scissor_Relic.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left = clamp(gamepad1.right_stick_y);
        double right = clamp(gamepad1.left_stick_y);


        //GAMEPAD 1 SECTION
        //For half speed and full speed toggle
        if (gamepad1.right_bumper) {
            engine.SetMaxMotorPower(0.5d);
        }
        else if (gamepad1.left_bumper){
            engine.SetMaxMotorPower(1.0d);
        }

        //This controls the mechanum movement staments as well as the joysticks. Please refer to sampleOp_Engine Module
        //For more information and how the joysticks and robot moves.
        if (!gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.dpad_up) {
            engine.SetSpeed(left, right);
        } else {
            engine.Move(GetInputs(gamepad1), 1.0d);
        }
        if (gamepad2.x) {
            liftMode = LiftType.Relic;
        }
        else if (gamepad2.b) {
            liftMode = LiftType.Manual;
        }
       // Manual Mode
        if (liftMode == LiftType.Manual) {
            //GAMEPAD 2 SECTION
            //controls lift motor right(tower) power inputs. Controller using triggers. Also controls locking sequence.

            if (_leftTriggerDown) {
                if (gamepad2.left_trigger <= 0.10d) {
                    _leftTriggerDown = false;
                }
            } else {
                if (gamepad2.left_trigger >= 0.90d) {
                    _leftTriggerDown = true;
                    _lockOn = !_lockOn;
                    _lockPosition = lift_motor_Right.getCurrentPosition();
                }
            }

            if (gamepad2.right_trigger >= 0.10d) {
                lift_motor_Right.setPower(1.0d);
             //   _lockOn = false;
            } else {
                lift_motor_Right.setPower(0.0d);
            }

            //TOGGLE for lock on tower lift
            if (_lockOn) {
                if (lift_motor_Right.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                    lift_motor_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                lift_motor_Right.setPower(0.5d);
                lift_motor_Right.setTargetPosition(_lockPosition);

                if (gamepad2.left_stick_y >= 0.1d) {
                    topClaw.setPosition(1.0d);
                }
                else if (gamepad2.left_stick_y <= -0.1d) {
                    topClaw.setPosition(0.45d);
                }

                if (gamepad2.right_stick_y >= 0.1d) {
                    botClaw.setPosition(1.0d);
                }
                else if (gamepad2.right_stick_y <= -0.1d) {
                    botClaw.setPosition(0.3d);
                }
            }

            else{
                lift_motor_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                if (gamepad2.left_stick_y >= 0.1d) {
                    topClaw.setPosition(1.0d);
                }
                else if (gamepad2.left_stick_y <= -0.1d) {
                    topClaw.setPosition(0.40d);
                }

                if (gamepad2.right_stick_y >= 0.1d) {
                    botClaw.setPosition(1.0d);
                }
                else if (gamepad2.right_stick_y <= -0.1d) {
                    botClaw.setPosition(0.0d);
                }
            }

            //Controls lift motor left(drawerslide) power inputs. Controlled using bumpers.
            if (gamepad2.right_bumper) {
                lift_motor_Left.setPower(1.0d);
            }
            else if (gamepad2.left_bumper) {
                lift_motor_Left.setPower(-1.0d);
            }
            else {
                lift_motor_Left.setPower(0.0d);
            }
        }

        if (gamepad2.dpad_down && scissor_Relic.getCurrentPosition() >= 100 || gamepad2.y) {
            if (scissor_Relic.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                scissor_Relic.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            scissor_Relic.setPower(-1.0d);
        }
        //For the Scissor Lift, the old value is 3220
        else if (gamepad2.dpad_up && scissor_Relic.getCurrentPosition() <= 2200) {
            if (scissor_Relic.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                scissor_Relic.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            scissor_Relic.setPower(1.0d);
        }
        else if(gamepad2.a){
            if (scissor_Relic.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                scissor_Relic.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (relic_Motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
                relic_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            relic_Motor.setPower(1.0d);
            relic_Motor.setTargetPosition(850);

            scissor_Relic.setPower(1.0d);
            scissor_Relic.setTargetPosition(_scissPosition1);
        }
        else if (gamepad2.b){
            if (scissor_Relic.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                scissor_Relic.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (relic_Motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
                relic_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            scissor_Relic.setPower(1.0d);
            scissor_Relic.setTargetPosition(_scissPosition2);

            relic_Motor.setPower(1.0d);
            relic_Motor.setTargetPosition(1970);
        }
        else {
            if (scissor_Relic.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                scissor_Relic.setPower(0.0d);
            }
        }



        if (gamepad2.dpad_right && relic_Motor.getCurrentPosition() <= 1975){
            if (relic_Motor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                relic_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            relic_Motor.setPower(1.0d);
        }
        else if (gamepad2.dpad_left && relic_Motor.getCurrentPosition() >= 50 || gamepad2.x){
            if (relic_Motor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                relic_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            relic_Motor.setPower(-1.0d);
        }
        else {
            if (relic_Motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                relic_Motor.setPower(0.0d);
            }

        }

        if (gamepad1.a) {
            gripLeft.setPosition(0.0d);
            gripRight.setPosition(0.0d);
        }
        else if (gamepad1.b) {
            gripLeft.setPosition(1.0d);
            gripRight.setPosition(1.2d);
        }
        else if (gamepad1.x) {
            gripLeft.setPosition(0.3d);
            gripRight.setPosition(0.45d);
        }

        if (gamepad1.y){
            clawRollerTopLeft.setPosition(0.5d);
            clawRollerTopRight.setPosition(0.5d);

        }



        telemetry.addData("Scissor Lift: ", scissor_Relic.getCurrentPosition());
        telemetry.addData("Relic Motor: ", relic_Motor.getCurrentPosition());

        telemetry.addData("Lock: ", _lockOn);
        telemetry.addData("Lift Position: ", _lockPosition);
        telemetry.addData("Trigger", _leftTriggerDown);

        telemetry.addData("Left Lift: ", lift_motor_Left.getCurrentPosition());
        telemetry.addData("Right Lift: ", lift_motor_Right.getCurrentPosition());

        telemetry.addData("Left: ", lift_motor_Left.getPower());
        telemetry.addData("Right: ", lift_motor_Right.getPower());

        telemetry.addData("leftLift: ", lift_motor_Left.getPower());
        telemetry.addData("rightLift: ", lift_motor_Right.getPower());

        telemetry.addData("FrontLeft: ", engine.GetPosition(SampleOp_EngineModule.EngineMotor.FrontLeft));
        telemetry.addData("FrontRight: ", engine.GetPosition(SampleOp_EngineModule.EngineMotor.FrontRight));
        telemetry.addData("BackLeft: ", engine.GetPosition(SampleOp_EngineModule.EngineMotor.BackLeft));
        telemetry.addData("BackRight: ", engine.GetPosition(SampleOp_EngineModule.EngineMotor.BackRight));



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
