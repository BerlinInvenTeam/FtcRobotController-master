/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.teamcode.HardwarePushbot;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Pushbot: Teleop Tank", group="Pushbot")

//@Disabled
public class PushbotTeleopTank_Iterative extends OpMode{

    /* Declare OpMode members. */
    HardwarePushbot robot       = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    double          STATIONARY_SERVO = 0.5;
    double          FULL_SPEED_RIGHT_SERVO = 0;
    double          FULL_SPEED_LEFT_SERVO = 1;
    final double    MID_SERVO = 0.5;
    final double    BUCKET_POS = 0.3;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Berlin InvenTeam");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        robot.swivelBase.setPosition(robot.MID_SERVO);
        robot.bucketClaw.setPosition(robot.MID_SERVO);
        robot.bucket.setPosition(BUCKET_POS);}

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    public void sleep(int milis){
        try {
            Thread.sleep(milis);
        } catch (Exception e){}
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;
        double bucket;
        double cntPower;

        final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
        final int CYCLE_MS = 50;     // period of each cycle
        final double MAX_POS = 1.0;     // Maximum rotational position
        final double MIN_POS = 0.0;     // Minimum rotational position
        double bucketPosition = .5;       // set bucket to half way up.
        final double BUCKET_MAX_POS = .3;     // Maximum rotational position
        final double BUCKET_MIN_POS = .1;     // Minimum rotational position

        // Define class members
        double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
        boolean rampUp = true;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;

        bucket = -gamepad2.right_stick_y;


        robot.leftRearDrive.setPower(left);
        robot.rightRearDrive.setPower(right);
        robot.leftFrontDrive.setPower(left);
        robot.rightFrontDrive.setPower(right);

        robot.bucketDrive.setPower(bucket);

        // check the gamepad buttons and if pressed, increment the appropriate position
        // variable to change the servo location.

        // move bucket down on A button if not already at lowest position.
        //if (gamepad1.a && bucketPosition > MIN_POS)

        if (gamepad2.a) {
            robot.bucket.setPosition(BUCKET_MAX_POS);
            //telemetry.addData("Keypad", "pad_a clicked. position = " + bucketPosition);
        }

        if (gamepad2.b) {
            robot.bucket.setPosition(BUCKET_MIN_POS);
            //telemetry.addData("Keypad", "pad_b clicked. position = " + bucketPosition);
        }

        // Set continuous servo power level and direction.
        if (gamepad2.dpad_left) {
            cntPower = 1;
            robot.turnCarousel.setPower(cntPower);
            telemetry.addData("Keypad", "dpad_left clicked. power = " + cntPower);
        } else if (gamepad2.dpad_right) {
            cntPower = -1;
            robot.turnCarousel.setPower(cntPower);
            telemetry.addData("Keypad", "dpad_right clicked. power = " + cntPower);
        } else {
            cntPower = 0;
            robot.turnCarousel.setPower(cntPower);

        }

        if (gamepad2.x) {
            robot.bucketClaw.setPosition(position = MAX_POS);
            telemetry.addData("Keypad", "pad_x clicked. power = " + position);
        } else if (gamepad2.y) {
            robot.bucketClaw.setPosition(position = MIN_POS);
            telemetry.addData("Keypad", "pad_y clicked. power = " + position);
        }
    }

        /*
        if (gamepad2.right_bumper) {
            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            robot.swivelBase.setPosition(robot.MID_SERVO + clawOffset);
            robot.swivelBase.setPosition(1);
        }
        else if (gamepad2.left_bumper) {
            clawOffset = Range.clip(clawOffset, 0.5, -0.5);
            robot.swivelBase.setPosition(0);
            clawOffset -= CLAW_SPEED;
        }
         */
        /*
        // slow the servo, according to the rampUp (direction) variable.
        if (gamepad2.right_bumper) {
            if (rampUp){
                // Keep stepping up until we hit the max value.
                position += INCREMENT;
                if (position >= MAX_POS) {
                     robot.swivelBase.setPosition(position = MAX_POS);
                    rampUp = !rampUp;   // Switch ramp direction
                }
            }
        }
        else if (gamepad2.left_bumper) {
            if (!rampUp)
                // Keep stepping down until we hit the min value.
                position -= INCREMENT;
                if (position <= MIN_POS) {
                    robot.swivelBase.setPosition(position = MIN_POS);
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }

        }
         */
        /*
        // Use gamepad left & right Bumpers to open and close the claw
        if (gamepad1.right_bumper)
            clawOffset += CLAW_SPEED;FULL_SPEED_RIGHT_SERVO
        else if (gamepad1.left_bumper)
            clawOffset -= CLAW_SPEED;

        // Move both servos to new position.  Assume servos are mirror image of each other.
        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        robot.swivelBase.setPosition(robot.MID_SERVO + clawOffset);
        robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);

        // Use gamepad buttons to move the arm up (Y) and down (A)
        if (gamepad1.y)
            robot.swivelBase.setPower(robot.ARM_UP_POWER);
        else if (gamepad1.a)
            robot.swivelBase.setPower(robot.ARM_DOWN_POWER);
        else
            robot.swivelBase.setPower(0.0);

        // Send telemetry message to signify robot running;
        telemetry.addData("claw",  "Offset = %.2f", clawOffset);
        */
        /*
        telemetry.addData("left",  "%f", left);
        telemetry.addData("right", "%f", right);
        */
    //}

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop(){
    }
}