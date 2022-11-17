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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name="Mecanum: TeleOp", group="Mecanum" )
public class MecanumTeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareMecanum robot = new HardwareMecanum(); //use Mecanum's Hardware

    @Override
    public void runOpMode() throws InterruptedException {

        // Declare our motors
        // Make sure your ID's match your configuration

        double x1; //leftright
        double y1; //frontback

        double fortyfiveInRads = -Math.PI/4;
        double cosine45 = Math.cos(fortyfiveInRads);
        double sine45 = Math.sin(fortyfiveInRads);

        double x2; //left&back
        double y2; //right&front

        robot.init(hardwareMap);

        //send telemetry message to signify robot waiting:
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            y1 = -gamepad1.left_stick_y; // Remember, this is reversed!
            x1 = gamepad1.left_stick_x;  // Counteract imperfect strafing

            //need to rotate 45 degrees
            y2 = y1*cosine45 + x1*sine45;
            x2 = x1*cosine45 - y1*sine45;
            //double rx = gamepad1.right_stick_x;

            robot.leftFrontDrive.setPower(x2*.75);
            robot.rightRearDrive.setPower(x2*.75);
            robot.leftRearDrive.setPower(y2*.75);
            robot.rightFrontDrive.setPower(y2*.75);

            telemetry.addData("x1", "%.2f" , x1);
            telemetry.addData("y1", "%.2f" , y1);
            telemetry.update();

            sleep(50);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
           //double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            //double frontLeftPower = (y + x + rx) / denominator;
            //double backLeftPower = (y - x + rx) / denominator;
            //double frontRightPower = (y - x - rx) / denominator;
            //double backRightPower = (y + x - rx) / denominator;


        }
    }
}

