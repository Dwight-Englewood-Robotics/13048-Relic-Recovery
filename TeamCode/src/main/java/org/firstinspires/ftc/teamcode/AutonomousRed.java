package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
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


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Bot;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutonomousRed",group="Auton")
//@Disabled
public class AutonomousRed extends OpMode {
    // 1220 ticks revelation
    // 63
    // 90
    // 100
    //13 cm is diameter of wheel
    // Declare OpMode members.
    private ElapsedTime timer = new ElapsedTime();
    Bot robot = new Bot();
    int commandNum = 0;
    boolean red;
    int turned = 0;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, false);
//        robot.resetServo();
//        robot.resetEncoders();
        //telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        robot.downColor();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch(commandNum) {
            case 0:

                if(timer.milliseconds() > 5000) {
                    commandNum = 1;
                    timer.reset();
                    //robot.upColor();
                    red = robot.isRed();
                    break;
                }
                telemetry.addData("Red? ",  red);
                break;
            case 1:
                if(!red){
                    telemetry.addData("Red: ", true);
                    telemetry.update();
                    if(timer.milliseconds() < 1000){
                        robot.strafe(1.0);
                        robot.upColor();
                    }
                    else {
                        robot.strafe(0);
                        robot.upColor();
                        commandNum = -1;
                        break;
                    }
                }
                else{
                    telemetry.addData("Blue: ", true);
                    telemetry.update();
                    if(robot.adjustHeading(45)) {
                        commandNum = 2;
                        timer.reset();
                        break;
                    }
                }
                break;
            case 2:

                if(timer.milliseconds() > 1000){
                    commandNum = 3;
                    timer.reset();
                }
                robot.upColor();
                break;
            case 3:
                if(timer.milliseconds() > 1000){
                    robot.drive(0, 0, 0);
                    commandNum = -1;
                    break;
                }
                robot.drive(1, 0, 0);
                break;
        }

        telemetry.update();
//        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("done", true);
        telemetry.update();
    }

}