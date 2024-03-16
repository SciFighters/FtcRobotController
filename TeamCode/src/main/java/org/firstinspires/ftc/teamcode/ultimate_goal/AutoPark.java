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
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR NEAR PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.ultimate_goal;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ultimate_goal.util.BananaPipeline;
import org.firstinspires.ftc.teamcode.ultimate_goal.util.CvCam;
import org.firstinspires.ftc.teamcode.ultimate_goal.util.DriveClass;
import org.firstinspires.ftc.teamcode.ultimate_goal.util.GameClass;
import org.firstinspires.ftc.teamcode.ultimate_goal.util.Location;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(group = "Linear Opmode")
@Disabled
public class AutoPark extends LinearOpMode {
	final double tile = 0.6;
	//Location parkPos = new Location(-0.8, 2);
	Location startingPosition = new Location(0,0);
	private DriveClass robot = new DriveClass(this, DriveClass.ROBOT.COBALT, startingPosition).useEncoders();
	private GameClass game = new GameClass(this);    // Declare OpMode members.

	// main functions ==============================================================================

	@Override
	public void runOpMode() {

		telemetry.addData("Status", "Initialized");
		telemetry.update();
		robot.init(hardwareMap);
		game.init(hardwareMap);
		// Wait for the game to start (driver presses PLAY)
		waitForStart();
		sleep(135 * 1000); //2:15 minutes
		robot.drive(2, 0, 1, 0, false);
	}
}
