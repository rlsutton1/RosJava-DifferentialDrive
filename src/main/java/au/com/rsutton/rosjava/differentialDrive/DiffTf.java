package au.com.rsutton.rosjava.differentialDrive;

import geometry_msgs.Transform;
import geometry_msgs.TransformStamped;
import nav_msgs.Odometry;

import org.apache.commons.lang.NotImplementedException;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import std_msgs.Int16;

/*
 * 
 * Ported to java by Robert Sutton 2014
 * 
 diff_tf.py - follows the output of a wheel encoder and
 creates tf and odometry messages.
 some code borrowed from the arbotix diff_controller script
 A good reference: http://rossum.sourceforge.net/papers/DiffSteer/

 Copyright (C) 2012 Jon Stephan. 

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 ----------------------------------
 Portions of this code borrowed from the arbotix_python diff_controller.

 diff_controller.py - controller for a differential drive
 Copyright (c) 2010-2011 Vanadium Labs LLC.  All right reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of Vanadium Labs LLC nor the names of its 
 contributors may be used to endorse or promote products derived 
 from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

public class DiffTf extends AbstractNodeMain
{

	private int rate;
	private int ticks_meter;
	private double base_width;
	private String base_frame_id;
	private String odom_frame_id;
	private int encoder_min;
	private int encoder_max;
	private double encoder_low_wrap;
	private double encoder_high_wrap;
	private double t_delta;
	private double t_next;
	private Double enc_left;

	private double left;
	private Double right;
	private int lmult;
	private int prev_lencoder;
	private int rmult;
	private int prev_rencoder;
	private double x;
	private double y;
	private double th;
	private double dx;
	private double dr;
	private long then;
	private Double enc_right;
	private String nameSpace;

	public DiffTf(String nameSpace)
	{
		this.nameSpace = nameSpace;
		// initialize with sensible defaults
		rate = 10;
		ticks_meter = 50;
		base_width = 0.245; // The wheel base width in meters

		base_frame_id = "base_link";// # the name of the base frame of the robot
		odom_frame_id = "odom";// # the name of the odometry reference frame

		encoder_min = -32768;
		encoder_max = 32768;
		encoder_low_wrap = (encoder_max - encoder_min) * 0.3 + encoder_min;
		encoder_high_wrap = (encoder_max - encoder_min) * 0.7 + encoder_min;

		t_delta = 1.0 / rate;
		t_next = System.currentTimeMillis() + t_delta;

		// internal data
		enc_left = null;// # wheel encoder readings
		enc_right = null;//
		left = 0d; // # actual values coming back from robot
		right = 0d;
		lmult = 0;
		rmult = 0;
		prev_lencoder = 0;
		prev_rencoder = 0;
		x = 0;// # position in xy plane
		y = 0;
		th = 0;
		dx = 0;// # speeds in x/rotation
		dr = 0;
		then = System.currentTimeMillis();
	}

	@Override
	public void onStart(ConnectedNode connectedNode)
	{
		// log = connectedNode.getLog();

		final Publisher<Odometry> odomPub = Topic.ODOM.newPublisher(
				connectedNode, nameSpace);

		final Publisher<TransformStamped> odomBroadcaster = Topic.TF
				.newPublisher(connectedNode, nameSpace);

		// This CancellableLoop will be canceled automatically when the node
		// shuts
		// down.
		connectedNode.executeCancellableLoop(new CancellableLoop()
		{

			@Override
			protected void setup()
			{
			}

			@Override
			protected void loop() throws InterruptedException
			{
				update();
				Thread.sleep(rate);
			}

			private void update()
			{
				long now = System.currentTimeMillis();
				if (now > t_next)
				{
					long elapsed = now - then;
					then = now;
					elapsed /= 1000;

					// calculate odometry

					double d_left = 0;
					double d_right = 0;
					if (enc_left != null)

					{
						d_left = (left - enc_left) / ticks_meter;
						d_right = (right - enc_right) / ticks_meter;
					}
					enc_left = left;
					enc_right = right;

					// distance traveled is the average of the two wheels
					double d = (d_left + d_right) / 2;
					// this approximation works (in radians) for small angles
					th = (d_right - d_left) / base_width;
					// calculate velocities
					dx = d / elapsed;
					dr = th / elapsed;

					if (d != 0)
					{
						// calculate distance traveled in x and y
						x = Math.cos(th) * d;
						y = -Math.sin(th) * d;
						// calculate the final position of the robot
						x = x + (Math.cos(th) * x - Math.sin(th) * y);
						y = y + (Math.sin(th) * x + Math.cos(th) * y);
					}
					if (th != 0)
					{
						th = th + th;
					}

					// publish the odom information
					TransformStamped transform = odomBroadcaster.newMessage();
					transform.getTransform().getRotation().setX(0);
					transform.getTransform().getRotation().setY(0);
					transform.getTransform().getRotation()
							.setZ(Math.sin(th / 2));
					transform.getTransform().getRotation()
							.setW(Math.cos(th / 2));
					transform.getTransform().getTranslation().setX(x);
					transform.getTransform().getTranslation().setY(y);
					transform.getTransform().getTranslation().setZ(0);
					transform.getHeader().setStamp(new Time());
					transform.setChildFrameId(odom_frame_id);
					transform.getHeader().setFrameId(base_frame_id);
					odomBroadcaster.publish(transform);

					Odometry odom = odomPub.newMessage();

					odom.getHeader().setStamp(new Time());
					odom.getHeader().setFrameId(odom_frame_id);
					odom.getPose().getPose().getPosition().setX(x);
					odom.getPose().getPose().getPosition().setY(y);
					odom.getPose().getPose().getPosition().setZ(0);
					odom.getPose()
							.getPose()
							.setOrientation(
									transform.getTransform().getRotation());
					odom.setChildFrameId(base_frame_id);
					odom.getTwist().getTwist().getLinear().setX(dx);
					odom.getTwist().getTwist().getLinear().setY(0);
					odom.getTwist().getTwist().getAngular().setZ(dr);
					odomPub.publish(odom);

				}
			}
		});

		Subscriber<Int16> sub_lwheel = Topic.LWHEEL.newSubscriber(
				connectedNode, nameSpace);
		sub_lwheel.addMessageListener(new MessageListener<Int16>()
		{
			@Override
			public void onNewMessage(Int16 message)
			{
				short enc = message.getData();
				if (enc < encoder_low_wrap && prev_lencoder > encoder_high_wrap)
				{
					lmult = lmult + 1;
				}

				if (enc > encoder_high_wrap && prev_lencoder < encoder_low_wrap)
				{
					lmult = lmult - 1;
				}

				left = 1.0 * (enc + lmult * (encoder_max - encoder_min));
				prev_lencoder = enc;
			}
		});
		Subscriber<Int16> sub_rwheel = Topic.RWHEEL.newSubscriber(
				connectedNode, nameSpace);
		sub_rwheel.addMessageListener(new MessageListener<Int16>()
		{
			@Override
			public void onNewMessage(Int16 message)
			{
				short enc = message.getData();
				if (enc < encoder_low_wrap && prev_rencoder > encoder_high_wrap)
				{
					rmult = rmult + 1;
				}

				if (enc > encoder_high_wrap && prev_rencoder < encoder_low_wrap)
				{
					rmult = rmult - 1;
				}

				right = 1.0 * (enc + rmult * (encoder_max - encoder_min));
				prev_rencoder = enc;
			}
		});

	}

	@Override
	public GraphName getDefaultNodeName()
	{
		throw new NotImplementedException();
	}
}