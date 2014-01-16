package au.com.rsutton.rosjava.differentialDrive;

import java.util.LinkedList;
import java.util.List;

import org.apache.commons.lang.NotImplementedException;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import std_msgs.Float32;
import std_msgs.Int16;

/*
 * 
 * Ported to java by Robert Sutton 2014
 * 
 pid_velocity - takes messages on wheel_vtarget 
 target velocities for the wheels and monitors wheel for feedback

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


 */
public class PidVelocity extends AbstractNodeMain
{

	volatile private double motor = 0;
	volatile private float target;
	private float vel;
	private double integral;
	private float error;
	private double derivative;
	private float previous_error;
	private double wheel_prev;
	volatile private double wheel_latest;
	volatile private int wheel_mult;
	volatile private int prev_encoder;
	private long then;
	private double Kp;
	private double Ki;
	private double Kd;
	private int out_min;
	private int out_max;
	private int rate;
	// private int rolling_pts;
	private int timeout_ticks;
	private int ticks_per_meter;
	private double vel_threshold;
	private int encoder_min;
	private int encoder_max;
	private double encoder_low_wrap;
	private double encoder_high_wrap;
	private List<Double> prev_vel = new LinkedList<Double>();
	private long prev_pid_time;
	volatile private int ticks_since_target;
	protected long dt_duration;
	protected double dt;
	private String nameSpace;

	public PidVelocity(String nameSpace)
	{
		// initialize with sensible defaults

		this.nameSpace = nameSpace;

		target = 0;
		motor = 0;
		vel = 0;
		integral = 0;
		error = 0;
		derivative = 0;
		previous_error = 0;
		wheel_prev = 0;
		wheel_latest = 0;
		then = System.currentTimeMillis();
		wheel_mult = 0;
		prev_encoder = 0;

		Kp = 10;
		Ki = 10;
		Kd = 0.001;
		out_min = -255;
		out_max = 255;
		double hertz = 30;
		rate = (int) (1000d / hertz);
		timeout_ticks = (int) (hertz * 4); // about 4 seconds
		ticks_per_meter = 20;
		vel_threshold = 0.001;
		encoder_min = -32768;
		encoder_max = 32768;
		encoder_low_wrap = (encoder_max - encoder_min) * 0.3 + encoder_min;
		encoder_high_wrap = (encoder_max - encoder_min) * 0.7 + encoder_min;
		prev_vel.add(0.0d);
		prev_vel.add(0.0d);
		wheel_latest = 0.0;
		prev_pid_time = System.currentTimeMillis();
		// rospy.logdebug("%s got Kp:%0.3f Ki:%0.3f Kd:%0.3f tpm:%0.3f" %
		// (nodename, Kp, Ki, Kd, ticks_per_meter))

	}

	@Override
	public void onStart(ConnectedNode connectedNode)
	{
		// log = connectedNode.getLog();

		final Publisher<Float32> pub_motor = Topic.MOTOR_CMD.newPublisher(
				connectedNode, nameSpace);
		final Publisher<Float32> pub_vel = Topic.WHEEL_VEL.newPublisher(
				connectedNode, nameSpace);

		// This CancellableLoop will be canceled automatically when the node
		// shuts
		// down.
		connectedNode.executeCancellableLoop(new CancellableLoop()
		{

			@Override
			protected void setup()
			{
				ticks_since_target = timeout_ticks;
				wheel_prev = wheel_latest;
				then = System.currentTimeMillis();
				wheel_prev = wheel_latest;

			}

			@Override
			protected void loop() throws InterruptedException
			{

				spinOnce();
				Thread.sleep(rate);

			}

			private void spinOnce()
			{

				previous_error = 0.0f;
				prev_vel.clear();
				prev_vel.add(0.0d);
				prev_vel.add(0.0d);
				integral = 0.0f;
				error = 0;
				derivative = 0.0f;
				vel = 0.0f;

				while (ticks_since_target < timeout_ticks)
				{
					calcVelocity();
					doPid();
					Float32 mes = pub_motor.newMessage();
					mes.setData((float) motor);
					pub_motor.publish(mes);
					try
					{
						Thread.sleep(rate);
					} catch (InterruptedException e)
					{
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					ticks_since_target++;
					if (ticks_since_target == timeout_ticks)
					{
						Float32 mes2 = pub_motor.newMessage();
						mes2.setData(0);
						pub_motor.publish(mes2);
					}
				}

			}

			private void doPid()
			{
				long pid_dt_duration = System.currentTimeMillis()
						- prev_pid_time;
				double pid_dt = pid_dt_duration / 1000.0d;
				pid_dt = Math.max(1, pid_dt);
				prev_pid_time = System.currentTimeMillis();
				error = target - vel;
				integral = integral + (error * pid_dt);
				derivative = (error - previous_error) / pid_dt;
				previous_error = error;

				motor = (Kp * error) + (Ki * integral) + (Kd * derivative);
				if (motor > out_max)
				{
					motor = out_max;
					integral = integral - (error * pid_dt);
				}
				if (motor < out_min)
				{
					motor = out_min;
					integral = integral - (error * pid_dt);
				}
				if (target == 0)
				{
					motor = 0;
				}
			}

			private void calcVelocity()
			{
				dt = (System.currentTimeMillis() - then) / 1000;
				dt = Math.max(dt, 1);
				if (wheel_latest == wheel_prev)
				{
					double cur_vel = ((1.0d / ticks_per_meter) / dt);
					if (Math.abs(cur_vel) < vel_threshold)
					{
						appendVel(0);
						calcRollingVel();
					} else
					{
						if (Math.abs(cur_vel) < vel)
						{
							appendVel(cur_vel);
							calcRollingVel();
						}
					}
				} else
				{
					double cur_vel = (wheel_latest - wheel_prev) / dt;
					appendVel(cur_vel);
					calcRollingVel();
					wheel_prev = wheel_latest;
					then = System.currentTimeMillis();
				}
				Float32 msg = pub_vel.newMessage();
				msg.setData(vel);
				pub_vel.publish(msg);

			}

			private void calcRollingVel()
			{
				double size = prev_vel.size();
				double total = 0;
				for (Double val : prev_vel)
				{
					total += val;
				}
				vel = 0;
				if (size > 0)
				{
					vel = (float) (total / size);
				}

			}

			private void appendVel(double cur_vel)
			{
				prev_vel.add(cur_vel);
				prev_vel.remove(0);

			}
		});

		Subscriber<Int16> wsubscriber = Topic.WHEEL.newSubscriber(
				connectedNode, nameSpace);
		wsubscriber.addMessageListener(new MessageListener<Int16>()
		{
			@Override
			public void onNewMessage(Int16 message)
			{
				short enc = message.getData();
				if (enc < encoder_low_wrap && prev_encoder > encoder_high_wrap)
				{
					wheel_mult = wheel_mult + 1;
				}
				if (enc > encoder_high_wrap && prev_encoder < encoder_low_wrap)
				{
					wheel_mult = wheel_mult - 1;
				}
				wheel_latest = 1.0d
						* (enc + wheel_mult * (encoder_max - encoder_min))
						/ ticks_per_meter;
				prev_encoder = enc;

			}
		});

		Subscriber<Float32> wtsubscriber = Topic.WHEEL_VTARGET.newSubscriber(
				connectedNode, nameSpace);
		wtsubscriber.addMessageListener(new MessageListener<Float32>()
		{
			@Override
			public void onNewMessage(Float32 message)
			{
				target = message.getData();
				ticks_since_target = 0;

			}
		});

	}

	@Override
	public GraphName getDefaultNodeName()
	{
		throw new NotImplementedException();
	}
}