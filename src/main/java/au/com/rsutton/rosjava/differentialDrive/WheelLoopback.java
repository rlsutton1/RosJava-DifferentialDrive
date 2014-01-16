package au.com.rsutton.rosjava.differentialDrive;

import org.apache.commons.lang.NotImplementedException;
import org.apache.commons.logging.Log;
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
 * Ported to java by Robert Sutton 2014
 * 
 #   Copyright 2012 Jon Stephan
 #   jfstepha@gmail.com
 #
 #   This program is free software: you can redistribute it and/or modify
 #   it under the terms of the GNU General Public License as published by
 #   the Free Software Foundation, either version 3 of the License, or
 #   (at your option) any later version.
 #
 #   This program is distributed in the hope that it will be useful,
 #   but WITHOUT ANY WARRANTY; without even the implied warranty of
 #   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 #   GNU General Public License for more details.
 #
 #   You should have received a copy of the GNU General Public License
 #   along with this program.  If not, see <http://www.gnu.org/licenses/>.

 */
public class WheelLoopback extends AbstractNodeMain
{

	private int rate;
	private double timeout_secs;
	private int ticks_meter;
	private float velocity_scale;
	private float latest_motor;
	private int wheel;
	private double secs_since_target;
	private long then;
	private long latest_msg_time;
	private Log log;
	private float velocity;
	private float seconds_per_tick;
	private long elapsed;
	private String nameSpace;

	public WheelLoopback(String nameSpace)
	{
		// initialize with sensible defaults
		this.nameSpace= nameSpace;

		rate = 200;
		timeout_secs = 0.5;
		ticks_meter = 50;
		velocity_scale = 255.0f;
		latest_motor = 0;
		wheel = 0;
	}

	@Override
	public void onStart(ConnectedNode connectedNode)
	{
		log = connectedNode.getLog();

		final Publisher<Int16> publisher = Topic.WHEEL.newPublisher(connectedNode, nameSpace);
		// This CancellableLoop will be canceled automatically when the node
		// shuts
		// down.
		connectedNode.executeCancellableLoop(new CancellableLoop()
		{

			@Override
			protected void setup()
			{
				// from spin method
				secs_since_target = timeout_secs;
				then = System.currentTimeMillis();
				latest_msg_time = System.currentTimeMillis();
				log.info("-D- spinning");
			}

			@Override
			protected void loop() throws InterruptedException
			{

				if (secs_since_target < timeout_secs)
				{
					spinOnce();

					secs_since_target = (System.currentTimeMillis() - latest_msg_time) / 1000;
				} else
				{
					secs_since_target = (System.currentTimeMillis() - latest_msg_time) / 1000;
					velocity = 0;
				}
				Thread.sleep(rate);

			}

			private void spinOnce()
			{
				velocity = latest_motor / velocity_scale;
				if (Math.abs(velocity) > 0)
				{
					seconds_per_tick = Math.abs(1 / (velocity * ticks_meter));
					elapsed = (System.currentTimeMillis() - then) / 1000;
					log.info("spinOnce: vel=" + velocity + " sec/tick = "
							+ seconds_per_tick + " elapsed=" + elapsed);
					if (elapsed > seconds_per_tick)
					{
						log.info("incrementing wheel");
						if (velocity > 0)
						{
							wheel++;
						} else
						{
							wheel--;
						}
						Int16 value = publisher.newMessage();
						value.setData((short) wheel);
						publisher.publish(value);

						then = System.currentTimeMillis();
					}
				}
			}
		});

		Subscriber<Float32> subscriber = Topic.MOTOR.newSubscriber(connectedNode, nameSpace);
		subscriber.addMessageListener(new MessageListener<Float32>()
		{
			@Override
			public void onNewMessage(Float32 message)
			{
				latest_motor = message.getData();
				latest_msg_time = System.currentTimeMillis();

			}
		});

	}

	@Override
	public GraphName getDefaultNodeName()
	{
		throw new NotImplementedException();
	}
}