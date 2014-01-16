package au.com.rsutton.rosjava.differentialDrive;

import geometry_msgs.Twist;

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

/*

 Ported to java by Robert Sutton 2014

 twist_to_motors - converts a twist message to motor commands.  Needed for navigation stack


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
public class TwistTwoMotors extends AbstractNodeMain
{

	private double w;
	private int rate;
	private int timeout_ticks;
	private double left;
	private double right;
	private Log log;
	private int idle;
	// private long then;
	private int ticks_since_target;
	protected double dx;
	protected double dr;
	private String nameSpace;

	public TwistTwoMotors(String nameSpace)
	{
		// initialize with sensible defaults

		this.nameSpace = nameSpace;
		w = 0.2; // base width
		rate = 50;
		timeout_ticks = 2;
		left = 0;
		right = 0;
	}

	@Override
	public void onStart(ConnectedNode connectedNode)
	{
		log = connectedNode.getLog();

		final Publisher<Float32> pub_lmotor = Topic.LWHEEL_VTARGET.newPublisher(connectedNode, nameSpace);
		final Publisher<Float32> pub_rmotor = Topic.RWHEEL_VTARGET.newPublisher(connectedNode, nameSpace);

		// This CancellableLoop will be canceled automatically when the node
		// shuts
		// down.
		connectedNode.executeCancellableLoop(new CancellableLoop()
		{

			@Override
			protected void setup()
			{
				// from spin method
				idle = 10; // rate
				// then = System.currentTimeMillis();
				ticks_since_target = timeout_ticks;
			}

			@Override
			protected void loop() throws InterruptedException
			{

				if (ticks_since_target < timeout_ticks)
				{
					spinOnce();
					Thread.sleep(rate);
				}
				Thread.sleep(idle);

			}

			private void spinOnce()
			{
				right = (1.0f * dx + dr * w / 2.0f);
				left = (1.0f * dx - dr * w / 2.0f);
				Float32 value = pub_lmotor.newMessage();
				value.setData((float) left);
				pub_lmotor.publish(value);

				value = pub_rmotor.newMessage();
				value.setData((float) right);
				pub_rmotor.publish(value);

				ticks_since_target++;

			}
		});

		Subscriber<Twist> subscriber = Topic.TWIST.newSubscriber(connectedNode, nameSpace);
		subscriber.addMessageListener(new MessageListener<Twist>()
		{
			@Override
			public void onNewMessage(Twist message)
			{
				ticks_since_target = 0;
				dx = message.getLinear().getX();
				dr = message.getAngular().getZ();
				// dy = message.getLinear().getY();
			}
		});

	}

	@Override
	public GraphName getDefaultNodeName()
	{
		throw new NotImplementedException();
	}
}