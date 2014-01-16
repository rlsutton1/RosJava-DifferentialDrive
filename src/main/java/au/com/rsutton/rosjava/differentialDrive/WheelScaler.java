package au.com.rsutton.rosjava.differentialDrive;

import org.apache.commons.lang.NotImplementedException;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import std_msgs.Int16;

/*

 Ported to java by Robert Sutton 2014

 wheel_scaler
 scales the wheel readings (and inverts the sign)

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
public class WheelScaler extends AbstractNodeMain
{

	float scale = 1;
	private String nameSpace;

	// private Log log;

	public WheelScaler(String nameSpace)
	{
		this.nameSpace = nameSpace;
	}

	@Override
	public void onStart(ConnectedNode connectedNode)
	{
		// log = connectedNode.getLog();

		// scaling for left wheel
		final Publisher<Int16> lwheel = Topic.LWHEEL_SCALED.newPublisher(
				connectedNode, nameSpace);
		final Subscriber<Int16> lWheelSubscriber = Topic.LWHEEL.newSubscriber(
				connectedNode, nameSpace);
		scalerProcessor(lwheel, lWheelSubscriber);

		//scaling for right wheel
		final Publisher<Int16> rwheel = Topic.RWHEEL_SCALED.newPublisher(
				connectedNode, nameSpace);
		final Subscriber<Int16> rWheelSubscriber = Topic.RWHEEL.newSubscriber(
				connectedNode, nameSpace);
		scalerProcessor(rwheel, rWheelSubscriber);

	}

	private void scalerProcessor(final Publisher<Int16> wheelPublisher,
			Subscriber<Int16> wheelSubscriber)
	{
		wheelSubscriber.addMessageListener(new MessageListener<Int16>()
		{
			@Override
			public void onNewMessage(Int16 message)
			{
				Int16 value = wheelPublisher.newMessage();
				System.out.println("Wheel scaler received "
						+ message.getData());
				value.setData((short) (message.getData() * scale));
				wheelPublisher.publish(value);

			}
		});
	}

	@Override
	public GraphName getDefaultNodeName()
	{
		throw new NotImplementedException();
	}
}