package au.com.rsutton.rosjava.differentialDrive;

import geometry_msgs.Twist;

import java.util.Collection;
import java.util.LinkedList;
import java.util.concurrent.BrokenBarrierException;
import java.util.concurrent.CyclicBarrier;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import junit.framework.TestCase;

import org.junit.Test;
import org.ros.address.InetAddressFactory;
import org.ros.concurrent.CancellableLoop;
import org.ros.concurrent.DefaultScheduledExecutorService;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeFactory;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeListener;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import std_msgs.Float32;
import std_msgs.Int16;

public class PidVelocityTest extends TestCase
{

	volatile double expectedLWheelScaled;
	volatile double expectedRWheelScaled;

	CyclicBarrier cb = new CyclicBarrier(2);

	CyclicBarrier testBarrier = new CyclicBarrier(2);

	private void createNode(Collection<NodeListener> nodes)
			throws InterruptedException, BrokenBarrierException
	{
		String host = InetAddressFactory.newNonLoopback().getHostName();
		NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(host);
		DefaultNodeFactory factory = new DefaultNodeFactory(
				new DefaultScheduledExecutorService());

		nodeConfiguration.setNodeName(GraphName.of("randomName"));
		factory.newNode(nodeConfiguration, nodes);

	}

	@Test
	public void test1() throws InterruptedException, BrokenBarrierException
	{
		Collection<NodeListener> nodes = new LinkedList<NodeListener>();
		nodes.add(new PidVelocity("testing"));
		nodes.add(setupTestNode("testing"));
		createNode(nodes);

		testBarrier.await();
		Thread.sleep(500000);
	}

	NodeListener setupTestNode(final String nameSpace)
	{
		return new NodeListener()
		{

			@Override
			public void onStart(ConnectedNode connectedNode)
			{
				final Subscriber<Float32> motorCmd = Topic.MOTOR_CMD
						.newSubscriber(connectedNode, nameSpace);
				motorCmd.addMessageListener(new MessageListener<Float32>()
				{
					@Override
					public void onNewMessage(Float32 message)
					{
						System.out.println("MotorCMD test Recevied "
								+ message.getData());

					}
				});

				final Subscriber<Float32> rwheel = Topic.WHEEL_VEL
						.newSubscriber(connectedNode, nameSpace);
				rwheel.addMessageListener(new MessageListener<Float32>()
				{
					@Override
					public void onNewMessage(Float32 message)
					{
						System.out.println("WHEEL_VEL test Recevied "
								+ message.getData());

					}
				});

				final Publisher<Int16> wheel = Topic.WHEEL.newPublisher(
						connectedNode, nameSpace);

				final Publisher<Float32> wheel_vtarget = Topic.WHEEL_VTARGET.newPublisher(
						connectedNode, nameSpace);

				connectedNode.executeCancellableLoop(new CancellableLoop()
				{

					@Override
					protected void loop() throws InterruptedException
					{
						for (double i = -10000; i < 10000; i++)
						{
							Int16 message = wheel.newMessage();
							message.setData((short) 00);
							wheel.publish(message);
							
							Float32 wvt = wheel_vtarget.newMessage();
							wvt.setData((float) (i/100d));
							wheel_vtarget.publish(wvt);
							
							Thread.sleep(1000);
						}

						try
						{
							testBarrier.await();
						} catch (InterruptedException | BrokenBarrierException e)
						{
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
					}
				});

			}

			@Override
			public void onShutdown(Node node)
			{
				// TODO Auto-generated method stub

			}

			@Override
			public void onShutdownComplete(Node node)
			{
				// TODO Auto-generated method stub

			}

			@Override
			public void onError(Node node, Throwable throwable)
			{
				// TODO Auto-generated method stub

			}
		};
	}
}
