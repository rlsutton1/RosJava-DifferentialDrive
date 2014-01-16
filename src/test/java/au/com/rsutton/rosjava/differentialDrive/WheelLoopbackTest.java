package au.com.rsutton.rosjava.differentialDrive;

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

public class WheelLoopbackTest extends TestCase
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
		nodes.add(new WheelLoopback("testing"));
		nodes.add(setupTestNode("testing"));
		createNode(nodes);

		testBarrier.await();
	}

	NodeListener setupTestNode(final String nameSpace)
	{
		return new NodeListener()
		{

			@Override
			public void onStart(ConnectedNode connectedNode)
			{
				final Subscriber<Int16> wheel = Topic.WHEEL.newSubscriber(
						connectedNode, nameSpace);
				wheel.addMessageListener(new MessageListener<Int16>()
				{
					@Override
					public void onNewMessage(Int16 message)
					{
						System.out.println("Wheel Loopback test Recevied "
								+ message.getData());
						
					}
				});

				final Publisher<Float32> motor = Topic.MOTOR.newPublisher(
						connectedNode, nameSpace);

				connectedNode.executeCancellableLoop(new CancellableLoop()
				{

					@Override
					protected void loop() throws InterruptedException
					{
						for (int i = -1000; i < 1000; i++)
						{
							Float32 message = motor.newMessage();
							
							message.setData((short) 5);
							motor.publish(message);
							System.out.println("Wheel scaler test Send "
									+ message.getData());
							Thread.sleep(100);
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
