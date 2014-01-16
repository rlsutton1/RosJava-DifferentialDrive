package au.com.rsutton.rosjava.differentialDrive;

import geometry_msgs.TransformStamped;
import geometry_msgs.Twist;

import java.util.Collection;
import java.util.LinkedList;
import java.util.concurrent.BrokenBarrierException;
import java.util.concurrent.CyclicBarrier;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import junit.framework.TestCase;
import nav_msgs.Odometry;

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

public class DiffTfTest extends TestCase
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
		nodes.add(new DiffTf("testing"));
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
				final Subscriber<Odometry> odom = Topic.ODOM.newSubscriber(
						connectedNode, nameSpace);
				odom.addMessageListener(new MessageListener<Odometry>()
				{
					@Override
					public void onNewMessage(Odometry message)
					{
						System.out.println("odom test Recevied "
								+ message.getChildFrameId()
								+ " "
								+ message.getHeader().getFrameId()
								+ " "
								+ message.getPose().getPose().getPosition()
										.getX()
								+ " "
								+ message.getPose().getPose().getPosition()
										.getY());

					}
				});

				final Subscriber<TransformStamped> rwheel = Topic.TF
						.newSubscriber(connectedNode, nameSpace);
				rwheel.addMessageListener(new MessageListener<TransformStamped>()
				{
					@Override
					public void onNewMessage(TransformStamped message)
					{
						System.out.println("transform test Recevied "
								+ message.getChildFrameId()
								+ " "
								+ message.getHeader().getFrameId()
								+ " "
								+ message.getTransform().getTranslation()
										.getX()
								+ " "
								+ message.getTransform().getTranslation()
										.getY());

					}
				});

				final Publisher<Int16> wheel = Topic.RWHEEL.newPublisher(
						connectedNode, nameSpace);

				final Publisher<Int16> wheel_vtarget = Topic.LWHEEL
						.newPublisher(connectedNode, nameSpace);

				connectedNode.executeCancellableLoop(new CancellableLoop()
				{

					@Override
					protected void loop() throws InterruptedException
					{
						for (double i = -10000; i < 10000; i++)
						{
							Int16 message = wheel.newMessage();
							message.setData((short) i);
							wheel.publish(message);

							message = wheel_vtarget.newMessage();
							message.setData((short) i);
							wheel_vtarget.publish(message);

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
