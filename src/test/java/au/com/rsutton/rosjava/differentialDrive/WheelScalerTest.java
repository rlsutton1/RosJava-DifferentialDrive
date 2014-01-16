package au.com.rsutton.rosjava.differentialDrive;

import java.util.Collection;
import java.util.LinkedList;
import java.util.concurrent.BrokenBarrierException;
import java.util.concurrent.CyclicBarrier;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import org.junit.BeforeClass;
import org.junit.Test;
import org.ros.address.InetAddressFactory;
import org.ros.concurrent.CancellableLoop;
import org.ros.concurrent.DefaultScheduledExecutorService;
import org.ros.internal.node.topic.SubscriberIdentifier;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeFactory;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeListener;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.PublisherListener;
import org.ros.node.topic.Subscriber;

import std_msgs.Int16;
import junit.framework.TestCase;

public class WheelScalerTest extends TestCase
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
		nodes.add(new WheelScaler("testing"));
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
				final Subscriber<Int16> lwheelScaled = Topic.LWHEEL_SCALED
						.newSubscriber(connectedNode, nameSpace);
				lwheelScaled.addMessageListener(new MessageListener<Int16>()
				{
					@Override
					public void onNewMessage(Int16 message)
					{
						System.out.println("LWheel scaler test Recevied "
								+ message.getData());
						assertTrue("LWheel expected " + expectedLWheelScaled
								+ " got " + message.getData(),
								message.getData() == expectedLWheelScaled);
						try
						{
							cb.await(20, TimeUnit.SECONDS);
						} catch (InterruptedException | BrokenBarrierException
								| TimeoutException e)
						{
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
					}
				});

				final Subscriber<Int16> rwheelScaled = Topic.RWHEEL_SCALED
						.newSubscriber(connectedNode, nameSpace);
				rwheelScaled.addMessageListener(new MessageListener<Int16>()
				{

					@Override
					public void onNewMessage(Int16 message)
					{
						System.out.println("RWheel scaler test Recevied "
								+ message.getData());
						assertTrue("RWheel expected " + expectedRWheelScaled
								+ " got " + message.getData(),
								message.getData() == expectedRWheelScaled);
						try
						{
							cb.await(20, TimeUnit.SECONDS);
						} catch (InterruptedException | BrokenBarrierException
								| TimeoutException e)
						{
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
					}
				});

				final Publisher<Int16> lWheelRaw = Topic.LWHEEL.newPublisher(
						connectedNode, nameSpace);

				final Publisher<Int16> rWheelRaw = Topic.RWHEEL.newPublisher(
						connectedNode, nameSpace);

				connectedNode.executeCancellableLoop(new CancellableLoop()
				{

					@Override
					protected void loop() throws InterruptedException
					{
						for (int i = -1000; i < 1000; i++)
						{
							Int16 message = lWheelRaw.newMessage();
							expectedLWheelScaled = i;
							message.setData((short) i);
							lWheelRaw.publish(message);
							System.out.println("Wheel scaler test Send "
									+ message.getData());
							try
							{
								cb.await(20, TimeUnit.SECONDS);
							} catch (InterruptedException
									| BrokenBarrierException | TimeoutException e)
							{
								// TODO Auto-generated catch block
								e.printStackTrace();
								cb.reset();
							}
						}

						for (int i = -1000; i < 1000; i++)
						{
							Int16 message = rWheelRaw.newMessage();
							expectedRWheelScaled = i;
							message.setData((short) i);
							rWheelRaw.publish(message);
							System.out.println("RWheel scaler test Send "
									+ message.getData());
							try
							{
								cb.await(20, TimeUnit.SECONDS);
							} catch (InterruptedException
									| BrokenBarrierException | TimeoutException e)
							{
								// TODO Auto-generated catch block
								e.printStackTrace();
								cb.reset();
							}
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
