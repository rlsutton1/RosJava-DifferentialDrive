package au.com.rsutton.rosjava.differentialDrive;

import geometry_msgs.TransformStamped;
import geometry_msgs.Twist;
import nav_msgs.Odometry;

import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import std_msgs.Float32;
import std_msgs.Int16;

public enum Topic
{
	LWHEEL_SCALED("lwheel_scaled", Int16._TYPE), RWHEEL_SCALED("rwheel_scaled",
			Int16._TYPE), LWHEEL("lwheel", Int16._TYPE), RWHEEL("rwheel",
			Int16._TYPE), MOTOR("motor", Float32._TYPE), WHEEL("wheel",
			Int16._TYPE), LWHEEL_VTARGET("lwheel_vtarget", Float32._TYPE), RWHEEL_VTARGET(
			"rwheel_vtarget", Float32._TYPE), TWIST("twist", Twist._TYPE), WHEEL_VTARGET(
			"wheel_vtarget", Float32._TYPE), MOTOR_CMD("motor_cmd",
			Float32._TYPE), WHEEL_VEL("wheel_vel", Float32._TYPE), ODOM("odom",
			Odometry._TYPE), TF("tf",TransformStamped._TYPE);

	private String topicName;
	private String type;

	Topic(String topicName, String type)
	{
		this.topicName = topicName;
		this.type = type;

	}

	<T> Publisher<T> newPublisher(ConnectedNode connectedNode, String nameSpace)
	{
		return connectedNode.newPublisher(this.getGraphName(nameSpace), type);
	}

	private GraphName getGraphName(String nameSpace)
	{

		return GraphName.of(nameSpace + "/" + topicName);
	}

	<T> Subscriber<T> newSubscriber(ConnectedNode connectedNode,
			String nameSpace)
	{
		return connectedNode.newSubscriber(this.getGraphName(nameSpace), type);
	}
}
