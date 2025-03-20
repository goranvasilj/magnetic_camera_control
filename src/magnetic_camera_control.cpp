#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

tf::TransformListener* transform_listener;
// Callback functions for subscribers

double QuaternionToHeading(double qw, double qx, double qy, double qz) {
	double siny = +2.0 * (qw * qz + qy * qx);
	double cosy = +1.0 - 2.0 * (qx * qx + qz * qz);
	double heading1 = atan2(siny, cosy);      // in radians
	return heading1;
}

void get_powerline_pose(double &x, double &y, double &z, double &heading) {
	tf::StampedTransform transform;
	ros::Time t = ros::Time(0);
	try {
		transform_listener->lookupTransform("/power_line0","/magnetometer_center"
				, t, transform);

	} catch (tf::TransformException ex) {
		ROS_ERROR("%s", ex.what());
		return;
	}
	double qw = transform.getRotation().getW();
	double qx = transform.getRotation().getX();
	double qy = transform.getRotation().getY();
	double qz = transform.getRotation().getZ();

	double heading1 = QuaternionToHeading(qw, qx, qy, qz);
	heading = heading1;
	heading = heading + 3.14159/2;
	if (heading>3.14159/2)
	{
		heading=heading-3.14159;
	}
	x = transform.getOrigin().x();
	y = transform.getOrigin().y();
	z = transform.getOrigin().z();

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "magnetic_camera_control");
	ros::NodeHandle nh;
	ros::NodeHandle nh_ns("~");
	transform_listener=new tf::TransformListener();

	std::string port, topic, cmd_roll, cmd_pitch, cmd_yaw, cmd_zoom;

	nh_ns.param("cmd_roll", cmd_roll, (std::string) "/camera_cmd_roll");
	nh_ns.param("cmd_pitch", cmd_pitch, (std::string) "/camera_cmd_pitch");
	nh_ns.param("cmd_yaw", cmd_yaw, (std::string) "/camera_cmd_yaw");
	nh_ns.param("cmd_zoom", cmd_zoom, (std::string) "/camera_cmd_zoom");

	// Create publishers
	ros::Publisher pub_yaw = nh.advertise < std_msgs::Float32 > (cmd_yaw, 10);
	ros::Publisher pub_pitch = nh.advertise < std_msgs::Float32 > (cmd_pitch, 10);

	ros::Rate loop_rate(2); // 10 Hz

	while (ros::ok()) {
		// Prepare and publish messages
		std_msgs::Float32 msg1, msg2;
		msg1.data = 1.0; // Example value
		msg2.data = 2.0; // Example value

		double x, y, z, yaw;
		get_powerline_pose(x, y, z, yaw);
		double yaw_command = -yaw;
		double pitch_command = atan(z / sqrt(x * x + y * y));
		std::cout<<"yaw pitch "<<-yaw_command/3.14159*180+180<<" "<<pitch_command/3.14159*180<<std::endl;

		std_msgs::Float32 yaw_message;
		std_msgs::Float32 pitch_message;
		yaw_message.data=-yaw_command/3.14159*180+180;
		pitch_message.data=-pitch_command/3.14159*180;

		pub_yaw.publish(yaw_message);
		ros::spinOnce();
		loop_rate.sleep();

		//pub_pitch.publish(pitch_message);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

