//Naveen Chandra R
//160432
//naveencr@iitk.ac.in

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

// These are conventions followed by costmap:
// cost = 100 => direct obstacle
// cost = 99 => obstacle within inscribed radius
#define COLLISION_THRESHOLD	99

class Node 	// a state (x, y, psi)
{
public:
	int x = 0;				// x-coordinate of current cell
	int y = 0;				// y-coordinate of current cell
	double psi = 0.0;		// in radians

	int parent_idx = -1;	// pointer index for parent node within the tree
};

// RRT utility functions
inline bool angleCheck(const Node& parent, const Node& sample, const double max_angle);
inline bool checkConstraint(const Node& current, const nav_msgs::OccupancyGrid& cfg_space);
bool localPlanner(const Node& parent, const Node& sample, const nav_msgs::OccupancyGrid& cfg_space);
Node randSample(const nav_msgs::OccupancyGrid& cfg_space);

// Callback functions
void cfgSpaceCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_msg,
	nav_msgs::OccupancyGrid& cfg_space, bool& cfg_space_received);

// Common math functions
double angWrap(const double x);
inline double distance(const Node& n1, const Node& n2);

int main(int argc, char **argv)
{
	// Initializing node
	ros::init(argc, argv, "rrt_planner");
	ros::NodeHandle nh;
	ros::Time t;


	// Getting parameters
	int max_nodes = 5000, max_iterations = 1000000;
	double delta_step = 5.0, max_angle = 50.0, initial_yaw = 180.0;
	std::vector<double> initial_position (0, 0);
	std::vector<double> goal_position (2, 2);
	double checking_freq = 10.0, publishing_freq = 0.5;
	nh.param("max_nodes", max_nodes, 10000);				// the maximum number of nodes to include in the tree
	nh.param("max_iterations", max_iterations, 1000000);	// the maximum number of iterations to run before halting
	nh.param("delta_step", delta_step, (double)5);			// the maximum allowable size of a step
	nh.param("max_angle", max_angle, (double)50.0);			// maximum allowable turning angle between consecutive steps
	max_angle = max_angle*M_PI/180.0;
	nh.getParam("initial_position", initial_position);		// initial co-ordinates (x, y) in the cfg_space
	nh.param("initial_yaw", initial_yaw, (double)0.0);		// initial yaw in degrees
	initial_yaw = initial_yaw*M_PI/180.0;
	nh.getParam("goal_position", goal_position);			// goal co-ordinates (x, y) in the cfg_space

	nav_msgs::OccupancyGrid cfg_space;
	bool cfg_space_received = false;

	// Creating subscriber and publisher objects
	ros::Subscriber sub_cfg = nh.subscribe<nav_msgs::OccupancyGrid>("/cfg_space", 1,
	boost::bind(cfgSpaceCallback, _1, boost::ref(cfg_space), boost::ref(cfg_space_received)));

	ros::Publisher pub_nodes = nh.advertise<nav_msgs::OccupancyGrid>("/nodes", 1);	// for vizualization
	ros::Publisher pub_path = nh.advertise<nav_msgs::Path>("/path", 1);

	// std::vector<Node> tree;

	ros::Rate publishing_rate(0.2);							// updating every 5 seconds
	while (ros::ok())
	{
		// Getting configuration space
		ros::Rate checking_rate(10);
		while (ros::ok() && !cfg_space_received)
		{
			ros::spinOnce();
			checking_rate.sleep();
		}
		ROS_INFO_STREAM("Configuration space received.");

		// Nodes for initial and goal positions (converted from metres to cells)
		Node root;
		root.x = (initial_position[0] - cfg_space.info.origin.position.x)/cfg_space.info.resolution;
		root.y = (initial_position[1] - cfg_space.info.origin.position.y)/cfg_space.info.resolution;
		root.psi = initial_yaw;

		Node goal;
		goal.x = (goal_position[0] - cfg_space.info.origin.position.x)/cfg_space.info.resolution;
		goal.y = (goal_position[1] - cfg_space.info.origin.position.y)/cfg_space.info.resolution;

		//////////
		// TODO //
		//////////

		// In this part of the code, you need to build the tree as per the RRT
		// algorithm given in the assignment. Here, you'll be using all the
		// functions from this file in order to run the main RRT loop. Some part of
		// the code has been already given. You have to fill in the rest.

		ROS_INFO_STREAM("Initializing RRT...");
		srand(time(NULL));	// seeding the rand() function
		
		std::vector<Node> tree;
		tree.push_back(root);

		int num_iterations = 0;
		Node sample;
		while (tree.size() < max_nodes && num_iterations < max_iterations)
		{
			// Generating a random sample in freespace (only x & y are set)
			sample = randSample(cfg_space);

			// Find the nearest neighbour of the sample from the tree
			double curr_dist, min_distance;
			int min_index;
			min_distance = std::numeric_limits<double>::infinity();

			for (int i = 0; i < tree.size(); i++)
			{
				curr_dist = distance(tree[i], sample);
				if (min_distance > curr_dist)
				{
					min_distance = curr_dist;
					min_index = i;
				}

			}
			// std::cout << "tree: " << sample.parent_idx << std::endl;
			// std::cout << "min: " << min_index << std::endl;
			// checking for log errors

			// Set this nearest neighbour as the parent of the sample
       		sample.parent_idx = min_index;
			// Running some checks in increasing order of computational complexity
			// bool all_passed = false;
			// Check 1: Does the sample co-incide with its parent?
			if (min_distance > 0)
			{
				// Updating psi based on heading direction from parent to sample
				sample.psi = std::atan2(sample.y - tree[sample.parent_idx].y, sample.x - tree[sample.parent_idx].x);

				// Check 2: Is the steer angle within the acceptable range?
				if (angleCheck(tree[sample.parent_idx], sample, max_angle))
				{
					// Check 3: Is the path from parent to sample free of obstacles?
					if (localPlanner(tree[sample.parent_idx], sample, cfg_space))
					{
						// Truncate the edge length to delta_step if it is larger than that
						// (update the location of the sample accordingly)
						if (min_distance > delta_step)
						{
							sample.x = tree[sample.parent_idx].x +	(int)(((double)(sample.x - tree[sample.parent_idx].x))*(delta_step/min_distance));
							sample.y = tree[sample.parent_idx].y +	(int)(((double)(sample.y - tree[sample.parent_idx].y))*(delta_step/min_distance));

						}
						// std::cout << "sample: " << sample.x << " " << sample.y << " " << sample.parent_idx << " " << sample.psi << std::endl;
						// checking for log errors

						// Adding node to tree
						tree.push_back(sample);
					}
				}
			}

			num_iterations++;
			ROS_INFO_STREAM_THROTTLE(0.5, num_iterations<<" iterations completed.");
		}

		/////////////////////
		// End of RRT Loop //
		/////////////////////

		// Makeshift method for visualizing all nodes in the tree
		nav_msgs::OccupancyGrid rrt_nodes;
		rrt_nodes.header = cfg_space.header;
		rrt_nodes.info = cfg_space.info;
		rrt_nodes.data.resize(cfg_space.data.size());
		for (int i = 0; i < rrt_nodes.data.size(); i++)
			rrt_nodes.data[i] = -1;
		// costmap gives colours from (1->98)
		for (int i = 0; i < tree.size(); i++)
		{
			rrt_nodes.data[tree[i].y*rrt_nodes.info.width + tree[i].x] = 50;
		}
		pub_nodes.publish(rrt_nodes);


		ROS_INFO_STREAM("Selecting best path...");
		// Choosing best path
		double curr_dist, min_distance;
		int min_index;
		min_distance = std::numeric_limits<double>::infinity();
		for (int i = 0; i < tree.size(); i++)
		{
			curr_dist = distance(tree[i], goal);
			if (min_distance > curr_dist)
			{
				min_distance = curr_dist;
				min_index = i;
			}
		}

		Node closest_node = tree[min_index];

		// Tracing the path back to the initial position
		std::vector<Node> path_rev;
		path_rev.push_back(closest_node);
		while (path_rev.back().parent_idx != -1)
		{
			path_rev.push_back(tree[path_rev.back().parent_idx]);
		}

		// Reversing the path, transforming it back into real-world units & into the map frame before publishing
		geometry_msgs::PoseStamped current_pose;
		nav_msgs::Path path_msg;
		path_msg.header = cfg_space.header;
		current_pose.header = cfg_space.header;
		for (int i = path_rev.size()-1; i >= 0; i--)
		{
			current_pose.pose.position.x = ((double)path_rev[i].x)*cfg_space.info.resolution + cfg_space.info.origin.position.x;
			current_pose.pose.position.y = ((double)path_rev[i].y)*cfg_space.info.resolution + cfg_space.info.origin.position.y;
			current_pose.pose.orientation.z = std::sin(path_rev[i].psi/2);
			current_pose.pose.orientation.w = std::cos(path_rev[i].psi/2);
			path_msg.poses.push_back(current_pose);
		}

		pub_path.publish(path_msg);

		cfg_space_received = false;
		publishing_rate.sleep();
	}
	// Exiting Node
	ROS_INFO_STREAM("Shutting down node...");
	ros::shutdown();
	return 0;
}

///////////////////////////
// RRT utility functions //
///////////////////////////

inline bool angleCheck(const Node& parent, const Node& sample, const double max_angle)
{
	// Returns true if turning angle is within bounds
	return (std::abs(angWrap(sample.psi - parent.psi)) <= max_angle);
}

inline bool checkConstraint(const Node& current, const nav_msgs::OccupancyGrid& cfg_space)
{
	// Returns true if the current state is not a state of collision
	return (cfg_space.data[current.y*cfg_space.info.width + current.x] < COLLISION_THRESHOLD &&
			cfg_space.data[current.y*cfg_space.info.width + current.x] >= 0);
}

bool localPlanner(const Node& parent, const Node& sample, const nav_msgs::OccupancyGrid& cfg_space)
{
	//////////
	// TOOD //
	//////////

	// Given two nodes (and the occupancy grid for the configuration space), you
	// need to return a single Boolean value representing whether  the straight
	// line joining them is collision-free. To do this, you  need to check every
	// cell in the occupancy grid that lies along this line. If all of them are
	// collision-free, then the path joining the two nodes is also
	// collision-free, and you should return true (otherwise false).

	// Coding tip: The simplest way to do this is to use two variables x and y
	// and then move from the parent node to the current sample in increments of
	// delta_x and delta_y (such that their norm is 1 cell). Note that while x
	// and y will be floating point values, your current location must be
	// typecasted to integers while checking the possibility of collision at a
	// particular cell using checkConstraint

	// Initial State
	Node current = parent;

	//loop for the local planner

	int num_steps = (int)(distance(parent,sample)/5); // Dividing by step size to get integer
	double current_x = current.x, current_y = current.y;
	double delta_x = std::cos(current.psi), delta_y = std::sin(current.psi);
	for (int i = 0; i < num_steps; i++)
	{
		current_x = current_x + delta_x;	current_y = current_y + delta_y;
		current.x = (int)current_x;		current.y = (int)current_y;
		if (!checkConstraint(current, cfg_space))
			return false;
	}

	return true;
}

Node randSample(const nav_msgs::OccupancyGrid& cfg_space)
{
	// Gives random node in freespace=
	int map_width = cfg_space.info.width;
	int map_height = cfg_space.info.height;
	Node sample;
	while(true)
	{
		sample.x = rand()%map_width;
		sample.y = rand()%map_height;
		if (cfg_space.data[sample.y*map_width + sample.x] < COLLISION_THRESHOLD &&
		 	cfg_space.data[sample.y*map_width + sample.x] >= 0)
			break;
	}

	return sample;
}

////////////////////////
// Callback functions //
////////////////////////

void cfgSpaceCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_msg,
	nav_msgs::OccupancyGrid& cfg_space, bool& cfg_space_received)
{
	cfg_space.header = occupancy_msg->header;
	cfg_space.info = occupancy_msg->info;
	cfg_space.data = occupancy_msg->data;

	cfg_space_received = true;
}

///////////////////////////
// Common math functions //
///////////////////////////

double angWrap(double x)
{
	// Wraps an angle to a value between (-PI, PI]
	while (x <= -M_PI)
		x = x + 2*M_PI;
	while (x > M_PI)
		x = x - 2*M_PI;
	return x;
}

inline double distance(const Node& n1, const Node& n2)
{
	// Gives the eucledian distance between nodes n1 and n2
	return std::sqrt((double)((n2.x-n1.x)*(n2.x-n1.x) + (n2.y-n1.y)*(n2.y-n1.y)));
}
