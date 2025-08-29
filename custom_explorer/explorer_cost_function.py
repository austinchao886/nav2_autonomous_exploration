import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np
from omegaconf import OmegaConf
from ament_index_python.packages import get_package_share_directory
import os

class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info("Explorer Node Started")

        # Load Config
        package_share = get_package_share_directory('custom_explorer')
        config_path = os.path.join(package_share, 'config', 'config.yaml')
        self.config = OmegaConf.load(config_path)

        # Subscriber to the map topic
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        # Action client for navigation
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Visited frontiers set
        self.visited_frontiers = set()

        # Map and position data
        self.map_data = None
        self.robot_position = (0, 0)  # Placeholder, update from localization

        # Timer for periodic exploration
        self.timer = self.create_timer(self.config.explore_rate, self.explore)
        
    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info("Map received")

    def navigate_to(self, x, y):
        """
        Send navigation goal to Nav2.
        """
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.w = 1.0  # Facing forward

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg

        self.get_logger().info(f"Navigating to goal: x={x}, y={y}")

        # Wait for the action server
        self.nav_to_pose_client.wait_for_server()

        # Send the goal and register a callback for the result
        send_goal_future = self.nav_to_pose_client.send_goal_async(nav_goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Handle the goal response and attach a callback to the result.
        """
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warning("Goal rejected!")
            return

        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_complete_callback)

    def navigation_complete_callback(self, future):
        """
        Callback to handle the result of the navigation action.
        """
        try:
            result = future.result().result
            self.get_logger().info(f"Navigation completed with result: {result}")
        except Exception as e:
            self.get_logger().error(f"Navigation failed: {e}")

    def find_frontiers(self, map_array):
        """
        Detect frontiers in the occupancy grid map.
        """
        frontiers = []
        rows, cols = map_array.shape

        # Iterate through each cell in the map
        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if map_array[r, c] == 0:  # Free cell
                    # Check if any neighbors are unknown
                    neighbors = map_array[r-1:r+2, c-1:c+2].flatten()
                    if -1 in neighbors:
                        frontiers.append((r, c))

        self.get_logger().info(f"Found {len(frontiers)} frontiers")
        return frontiers

    def choose_frontier(self, frontiers, map_array):
        """
        Choose a frontier based on a cost/utility function.
        Uses self.config.strategy, self.config.alpha, and self.config.beta directly.
        """
        robot_row, robot_col = self.robot_position
        best_score = -float('inf')
        chosen_frontier = None

        strategy = self.config.strategy 
        alpha = self.config.alpha 
        beta = self.config.beta 
        
        for frontier in frontiers:
            if frontier in self.visited_frontiers:
                continue

            # Distance to robot (Euclidean in grid coordinates)
            distance = np.sqrt((robot_row - frontier[0])**2 + (robot_col - frontier[1])**2)

            if distance < 1e-6:
                continue  # skip if robot is already at frontier

            # Information gain = number of unknown cells in local window
            r, c = frontier
            window = map_array[max(0, r-5):r+6, max(0, c-5):c+6]
            info_gain = np.sum(window == -1)

            # Cost function selection
            if strategy == "nearest":
                score = -distance  # smaller distance is better
            elif strategy == "ratio":
                score = info_gain / distance
            elif strategy == "weighted":
                score = alpha * info_gain - beta * distance
            else:
                self.get_logger().warning(f"Unknown strategy {strategy}, fallback to nearest")
                score = -distance

            if score > best_score:
                best_score = score
                chosen_frontier = frontier

        if chosen_frontier:
            self.visited_frontiers.add(chosen_frontier)
            self.get_logger().info(
                f"Chosen frontier {chosen_frontier} with score={best_score:.2f} using {strategy} strategy")
        else:
            self.get_logger().warning("No valid frontier found")

        return chosen_frontier
    

    def finish_if_map_complete(self, frontiers, threshold=None):
        """
        Finish exploration if the number of frontiers is below a threshold.
        """
        if threshold is None:
            threshold = getattr(self, 'threshold', 5)
        if len(frontiers) <= threshold:
            self.get_logger().info(f"Frontiers below threshold ({threshold}). Exploration complete!")
            # self.shutdown_robot()
            # Optionally, stop the timer to halt further exploration
            self.timer.cancel()
            return True
        return False

    def explore(self):
        if self.map_data is None:
            self.get_logger().warning("No map data available")
            return

        # Convert map to numpy array
        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width))

        # Detect frontiers
        frontiers = self.find_frontiers(map_array)

        # Finish if frontiers are below threshold
        if self.finish_if_map_complete(frontiers):
            return

        chosen_frontier = self.choose_frontier(
            frontiers,
            map_array
        )

        if not chosen_frontier:
            self.get_logger().warning("No frontiers to explore")
            return

        # Convert the chosen frontier to world coordinates
        goal_x = chosen_frontier[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x
        goal_y = chosen_frontier[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y

        # Navigate to the chosen frontier
        self.navigate_to(goal_x, goal_y)

def main(args=None):
    rclpy.init(args=args)
    explorer_node = ExplorerNode()

    try:
        explorer_node.get_logger().info("Starting exploration...")
        rclpy.spin(explorer_node)
    except KeyboardInterrupt:
        explorer_node.get_logger().info("Exploration stopped by user")
    finally:
        explorer_node.destroy_node()
        rclpy.shutdown()
