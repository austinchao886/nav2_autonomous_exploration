import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np
from omegaconf import OmegaConf
from ament_index_python.packages import get_package_share_directory
import os
from collections import deque
from visualization_msgs.msg import Marker, MarkerArray
import geometry_msgs.msg
class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info("Explorer Node Started")

        # Load Config
        package_share = get_package_share_directory('custom_explorer')
        config_path = os.path.join(package_share, 'config', 'config.yaml')
        self.config = OmegaConf.load(config_path)
        
        # Frontier clustering & filtering parameters
        self.min_cluster_size = 3  # Minimum size of frontier cluster to consider
        self.obstacle_ratio_thresh = 0.5  
        self.rho_threshold = 0.2      
        self.patch_radius = 3         

        self.current_target = None     

        # Subscriber to the map topic
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        
        # Subscriber to the odometry topic (if needed for robot position)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.path_publisher = self.create_publisher(
            Path, '/exploration_path', 10)
        
        self.exploration_path = Path()
        self.exploration_path.header.frame_id = 'odom'
        
        self.prev_odom = None
        self.total_distance = 0.0

        # Publisher for frontier visualization
        self.frontier_pub = self.create_publisher(
            MarkerArray,
            'frontier_markers',
            10
        )

        # Action client for navigation
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # Visited frontiers set
        self.visited_frontiers = set()

        # Map and position data
        self.map_data = None
        self.robot_position = (0, 0)  # Placeholder, update from localization

        # Timer for periodic exploration
        self.timer = self.create_timer(self.config.explore_rate, self.explore)

        # Start the exploration timer
        self.start_time = self.get_clock().now()

        # Get Distance and Time Information
        self.print_information_timer = self.create_timer(5.0, self.print_information)

    def world_to_grid(self, x, y):
        res = self.map_data.info.resolution
        ox  = self.map_data.info.origin.position.x
        oy  = self.map_data.info.origin.position.y
        c = int((x - ox) / res)
        r = int((y - oy) / res)
        return (r, c)
    
    def grid_to_world(self, r, c):
        """Convert grid index (row, col) -> world (x, y) in map frame."""
        res = self.map_data.info.resolution
        ox  = self.map_data.info.origin.position.x
        oy  = self.map_data.info.origin.position.y
        x = c * res + ox
        y = r * res + oy
        return x, y


    def odom_callback(self, msg): # msg is respect to world frame
        if self.prev_odom is None:
            self.prev_odom = msg
            return
        
        # Update robot position
        distance = np.sqrt(
            (msg.pose.pose.position.x - self.prev_odom.pose.pose.position.x) ** 2 +
            (msg.pose.pose.position.y - self.prev_odom.pose.pose.position.y) ** 2
        )

        # Draw Path
        path_pose = PoseStamped()
        path_pose.header.frame_id = 'odom'
        path_pose.header.stamp = msg.header.stamp
        path_pose.pose = msg.pose.pose
        self.exploration_path.poses.append(path_pose)
        self.path_publisher.publish(self.exploration_path)

        self.total_distance += distance
        self.prev_odom = msg

        if self.current_target and self.map_data is not None:
            fr = self.current_target  # (row, col)
            tx, ty = self.grid_to_world(fr[0], fr[1])  # 注意順序 r=row, c=col
            dist = np.hypot(
                msg.pose.pose.position.x - tx,
                msg.pose.pose.position.y - ty
            )
            if dist < 0.3:
                self.visited_frontiers.add(fr)
                self.current_target = None

        if self.map_data is None:
            return
        
        self.robot_position = self.world_to_grid(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
    
    def print_information(self):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.get_logger().info(
            f"Elapsed Time: {elapsed_time:.2f} seconds, Total Distance Travelled: {self.total_distance:.2f} meters"
        )

    def publish_frontier_clusters(self, clusters, centroids):
        marker_array = MarkerArray()
        marker_id = 0

        # Show frontier clusters
        for cluster in clusters:
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "cluster_cells"
            m.id = marker_id
            marker_id += 1

            m.type = Marker.SPHERE_LIST
            m.action = Marker.ADD

            m.scale.x = 0.05   
            m.scale.y = 0.05
            m.scale.z = 0.05

            m.color.r = 0.2
            m.color.g = 0.8
            m.color.b = 1.0
            m.color.a = 0.8

            for (r, c) in cluster:
                p = geometry_msgs.msg.Point()
                p.x = c * self.map_data.info.resolution + self.map_data.info.origin.position.x
                p.y = r * self.map_data.info.resolution + self.map_data.info.origin.position.y
                p.z = 0.05
                m.points.append(p)

            marker_array.markers.append(m)

        # Show cluster centroid 
        for (r, c) in centroids:
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "cluster_centroid"
            m.id = marker_id
            marker_id += 1

            m.type = Marker.SPHERE
            m.action = Marker.ADD

            m.scale.x = 0.15
            m.scale.y = 0.15
            m.scale.z = 0.15

            m.color.r = 1.0
            m.color.g = 0.3
            m.color.b = 0.2
            m.color.a = 1.0

            m.pose.position.x = c * self.map_data.info.resolution + self.map_data.info.origin.position.x
            m.pose.position.y = r * self.map_data.info.resolution + self.map_data.info.origin.position.y
            m.pose.position.z = 0.1

            marker_array.markers.append(m)

        self.frontier_pub.publish(marker_array)


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
            self.get_logger().info(
                f"Navigation completed with result: {result}")
        except Exception as e:
            self.get_logger().error(f"Navigation failed: {e}")

    def cluster_frontiers(self, candidate_mask: np.ndarray):
        rows, cols = candidate_mask.shape
        visited = np.zeros_like(candidate_mask, dtype=bool)
        clusters = []

        neighbor_offsets = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1),
        ]       

        for r in range(rows):
            for c in range(cols):
                if candidate_mask[r,c] != 1 or visited[r,c]:
                    continue
                # Start a new cluster
                cluster = []
                queue = deque()
                queue.append((r,c))
                visited[r,c] = True

                while queue:
                    cr, cc = queue.popleft()
                    cluster.append((cr,cc))
                    for dr, dc in neighbor_offsets:
                        nr, nc = cr + dr, cc + dc
                        # Frontier bounds check
                        if nr < 0 or nr >= rows or nc < 0 or nc >= cols:
                            continue
                        # only add unvisited frontier cells
                        if candidate_mask[nr, nc] == 1 and not visited[nr, nc]:
                            visited[nr, nc] = True
                            queue.append((nr, nc))
                
                clusters.append(cluster)
        self.get_logger().info(f"[cluster_frontiers] Found {len(clusters)} clusters.")
        return clusters

    def is_obstacle_region(self, map_array, r, c):
        """Check whether this frontier centroid is too close to obstacles."""
        pr = self.patch_radius
        rows, cols = map_array.shape

        r0 = max(0, r - pr)
        r1 = min(rows, r + pr + 1)
        c0 = max(0, c - pr)
        c1 = min(cols, c + pr + 1)

        patch = map_array[r0:r1, c0:c1]

        total = patch.size
        obstacles = np.sum(patch == 100)

        obstacle_ratio = obstacles / total

        return obstacle_ratio > self.obstacle_ratio_thresh
    
    def is_valid_boundary(self, map_array, r, c):
        """Check boundary quality (ρ filter)."""
        pr = self.patch_radius
        rows, cols = map_array.shape

        r0 = max(0, r - pr)
        r1 = min(rows, r + pr + 1)
        c0 = max(0, c - pr)
        c1 = min(cols, c + pr + 1)

        patch = map_array[r0:r1, c0:c1]

        total = patch.size
        unknowns = np.sum(patch == -1)

        unknown_ratio = unknowns / total

        # ρ = 1 - 2 * | unknown_ratio - 0.5 |
        rho = 1 - 2 * abs(unknown_ratio - 0.5)

        return rho >= self.rho_threshold

    def find_frontiers(self, map_array):
        """
        Detect frontiers in the occupancy grid map.
        """
        #### Test ####
        self.get_logger().info(f'[find_frontiers] Map shape: {map_array.shape}, Free cells: {np.sum(map_array==0)}, Unknown cells: {np.sum(map_array==-1)}')
        frontiers = []
        rows, cols = map_array.shape

        candidate_mask = np.zeros_like(map_array, dtype=np.uint8)

        # Iterate through each cell in the map
        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if map_array[r, c] == 0:  # Free cell
                    # Check if any neighbors are unknown
                    neighbors = map_array[r-1:r+2, c-1:c+2].flatten()
                    if -1 in neighbors:
                        candidate_mask[r, c] = 1

        # Extract frontier points from candidate mask
        clusters = self.cluster_frontiers(candidate_mask)

        for cluster in clusters:
            # Filter small clusters
            if len(cluster) < self.min_cluster_size:
                self.get_logger().info(f"cluster size={len(cluster)} -> dropped (too small)")
                continue

            # Compute the centroid of the cluster
            if len(cluster) == 0:
                continue

            centroid_r = int(np.mean([pt[0] for pt in cluster]))
            centroid_c = int(np.mean([pt[1] for pt in cluster]))

            # Obstacle filter
            if self.is_obstacle_region(map_array, centroid_r, centroid_c):
                self.get_logger().info(f"cluster at ({centroid_r},{centroid_c}) -> dropped (obstacle)")
                continue

            # Boundary rho filter
            if not self.is_valid_boundary(map_array, centroid_r, centroid_c):
                self.get_logger().info(f"cluster at ({centroid_r},{centroid_c}) -> dropped (rho)")
                continue
            
            frontiers.append((centroid_r, centroid_c))

        self.get_logger().info(
            f"[find_frontiers]Candidate cells: {int(candidate_mask.sum())}, "
            f"clusters found: {len(clusters)}, centroids={len(frontiers)}"
        )
        return clusters, frontiers

    def choose_frontier(self, clusters, frontiers, map_array):
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
            # distance = np.sqrt(
            #     (robot_row - frontier[0])**2 + (robot_col - frontier[1])**2)
            distance_grid = np.sqrt(
                (robot_row - frontier[0])**2 + (robot_col - frontier[1])**2)

            distance = distance_grid * self.map_data.info.resolution

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
                self.get_logger().warning(
                    f"Unknown strategy {strategy}, fallback to nearest")
                score = -distance

            if score > best_score:
                best_score = score
                chosen_frontier = frontier

        if chosen_frontier:
            self.current_target = chosen_frontier
            self.get_logger().info(
                f"Chosen frontier {chosen_frontier} with score={best_score:.2f} using {strategy} strategy")
        else:
            self.get_logger().warning("No valid frontier found")

        return chosen_frontier

    def finish_if_map_complete(self, frontiers, threshold=None):
        """
        Finish exploration if the number of frontiers is below a threshold.
        """
        # if threshold is None:
        #     threshold = getattr(self, 'threshold', 5)
        # if len(frontiers) <= threshold:
        #     self.get_logger().info(
        #         f"Frontiers below threshold ({threshold}). Exploration complete!")
        #     # self.shutdown_robot()
        #     # Optionally, stop the timer to halt further exploration
        #     self.timer.cancel()
        #     return True
        return False

    def explore(self):
        
        if self.map_data is None:
            self.get_logger().warning("No map data available")
            return

        # Convert map to numpy array
        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width))

        # Detect frontiers
        clusters, centroids = self.find_frontiers(map_array)
        self.publish_frontier_clusters(clusters, centroids)

        frontiers = centroids

        self.get_logger().info(
            f"Exploration: {len(frontiers)} frontier regions detected."
        )

        # Finish if frontiers are below threshold
        if self.finish_if_map_complete(frontiers):
            return

        chosen_frontier = self.choose_frontier(
            clusters,
            frontiers,
            map_array
        )

        if not chosen_frontier:
            self.get_logger().warning("No frontiers to explore")
            return

        # Convert the chosen frontier to world coordinates
        goal_x = chosen_frontier[1] * self.map_data.info.resolution + \
            self.map_data.info.origin.position.x
        goal_y = chosen_frontier[0] * self.map_data.info.resolution + \
            self.map_data.info.origin.position.y

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
