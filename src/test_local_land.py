
"""
test local land
"""



import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleCommand, LandingTargetPose, VehicleLocalPosition
#import multiprocessing

import time

TARGET_POSE=(2,3,0)


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        
        
        
        super().__init__('local_position_landing_node')

        self.hb=0
        self.dt=0.1
        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.vehicle_landing_publisher = self.create_publisher(
            LandingTargetPose, '/fmu/in/vehicle_command', qos_profile)
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)

        # Create a timer to publish control commands
        # time.sleep(3000)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.start_land()
        
        # self.local_land()
    
    def start_land(self):
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_PRECLAND
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
    
    def timer_callback(self):
        self.local_land()
    

    def local_land(self):
            """Publish a vehicle command."""
            """
            # Relative position of precision land target in navigation (body fixed, north aligned, NED) and inertial (world fixed, north aligned, NED) frames

uint64 timestamp		# time since system start (microseconds)

bool is_static			# Flag indicating whether the landing target is static or moving with respect to the ground

bool rel_pos_valid		# Flag showing whether relative position is valid
bool rel_vel_valid		# Flag showing whether relative velocity is valid

float32 x_rel			# X/north position of target, relative to vehicle (navigation frame) [meters]
float32 y_rel			# Y/east position of target, relative to vehicle (navigation frame) [meters]
float32 z_rel			# Z/down position of target, relative to vehicle (navigation frame) [meters]

float32 vx_rel			# X/north velocity  of target, relative to vehicle (navigation frame) [meters/second]
float32 vy_rel			# Y/east velocity of target, relative to vehicle (navigation frame) [meters/second]

float32 cov_x_rel		# X/north position variance [meters^2]
float32 cov_y_rel		# Y/east position variance [meters^2]

float32 cov_vx_rel		# X/north velocity variance [(meters/second)^2]
float32 cov_vy_rel		# Y/east velocity variance [(meters/second)^2]

bool abs_pos_valid		# Flag showing whether absolute position is valid
float32 x_abs			# X/north position of target, relative to origin (navigation frame) [meters]
float32 y_abs			# Y/east position of target, relative to origin (navigation frame) [meters]
float32 z_abs			# Z/down position of target, relative to origin (navigation frame) [meters]
            """
            
            
            # if(self.vehicle_local_position):
                
            dist=self.calc_dist3()
            
            print(dist)
            print("-"*10)
            
            msg = LandingTargetPose()
            msg.is_static = True
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            
            msg.rel_pos_valid = True
            msg.rel_vel_valid = False
            msg.x_rel = dist[0]
            msg.y_rel = dist[1]
            msg.z_rel = dist[2]
            
            
            self.vehicle_command_publisher.publish(msg)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position
        # print("got locla position")

    def calc_dist3(self, ):
        dx=self.vehicle_local_position.x-TARGET_POSE[0]
        dy=self.vehicle_local_position.y-TARGET_POSE[1]
        dz=self.vehicle_local_position.z-TARGET_POSE[2]

        return -dx,-dy,-dz

    
def main(args=None) -> None:
    print('attenp local land...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
