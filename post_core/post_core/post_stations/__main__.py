import sys
import argparse
import rclpy
from rclpy.utilities import remove_ros_args
from .registry import get_station_class  
from .stations import *  # Ensure stations are registered

def main():
    argv = remove_ros_args(sys.argv)

    parser = argparse.ArgumentParser(description="Launch a station node")
    parser.add_argument('--type', required=True, help='Station type to run')
    parser.add_argument('--name', required=True, help='Station node name')
    
    args = parser.parse_args(argv[1:])  # skip script name
    
    print(f"Starting station type: {args.type} with node name: {args.name}")

    station_cls = get_station_class(args.type)

    rclpy.init()
    node = station_cls(name=args.name)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

