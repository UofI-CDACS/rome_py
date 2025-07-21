import sys
import argparse
import asyncio
import rclpy
from rclpy.utilities import remove_ros_args
from .registry import get_station_class  

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
    async def runner():
        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                await asyncio.sleep(0)  # let async tasks progress
        finally:
            node.destroy_node()
            rclpy.shutdown()

    asyncio.run(runner())

if __name__ == "__main__":
    main()

