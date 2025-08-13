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
    parser.add_argument('--timeout', required=False, default=0.0001, help='Specifies timeout for parcel processing (default: 0.0001 seconds)')
    parser.add_argument('--lossmode', required=False, default='lossy', help='Specifies loss mode for parcel transmission (default: lossy)')
    parser.add_argument('--depth', required=False, default=10, type=int, help='QoS depth for parcel topics (default: 10)')
    args = parser.parse_args(argv[1:])

    print(f"Starting station type: {args.type} with node name: {args.name} and timeout: {args.timeout}")

    station_cls = get_station_class(args.type)

    rclpy.init()
    # Pass loss_mode and depth to constructor
    node = station_cls(name=args.name, loss_mode=args.lossmode, depth=args.depth)
    
    async def runner():
        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=float(args.timeout))
                await asyncio.sleep(0.0)  # Small delay for async tasks
        except SystemExit:
            rclpy.logging.get_logger("Quitting").info('Done')
        finally:
            node.destroy_node()
            rclpy.shutdown()

    asyncio.run(runner())

if __name__ == "__main__":
    main()

