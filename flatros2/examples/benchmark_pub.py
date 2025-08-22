#!/usr/bin/env python3

# Imports for time, ROS 2, message types, and numpy
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from benchmark_msgs.msg import BenchmarkData
from flatros2 import Flat
from flatros2.publisher import FlatPublisher
import numpy as np
import argparse

# Default message sizes for benchmarking (in bytes)
# SIZES = [10, 100_000, 1_000_000, 4_000_000]  # in bytes
SIZES = [1_000_000]  # in bytes


def generate_sizes_linear(start, stop, step=1):
    """Generate a list of sizes from start to stop with a given step."""
    return list(range(start, stop + 1, step))

class BenchmarkPublisher(Node):
    """
    ROS 2 node for publishing benchmark messages of varying sizes.
    Publishes messages in either fixed_size or sweep mode.
    """
    # ANSI color codes for consistent use
    BOLD = '\033[1m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    CYAN = '\033[36m'
    RED = '\033[31m'
    RESET = '\033[0m'

    def __init__(self, mode="fixed_size", duration=10, sizes=None, period_ms=1000):
        super().__init__('benchmark_publisher', start_parameter_services=False, enable_rosout=False, enable_logger_service=False)
        self.mode = mode
        self.duration = duration
        self.period_ms = period_ms
        if mode == "sweep":
            # Sweep mode: test a range of message sizes
            sweep_start = 1
            sweep_stop = 2_000_000
            sweep_step = 20000
            print(f"{self.CYAN}{self.BOLD}Sweep mode:{self.RESET} going to publish messages from {self.YELLOW}{sweep_start}{self.RESET} to {self.YELLOW}{sweep_stop}{self.RESET} bytes with a step of {self.YELLOW}{sweep_step}{self.RESET}.")
            self.sizes = generate_sizes_linear(sweep_start, sweep_stop, sweep_step)
            self.index = 0
            self.publisher = self.make_flat_publisher(self.sizes[self.index])
            self.timer = self.create_timer(self.period_ms / 1000.0, self.sweep_send)
        else:
            # Fixed size mode: repeatedly send a fixed size
            self.sizes = sizes if sizes is not None else SIZES
            self.index = 0
            self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
            self.publisher = self.make_flat_publisher(self.sizes[self.index])
            print(f"{self.CYAN}🔁 Now sending {self.YELLOW}{self.sizes[self.index]}{self.RESET} bytes")
            self.timer = self.create_timer(self.period_ms / 1000.0, self.fixed_size_send)

    def make_flat_publisher(self, size):
        """Create a FlatPublisher for a given message size."""
        msg = BenchmarkData(header=Header(), data=np.zeros(size, dtype=np.uint8))
        return FlatPublisher(self, Flat(msg), 'benchmark_topic', 10)

    def fixed_size_send(self):
        """Send messages of a fixed size for a set duration."""
        now_sec = self.get_clock().now().seconds_nanoseconds()[0]
        if now_sec - self.start_time >= self.duration:
            self.index += 1
            if self.index >= len(self.sizes):
                self.get_logger().info(f"{self.GREEN}{self.BOLD}✅ All sizes sent.{self.RESET}")
                rclpy.shutdown()
                return
            self.current_size = self.sizes[self.index]
            self.publisher = self.make_flat_publisher(self.current_size)
            self.start_time = now_sec
            self.get_logger().info(f"{self.CYAN}🔁 Now sending {self.YELLOW}{self.current_size}{self.RESET} bytes")
            return

        msg = self.publisher.borrow_loaned_message()
        now = self.get_clock().now().to_msg()
        msg.header.stamp.sec = now.sec
        msg.header.stamp.nanosec = now.nanosec
        t0 = time.perf_counter()
        self.publisher.publish_loaned_message(msg)
        t1 = time.perf_counter()
        # print(f"{self.GREEN}Published {self.sizes[self.index]} bytes in {self.YELLOW}{1e3*(t1 - t0):.2f}{self.RESET} ms")

    def sweep_send(self):
        """Send one message for each size in the sweep list."""
        if self.index >= len(self.sizes):
            self.get_logger().info(f"{self.GREEN}{self.BOLD}✅ Sweep complete.{self.RESET}")
            rclpy.shutdown()
            return
        size = self.sizes[self.index]
        self.publisher = self.make_flat_publisher(size)
        msg = self.publisher.borrow_loaned_message()
        now = self.get_clock().now().to_msg()
        msg.header.stamp.sec = now.sec
        msg.header.stamp.nanosec = now.nanosec
        msg.data[:] = np.random.randint(0, 255, size, dtype=np.uint8)
        t0 = time.perf_counter()
        self.publisher.publish_loaned_message(msg)
        t1 = time.perf_counter()
        # print(f"{self.GREEN}Published {size} bytes in {self.YELLOW}{1e3*(t1 - t0):.2f}{self.RESET} ms")
        self.index += 1

def main():
    """
    Main function for the benchmark publisher script.

    Parses command-line arguments and starts the publisher node.

    Args:
        --mode: Specifies the publishing mode ('duration' or 'sweep').
        --help: Displays instructions on how to run the script.
    """
    # ANSI color codes (for use outside the class)
    BOLD = '\033[1m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    CYAN = '\033[36m'
    RESET = '\033[0m'
    parser = argparse.ArgumentParser(
        description=f'{BOLD}{GREEN}Benchmark Publisher{RESET}\n\n'
                    f'{CYAN}Publishes messages of varying sizes to benchmark performance.\n{RESET}',
        epilog=(
            f'{BOLD}{YELLOW}Usage Examples:{RESET}\n'
            f'  {CYAN}python3 benchmark_pub.py --mode sweep{RESET}         {YELLOW}# Sweep through many sizes{RESET}\n'
            f'  {CYAN}python3 benchmark_pub.py --duration 10{RESET}        {YELLOW}# Fixed size(s), 10s each{RESET}\n'
            f'\n'
            f'{BOLD}{YELLOW}Options:{RESET}\n'
            f'  {GREEN}--mode sweep|fixed_size{RESET}   Sweep (default) or fixed-size mode.\n'
            f'  {GREEN}--duration N{RESET}            Duration (seconds) for each size in fixed-size mode. Implies --mode fixed_size.\n'
            f'\n'
            f'{BOLD}{YELLOW}Sweep mode:{RESET}\n'
            f'  Publishes messages from 1 to 2,000,000 bytes with a step of 20,000.\n'
            f'{BOLD}{YELLOW}Fixed size mode:{RESET}\n'
            f'  Publishes each size for the specified duration (default: 10s per size).\n'
            f'\n'
            f'{BOLD}{YELLOW}Subscriber:{RESET}\n'
            f'  In another terminal, run the subscriber to receive and log messages:\n'
            f'    {CYAN}./install/flatros2/lib/flatros2/benchmark_sub --duration 30{RESET}\n'
            f'\n'
            f'{BOLD}{YELLOW}Analysis:{RESET}\n'
            f'  Analyze results with: {CYAN}python3 /workspace/analyze_latency.py{RESET}\n'
        )
    )
    parser.add_argument('--mode', choices=['fixed_size', 'sweep'], default='sweep',
                        help=f'{GREEN}sweep{RESET} (default) or {GREEN}fixed_size{RESET}')
    parser.add_argument('--sizes', type=str, default=None,
                        help=f'{GREEN}Comma-separated list of message sizes (bytes) for fixed_size mode. E.g. --sizes 10,1000,100000{RESET}')
    parser.add_argument('--duration', type=int, default=None,
                        help=f'{GREEN}Duration in seconds to publish each message size in fixed_size mode. If set, mode is forced to fixed_size. (default: None){RESET}')
    parser.add_argument('--period', type=int, default=1000,
                        help=f'{GREEN}Period in milliseconds between publishes (applies to both modes, default: 1000 ms){RESET}')
    import sys
    args = parser.parse_args()
    # If --duration or --sizes is provided, force mode to 'fixed_size'
    mode = args.mode
    duration = args.duration
    sizes = None
    period_ms = args.period
    if args.sizes is not None:
        try:
            sizes = [int(s) for s in args.sizes.split(',') if s.strip()]
            if not sizes:
                raise ValueError
        except Exception:
            print(f'{BOLD}Error:{RESET} --sizes must be a comma-separated list of positive integers.')
            sys.exit(1)
        mode = 'fixed_size'
    if duration is not None:
        mode = 'fixed_size'
        if duration <= 0:
            print(f'{BOLD}Error:{RESET} --duration must be a positive integer.')
            sys.exit(1)
    else:
        duration = 10  # fallback default
    if len(sys.argv) == 1:
        print(f'{YELLOW}No arguments passed. Defaulting to sweep mode.{RESET}')
    rclpy.init()
    try:
        rclpy.spin(BenchmarkPublisher(mode=mode, duration=duration, sizes=sizes, period_ms=period_ms))
    except KeyboardInterrupt:
        print(f"{YELLOW}Interrupted by user. Exiting...{RESET}")
    finally:
        rclpy.try_shutdown()

if __name__ == '__main__':
    # ANSI color codes (for use outside the class)
    BOLD = '\033[1m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    CYAN = '\033[36m'
    RESET = '\033[0m'
    print(f'{BOLD}{GREEN}Starting Benchmark Publisher...{RESET}')
    print(f'{CYAN}This program benchmarks the publishing performance by sending messages of varying sizes.{RESET}')
    print(f'Use the {YELLOW}"--mode"{RESET} argument to select between {GREEN}"fixed_size"{RESET} and {GREEN}"sweep"{RESET} modes.')
    print(f'{YELLOW}Note:{RESET} In another terminal, run the {GREEN}subscriber{RESET} to receive and log the messages:')
    print(f'  {CYAN}./install/flatros2/lib/flatros2/benchmark_sub --duration 30{RESET}')
    print(f'{YELLOW}Tip:{RESET} You can add {CYAN}-h{RESET} for help or change the duration as needed.')
    main()
