## Benchmarking Flatros2 Performance

This section describes a benchmarking setup to measure the latency of large messages using Flatros2 with Iceoryx2. The setup includes custom publisher and subscriber nodes with flexible CLI options and robust diagnostics, as well as an analysis script for results.

### Components

- **Publisher (Python, `benchmark_pub.py`):**
    - Publishes messages of configurable sizes, either sweeping through a range or sending fixed sizes for a set duration.
    - Supports CLI arguments:
        - `--sizes`: Comma-separated list of message sizes in bytes (e.g., `--sizes 10,1000,10000`).
        - `--mode`: `sweep` (vary sizes) or `fixed_size` (single size).
        - `--duration`: Duration in seconds to send messages (for `fixed_size` mode).
        - `--period`: Period in milliseconds between messages.
        - Prints the current message size before publishing in `fixed_size` mode.

- **Subscriber (C++, `benchmark_sub`):**
    - Receives messages and logs latency to `latency_log.csv`.
    - Supports CLI arguments:
        - `--log`: Enable/disable CSV logging.
        - `--debug`: Enable debug output.
        - `--duration`/`-d`: Duration in seconds to run.
        - Prints a clear warning at startup about how to kill the process (since Ctrl+C may not work; use `pkill` as below).

- **Analyzer (Python, `analyze_latency.py`):**
    - Reads and cleans the latency CSV file.
    - Plots latency vs. message size and saves the figure as `latency_vs_size.png`.
    - Prints mean, min, max latency for message size ranges.

### Instructions

#### 1. Run the Subscriber

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_iceoryx2_cxx
./install/flatros2/lib/flatros2/benchmark_sub --duration 5 --log true --debug
```

> [!NOTE]
> At startup, the subscriber prints a warning about how to kill the process. If Ctrl+C does not work, use the `pkill` command below.

#### 2. Run the Publisher

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_iceoryx2_cxx
python3 install/flatros2/lib/flatros2/benchmark_pub.py --mode fixed_size --sizes 10,1000,10000 --period 1000
```

You can also use `--mode fixed_size --duration 10` to send a fixed size for a set duration.

#### 3. Kill Existing Iceoryx2 State (Recommended)

```bash
pkill -9 -f 'benchmark_pub|benchmark_sub|ros2'
sudo rm -f /dev/shm/iox2_*
sudo rm -rf /tmp/iceoryx2
```

The subscriber will log latency data to `latency_log.csv`.

#### 4. Analyze Results

Use the provided script to generate a plot and summary statistics:

```bash
python3 analyze_latency.py
```

This script:
- Reads and cleans the latency CSV file
- Plots latency vs. message size and saves the figure as `latency_vs_size.png`
- Prints mean, min, max latency for message size ranges

You can find the script in the root directory as `analyze_latency.py`.


#### 5. Getting a CPU flamegraph

To analyze CPU usage and bottlenecks during benchmarking, you can generate a CPU flamegraph using Linux's `perf` tool. This is useful for both the publisher and subscriber processes.

> **Note:**
> For best results, build your workspace with debug symbols enabled. This ensures that the flamegraph will show function names and call stacks correctly.
>
> Use the following command to build with debug symbols:
> ```bash
> colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
> ```

**Step-by-step instructions:**

1. **Start the subscriber in one terminal:**

        ```bash
        ./install/flatros2/lib/flatros2/benchmark_sub -d 60 --log false
        ```

2. **Start the publisher in another terminal:**

        ```bash
        python3 install/flatros2/lib/flatros2/benchmark_pub.py --duration 60 --period 10 --sizes 2000000
        ```

3. **Find the process IDs (PIDs) for both processes in a third terminal:**

        ```bash
        pgrep -fl benchmark_sub
        pgrep -fl benchmark_pub.py
        ```
        Note the PIDs for use in the next step.

4. **Record a perf trace for either process (replace <PID> with the actual PID):**

        ```bash
        sudo perf record -F 99 -p <PID> --call-graph dwarf -- sleep 5
        ```
        - This will record 5 seconds of profiling data for the selected process.
        - You can run this for both publisher and subscriber (in separate terminals or sequentially).

5. **Generate a flamegraph:**

        - First, install Flamegraph scripts if you haven't already:
            ```bash
            git clone https://github.com/brendangregg/Flamegraph.git
            export PATH=$PATH:$(pwd)/Flamegraph
            ```
        - Then, generate the flamegraph SVG:
            ```bash
            sudo perf script | stackcollapse-perf.pl | flamegraph.pl > flamegraph.svg
            ```
        - Open `flamegraph.svg` in your browser to explore CPU usage.

**Tips:**
- You may need to install `perf` and `dwarves` (for DWARF call graph support):
    ```bash
    sudo apt update && sudo apt install linux-tools-common linux-tools-$(uname -r) dwarves
    ```
- For more details, see: https://www.brendangregg.com/flamegraphs.html
