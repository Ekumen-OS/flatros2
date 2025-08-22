# flatros2

A proof-of-concept demonstrating zero-copy communication in ROS 2 between C++ and Python using FlatBuffers and Eclipse iceoryx.

[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## 🌐 Overview

This repository provides a working prototype for high-performance messaging in ROS 2. The standard ROS 2 messaging pipeline often involves multiple data copies and serialization/deserialization steps, which can become a bottleneck for high-throughput applications, especially when dealing with large data payloads like images or point clouds.

`flatros2` tackles this problem by combining:

1. [`flatbuffers`](https://github.com/google/flatbuffers): a highly efficient cross-platform serialization library from Google. It allows accessing serialized data without a parsing/unpacking step, enabling zero-copy access.
2.  [`Eclipse Iceoryx2`](https://github.com/eclipse-iceoryx/iceoryx2): a true zero-copy inter-process communication (IPC) middleware that uses shared memory.

By integrating these technologies, `flatros2` achieves significantly lower latency and CPU usage for intra-machine communication. This prototype demonstrates a C++/Python image processing pipeline where large image data is passed between nodes with minimal overhead.

## 🌟 Features

*   **Near Zero-Copy:** Drastically reduces data copies between publishers and subscribers on the same machine.
*   **Efficient Serialization:** Uses Google's FlatBuffers for memory-efficient, access-without-parsing data representation.
*   **C++/Python Interoperability:** Seamlessly pass large data between C++ and Python nodes without serialization/deserialization overhead in the Python process.

## 🚀 Getting Started

The easiest way to get started is by using the provided Visual Studio Code Dev Container, which handles all dependencies and configuration automatically.

### Prerequisites

*   Docker
*   Visual Studio Code
*   Dev Containers extension for VS Code

### Installation and Build

1.  Clone the repository:
    ```bash
    git clone https://github.com/Ekumen-OS/flatros2.git
    cd flatros2
    ```

2.  Open the cloned repository folder in Visual Studio Code.

3.  When prompted, click on **"Reopen in Container"**. This will build the Docker image specified in `.devcontainer/Dockerfile`, which includes ROS 2 Jazzy and all other necessary dependencies.

4.  Once the container is running and you have a terminal open inside VS Code, continue with the workspace setup instructions below.

## Workspace Setup Instructions

To set up the recommended workspace layout for Flatros2 and ROS 2 core development:

### 1. Create Workspaces and Source Folders

```bash
# Create the main Flatros2 workspace
mkdir -p ~/flatros_ws/src

# Create a separate workspace for ROS 2 core (optional)
mkdir -p ~/ros2-core/src
```

### 2. Import Repositories with vcs

- For the Flatros2 workspace, use the provided `iceoryx.repos` file:

```bash
cd ~/flatros_ws/src
vcs import < /workspace/src/iceoryx.repos
```

- For the ROS 2 core workspace, use the provided `ros2-core.repos` file:

This step is optional! Is just to have the source code be able
to instrument code and debug better.

```bash
cd ~/ros2-core/src
vcs import < /workspace/src/ros2-core.repos
```

Continue with the build and setup instructions as described below.

---

## Troubleshooting & Setup Tips

If you encounter build or permission errors, try the following:

**Fix permissions so colcon can write logs/builds:**

```bash
sudo chown -R developer:ekumen .
```

**Fix missing ACL headers needed by iceoryx:**

```bash
sudo apt update && sudo apt install -y libacl1-dev
```

## Running the Demo

The demo consists of three nodes that form an image processing pipeline:

1.  `image_capture_node.py`: A Python node that captures images from a webcam and publishes them.
2.  `edge_detector_node`: A C++ node that subscribes to the raw images, performs Canny edge detection using OpenCV, and publishes the resulting image.
3.  `image_viewer_node.py`: A Python node that subscribes to both the raw and the edge-detected images and displays them using OpenCV.

To run the demo, open three separate terminals inside the dev container (`Ctrl+Shift+5` or `Terminal > New Terminal`).

> [!IMPORTANT]
> The `image_capture_node.py` requires a webcam. If you don't have one, you can modify it to publish a static image instead.

---

**Terminal 1: Run the Image Capture Node**

```bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_iceoryx2_cxx
ros2 run flatros2 image_capture_node.py --ros-args -r image:=image/raw
```

Always the `RMW_IMPLEMENTATION` environment variable to `rmw_iceoryx2_cxx` to enable the zero-copy transport.

---

**Terminal 2: Run the Edge Detection Node**

```bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_iceoryx2_cxx
ros2 run flatros2 edge_detector_node --ros-args -r input_image:=image/raw -r output_image:=image/edges
```

---

**Terminal 3: Run the Image Viewer Node**

```bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_iceoryx2_cxx
ros2 run flatros2 image_viewer_node.py --ros-args -r image:=image/edges
```

You should see the real-time edge-detected video stream pop up in a separate window.

> [!CAUTION]
> Iceoryx2 is a still very much a work in progress. Signal handling and QoS support may be lacking.

## License

This project is licensed under the Apache 2.0 License - see the [LICENSE](./LICENSE) file for details.

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
