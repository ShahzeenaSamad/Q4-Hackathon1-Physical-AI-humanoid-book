# Troubleshooting Guide

Common issues and solutions for ROS 2, Simulation, and NVIDIA Isaac platforms.

---

## 1. ROS 2 Issues

### Command not found: `ros2`
*   **Cause**: The ROS 2 environment has not been sourced in the current terminal.
*   **Solution**: Run `source /opt/ros/humble/setup.bash`. To automate this, add it to your `~/.bashrc`.

### Nodes cannot see each other
*   **Cause**: Mismatched `ROS_DOMAIN_ID` or network firewall.
*   **Solution**: Check `echo $ROS_DOMAIN_ID`. Ensure all machines use the same ID (default is 0).

### Build failures (`colcon build`)
*   **Cause**: Missing dependencies.
*   **Solution**: Run `rosdep install -i --from-path src --rosdistro humble -y`.

---

## 2. Gazebo Simulation Issues

### Simulation is extremely slow
*   **Cause**: `real_time_factor` < 1.0 due to high computational load.
*   **Solution**: Simplify models, reduce sensor update rates (Hz), or lower the resolution of simulated cameras.

### Gazebo crashes on startup
*   **Cause**: GPU driver issues or incompatible environment variables.
*   **Solution**: Try running with `export QT_QPA_PLATFORM=xcb` or update your NVIDIA drivers.

---

## 3. NVIDIA Isaac Sim Issues

### Black screen or crash in Isaac Sim
*   **Cause**: Incompatible NVIDIA drivers or out-of-memory (OOM) on GPU.
*   **Solution**: Ensure you have at least 8GB VRAM and NVIDIA Driver 525+.

### ROS 2 Bridge not working
*   **Cause**: The ROS 2 Extension is not enabled in Isaac Sim.
*   **Solution**: Go to `Window` -> `Extension Manager`, search for "ROS 2 Bridge" and enable it.

---

## 4. AI & VLA Issues

### Out of Memory (CUDA OOM)
*   **Cause**: Loading a Large Language Model or Vision-Language Model that exceeds GPU capacity.
*   **Solution**: Use a smaller model (e.g., `distil-whisper`) or use 4-bit/8-bit quantization.

### High Latency in Voice Commands
*   **Cause**: Using a slow CPU for transcription.
*   **Solution**: Use `faster-whisper` and ensure it's utilizing the GPU (`device="cuda"`).
