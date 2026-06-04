# Camera Pick Pipeline Setup Guide

This project is a ROS 2-based camera pipeline for real-time object detection and pose estimation using YOLO and Intel RealSense cameras.

## Prerequisites

- Python 3.8 or higher
- pip (Python package manager)
- ROS 2 (if using ROS features)

## Virtual Environment Setup

### 1. Create a Virtual Environment

```bash
python3 -m venv .venv
```

### 2. Activate the Virtual Environment

**On Linux/macOS:**
```bash
source .venv/bin/activate
```

**On Windows:**
```bash
.venv\Scripts\activate
```

### 3. Upgrade pip

```bash
pip install --upgrade pip
```

### 4. Install Dependencies

```bash
pip install -r requirements.txt
```

### 5. Install the Package (Optional)

If you want to use the package as an installed module:

```bash
pip install -e .
```

## Deactivating the Virtual Environment

When you're done working, deactivate the virtual environment:

```bash
deactivate
```

## Project Structure

```
camera/
├── .venv/                    # Virtual environment directory
├── src/
│   └── my_pick_pipeline/
│       ├── my_pick_pipeline/
│       │   ├── __init__.py
│       │   └── realtime_inference.py
│       ├── test/
│       ├── setup.py
│       └── package.xml
├── requirements.txt          # Python dependencies
└── README.md                 # This file
```

## Dependencies

The project requires the following main packages:

- **OpenCV** (`opencv-python`) - Computer vision library
- **NumPy** - Numerical computing
- **Ultralytics YOLO** - Object detection framework
- **pyrealsense2** - Intel RealSense camera SDK
- **ROS 2** - Robotics middleware (rclpy, geometry_msgs, robot_msgs)

See `requirements.txt` for the complete list of dependencies.

## Running the Project

To run the real-time inference script:

```bash
# Make sure virtual environment is activated
source .venv/bin/activate

# Run the pose estimator
pose_estimator
```

Or directly:

```bash
python -m my_pick_pipeline.realtime_inference
```

## Troubleshooting

### Virtual Environment Not Activating

Ensure you're in the correct directory and using the correct activation command for your OS.

### Missing Dependencies

If you encounter import errors, reinstall dependencies:

```bash
pip install -r requirements.txt --force-reinstall
```

### ROS 2 Dependencies

If ROS 2 packages are not found, ensure ROS 2 is properly installed and sourced:

```bash
source /opt/ros/<distro>/setup.bash
```

Replace `<distro>` with your ROS 2 distribution (e.g., `humble`, `iron`).

## Development

For development with testing:

```bash
pip install -e ".[test]"
pytest
```

## License

See setup.py for license information.
