#!/bin/bash
# Script to wait for all required sensor topics to be publishing data before starting RTAB-Map

echo "Waiting for sensor topics to publish data..."

# List of required topics (LiDAR disabled for pure RGB-D SLAM)
TOPICS=(
    "/sim_camera/image_raw"
    "/sim_camera/depth"
    "/sim_camera/camera_info"
    "/odom"
    "/imu/data"
)

# Wait for each topic with timeout
TIMEOUT=120  # seconds (increased for depth model loading)
START_TIME=$(date +%s)

for topic in "${TOPICS[@]}"; do
    echo "  Waiting for data on topic: $topic"

    while true; do
        # Try to receive one message from the topic (with 2 second timeout per attempt)
        if timeout 2s ros2 topic echo "$topic" --once > /dev/null 2>&1; then
            echo "    ✓ Topic $topic is publishing data"
            break
        fi

        # Check global timeout
        CURRENT_TIME=$(date +%s)
        ELAPSED=$((CURRENT_TIME - START_TIME))

        if [ $ELAPSED -ge $TIMEOUT ]; then
            echo "    ✗ Timeout waiting for data on topic $topic"
            echo "    Topic may be advertised but not publishing yet"
            exit 1
        fi

        # Small delay before retry
        sleep 0.5
    done
done

echo ""
echo "================================================"
echo "All sensor topics are publishing data!"
echo "Starting RTAB-Map now..."
echo "================================================"
echo ""
exit 0
