#!/bin/bash

# Output file with timestamp
OUTPUT_FILE="pose_test_results_$(date +%Y%m%d_%H%M%S).log"

echo "Running pose_test 10 times..."
echo "Output will be saved to: $OUTPUT_FILE"
echo "========================================" | tee "$OUTPUT_FILE"

for i in {1..50}; do
    echo "" | tee -a "$OUTPUT_FILE"
    echo "========================================" | tee -a "$OUTPUT_FILE"
    echo "Run $i of 50 - $(date)" | tee -a "$OUTPUT_FILE"
    echo "========================================" | tee -a "$OUTPUT_FILE"
    
    ros2 run writing_robot_control pose_test \
        --ros-args \
        -p poses_file:=src/writing_robot_description/config/delivery_poses.yaml \
        -p validate:=true \
        -p telemetry:=true \
        -p urdf_file:=src/writing_robot_description/urdf/koch_v11_arm_real.urdf \
        -p power_monitoring:=true \
        -p movement_time:=0.5  2>&1 | tee -a "$OUTPUT_FILE"
    
    # Check if command succeeded
    if [ ${PIPESTATUS[0]} -ne 0 ]; then
        echo "ERROR: Run $i failed!" | tee -a "$OUTPUT_FILE"
    else
        echo "Run $i completed successfully" | tee -a "$OUTPUT_FILE"
    fi
    
    # Optional: add a small delay between runs
    # sleep 1
done

echo "" | tee -a "$OUTPUT_FILE"
echo "========================================" | tee -a "$OUTPUT_FILE"
echo "All 10 runs completed - $(date)" | tee -a "$OUTPUT_FILE"
echo "Results saved to: $OUTPUT_FILE" | tee -a "$OUTPUT_FILE"
