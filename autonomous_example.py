"""
This is a tiny autonomous system that should be able to finish a lap in an empty map with only cones. 
Use the following settings.json:

{
  "SettingsVersion": 1.2,
  "Vehicles": {
    "FSCar": {
      "DefaultVehicleState": "",
      "EnableCollisionPassthrogh": false,
      "EnableCollisions": true,
      "AllowAPIAlways": true,
      "RC":{
          "RemoteControlID": -1
      },
      "Sensors": {
        "Gps" : {
          "SensorType": 3,
          "Enabled": true
        },
        "Lidar": {
          "SensorType": 6,
          "Enabled": true,
          "X": 1.3, "Y": 0, "Z": 0.1,
          "Roll": 0, "Pitch": 0, "Yaw" : 0,
          "NumberOfLasers": 1,
          "PointsPerScan": 500,
          "VerticalFOVUpper": 0,
          "VerticalFOVLower": 0,
          "HorizontalFOVStart": -90,
          "HorizontalFOVEnd": 90,
          "RotationsPerSecond": 10,
          "DrawDebugPoints": true
        }
      },
      "Cameras": {},
      "X": 0, "Y": 0, "Z": 0,
      "Pitch": 0, "Roll": 0, "Yaw": 0
    }
  }
}
"""

"""
This is a tiny autonomous system that can finish a lap in an empty map with only cones.
It combines lidar-based autonomous driving with a live camera feed.
"""

import sys
import os
import time
import numpy
import math
import matplotlib.pyplot as plt
import cv2  # OpenCV for displaying the camera feed

# Add the fsds package to the Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import fsds

# Connect to the simulator
client = fsds.FSDSClient()
client.confirmConnection()
print("Connected to Simulator")

# Enable API control
client.enableApiControl(True)

# Autonomous system constants
max_throttle = 0.2  # m/s^2
target_speed = 4  # m/s
max_steering = 0.3
cones_range_cutoff = 7  # meters

# Lidar-based cone detection
def pointgroup_to_cone(group):
    average_x = sum(point['x'] for point in group) / len(group)
    average_y = sum(point['y'] for point in group) / len(group)
    return {'x': average_x, 'y': average_y}

def distance(x1, y1, x2, y2):
    return math.sqrt(math.pow(abs(x1 - x2), 2) + math.pow(abs(y1 - y2), 2))

def find_cones():
    # Get lidar data
    lidardata = client.getLidarData(lidar_name='Lidar')
    if len(lidardata.point_cloud) < 3:
        return []  # No points detected

    # Convert lidar point cloud to xyz coordinates
    points = numpy.array(lidardata.point_cloud, dtype=numpy.dtype('f4'))
    points = numpy.reshape(points, (int(points.shape[0] / 3), 3))

    # Group points into cones
    current_group = []
    cones = []
    for i in range(1, len(points)):
        distance_to_last_point = distance(points[i][0], points[i][1], points[i - 1][0], points[i - 1][1])
        if distance_to_last_point < 0.1:  # Points closer than 10 cm belong to the same group
            current_group.append({'x': points[i][0], 'y': points[i][1]})
        else:
            if len(current_group) > 0:
                cone = pointgroup_to_cone(current_group)
                if distance(0, 0, cone['x'], cone['y']) < cones_range_cutoff:
                    cones.append(cone)
                current_group = []
    return cones

def calculate_steering(cones):
    # Calculate steering based on average y position of cones
    average_y = sum(cone['y'] for cone in cones) / len(cones)
    return -max_steering if average_y > 0 else max_steering

def calculate_throttle():
    # Calculate throttle based on current speed
    gps = client.getGpsData()
    velocity = math.sqrt(math.pow(gps.gnss.velocity.x_val, 2) + math.pow(gps.gnss.velocity.y_val, 2))
    return max_throttle * max(1 - velocity / target_speed, 0)

# Main loop
while True:
    # 20 Hz loop
    plt.pause(0.05)
    plt.clf()
    plt.axis([-cones_range_cutoff, cones_range_cutoff, -2, cones_range_cutoff])

    # Lidar-based cone detection
    cones = find_cones()
    if len(cones) == 0:
        continue

    # Control car based on lidar data
    car_controls = fsds.CarControls()
    car_controls.steering = calculate_steering(cones)
    car_controls.throttle = calculate_throttle()
    car_controls.brake = 0
    client.setCarControls(car_controls)

    # Plot detected cones
    for cone in cones:
        plt.scatter(x=-1 * cone['y'], y=cone['x'])

    # Retrieve and display camera feed
    response = client.simGetImages([
        fsds.ImageRequest("examplecam", fsds.ImageType.Scene, False, False)
    ])
    if response:
        img1d = numpy.frombuffer(response[0].image_data_uint8, dtype=numpy.uint8)
        img_rgb = img1d.reshape(response[0].height, response[0].width, 3)
        cv2.imshow("Live Camera Feed", img_rgb)

    # Exit condition: Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up
cv2.destroyAllWindows()
plt.show()