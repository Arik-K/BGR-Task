# BGR-Task
# Formula Student Driverless Simulator - Assignment Submission

## Setup Process
1. **Simulator Setup**:
   - Downloaded and installed the Formula Student Driverless Simulator.
   - Cloned the repository using:
     ```
     git clone https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator
     ```
   - Installed Python dependencies:
     ```
     pip install -r requirements.txt
     ```
   - Started the simulator and verified it was running correctly.

2. **Configuration Changes**:
   - Modified the `settings.json` file to add the `examplecam` camera:
     ```json
     "Cameras": {
    "examplecam": {
        "CaptureSettings": [
        {
            "ImageType": 0,
            "Width": 785,
            "Height": 785,
            "FOV_Degrees": 90
        }
        ],
        "X": 1.0,
        "Y": 0.06,
        "Z": 1.1,
        "Pitch": -10.0,
        "Roll": 0.0,
        "Yaw": 0
    }
}

     }
     ```

3. **Running the Python Script**:
   - Ran the `autonomous_example.py` script to display the live camera feed.

---

## Modifications

### `settings.json`
- Added the `"examplecam"` camera with the above configuration.

### `autonomous_example.py`
- Added code to integrate and display the live camera feed using OpenCV:
   ```python
   # Retrieve and display camera feed
response = client.simGetImages([
    fsds.ImageRequest("examplecam", fsds.ImageType.Scene, False, False)
])
if response:
    img1d = numpy.frombuffer(response[0].image_data_uint8, dtype=numpy.uint8)
    img_rgb = img1d.reshape(response[0].height, response[0].width, 3)
    cv2.imshow("Live Camera Feed", img_rgb)

