# EcoDex

EcoDex is an innovative service designed to address the problems of garbage collection and segregation by automating the task and enabling tracking of waste collection based on location and type.

![Raged2](https://github.com/user-attachments/assets/80a0b8ff-2929-4a72-9ffd-c6024f904a11)


## Features
EcoDex consists of four main parts:

1. **Machine Learning Algorithm**
2. **ROS-Based Communication**
3. **Website-Based Monitoring**
4. **Manual Control for Debugging and Error Correction**

---

## Implementation

### **1. Machine Learning Algorithm**

#### Garbage Detection:
- Utilizes a MobileNet V2 SSD-based architectural model.
- Custom-trained on a dataset comprising 60 classes.

#### Distance and Angle Calculation:
- Uses camera focal length and horizontal field of view (HFoV) properties.
- **Focal Length Calculation Formula:**
  
  ```
  f = (w * d) / W
  ```
  
  - `w`: Width in pixels
  - `W`: Actual width of the object
  - `d`: Distance from the camera

- **Distance Calculation Formula:**
  
  ```
  d = (W * f) / w
  ```

- **Angle Calculation Formula:**
  
  ```
  theta = (width_of_frame / Hfov) * (centre_of_screen - centre_of_bounding_box)
  ```

#### Reward-Punishment ANN Model:
- Implements an artificial neural network (ANN)-based reward-punishment mechanism.
- Calculates delay for the bot to reach a particular distance, optimizing navigation based on detected objects.

---

### **2. ROS-Based Communication**
- Employs ROS1 as the primary tool for device communication.
- Tasks are modularized into multiple publisher and subscriber nodes:
  - Input from camera
  - Object detection
  - Distance and angle calculation
  - Data transmission to the microcontroller

- Final processed values are sent to an ESP microcontroller, which enables the bot to pick up and segregate garbage.

---

### **3. Website-Based Monitoring**
- A custom-built website provides user login and sign-in functionalities.

#### Key Features:
- **Data Storage:**
  - Tracks detected garbage data, categorized into biodegradable and non-biodegradable materials.
- **User-Based Monitoring:**
  - Each user or company can log in to access their bot's data.
  - Displays collected garbage details (e.g., weight in kilograms), bot location, collection points, and current bot status.

---

### **4. Manual Control-Based Debugging Method**

#### Purpose:
1. **Autonomous Functionality Backup:**
   - Ensures that if the bot's autonomous system fails, manual controls can take over seamlessly.

2. **Debugging and Dataset Collection:**
   - Allows users to collect new datasets for the MobileNet model.
   - Facilitates debugging and correcting hardware errors.
  
  ![Raged1](https://github.com/user-attachments/assets/71bddf7b-b2e3-427a-885a-094e0f9ba990)


---

## Contributors
- Ayush Acharya
- Joel Dantis
- Gobi E
- Arjun Deveraj

---

## Future Improvements
- Implement additional classes for waste detection.
- Upgrade to ROS2 for improved performance.
- Introduce real-time alert notifications on the website.

---

## License
This project is licensed under the [MIT License](LICENSE).
