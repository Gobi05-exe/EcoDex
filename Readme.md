# EcoDex

EcoDex is an innovative service designed to address the problems of garbage collection and segregation by automating the task and enabling tracking of waste collection based on location and type.

![Raged2](https://github.com/user-attachments/assets/80a0b8ff-2929-4a72-9ffd-c6024f904a11)


## Features
EcoDex consists of four main parts:

1. **Machine Learning Algorithm**
2. **ROS-Based Communication**
3. **Website-Based Monitoring**
4. **Manual Control for Debugging and Error Correction**

![Raged_Circuit_page-0001](https://github.com/user-attachments/assets/50b592a4-5999-41ff-a6d0-c19dd9225f8d)


---

## Implementation

### **1. Machine Learning Algorithm**

#### Garbage Detection:
- Utilizes an Ultralytics YOLOv8 architectural model.
- Custom-trained on the TACO dataset comprising 60 classes.

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

  ![image](https://github.com/user-attachments/assets/4730d42d-f5f9-43c2-82c5-b9e590202065)


- **Angle Calculation Formula:**
  
  ```
  theta = (width_of_frame / Hfov) * (centre_of_screen - centre_of_bounding_box)
  ```

#### Optimized navigation to target object:
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

  ![image](https://github.com/user-attachments/assets/e105761f-9daa-4441-9670-39ebdb04bcdf)



#### Key Features:
- **Data Storage:**
  - Tracks detected garbage data, categorized into biodegradable and non-biodegradable materials.
 
    ![image](https://github.com/user-attachments/assets/0e3d0383-83e1-4b9c-a5f6-099f3f9d07dd)

- **User-Based Monitoring:**
  - Each user or company can log in to access their bot's data.
 
   ![image](https://github.com/user-attachments/assets/b19f93f5-820b-4117-8bac-47a306777146)

   ![image](https://github.com/user-attachments/assets/40eb2417-4eee-4d7d-8baa-7901d6e6fc24)
 
    

  - Displays collected garbage details (e.g., weight in kilograms), bot location, collection points, and current bot status.
 
    ![image](https://github.com/user-attachments/assets/f868b826-f269-4363-941f-f7503855cefa)


---

### **4. Manual Control-Based Debugging Method**

#### Purpose:
1. **Autonomous Functionality Backup:**
   - Ensures that if the bot's autonomous system fails, manual controls can take over seamlessly.

2. **Debugging:**
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
