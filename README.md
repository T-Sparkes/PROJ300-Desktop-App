# PROJ-300 Desktop Application
Contains the desktop application written for project 300. Relevent Extracts from my dissertation are included below, they are far too verbose for a github readme but do provide a detailed explanation.  

<img width="854" height="482" alt="Screenshot 2026-05-27 170120" src="https://github.com/user-attachments/assets/2d802f1d-5c39-4516-a12d-dbed00f44d45" />

Application GUI.

## Dissertation Extracts:
### 7.4 Derivation for the system Estimation Problem.

<img width="488" height="312" alt="image" src="https://github.com/user-attachments/assets/6e37bf40-096b-4281-ba58-d169e7dbd03d" />

To apply the EKF to this specific system, the mathematical models must be derived. Figure[16] summarises the localisation problem, the goal is to estimate the pose of a differential drive robot, whose state transition is governed by its kinematics. Two UWB landmarks provide corrective measurements in the form of range measurements $r_{a}$ and $r_{b}$ from fixed location in the global frame.

The state vector $X_{k}$ at time k is defined as:
And the control vector $U_{k}$:

$$x_{k}=\begin{bmatrix}x_{k}\cr y_{k}\cr \theta_{k}\end{bmatrix}$$

$$u_{k}=\begin{bmatrix}\Delta L_{k}\cr \Delta R_{k}\end{bmatrix}$$

Where $\Delta L_{k}$ and $\Delta R_{k}$ are the linear distance travelled by each wheel during the timestep.

The motion model or state transition model is derived from the differential drive kinematic equations [2]. The non linear motion model used for the state prediction is given by:

$$x_{k}^{-}=f(x_{k-1},u_{k})=\begin{bmatrix}x_{k-1}+\Delta d_{k}\cos(\theta_{k-1}+\frac{\Delta\theta_{k}}{2})\cr y_{k-1}+\Delta d_{k}\sin(\theta_{k-1}+\frac{\Delta\theta_{k}}{2})\cr \theta_{k-1}+\Delta\theta_{k}\end{bmatrix}$$

Where $\Delta d_{k}$ and $\Delta\theta_{k}$ are calculated as:

$$\Delta d_{k}=\frac{\Delta R_{k}+\Delta L_{k}}{2}$$

$$\Delta\theta_{k}=\frac{\Delta R_{k}-\Delta L_{k}}{B}$$

And B is the wheelbase of the robot platform. This nonlinear function provides the accuracy for state predictions as it fully captures the kinematics of the robots movement.

In order to propagate the state covariance under the gaussian assumption, the Jacobian of the motion model is used, linearizing around the current state and is calculated as:

$$F_{k}=\frac{\partial f}{\partial x}|_{x_{k-1},u_{k}}=\begin{bmatrix}1&0&-\Delta d_{k}\sin(\theta_{k-1}+\frac{\Delta\theta_{k}}{2})\cr 0&1&\Delta d_{k}\cos(\theta_{k-1}+\frac{\Delta\theta_{k}}{2})\cr 0&0&1\end{bmatrix}$$

To derive the measurement model, let the landmarks A and B be located at points in the global frame, (x1,y1) and (x2,y2) respectively. The measurements are given in the form of ranges ra and rb, from figure [16], from the landmarks to the current location of the robot.

The measurement vector $Z_{k}$ containing the two range measurements can be described by a nonlinear function of $X_{k^{\prime}}$ $h(x_{k})$ using the euclidean distance from each landmark to the robot's current location at time step K. Giving the measurement function $h(x)$ as:

$$z_{k}=\begin{bmatrix}z_{1,k}\cr z_{2,k}\end{bmatrix}=h(x_{k})=\begin{bmatrix}\sqrt{(x_{k}-x_{1})^{2}+(y_{k}-y_{1})^{2}}\cr \sqrt{(x_{k}-x_{2})^{2}+(y_{k}-y_{2})^{2}}\end{bmatrix}$$

As described previously, the measurement function must be linear to calculate the Kalman gain, so is approximated by linearizing with the jacobian H around the current state. Resulting in $H_{k}$ as:

$$H_{k}=\frac{\partial h}{\partial x}|_{x_{k}}=\begin{bmatrix}\frac{x_{k}-x_{1}}{r_{1,k}}&\frac{y_{k}-y_{1}}{r_{1,k}}&0\cr \frac{x_{k}-x_{2}}{r_{2,k}}&\frac{y_{k}-y_{2}}{r_{2,k}}&0\end{bmatrix}$$

Where:

$$r_{1,k}=\sqrt{(x_{k}-x_{1})^{2}+(y_{k}-y_{1})^{2}}, r_{2,k}=\sqrt{(x_{k}-x_{2})^{2}+(y_{k}-y_{2})^{2}}$$

### 8.0 Host Software Design
The host software is a custom visualization and control application developed for Windows, written in C++. It provides a graphical user interface (GUI) that allows for real-time control and modification of system parameters, such as Kalman filter covariances. Additionally, the software includes data logging capabilities, graphing the system state, landmark range measurements, as well as the Kalman gains and state covariances. At the center of the application is a 2D visualization of the environment in which the system operates. The top-down view displays the landmark range measurements, landmark locations, the Kalman state estimate (represented by a robot graphic), and the current path and waypoints the robot is set to follow. 
8.1 Software Framework and Architecture
The application uses the SDL3[17] library as the rendering backend, while the ImGUI[18] and ImPlot[19] library is utilized for the user interface functions. The main application class is derived from a base class that provides virtual functions for event callbacks and an update function that is called during each application frame. Figure [19] shows the components of the application.

<img width="312" height="328" alt="image" src="https://github.com/user-attachments/assets/a6408bda-3777-495e-93d9-7350daef8038" />

Figure.[19]. Application Components.
The application includes a base class that handles rendering to the viewport at the center of the screen. This base class is used by the Landmark container, Kalman filter, and Grid renderer classes to visualize the landmarks and their ranges, the Kalman state estimate, and the world grid, respectively. Another base class, UIWindow, allows separate classes to inherit and implement their own UI panels. Each panel's logic and UI are automatically updated each frame. The class inheritance structure is shown in Appendix(C.1).

#### 8.2 Visualization and UI
The UI is composed of several classes, each separated by function. An overview of the layout is shown in Appendix (C.2).
#### 8.2.1 Serial Monitor
The Serial Monitor displays raw data packets received from the robot over the 2.4GHz connection and provides controls for selecting COM ports and baud rates.
#### 8.2.2 InfoBar
The InfoBar, located across the bottom of the screen, shows information such as the mouse position in world space and the connection status to the robot.
#### 8.2.3 Graph Window
The Graph Window plots the Kalman matrix data and landmark range measurements in real time.
#### 8.2.4 ConfigWindow
The ConfigWindow allows adjustment of system parameters, including landmark coordinates and Kalman filter settings.
#### 8.2.5 KalmanFilter Visualization
The KalmanFilter Visualization draws a robot graphic based on the current Kalman estimate and renders the corresponding covariance ellipse.

#### 8.3 Core Modules
The core of the software revolves around two main functions: the OnEvent function and the OnUpdate function. The OnEvent function is called whenever a new event is created, primarily by the Serial Interface. It handles incoming data and triggers Kalman filter operations. The main event handling logic is shown in Figure [20], Code in Appendix(C.3).

<img width="541" height="496" alt="image" src="https://github.com/user-attachments/assets/e9a3ac95-1af2-4079-9a13-f65362b45461" />

The OnUpdate function is called once per application frame and manages continuous operations such as UI updates, user input, and the calculation and management of robot commands and waypoints. Its structure is shown in Figure [21], code in Appendix(C.4).

<img width="399" height="638" alt="image" src="https://github.com/user-attachments/assets/aeab95ec-298b-48b3-9acb-64f44b126381" />

#### 8.3.1 OdomKalmanFilter
The OdomKalmanFilter class implements the Extended Kalman Filter (EKF) for the localization system. It has two main functions: the predict function, which takes an encoder data packet and performs the Kalman prediction step, and the landmark update function, which takes a landmark range packet and performs a Kalman update step for the associated landmark. The update function operates on an event-based system, being triggered by serial events dispatched from the Serial Interface, using the SDL3[17] event system as the backend.

#### 8.3.2 Landmark Container
The LandmarkContainer class manages the data associated with the two landmarks, A and B. It stores the positions of the landmarks and the calibration corrections applied to the raw range measurements.
Landmark range packets are received and decoded by the Serial Interface, which then dispatches an event containing the packet. In the event handling function, the raw landmark packet is passed to the LandmarkContainer, where calibration corrections are applied. The Kalman filter update function then retrieves the corrected range measurements from the container.

#### 8.3.3 Path Controller
The PathController class manages the list of waypoints for the robot to follow. It exposes functions for selecting the next waypoint and modifying waypoints through the UI, and stores the index of the current target waypoint. It also contains the motion controller logic, which calculates the target wheel velocities needed to reach the current waypoint based on the Kalman filter’s state estimate. The computed velocities are sent to the Serial Interface for transmission to the robot.

#### 8.4 Serial Communication Interface
The SerialInterface class handles reading and decoding packets sent from the mobile robot, as well as sending command packets from the software. Packets are transmitted over a 2.4 GHz radio link to an Arduino connected to the host PC, and then transferred to the software via serial communication, using the standard outlined in Section (9.0).

<img width="484" height="508" alt="image" src="https://github.com/user-attachments/assets/354dfea7-8ec3-4884-9dc8-13c3bc36eb2e" />

To prevent the main program from being blocked by serial operations, all communication occurs on a dedicated thread. This thread is created when a serial port is opened and destroyed when the port is closed. As shown in Figure [22], the thread reads incoming packets, verifies checksums to avoid corrupt data, and dispatches events containing the validated packets. The serial thread also monitors for new outgoing command packets and writes them to the serial port accordingly.

#### 8.5 Data Flow Summary
The core data flow into the application governs how encoder and range measurements are processed to estimate the robot’s pose and generate control commands in real-time. This system is implemented using modular components that handle serial communication, localization, control, and user interface visualization. The flow of data into the application is shown in Figure[23].

<img width="471" height="411" alt="image" src="https://github.com/user-attachments/assets/7d7835af-cf2f-43fb-ae72-28fc294f4818" />

The process begins with the serial interface providing decoded data packets received from the robot. Encoder packets are used by the Kalman filter to trigger a prediction step. Landmark packets, which contain UWB range data, are first passed to the LandmarkContainer. Here, a correction is applied using precomputed calibration data detailed in Section 5.4. The corrected range data is then passed to the OdomKalmanFilter, where it contributes to the correction step of the Extended Kalman Filter (EKF). It is also sent to the UI for graphing. Additionally, Kalman state estimates, covariance matrices, and Kalman gain matrices are passed to the UI for visualization. All packets are ultimately sent to the Serial Monitor for display in decoded form, as shown in Appendix(C.2).

<img width="488" height="408" alt="image" src="https://github.com/user-attachments/assets/31d54ee5-e5ba-4d05-807a-eaf124a11a41" />

The data flow out of the software begins with the UI as shown in figure[24]. Configuration parameters are passed from UI modules to the LandmarkContainer, OdomKalmanFilter, and PathController. This allows for real-time adjustments to waypoints, landmark positions, and Kalman filter parameters. When operating in waypoint mode, the PathController dispatches velocity command packets to the serial interface, which transmits them over radio to the robot.
