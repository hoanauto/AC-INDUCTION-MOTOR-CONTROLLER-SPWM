# 🔧 "Research on Converting Internal Combustion Engine Motorcycles into Electric Motorcycles"
## ĐỒ ÁN TỐT NGHIỆP_NGUYỄN TRỌNG HOAN_06/2025 Gồm có `File thuyết minh`, `các phiên bản code`

### LINK TỔNG: https://drive.google.com/drive/u/0/folders/13GII5-7eNStuxl2eF6Wzm4rQJNyuKq-C
![image](https://github.com/user-attachments/assets/a73bfbfd-27bf-44d1-a908-74815127b3b5)
![image](https://github.com/user-attachments/assets/945c577b-f9ac-4ce2-853d-a83fc339ffc2)

## 🔍 Project Overview 
The main objective of this research is to design and develop a low-cost control circuit for a three-phase asynchronous (AC induction) motor, integrating a speed sensor to regulate PWM signals. This aims to improve the speed control and torque response of electric motorcycles, ultimately enhancing the smoothness and efficiency of vehicle operation.

An STM32F103C8 microcontroller is employed as the core processing unit, responsible for generating and modulating PWM signals to control the switching of the DC voltage. The PWM pulse width is dynamically adjusted via the microcontroller to approximate a sinusoidal waveform at the output. This control method is based on the principles of a Variable Frequency Drive (VFD), where DC voltage is converted into a near-sinusoidal AC waveform using a lookup table. The output waveform can be further refined to closely resemble a pure sine wave.

In addition, the research team has developed a water-cooling system for the IGBT module to maintain stable thermal performance and prevent overheating during operation. This cooling solution significantly improves the durability and efficiency of the power electronics, ensuring optimal performance of the entire control system.

The team also investigates and designs a gear reduction system for the electric motorcycle, aiming to deliver higher torque output. This is particularly beneficial in scenarios requiring rapid acceleration or navigating difficult terrains.

A critical aspect of this project involves designing and implementing the electrical body system for a Honda Dream motorcycle. The system includes utility functions for the rider, such as lighting control and real-time vehicle monitoring. An LCD display is integrated to show essential metrics such as speed and system status, enhancing the user’s ability to monitor and manage the vehicle effectively, while improving the overall riding experience.

Finally, the team designs and fabricates the circuit boards (PCBs) using Altium Designer. These circuits are mounted onto the Honda Dream chassis, and the assembled vehicle undergoes road testing. Performance data is collected and analyzed to refine and optimize the design for future iterations.
