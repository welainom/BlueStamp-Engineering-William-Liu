# Ball Tracking Robot:

I chose the Ball Tracking Robot as my main project. It uses a computer vision Python library called OpenCv to detect if the ball is present within the camera frame, and then it controls motors and navigates the robot towards the ball. The robot uses a Raspberry Pi minicomputer, cameras, DC motors, and ultrasonic sensors to detect objects that might cause a crash.

| **Engineer** | **School** | **Area of Interest** | **Grade** |
|:--:|:--:|:--:|:--:|
| William L | Monta Vista High School | Computer Science | Incoming Sophomore

![Headstone Image](William_L (1).jpg)

# First Milestone:

<iframe width="560" height="315" src="https://www.youtube.com/embed/nw1HndpI-dI?si=KehLbm4MAK9ZHV8J" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

**Summary:**
For my first milestone, I made the robot move forward and turn on command. First, I built the frame for my robot, attaching the wheels and motors to the base. Second, I connected the L298H Driver Board, the DC Motors, and the Raspberry Pi. Lastly, I wrote testing code on the Raspberry Pi to make the robot move forward and turn.

**How it Works:**
The Raspberry Pi is the central controller of the whole project. It sends command signals to the L298H Driver Board, which controls the DC Motors. Each input pin on the L298H Driver Board is connected to a General Purpose Input Output (GPIO) pin on the Raspberry Pi, and the output pins are soldered to the DC Motors. In my code, I specify which pins I want to run GPIO operations on, and these are the pins controlling the Driver Board. This way, I can control the motors using the raspberry pi.

```python
# Left Motor
in1 = 17 # Forward 
in2 = 27 # Backward
ena = 4

# Right Motor
in3 = 2 # Foward 
in4 = 3 # Backward
enb = 14

# GPIO Setup
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(ena, GPIO.OUT)
```

This code specifies which GPIO pins the L298H driver board is connected to, and which pins correspond to which motors. GPIO pins are general purpose, meaning that they need setup code and need to be defined as either input or output. In the above code, I specify the in1, in2, and ena pins as output pins. in1 and in2 control the right motor.. in3 and in4 control the right motor.

```python
def forward():
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
```

Here, I defined a forward function, which makes the robot move forward. GPIO.HIGH and GPIO.LOW specify how the motor is to move, and combinations of these commands can make it move in other ways. in1 and in2 are connected to the out1 and out2 pins on the driver board. One is set to high and one is set to low, so 5V of current comes out. ena and enb are configured to be pwm pins, which stands for pulse width modulation. PWM is useful for controlling the average power or amplitude delivered by an electrical signal. After this configuration, each pin outputs a waveform with a certain frequency. This changes how fast the motors turn.

![Headstone Image](hbriddd.png)

This is a diagram of an H-bridge configuration, which is commonly used to control DC Motors. Q1, Q2, Q3, and Q4 are switches that control current flow. The H-bridge configuration, with its four switches, allows you to control the direction of the motor by selectively turning on pairs of transistors. For example, in my project, when I want the robot to move forward, I would turn on switches Q1 and Q4, while leaving Q2 and Q3 off. 

**Parts Used:**
- Raspberry Pi 4: A small minicomputer that controls everything in this project. Wired everything to this and used it to control power to the motors.
- DC Motors: Two electrical motors that use Direct Current (DC) to produce mechanical force. Uses an electromagnet to change the direction of current in the motor.
- L298H Driver Board: A H-bridge motor driver board that is used to connect the Pi to the DC motors. A H-bridge is an electronic circuit that switches the polarity of a voltage.
- 6V Power Source: Supplies the L298H driver board with power. 
- Portable Battery Source: Supplies the Raspberry Pi with power.

![Headstone Image](resized rpi setup.gif)
![Headstone Image](hbridge (1).jpg)

Left: Raspberry Pi 4, Right: L298H Driver Board

**Challenges:**
When completing this milestone, I faced a number of challenges. Firstly, connecting the Raspberry Pi to my computer took a whole day of trial and error until it finally connected. Additionally, the wiring was a challenge. It took me a while to find out which output and input pins on the L298H Driver Board corresponded with which motor. After I finished these tasks, writing code was much simpler. 

**What’s Next:**
After this, I will work on the color detection and ball tracking component of this project, and this seems like more coding, which I am looking forward to. 

# Code:

**First Milestone Code:**

```python
import RPi.GPIO as GPIO
import cv2
import numpy as np
from time import sleep

GPIO.setwarnings(False)

# Left Motor
in1 = 17 # Forward 
in2 = 27 # Backward
ena = 4

# Right Motor
in3 = 2 # Foward 
in4 = 3 # Backward
enb = 14

GPIO.setmode(GPIO.BCM)

GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(ena, GPIO.OUT)

GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
GPIO.setup(enb, GPIO.OUT)

power_a = GPIO.PWM(ena, 20)
power_a.start(60)

power_b = GPIO.PWM(enb, 20)
power_b.start(60)

def forward():
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)

def stop():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)
    
def turn_left():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
    
def turn_right():
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)
    

while(True):
	userInput = input()
	
	if (userInput == 'c'):
		forward()
	if (userInput == 'l'):
		turn_left()
	if (userInput == 'r'):
		turn_right()
	if (userInput == 'x'):
		stop()

```

# Starter Project: Calculator:

<iframe width="560" height="315" src="https://www.youtube.com/embed/q9Mwomvro4c?si=zozlNiDIa9Zl9xtW" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

**Summary:**
For my starter project, I chose to make a calculator. It is a simple four function calculator that can use decimals and goes up to six digits. This project involved a lot of soldering and connecting parts, and it was pretty challenging and took a lot of time.

**Parts used:**
- Monolithic Capacitor: Used for energy exchange and storage
- 3V Battery: Power supply of the calculator
- Buttons and Button Caps: Detects presses and sends signals to the STC controller
- Seven Segment Displays: Allows numbers to be displayed
- STC Controller: Microchip, performs operations fast and using little power
- Circuit board: Allows the connections to be possible
- Screws and Nuts: Fastens the parts together

**How it Works:**
The STC Controller has many small pins attached to the sides, and whenever a button is pressed, it sends a signal to one of the pins. Depending on which pins are on and which pins are off, the STC Controller will send a different signal to the seven segment displays and display your operations. A seven segment display allows numbers to be displayed by controlling current flow to each of seven segments, allowing many combinations of patterns to be displayed.

![Headstone Image](sevensegment.jpeg)
(forum.arduino.cc)

This is a diagram of a seven segment display. Each switch (lettered A-DP) controls the corresponding segment on the display by allowing current to flow to that segment.

**Challenges:**
Despite this being a starter project, it still came with many challenges. Firstly, the large number of components that needed connecting required a lot of very precise soldering, which was a challenge. Additionally, the instructions were rather unclear, and I spent quite a long time figuring out which parts to solder and how to connect them. 

**What's Next:**
After completing my start project, I will start planning my main project.


<!--
# Ball Tracking Robot
<!--Replace this text with a brief description (2-3 sentences) of your project. This description should draw the reader in and make them interested in what you've built. You can include what the biggest challenges, takeaways, and triumphs from completing the project were. As you complete your portfolio, remember your audience is less familiar than you are with all that your project entails!-->

<!-- You should comment out all portions of your portfolio that you have not completed yet, as well as any instructions: -->
<!--- This is an HTML comment in Markdown -->
<!--- Anything between these symbols will not render on the published site -->
<!--
| **Engineer** | **School** | **Area of Interest** | **Grade** |
|:--:|:--:|:--:|:--:|
| William L | Monta Vista High School | Computer Science | Incoming Sophomore

**Replace the BlueStamp logo below with an image of yourself and your completed project. Follow the guide [here](https://tomcam.github.io/least-github-pages/adding-images-github-pages-site.html) if you need help.**

![Headstone Image](logo.svg)
  
# Final Milestone

**Don't forget to replace the text below with the embedding for your milestone video. Go to Youtube, click Share -> Embed, and copy and paste the code to replace what's below.**

<iframe width="560" height="315" src="https://www.youtube.com/embed/F7M7imOVGug" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

For your final milestone, explain the outcome of your project. Key details to include are:
- What you've accomplished since your previous milestone
- What your biggest challenges and triumphs were at BSE
- A summary of key topics you learned about
- What you hope to learn in the future after everything you've learned at BSE



# Second Milestone

**Don't forget to replace the text below with the embedding for your milestone video. Go to Youtube, click Share -> Embed, and copy and paste the code to replace what's below.**

<iframe width="560" height="315" src="https://www.youtube.com/embed/y3VAmNlER5Y" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

For your second milestone, explain what you've worked on since your previous milestone. You can highlight:
- Technical details of what you've accomplished and how they contribute to the final goal
- What has been surprising about the project so far
- Previous challenges you faced that you overcame
- What needs to be completed before your final milestone 

# First Milestone

**Don't forget to replace the text below with the embedding for your milestone video. Go to Youtube, click Share -> Embed, and copy and paste the code to replace what's below.**

<iframe width="560" height="315" src="https://www.youtube.com/embed/CaCazFBhYKs" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

For your first milestone, describe what your project is and how you plan to build it. You can include:
- An explanation about the different components of your project and how they will all integrate together
- Technical progress you've made so far
- Challenges you're facing and solving in your future milestones
- What your plan is to complete your project

# Schematics 
Here's where you'll put images of your schematics. [Tinkercad](https://www.tinkercad.com/blog/official-guide-to-tinkercad-circuits) and [Fritzing](https://fritzing.org/learning/) are both great resoruces to create professional schematic diagrams, though BSE recommends Tinkercad becuase it can be done easily and for free in the browser. 

# Code
Here's where you'll put your code. The syntax below places it into a block of code. Follow the guide [here]([url](https://www.markdownguide.org/extended-syntax/)) to learn how to customize it to your project needs. 

```c++
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Hello World!");
}

void loop() {
  // put your main code here, to run repeatedly:

}
```

# Bill of Materials
Here's where you'll list the parts in your project. To add more rows, just copy and paste the example rows below.
Don't forget to place the link of where to buy each component inside the quotation marks in the corresponding row after href =. Follow the guide [here]([url](https://www.markdownguide.org/extended-syntax/)) to learn how to customize this to your project needs. 

| **Part** | **Note** | **Price** | **Link** |
|:--:|:--:|:--:|:--:|
| Item Name | What the item is used for | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
| Item Name | What the item is used for | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
| Item Name | What the item is used for | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |

# Other Resources/Examples
One of the best parts about Github is that you can view how other people set up their own work. Here are some past BSE portfolios that are awesome examples. You can view how they set up their portfolio, and you can view their index.md files to understand how they implemented different portfolio components.
- [Example 1](https://trashytuber.github.io/YimingJiaBlueStamp/)
- [Example 2](https://sviatil0.github.io/Sviatoslav_BSE/)
- [Example 3](https://arneshkumar.github.io/arneshbluestamp/)

To watch the BSE tutorial on how to create a portfolio, click here. -->
