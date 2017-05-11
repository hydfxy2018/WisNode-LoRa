When start to use WisNode Lora board work with the 5V level Arduino board, we will have the trouble that between WisNode Lora board and Arduino board can not be communicated by the serial port. After testing, we found the Arduino board serial port is 5V ,and WisNode Lora board serial port is the level of 3.3V; Level mismatch between two serial ports Lead to abnormal communication.
Now RAK fixed this issue and launch the Lora-shift board, With this converter board, you can plug in the Wisnode-lora shield with Arduino shiled, like the following picture showing: 

![](http://i.imgur.com/8hCO5Hm.png)

Lora-Shift board


![](http://i.imgur.com/nQfMFf4.png)


![](http://i.imgur.com/ZawHHVe.png)


![](http://i.imgur.com/LiWdtcC.png)


Only need to use wire connected Lora-Shift board’s 5V, 3.3V, and GND on WisNode Lora board , You can easily resolve this problem.


![](http://i.imgur.com/ZlO64bH.png)
