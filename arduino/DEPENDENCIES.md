# Arduino dependencies

Install Adafruit BNO055 and Adafruit Unified Sensor through the Arduino library manager.
Install the `microM` library for the DAGU Micro Magician Controller used by the sketch.
The available microM archive attributes Russell Cameron but contains no explicit license file, so its sources are not redistributed in this public bundle. Obtain the library supplied for the controller, or supply the motor adapter for your own board.

Reference sketch: V24 SOFT_GUARD + CMD_VEL_REJECTED, 348 ticks/revolution. Its presence in the recovered project does not prove that this exact variant is currently flashed on the robot.
UART is 115200 baud. Commands are `CMD_VEL,left_mps,right_mps`. This is not the early 19200-baud, 0–240 command protocol.
