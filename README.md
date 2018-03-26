# espnow_RC_RX

Project using the ESP8266 as transmitter and receiver for RC. Done with arduino for esp8266 ide. Uses the ack-less ESPNOW protocol. It uses PPMSUM output from receiver. Receiver PPM out is D8, easily reconfigurable.

2 chan servo output or tracked vehicle 4 pwm mix is also possible.

You will need the rceiver MAC to bind the transmitter. The MAC will be displayed in arduino serial monitor after reset.