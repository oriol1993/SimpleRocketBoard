# SimpleRocketBoard
## Instructions to dump the flash to a .txt file via serial port
<sub>(Works only for Unix based Operating Systems)</sub>
- Identification of serial ports available: ```% ls /dev/tty.*```
- Make sure you are in the folder you wish to save the data.
- Start recording the terminal to a _.log_ file: ```% script screen.log```
- Start the serial communication: ```% screen /dev/tty.serialPortToUse 115200```
- Wait for the BMP280 to connect.
- Type a ```4``` to the serial console to trigger the data dump.
- Wait for all the data to transfer.
- Press ```Ctrl+a ; k ; y``` to finish the screen app.
- Type ```exit``` to finish the script recording.
- Check that you have actually recieved all the data by typing ```% cat screen.log```. This should output the entire textfile into your terminal window.
