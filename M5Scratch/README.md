# M5Scratch
M5Scratch is Arduino IDE program whch communicate both M5Stack/M5StickC and Scratch 1.4 each other.
![Demo at M5StickC](https://gyazo.com/d8ec2ed78423488701dda4eb962741f4/raw)

Japanese information is at https://scrapbox.io/610t/M5Scratch .

# How to run
## master branch
- You must set your network SSID, PASSWD, and Scratch Host IP at network.h.
- Compile and upload M5Scratch.ino with Arduino IDE. 

## LovyanLauncher branch
- It's support M5Stack family with SD slot only (not support M5StickC).
- Install LovyanLauncher (https://github.com/lovyan03/M5Stack_LovyanLauncher) to your M5Stack.
- Compile M5Scratch.ino with Sketch -> Binary output, then create M5Scratch.bin
- Install M5Scratch.ino to / of microSD.
- Copy jpg/M5Scratch* to /jpg/ of microSD.
- Copy json/M5Scratch.json  to /json/ of microSD.
- Run LovyanLauncher -> SD Updater -> M5Scratch.

# How to add new variable and broadcast message
I will write detail later.

# TODO
- Better Scratch Host IP setting method
- New device support: M5StickC Plus, ATOM Matrix, Core 2, Wio Terminal
