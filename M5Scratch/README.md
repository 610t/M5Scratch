# M5Scratch
M5Scratch is Arduino IDE program whch communicate both M5Stack/M5StickC/ATOM Matrix and Scratch 1.4 each other.
![Demo at M5StickC](https://gyazo.com/d8ec2ed78423488701dda4eb962741f4/raw)

Japanese information is at https://scrapbox.io/610t/M5Scratch .

# How to run
## master branch
- You must set your network SSID, PASSWD, and Scratch Host IP at network.h.
- Compile and upload M5Scratch.ino with Arduino IDE. 

## LovyanLauncher branch
- It's support M5Stack family with SD slot only (not support M5StickC/ATOM Matrix).
- Install LovyanLauncher (https://github.com/lovyan03/M5Stack_LovyanLauncher) to your M5Stack.
- Compile M5Scratch.ino with Sketch -> Binary output, then create M5Scratch.bin.
- Install M5Scratch.bin to / of microSD.
- Copy jpg/M5Scratch* to /jpg/ of microSD.
- Copy json/M5Scratch.json  to /json/ of microSD.
- Run LovyanLauncher -> SD Updater -> M5Scratch.

## Setting at Scratch
- Mouse right click "[Slider] sensor value" and check "enable remote sensor connections".

![enable remote sensor](https://gyazo.com/92b159d8b26f698c4ed261e7243800c8/raw)

- Try code below:

![First step code](https://gyazo.com/79f6991e1172e79407f1c70c8fe6c33c/raw)


# How to add new variable and broadcast message
I will write detail later.

# TODO
- Better Scratch Host IP setting method
- New device support: M5StickC Plus, Core 2, Wio Terminal
