## First Lego League code
Code for our robotics team using LEGO SPIKE Prime and <a href="https://github.com/sanjayseshan/spikeprime-vscode">spikeprime-vscode</a>.\
The program deals with all of the logic and creates a simple to use robot control environment.\
It's quite simple, even our 4th graders use it!\
**(This readme is early stage and is missing a lot of critical information.)**

### How do I use it?
1. Copy ```main.py``` or clone repository.
2. Install the Spike Prime vscode extension.
3. Search for **```TODO:```** in the ```main.py``` and modify accordingly.
   * This involves changing PORT values and additional info.
4. Please read <a href="https://github.com/chm46e/FirstLegoLeague/edit/master/README.md#what-do-i-need-to-know">What do I need to know?</a>
   * Trust me, this is suuuper required.
6. Connect to the robot. (<a href="https://github.com/chm46e/FirstLegoLeague/edit/master/README.md#how-do-i-connect-to-the-robot">How do I connect to the robot?</a>)
7. Upload it to the robot. (look at the top-right buttons)

### What do I need to know?
(optional: more detailed description <a href="https://github.com/chm46e/FirstLegoLeague/edit/master/README.md#closer-look-at-the-program">here</a>)\
When you launch the program, it goes to a internal menu. You can move around in the internal menu just like in the external menu.\
The internal menu looks identical to external. (FIX: should be changed)\
You can get out of there with holding (minimal) both left and right buttons.
TODO

In a nutshell of the code:
* class API at line 39 to 456:\
This contains all of the logic on robot control. This can be completely ignored.
* main() at line 456 to 532:\
This contains the loop. TODO
* exec_0() exec_1() etc at line 532 to setup:

### Closer look at the program
The internal menu gives us direct control over the and has a slight performance improvement.\
TODO

### How do I connect to the robot?
#### Windows & bluetooth
It should work in windows right away without any weird issues.\
Just pair to the robot and in vscode, some COM devices should appear.\
Try them all, the right one will beep.

#### Windows & usb
Haven't used it. Probably is super simple.

#### Linux & bluetooth
With linux it's a bit more tricky.\
First pair to the robot through your regular interface.\
Then you need to figure out the bluetooth address of your robot.\
It looks something like this: ```XX:XX:XX:XX:XX:XX```

Now the most **important** part:\
You have to create a rfcomm device with this command: (replace bluetooth address)\
```sudo rfcomm bind /dev/rfcomm0 XX:XX:XX:XX:XX:XX 1```\
**NB!** You need to execute this command every time you **reboot**!!\
You probably need to install: ```bluez-deprecated```

Now in the vscode spike connect menu, clicking ```/dev/rfcomm0``` should work.

#### Linux & usb
Once you plug the usb to the robot, the linux kernel should create a device ```/dev/ttyACM0``` or similar.\
Through vscode menu, select ```/dev/ttyACM0``` and it should connect right away without issues.

#### macOS
Both types should be very similar to Linux.

