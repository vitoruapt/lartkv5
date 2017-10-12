Before you run this you have to give root permissions to Phidgets Interface:

In a new shelltype:

- lsusb

    It should tell you something like this:

    Bus 002 Device 013: ID 06c2:0045 Phidgets Inc. (formerly GLAB) PhidgetInterface Kit 8-8-8

    The numbers separated by a colon are the codes for vendor:product. Since we want to set up the rule so that all Phidgets, no matter what product, can be used without root privileges, we use the vendor code, which is 06c2.


- Navigate to /etc/udev/rules.d/.

    To make sure the Phidget udev rules are found first, we can create a file 10-persistent-usb.rules (all udev rule files need to end with .rules) and add one line to it. You can do this by right clicking in the folder and creating a new text document.

- sudo gedit 10-persistent-usb.rules:

    SUBSYSTEM=="usb", ATTRS{idVendor}=="06c2", MODE="0666", OWNER="user"


    Make sure to replace user with your user name. Your username will have no capital letters. You probably recognize the 06c2 from the vendor discussion above. We have added the match on SUBSYSTEM to search first within usb (within a possibly big database). The MODE sets read and write privileges for everyone to the device, and OWNER sets the owner to be you.

    Save the 10-persistent-usb.rules in /etc/udev/rules.d/ and then change its permissions so it can be read by all. Type in terminal:

- sudo chmod a+r /etc/udev/rules.d/10-persistent-usb.rules

    The udev rule is now set, and it just has to get read in.  To re-read and implement the rules without having to reset the daemon or reset the computer, you can use:

- sudo udevadm control --reload-rules