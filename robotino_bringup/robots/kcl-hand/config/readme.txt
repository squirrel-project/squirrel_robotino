# Communicate with the Kclhand with without root permission 

# Step 1, you need to identify the vendorID and productID of your USB device. For that, use lsusb command.
$ lsusb 
# Get the maxon device information
# For example
# Bus 001 Device 012: ID 0403:a8b0 Future Technology Devices International, Ltd 
# Then idVender = 0403, idProduct = a8b0

# Step 2, Create a new udev rule as follows.
$ sudo vi /etc/udev/rules.d/50-myusb.rules
# SUBSYSTEMS=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="a8b0", GROUP="users", MODE="0666"
# Replace "idVendor" and "idProduct" values with your own. MODE="0666" indicates the preferred permission of the USB device.

# Step 3, Reboot your machine or reload udev rules:
$ sudo udevadm control --reload
# Done! You can verify the permission of the USB device.


