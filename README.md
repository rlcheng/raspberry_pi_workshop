##Raspberry Pi Workshop

###Introduction and Starting Up
Welcome to the Raspberry Pi workshop. This workshop uses Raspberry Pi 3 and Zeros. All Pies are preconfigured so that one does not need to plug in a keyboard, mouse, and monitor to use (no BYODKM here). Just have your laptop ready with SSH capabilities.

For those with a Raspberry Pi 3 in front of them, plug in the ac adapter to turn on. For those with the Pi Zero, please take the usb power cord and plug into your laptop.

All Pi’s have an ip address label on them. They should all be `10.10.1.X`, X being unique. To log in, power on the Pi. For the Pi 3, wait 15-20 seconds. For the Pi Zero, wait 30 seconds.

From your console: `$ ssh -X pi@<type ip on the board here>`
example: `$ ssh -X pi@10.10.1.40`

Since it will be your first time connecting, it’ll ask: `Are you sure you want to continue connecting?`
Type: `yes`

It will then ask for a password. The password is: `raspberry`

Everything you need today is on the table:
 - breadboard
 - LEDs
 - 330 ohm resistors
 - jumper wires

If you are curious as to how these were setup, they were all set to use the wireless connection here and with the following turned on: SSH, I2C, and SPI. For details on how to do that refer to the links in the reference at the end of this guide.

To turn off the Raspberry Pi, from console: `$ sudo shutdown -h now`
Your SSH connection will immediately end. You will see a blinking green light next to the red power light on the Pi. Please wait until the green light stops flashing before you unplug the power cable.

It is very important that you don't just unplug power. Shutdown command must be entered. This is because the Raspberry Pi file system is on an SD card and they are prown to corruption. I've experienced embedded systems similar to the Raspberry Pi lose its file system after 3-4 abrupt power offs.

###Turning on the LED using Python
We'll first learn how to connect an LED light and how to turn it on. Go through this guide: [Turning on an LED with your Raspberry Pi's GPIO Pins](https://thepihut.com/blogs/raspberry-pi-tutorials/27968772-turning-on-an-led-with-your-raspberry-pis-gpio-pins). This guide has a lot of good information on giving you the basic knowledge that I didn't feel I need to recreate.

###Turning on the LED using C code
Now that you are able to turn on an LED using Python. Let's try doing the same thing in C. This will still retain the same LED setup.

Create a new file called `ledwp.c` (from console you can use nano to create the file: `$ nano ledwp.c`) with the following content:
```C
#include <stdio.h>
#include <wiringPi.h>

#define ON  1
#define OFF 0

int main (void)
{
  if (wiringPiSetupGpio() == -1)
    return 1;

  int port = 18;
  pinMode(port, OUTPUT);

  digitalWrite(port, ON);
  delay(1000);
  digitalWrite(port, OFF);

  return 0;
}
```

Once you are done typing the code. Save the file. If had followed my recommendation of using nano, hit `Control + X`, and then `Y` to save, and enter to keep the file name the same. This will exit you out of nano.

Now we are ready to compile the code: `$ gcc ledwp.c -o ledwp -lwiringPi`
To run the code: `sudo ./ledwp`

###Wiring Pi
The C code earlier uses the Wiring Pi library to access the GPIO. The Wiring Pi library actually comes with a GPIO utility that you can call directly in the command line: `$ gpio -g write 18 1`
Notice that the LED is now on. To turn it off: `$ gpio -g write 18 0`

The Wiring Pi library uses a different pin out than what we are using in this workshop. Their pin 0 is our pin 18. With the -g flag in the command above, it uses our pin numbers. The last argument is similar to the #define in our C code earlier: 1 means on and 0 means off.

###LED in C Again
So what if you don't want to be dependent on the Wiring Pi library? Is there a way to write code to turn on and off the LED? Yes you can. You can actually do direct register access. Let's create a `led.c` file and do the following (`$ nano led.c`):
```C
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>

#define BCM2708_PERI_BASE        0x3F000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */

#define BLOCK_SIZE (4*1024)

int  mem_fd;
void *gpio_map;

volatile unsigned *gpio;

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))

#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0

void setup_io();

int main(int argc, char **argv)
{
    int port = 18;

    setup_io();

    INP_GPIO(port); // must use INP_GPIO before we can use OUT_GPIO
    OUT_GPIO(port);

    GPIO_SET = (1 << port);
    sleep(1);
    GPIO_CLR = (1 << port);

  return 0;
}

void setup_io()
{
    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
        printf("can't open /dev/mem \n");
        exit(-1);
    }

    gpio_map = mmap(
          NULL,                 //Any adddress in our space will do
          BLOCK_SIZE,           //Map length
          PROT_READ|PROT_WRITE, // Enable reading & writting to mapped memory
          MAP_SHARED,           //Shared with other processes
          mem_fd,               //File to map
          GPIO_BASE             //Offset to GPIO peripheral
    );

    close(mem_fd);

    if (gpio_map == MAP_FAILED) {
          printf("mmap error %d\n", (int)gpio_map);
          exit(-1);
    }

    gpio = (volatile unsigned *)gpio_map;
}
```

To compile this: `$ gcc led.c -o led`
To run the code: `$ sudo ./led`

###More to Explore with LEDs
Now that you are able to turn on the LED using Python and C, here are some things to try:
- What would you need to change in your code to make the led flash 5 times?
- What would you need ot change in the code to add more LEDs to turn on/off?

###References:
####Setup
http://www.makeuseof.com/tag/setup-wi-fi-bluetooth-raspberry-pi-3/
https://learn.sparkfun.com/tutorials/raspberry-pi-spi-and-i2c-tutorial#i2c-on-pi
https://www.raspberrypi.org/forums/viewtopic.php?t=128458&p=859002\
####LED
http://elinux.org/RPi_GPIO_Code_Samples
http://wiringpi.com/examples/blink/
http://wiringpi.com/the-gpio-utility/
