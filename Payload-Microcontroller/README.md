Soooo, here is a couple minute guide to figuring out how platformIO works:

## Getting Started
Make sure you have Platformio installed from the github extensions; when you do, you will see the extension show up on the lefthand side of your window. 

Do CTRL + SHIFT + P and type in "PlatformIO Home" and click on the first popup.

When you get to the Homepage, click "Open Project" and navigate your way to the "Payload Microcontroller" folder. Since we have a platformio.ini file, the extension should detect that it is a platformio project already, so no additional setup should be needed.

If you face any errors with getting started, please reach out to your project leads or JW


## File Directory 

All programs we will be editing and coding in will be in /src; all other files are for configuration

platformio.ini is the configuration file; we will be adding any additional libraries we need into this file (I spent like 2 months trying to add Boost into platformio and then quit NUFR for doing this bullshit work, since it wouldn't compile) 

If the current configuration does not work on your end, let JW or Burke know, we will help you debug


## Code Structure in /src

There are two types of files; .h and .cpp

You will learn this in CS211, but if you haven't taken it yet, here are some key notes for C++

### General Summary
Think of .h as the skeleton and .cpp as the body of our program. .h will contain all the function declarations and the #include needed for the .cpp file to work. 

For example, the file `altimeter.h` will contain all the function definitions and includes that `altimeter.cpp` will implement.

Note that since we want to have all the includes in the `.h`, we need `#pragma once` in that file for the includes to be added in the .cpp counterpart

Also note that when you are trying to include another file, you include the .h version, not the .cpp. For example, it is `#include <altimeter.h>`, not `#include <altimeter.cpp>`


For function definitions vs implementations, here is an example

foo.h: 
`private void foo(int arg);`

foo.cpp:
```#include <foo.h>

private void foo(int arg) {
    Serial.println("Foo");
}```

Notice that the .h has no body to the methods, and the cpp file matches the parameters/return type identically. This needs to be the case for the functions to work together.

Also notice that the .cpp version includes the .h counterpart, this is also required for this to work.

## Final notes
90% of the time, we will have some kind of outline for the code structure for you guys to fill out with functions already declared. In cases we don't and you're not sure what to work on, please reach out to your project leads to figure out how things will be assigned!




