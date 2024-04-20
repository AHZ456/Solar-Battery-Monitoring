- adc1_example_main.cpp: is the main file of the project, contains the actual code for monitoring.
- I2C.cpp: contains the body of the functions used in the main.cpp file (Current sensor functions, voltage measurement functions, time management...etc)
- I2C.hpp: contains the prototypes of the functions, the declarations of global variables, defines, includes, and structures. 
- cert.pem,json11.cpp,json11.hpp,Kconfig.projbuild,Thingspeak.h,wifi_utils.h: are necessary files for the Thingspeak cloud library.

## IMPORTANT NOTE: the I2C.hpp file contains the defines for WIFI password and name. You will need to modify them there later while following the steps of folder 3 of the Project. 
Comments in the program code will guide you to the aforementioned defines.
