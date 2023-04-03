# automated-plant-watering-psoc

A home automation project for watering my plants remotely while we go on vacation! :-)


## Notes on Hardware / Software Used

This project uses the Infineon PSoc [CYC8PROTO-062-4343W](https://www.mouser.com/ProductDetail/Infineon-Technologies/CY8CPROTO-062-4343W) board's functionalities for running logic, running a webservice, ADCs etc.

Additional hardware for watering: [Capacitive Moisture Sensors, 4 Channel Relay, Watering Pumps, Tubes](https://www.amazon.com/gp/product/B093V62VXD)

For multitasking, [FreeRTOS](https://www.freertos.org/) is used.

A lot of the baseline code for this repo has been taken from [Wifi Webserver](https://github.com/Infineon/mtb-example-wifi-https-server) from Infineon's github and has been modified for the needs of this application.
