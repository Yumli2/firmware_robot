# code_domino's for Decabot
This is the firmware for Decabot robot using code_domino. Decabot is a 10 x 10 x 10 cm Arduino compatible robot. 
The objective of the project is to deliver a complete educational platform to teach kids between 4 and 99 years to program a robot using domino pieces (virtual or tangible). We are developing four different interaction models, based on the abstraction capabilities of the users in different ages. They are:
* A basic pictographic domino for young toddlers who don't read, based on RFID. The pieces represent the moving capabilities of the robot;
* A basic android application where toddlers can draw paths, translate in virtual domino pieces, and upload to the robot.
* An advanced domino for kids, based on RFID, encompassing not only the movement pieces but complex program commands, as conditionals, functions, variables and parameters.
* An advanced android application, who replicates the tangible advanced domino and can be uploaded to the robot via Bluetooth. The app also permits to backup codes, share, and consult other codes. 

The basic robot is composed of: 
* Lasercut body in acrylic or MDF 2,5mm;
* 2 5V step motor 28byj48 with ULN2003 board driver;
* Hub shield with 74HC595 Shift Register for the motors;
* Ultrassonic sensor connector in shield;
* Gyroscope MPU-6050 GY-521 connector in shield;
* 6 operation buttons;
* Buzzer;
* Servo motor controlling pen;
* RFID RC-522 sensor connector in shield;
* Bluetooth HC-05 or HC-06 connector in shield;
* Extension bus for other modules (line follower, etc.).
Instructions to build your own Decabot will be found in www.codedomino.org, and in this github project.

## Getting Started

These instructions will get you a copy of the firmware up and running on your Decabot.

### Prerequisites

You need the Arduino IDE to upload this firmware to your Arduino based Turrai robot. Also you will need the following libraries:
* SPI version 1.1.1
* MFRC522 version 1.4.5
* Ultrasonic version 3.0.0
* EEPROM version 2.0.0
* Servo version 1.1.5
* Arduino UniqueID 1.0.9

Off course you need a Decabot complete robot. See www.codedomino.org to build or buy your robot.

### Installing

To install, simply use this files inside arduino sketch folder. Don't forget to install the libraries, using arduino's IDE menu Sketch -> Install Library -> Manage Library.

## Running the basic test

To run the basic code (hello world), simply upload the code to the robot using USB, and press the button E (second from right to left looking the robot frontal). The Decabot robot will draw a 10 x 10 cm suqare with a circle inside, then will position inside of it.

### Break down into end to end tests

To program a new code with the RFID dominoes, simply give a long press in any button: The robot start move looking forward the code, and will record it in the selected space memory. To execute, short press the same button. An exemple of code to try is listed above:

```
Start - Repeat < 4 - front - left - Stop repeat - Stop
```
This code will draw a 10 x 10 cm square.

## Built With

* [Arduino IDE](http://www.arduino.cc) - The web framework used

## Contributing

Please read [CONTRIBUTING.md](https://gist.github.com/PurpleBooth/b24679402957c63ec426) for details on our code of conduct, and the process for submitting pull requests to us.

## Versioning

We use [SemVer](http://semver.org/) for versioning. 

## Authors

* **Daniel Chagas** - *Concept, Hardware, Design, Mechanics, Firmware development, App development* - [triwaca](https://github.com/triwaca)

## Main Contributors

* **Patrick Martins** - RFID reading
* **Lucas de Lima** - Hardware and Hub Shield
* **Elizabeth Furtado** - Accademic research
* **Thais Silveira** - Accademic research and Presentation
* **Yuri Lima** - Educational Testing - [brasilmaker](https://github.com/brasilmaker)
* **Arthur Cavalcante** - Drawing and Testing

See also the list of [contributors](https://github.com/CodeDomino/contributors) who participated in this project.

## Thanks

* **Gênio Azul** - Support with FUNCAP funding
* **Herbert Rocha** - Great insights on materials and forms
* **Leo Costa** - Great ideas about animes!
* **Theo Chagas** - The first Decabot owner (Decabot reg. THE0)!
* **Uirá Porã** - The first who belived the idea!
* **Leonardo (from PPGIA)** - The first who bought the idea in 2015! - [lbcosta](https://github.com/lbcosta)
* **Everybody from LAPIN Lab**
* **Vasco Furtado**

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.

## Acknowledgments

* This project received funds from Funcap - Fundação Cearense de Apoio ao Desenvolvimento Científico e Tecnológico - Edital Inovafit II 2017.
* This projetc received support from Fundação Edson Queiroz  - Diretoria de Pesquisa, Desenvolvimento e Inovação - Universidade de Fortaleza.
