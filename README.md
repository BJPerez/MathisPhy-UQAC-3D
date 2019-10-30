# Maths and Physique Game Engine Project

Project made in 2019 for the "Mathématiques et Physique pour le jeu vidéo" class at UQAC.
The project aims to realize a simple physical game engine and to use it in a simple game.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

The things you need to install the software and how to install them

- A compiler, for obvious reasons,
- CMake, it will be used to generate the files for your environment.

Note that the project has been made with Visual Studio so using it could help you running the project. If you use Visual Studio Code, don't forget to add the CMake and CMake Tools extensions.


### Installing

A step by step series of examples that tell you how to get a development environment running

1. Clone the project on your computer

#### If you have Visual Studio or Visual Studio Code

2. Right click on the folder with the project name --> Open in Visual Studio
3. Once you're inside Visual Studio, generate everything. On VS19 Build --> BuildAll. You should see CMake working in the console output. When it's done you should see "Build All succeed" written in the consol output. 
4. Run the program. On VS19 Debug --> Start. You should see the OpenGLWindow. If Start button is disabled, check if the correct item is selected in the "Select Startup Item" drop-down list (on the top of the screen, on the right of the green arrow). The selected item should be MyExe.exe.

The process is similar on Visual Studio Code.

#### If you're not on Visual Studio

2. Use CMake to generate the files for your environment.

The next steps depend on your environment but you should be able to:

3. Compile the CMake generated files.
4. Run the program.

### Running

While the program is running, press A to generate a blob : an entity made of particles linked together with spring forces, these links comport lire rods when the elasticity limit is exceeded. The first blob generated can be controlled using keyboard arrows. Pressing Q splits the controlled blob in two separated parts, and pressing S reunites all blobs' particles into a big one.

## Authors

* **BERTRAND Axel BERA22029804**
* **DE CLERCQ Romain DECR28099806**
* **DURAND Quentin DURQ17039607**
* **PEREZ Benjamin PERB02119501**

## Links
The github project : https://github.com/BJPerez/MathsPhy-UQAC

The OpenGl tutorial: https://learnopengl.com/
