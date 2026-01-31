# FRC 2026 Robot Code - Team 1091

This repository contains the robot code for FRC Team 1091's 2026 season robot. 

## Prerequisites

To build and run this project, you should have the following installed:

*   **WPILib 2026**: It is recommended to use the WPILib installer for the current season.
*   **Java 17**: The project is configured to use Java 17 (standard for 2026 WPILib).
*   **Gradle**: Included via the Gradle wrapper (`./gradlew`).

## Getting Started

### Cloning the Repository

```bash
git clone https://github.com/Team1091/FRC-2026-Rebuilt.git
cd FRC-2026-Rebuilt
```

### Building the Project

To compile the code and check for errors, run:

```bash
./gradlew build
```

### Running Simulation

You can run the robot code in simulation on your development machine:

```bash
./gradlew simulateJava
```
This will launch the WPILib Simulation GUI, allowing you to test logic without a physical robot.

### Deploying to the Robot

To deploy the code to a RoboRIO, ensure you are connected to the robot's network (via USB, Ethernet, or Wi-Fi) and run:

```bash
./gradlew compileAndDeploy
```