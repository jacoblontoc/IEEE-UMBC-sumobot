# Sumobot Documentation

This is a comprehensive documentation of the sumobot created by Jacob Lontoc, Isaac Boteler, and Tommy Pham

![Completed sumobot](<Photos and Videos/after assembly.JPG>)

## Table of Contents

[Project Rules](##Guidelines)

[Soldering & Assembly](##Assembly)

[Programming](##Programming)

[Conclusion](##Conclusion)

## Guidelines

## Assembly

The images below show the robot during and after assembly.

![Before assembly](<Photos and Videos/before assembly.JPG>)

![Isometric view of the assembled robot](<Photos and Videos/isometric photo.JPG>)

![Top-down view of the robot](<Photos and Videos/topdown view.JPG>)

### Parts

There are an arrangement of modular parts

#### Front Sensor Array (line and proximity sensors)

![Front sensor array](<Photos and Videos/front sensor array.JPG>)

For line sensors, there are five line sensors that face straight downwards to help the Zumo distinquish between light and dark surfaces.

This means that the Zumo cannot detect RGB or luminosity values. It only detects colour based off the how much of it's own IR light bounces back to it.

> White/Light surfaces: Reflect most of the IR light back, resulting in a **high reflectance** value
> Black/Dark surfaces: Absorb the IR light, resulting in a **low reflectance** value.

There are also three proximity sensors that face directions away from the Zumo and hep detect nearby objects. The proximity sensors work of IR light, but they are designed to only detect light that is bounces back at the 38kHz frequency.

Based on the speed it takes for the receiver to get the bounced back IR signal, it will use that time to determine how far away the object or opponent was.

![Proximity sensors](<Photos and Videos/proximity sensors.JPG>)



#### Motors

Our Zumo utilizes **1:75 motors**, which offer a mixture of speed and physical pushback when being pushed back.

## Programming



## Conclusion