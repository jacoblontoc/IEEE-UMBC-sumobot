# IEEE UMBC Sumobot

Team project "Creatine Crackheads" for the IEEE Region 1 and Region 2 Joint Student Conference, Spring 2026.

This repository contains the code, documentation, and development notes for our Zumo 32U4 sumobot. The project placed #1 in the competition and finished ahead of 15 other teams.

## Team

- Isaac Boteler
- Jacob Lontoc
- Tommy Pham

## Project Overview

Our robot was built around the Zumo 32U4 platform for a 30-inch sumo ring. The software combines quick opponent detection, boundary awareness, aggressive cross-ring movement, and close-range attack behavior tuned for competition.

The repository is organized to preserve both the final code and the development process:

- `sumobot-starter-code/`: the current Arduino sketch and supporting sensor header.
- `documentation/`: project write-up, hardware notes, and media.
- `meeting-notes/`: dated planning and research notes from the build process.

## Robot Behavior Summary

The current sketch uses a simple state-driven competition flow:

- A countdown followed by a directional opening scan.
- A cross-ring drive to force early contact.
- An aggressive bounce-based search pattern when the opponent is not detected.
- A multi-phase attack and reacquire routine when the opponent is in range.
- Boundary detection to keep the robot inside the ring.

## Meeting Times

Every Friday between 1:00 and 3:00  
Location: Library

<details>
<summary>Dates</summary>
<ul><li>Pre-meeting - Jan 28th X </li>
<li>Meeting 1 - Feb 6th </li>
<li>Meeting 2 - Feb 13th  </li>
<li>Meeting 3 - Feb 20th </li>
<li>Meeting 4 - Feb 27th  </li>
<li>Meeting 5 - Mar 6th  </li>
<li>Meeting 6 - Mar 13th  </li>
<li>Meeting 7 - Mar 20th  </li></ul>
</details>

## Development Checklist

The original summarized checklist is kept here as part of the project record:

- [ ] Soldering Robot
- [ ] Creating state machine
- [ ] Programming basic states
- [ ] Programming attack state
- [ ] Programming search state
- [ ] Programming defense state?

## Key Dates

Day END - Mar 19th

Day COMPETITION - Mar 20th

## Additional Documentation

- [Project documentation](documentation/project-documentation.md)
- [Meeting note 01](meeting-notes/meeting-01-2026-01-28.md)
- [Meeting note 02](meeting-notes/meeting-02-2026-02-06.md)
- [Meeting note 03](meeting-notes/meeting-03-2026-02-13.md)