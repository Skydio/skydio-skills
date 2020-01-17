# Getting Started with the Skydio Skills SDK


## NOTE: This is now only available to approved developers.


With this guide, you'll be able to create new Skills for Skydio 2.
Skills are python scripts that run onboard the drone and provide basic UI elements inside the app.

![animation](assets/images/roof-inspection.gif)


## Skillset

We've put together a skillset of sample skills that you can use to learn how the Skills SDK works.

### Included Skills

- [Polygon Path](skillset/polygon_path.py): Fly a path in the shape of a user-defined polygon.
- [Property Tour](skillset/property_tour.py): Perform a series of cinematic motions to record a real estate video.
- [Roof Inspection](skillset/roof_inspection.py): Fly a configurable scanning pattern over the roof of a house.
- [Security Bot](skillset/security_bot.py): Follow anyone that gets within range of a home point, then return.
- [Party Mode](skillset/party_mode.py): Automatically follow subjects for 15 seconds at a time within a defined area.
- [Com Link](skillset/com_link.py): Communicate with a Skill from an external client using HTTP.
- [Remote Control](skillset/remote.py): Fly directly from a computer.

## Client

Included is a [Client](client/README.md) python module which demonstrates how to control
the vehicle and communicate with a skill directly from your computer over WiFi.
