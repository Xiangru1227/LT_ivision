# iVision Firmware Release Notes – Version 1.05
**Release Date:** 2025-08-13
**Applicable Hardware:** [iLT, RCore2]  
---

## Summary
iVision firmware update for API Laser Trackers using Jetson Nano module with improved Auto Lock Performance. 
---

## New Features & Enhancements
- Autolock is independent of device's camera intrinsic matrix.
- cam_calibration.json file has raw pixel values (instead of normalized values) for better readability.
- Smooth motion using high frequency step wise (configurable value) commands to the radian firmware.
- Video Streaming with higher exposure for better visibility.
- Better centroid detection in close distances when 4 LEDs are distinctly visible on the screen.

---

## Bug Fixes
- Supports both old and new versions of cam_calibration.json file. 
- If parallax calibration is performed, will write it in the new (raw pixels) format.

---

## Known Issues
- Sporadic Jog to the left side of the screen.
---

## Upgrade Instructions
1. Ensure the device is powered and connected and you are able to ping the IP address 192.168.0.168.
2. Download the firmware update tool from https://apisecure-my.sharepoint.com/:u:/r/personal/chetan_sadhu_apimetrology_com/Documents/Microsoft%20Teams%20Chat%20Files/APILaserTrackerFirmwareUpdateToolSetup.zip?csf=1&web=1&e=z7JIAT
3. After installing, open and connect the tracker using the approprite IP address.
4. Choose the Firmware Update .apiltfw extention file from your download location and click Verify and Upload.
5. Once the update is successful, restart the unit.
---

## Version Information

| Component / Module | Previous Version | New Version |
|--------------------|------------------|-------------|
| ivision            | 1.04             | 1.05        |
| radian             |                  |             |
| SDK_uservice       |                  |             |

---

##  Support
For help or questions, contact: **abhay.kulkarni@apimetrology.com** or visit (https://apimetrology.com/contact-api/).

---
