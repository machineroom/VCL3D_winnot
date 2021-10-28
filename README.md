James Wilson mucking about with port to Mac/Linux, i.e. anything not windows, of github.com:MarekKowalski/LiveScan3D.git (JUST the client)
Started from the master branch. We might want to re-sync with the AzureKinect branch.  
Uses freenect2 to talk to the xbox one kinect (aka kinect2)  

Building on Mac (Big Sur 11.6):
- Install libfreenect2 from source:
    - `git clone git@github.com:OpenKinect/libfreenect2.git`
    - `cd libfreenect2; mkdir build; cd build`
    - `cmake ..`
    - `make`
    - `make install`
- Install a bunch of stuff with brew: (at least) opencv, glfw, zstd
- Return to this code
- Make a `build` directory and cd into it
- You'll likely need to `export PKG_CONFIG_PATH="/usr/local/opt/opencv/lib/pkgconfig:/usr/local/lib/pkgconfig"` so cmake can find the dev packages
- run `cmake ..` and if all good then make
- If all goes well you'll get a `client` executable

Building on Linux (Ubuntu 21.04):
- Very similar to Mac, but use apt instaead of brew and you might need to set PKG_CONFIG_PATH to just `/usr/local/lib/pkgconfig`

Special stuff
- Linux: may need to run client as root to access usb device
 
Useful stuff
- https://ed.ilogues.com/Tutorials/kinect2/kinect3.html
- http://www.bryancook.net/2014/03/mapping-between-kinect-color-and-depth.html
- Sometimes the kinect will lock-up and restarting the app won't find the device. In this case hot-plug the USB device or reset it (usbreset)

Running livescan server on Linux:
 - Follow steps [here](https://www.dedoimedo.com/computers/wine-dotnet-mono.html) to install wine (tested on Ubuntu 21.04). Install mono-complete, winetricks, etc.
 - Build the server code (probably on windows) or grab exe from [github release](https://github.com/MarekKowalski/LiveScan3D/releases/tag/v1.0.1)
 - wine LiveScanServer.exe
 

# LiveScan3D #
LiveScan3D is a system designed for real time 3D reconstruction using multiple AzureKinect or Kinect v2 depth sensors simultaneously at real time speed. The code for working with Kinect v2 is in the master branch, and the v1.x.x releases. If you want to work with Azure Kinect please use the appropriately named branch.

For both sensors the produced 3D reconstruction is in the form of a coloured point cloud, with points from all of the Kinects placed in the same coordinate system. The point cloud stream can be visualized, recorded or streamed to a HoloLens or any Unity application. The code for streaming to Unity and HoloLens is available in the [LiveScan3D-Hololens repository](https://github.com/MarekKowalski/LiveScan3D-Hololens).

Possible use scenarios of the system include:
  * capturing an object’s 3D structure from multiple viewpoints simultaneously,
  * capturing a “panoramic” 3D structure of a scene (extending the field of view of one sensor by using many),
  * streaming the reconstructed point cloud to a remote location,
  * increasing the density of a point cloud captured by a single sensor, by having multiple sensors capture the same scene.

You will also find a short presentation of LiveScan3D in the video below (click to go to YouTube):
[![YouTube link](http://img.youtube.com/vi/9y_WglwpJtE/0.jpg)](http://www.youtube.com/watch?v=9y_WglwpJtE)

In our system each sensor is governed by a separate instance of a client app, which is connected to a server. The client apps can either run on separate machines or all on the same machine (only for Azure Kinect). The server allows the user to perform calibration, filtering, synchronized frame capture, and to visualize the acquired point cloud live.

## How to use it ##
To start working with our software you will need a Windows machine with and at least a single Kinect device. You can either build LiveScan3D from source, for which you will need Visual Studio 2019, or you can download the binary release.
Both the binary and source distributions contain a manual (in the docs directory) which contains the steps necessary to start working with our software (it won't take more than a couple of minutes to set up).

## Where to get help ##
If you have any problems feel free to contact us: Marek Kowalski <m.kowalski@ire.pw.edu.pl>, Jacek Naruniec <j.naruniec@ire.pw.edu.pl>. We usually answer emails quickly (our timezone is CET).

For details regarding the methods used in LiveScan3D you can take a look at our article: [LiveScan3D: A Fast and Inexpensive 3D Data Acquisition System for Multiple Kinect v2 Sensors](https://www.researchgate.net/publication/308807023_Livescan3D_A_Fast_and_Inexpensive_3D_Data_Acquisition_System_for_Multiple_Kinect_v2_Sensors).

## Licensing ##
While all of our code is licensed under the MIT license, the 3rd party libraries have different licenses:
  * nanoflann - https://github.com/jlblancoc/nanoflann, BSD license
  * OpenCV - https://github.com/Itseez/opencv, 3-clause BSD license
  * OpenTK - https://github.com/opentk/opentk, MIT/X11 license
  * ZSTD - https://github.com/facebook/zstd, BSD license
  * SocketCS - http://www.adp-gmbh.ch/win/misc/sockets.html

If you use this software in your research, then please use the following citation:

Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 3D Data
Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 2015 International Conference on, Lyon, France, 2015

## Authors ##
  * Marek Kowalski <m.kowalski@ire.pw.edu.pl>, homepage: http://home.elka.pw.edu.pl/~mkowals6/
  * Jacek Naruniec <j.naruniec@ire.pw.edu.pl>, homepage: http://home.elka.pw.edu.pl/~jnarunie/
