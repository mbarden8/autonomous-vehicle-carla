# CMDA Capstone Project: Self-Driving Vehicle w/ CARLA

## CARLA Installation

This project uses CARLA 0.8.4 for its work. You can download a zip file of the binary with the following link:

[CARLA_0.8.4_Windows_Download.zip](https://d18ky98rnyall9.cloudfront.net/uuTN7y7rEemnrA4AsaAhFA_bbb340f02eeb11e9a59e73356fd63643_CarlaUE4Windows.zip?Expires=1670630400&Signature=e~GmXO3ZIWv6SZ3HmTAaPqpvi0roH7THQ1TxmFszSMJyR5LaHTEgyyNYbRP2nhOyK-SURIXE~cerD~iNwFH2fpcUaBj~yh~XB7sI70WGEkTDcmyR0v1cnRQRlUfMBouWNEYyZWotQrctEhGVQ9aPn9TTR6xKQYRdmazCabPFZMY_&Key-Pair-Id=APKAJLTNE6QMUY6HBC5A)

**Note: this is a Windows 10 binary. We were unable to work CARLA on an EC2 linux instance.**

After the zip file is finished installing, unzip it to your directory of choice. For the remainder of the README, we will assume you have the CarlaUE4.exe executable under C:/CarlaSimulator/

## Ensure CARLA installation is working properly

To ensure the CARLA installation worked correctly, open up a terminal and navigate to where you have CARLA installed (C:/CarlaSimulator/)

From the command line, type:

CarlaUE4.exe -windowed -carla-no-networking

This should load up the default town map of CARLA.

## Configure Python

We were only able to work our scripts with Python 3.6. If you don't have python 3.6 downloaded, install it and add it to your system's PATH variable.

Install all python requirements by navigating to C:/CarlaSimulator/ and running this command:


python -m pip install -r requirements.txt --user

This will install all dependencies we will use in our project.

## Test basic python script with CARLA

Navigate to the folder C:/CarlaSimulator/PythonClient and then run the following command:

python manual_control.py

This should boot up that town map we saw earlier and you should be able to control your vehicle with your arrow keys (GTA style).

## Running our script with CARLA

### Boot up Race Track Map

Inside the directory C:/CarlaSimulator/, enter the following command:

CarlaUE4.exe /Game/Maps/RaceTrack -windowed -carla-server -benchmark -fps=30

This will load up the race track map at 30 frames per second. As of right now, CARLA is in server mode, waiting for you to interact with it in a python script. Let's do that now.

### Running the script

Download the source code files here. Then create a new directory under C:/CarlaSimulator/PythonClient, you can name this directory as you please, just make sure to place all files from this repo inside that directory.

Now, downgrade your version of matplotlib to 3.0.0, after doing so you should be able to run our script fine.

Navigate to the folder where the source code is and enter the following command:

python module_7.py

This should cause the vehicle to start moving around on its own and driving down the race track!

Feel free to play with the code and modify the vehicle's controller algorithm. Have fun!

## Some Lit Review on the CARLA Environment

[CARLA Overview](http://proceedings.mlr.press/v78/dosovitskiy17a/dosovitskiy17a.pdf)
