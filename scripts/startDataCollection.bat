set nSet=%1
echo on
echo "Recording data"
echo off
start yarpdatadumper --name data/set%nSet%/ft
start yarpdatadumper --name data/set%nSet%/OmegaPosition
start yarpdatadumper --name data/set%nSet%/fingerData

set /p DUMMY=Hit ENTER to continue...
echo on
echo "Connecting ports"
echo off
yarp connect /OmegaATI/ft /data/set%nSet%/ft
yarp connect /OmegaATI/omegaPosition  /data/set%nSet%/OmegaPosition
yarp connect /SkinTableTop/skin/fingertip /data/set%nSet%/fingerData 

set /p DUMMY=Hit ENTER to continue...
echo on
echo "Stopping"
echo off
yarp disconnect /OmegaATI/ft /data/set%nSet%/ft 
yarp disconnect /OmegaATI/omegaPosition  /data/set%nSet%/OmegaPosition
yarp disconnect /SkinTableTop/skin/fingertip /data/set%nSet%/fingerData 

