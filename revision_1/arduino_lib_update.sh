# Updates the libraries in the arduino folder
# Only temporary until another solution is figured out

# Go to libarary directory
cd ~/Documents/Arduino/libraries/

# Checkout only module code
svn checkout https://github.com/The1TrueJoe/Drive-Modules/trunk/revision_1/std_modules

# Go into the new folder and include full submodule repos
cd std_modules

if [ -d "arduino-mcp2515" ]; then
    cd arduino-mcp2515
    git pull
    cd ..
else 
    git clone https://github.com/GSSM-AutoGolfCart/arduino-mcp2515
fi

if [ -d "arduino-mcp4xxx" ]; then
    cd arduino-mcp2515
    git pull
    cd ..
else 
    git clone https://github.com/GSSM-AutoGolfCart/arduino-mcp4xxx
fi

if [ -d "ArduinoCore-avr" ]; then
    cd ArduinoCore-avr
    git pull
    cd ..
else 
    git clone https://github.com/GSSM-AutoGolfCart/ArduinoCore-avr
fi

echo "Done!"