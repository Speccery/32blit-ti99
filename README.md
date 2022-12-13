# 32Blit-ti99

This is an emulator for the TI-99/4A. I have tested it on three platforms:
- macos
- Pimoroni's [32blit](https://shop.pimoroni.com/products/32blit-dev-kit?variant=32337896865875)
- Pimoroni's [picosystem](https://shop.pimoroni.com/products/picosystem?variant=32369546985555)

The build system uses the conventions of the 32blit SDK, which is a nice cross platform system.

### Building
Built as is normal for 32blit SDK. The last time I built this I used the shell files I created (this is really for me to remember how this was done).

*run-cmake-pico.sh*

```
cd build.pico
cmake .. -DCMAKE_TOOLCHAIN_FILE=../../../32blit-sdk/pico.toolchain -DPICO_BOARD=pimoroni_picosystem -D32BLIT_DIR=/Users/erikpiehl/Developer/32blit-sdk
make -j7
```

### Limitations
- Frame rate on the Picosystem is lagging, the RP2040 seems not really to be fast enough. Using only one core though. The system compensates for this, so the CPU speed is kept at 3MHz but frames are dropped.
- VDP sprite collision detection missing.
- On the 32blit (and obviously the mac) it runs on full speed. 
- Sound support is still missing.
- Cartridge image is hardcoded, so only one is supported per build.
- No keyboard support to speak of, since the devices only have D-pad and some buttons. In order to select cartridge, I have mapped the D-pad left and right directions not only to joystick 1 but also to keyboard buttons 1 and 2. Yes this is not great.
- And probably many others.

32Blit boilerplate readme follows:

# 32Blit

![Build](https://github.com/32blit/32blit-boilerplate/workflows/Build/badge.svg)

This is a basic template for starting 32blit projects. It shows the basic
code layout and asset pipeline, hopefully giving folk a starting point for
any new projects.

It's based on the original `template` project from the 
[32Blit Beta](https://github.com/pimoroni/32blit-beta), with added asset
handling, and some tidying up to fit in with how I do things.

## Usage

[Use this template](https://github.com/32blit/32blit-boilerplate/generate) to
generate your own project.

* Edit the CMakeList.txt file to set the name of your project
* Edit the metadata.yml file to set the information for your project
* Edit the LICENSE file to set your name on the license
* Write lots of super cool code!

You should then be able to follow the usual build instructions.

For local builds this is:
```
mkdir build
cd build
cmake -D32BLIT_DIR=/path/to/32blit-sdk/ ..
```

Platform/Editor specific insctuctions [can be found in the main 32blit repo](https://github.com/pimoroni/32blit-beta#more-docs)
(For Visual Studio, you should follow the "Option 2" instructions, as the boilerplate does not contain a solution file)
