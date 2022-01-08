# Programming in Bare-Metal approach with GNU-GCC TOOLCHAIN

This is a very basic scheleton of a development enviromnent for the STM32F4 microcontrollers family.<br>
Starting with this basic project is possible to learn and build some skills in using the GNU-GCC toolchain for developing embedded firmware<br>

## Tools

- 1) Make
- 2) GCC
- 3) GDB
- 4) ST-LINK utilities (set of tools used to flash the code on your board)
- 5) OpenOCD (tool used to open a gdb server on the ST-LINK programmer, mandatory in order to debug code remotely from your pc to the board)
- 6) STM32F4 board (physical board used for in-field testing)
- 7) Renode (used as simulator of the physical system)
- 8) Terminal

(every IDE on the market makes use of this set of tools, but here you have full control of them, for free)

## Project description

This code is the most basic possible, it just makes the LED2 blink with a certain period.
The difficult part of this project is indeed all the part relative to the usage of the Makefile, linker script, and all other tools (like GDB) that are mandatory to be learned for a bare-metal approach.

[documentation](./docs) you can find all manuals and schematics of the board.<br>
[libraries](./inc) header files from ST-Microelectronics. This is the place also for custom header files.<br>
[binaries](./build) here you can find all object files (result of the compilation and linking process, look at **makefile** for further infos).
**There are two important files, .elf and .bin.**
The .elf is executable version of your code, this file must be used by gdb for debugging purpose.<br>
The .bin file which is the pure binary version of your application, this file is the one that is going to be flashed in the physical board.<br>
[source files](./src) this is the place for the source files (C code or even ASM (ARM) code should be placed here).
There are two files that basically contain the startup code for your board. **Do not Edit Them if you are not aware of what are you doing!!**<br>
[main.c](./main.c) this is the main() of your application.<br>
[renode script](./renode_sim.resc) this is the configuration script for Renode.<br>
[linker script](./LinkerScript.ld) this is the linker script used by gcc for linking phase, is is the basic version that you can find in the GCC-ARM-NONE-EABI toolchain (cross-compiler), with some little adaptations for our specific board. **Do not touch this file if you are not aware of what you doing!!**
[makefile](./Makefile) shell usage:<br>
1) *make*: compiles the code and generates .bin, .elf files.<br>
2) *make install*: compiles and flashes the binary (.bin) on the board through the ST-LINK connection.<br>
3) *make connect*: starts a gdb server on localhost:3333 port with your board, this is used to allow gdb to connect to that port and start a remote debug session. **see next chapter**<br>
4) *make sim*: starts gdb server on localhost:3333 port, this time not on the board but on the simulator Renode.<br>
5) *make clean*: cleans every file in build directory, this procedure should be done everytime a new compilation is required..<br>
<br>

## GDB SERVER

When your code is correcly flashed on the board, or on Renode simulator, it is possible to start a GDB remote debug session, remember to use *make connect* command if you intend to debug on the physical board.<br>
The following commands are sufficient, remember to change the paths if necessary..<br>
1) open GDB on .elf file.<br>
    <*path*>/arm-none-eabi-gdb -tui <*path*>/LedSwitch/build/LED2_switch.elf <br>
    if you are not willing to use any GUI just don't use *-tui* option<br>

2) After step 1, you should be in gdb program. what now? instruct gdb to start debugging on the remote target by typing this command in the gdb's prompt: <br>
*target extended localhost:3333*<br>
<br>
That's it, now you are debugging on the target :).
