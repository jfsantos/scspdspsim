This is an adaptation of the SCSP DSP interpreter implemented in [aosdk](https://github.com/nmlgc/aosdk) with added debugging information to help understand DSP code dumps from the Linker tool provided in the Sega Saturn SDK. It does a disassembly of sorts by explaining what each step in the program does (since SEGA never released the instruction set or operands for the DSP, I guess this is the second best thing we can have).

Future features:
- [ ] Test DSP code on PCM audio: this is mostly implemented except that we currently do not allocate the SCSP RAM buffer or have anything in it. Need to figure out if we need anything in it except for audio samples.
- [ ] Run DSP code step-by-step so we can look at the values of registers

