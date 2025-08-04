#include "ao.h"
#include "cpuintrf.h"
#include <assert.h>
#include <malloc.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

// PACK function: Converts a 32-bit signed integer (24.8 fixed-point format) to
// a custom 16-bit floating-point format. The format is: 1-bit sign, 4-bit
// exponent, 11-bit mantissa.
static UINT16 PACK(INT32 val) {
  UINT32 temp;
  int sign, exponent, k;

  // Extract the sign bit
  sign = (val >> 23) & 0x1;
  // Calculate the absolute value for exponent calculation
  temp = (val ^ (val << 1)) & 0xFFFFFF;
  exponent = 0;
  // Find the leading '1' to determine the exponent
  for (k = 0; k < 12; k++) {
    if (temp & 0x800000)
      break;
    temp <<= 1;
    exponent += 1;
  }
  // Shift the mantissa and mask it to 11 bits
  if (exponent < 12)
    val = (val << exponent) & 0x3FFFFF;
  else
    val <<= 11;
  val >>= 11;
  val &= 0x7FF;

  // Assemble the final 16-bit value
  val |= sign << 15;
  val |= exponent << 11;

  return (UINT16)val;
}

// UNPACK function: Converts the custom 16-bit floating-point format back to a
// 32-bit signed integer.
static INT32 UNPACK(UINT16 val) {
  int sign, exponent, mantissa;
  INT32 uval;

  // Extract sign, exponent, and mantissa
  sign = (val >> 15) & 0x1;
  exponent = (val >> 11) & 0xF;
  mantissa = val & 0x7FF;

  // Reconstruct the 24-bit value from mantissa
  uval = mantissa << 11;

  // Handle special cases for the exponent
  if (exponent > 11) {
    exponent = 11;
    uval |= sign << 22;
  } else
    uval |= (sign ^ 1) << 22;

  // Apply the sign and sign-extend to 32 bits
  uval |= sign << 23;
  uval <<= 8;
  uval >>= 8;

  // Apply the exponent by shifting
  uval >>= exponent;

  return uval;
}

//the DSP Context
struct _SCSPDSP
{
//Config
	UINT16 *SCSPRAM;
	UINT32 SCSPRAM_LENGTH;
	UINT32 RBP;	//Ring buf pointer
	UINT32 RBL;	//Delay ram (Ring buffer) size in words

//context

	INT16 COEF[64];		//16 bit signed
	UINT16 MADRS[32];	//offsets (in words), 16 bit
	UINT16 MPRO[128*4];	//128 steps 64 bit
	INT32 TEMP[128];	//TEMP regs,24 bit signed
	INT32 MEMS[32];	//MEMS regs,24 bit signed
	UINT32 DEC;

//input
	INT32 MIXS[16];	//MIXS, 24 bit signed
	INT16 EXTS[2];	//External inputs (CDDA)    16 bit signed

//output
	INT16 EFREG[16];	//EFREG, 16 bit signed

	int Stopped;
	int LastStep;
};

// SCSPDSP_Init: Initializes the DSP state.
void SCSPDSP_Init(struct _SCSPDSP *DSP) {
  memset(DSP, 0, sizeof(struct _SCSPDSP));
  DSP->RBL = 0x8000; // Ring Buffer Length (default)
  DSP->Stopped = 1;  // Start in a stopped state
}

// SCSPDSP_Start: Prepares the DSP for execution by finding the last non-zero
// instruction.
void SCSPDSP_Start(struct _SCSPDSP *DSP) {
  int i;
  DSP->Stopped = 0; // Set to running state
  // Find the last instruction that isn't all zeros.
  // This optimizes the main execution loop by only iterating over the used
  // program space.
  for (i = 127; i >= 0; --i) {
    UINT16 *IPtr = DSP->MPRO + i * 4;

    if (IPtr[0] != 0 || IPtr[1] != 0 || IPtr[2] != 0 || IPtr[3] != 0)
      break;
  }
  DSP->LastStep = i + 1; // Set the number of steps to execute.
}

// SCSPDSP_SetSample: Adds an audio sample to one of the input mixers.
// SEL is the mixer index, MXL is a volume control (though the implementation
// seems to ignore it).
void SCSPDSP_SetSample(struct _SCSPDSP *DSP, INT32 sample, int SEL, int MXL) {
  // The sample is added to the specified mixer channel.
  DSP->MIXS[SEL] += sample;
}

// SCSPDSP_Step: The main DSP emulation function. Executes one full program
// loop.
void SCSPDSP_Step(struct _SCSPDSP *DSP) {
  // Declare and initialize internal DSP registers and variables.
  INT32 ACC = 0;       // 26-bit accumulator
  INT32 SHIFTED = 0;   // 24-bit shifter output
  INT32 X = 0;         // 24-bit multiplier operand X
  INT32 Y = 0;         // 13-bit multiplier operand Y
  INT32 B = 0;         // 26-bit adder operand B
  INT32 INPUTS = 0;    // 24-bit input register
  INT32 MEMVAL = 0;    // Value read from memory
  INT32 FRC_REG = 0;   // 13-bit FRC register
  INT32 Y_REG = 0;     // 24-bit Y register
  UINT32 ADDR = 0;     // Memory address
  UINT32 ADRS_REG = 0; // 13-bit address register
  int step;

  // Do not run if the DSP is stopped.
  if (DSP->Stopped)
    return;

  // Clear the effect registers at the beginning of each step.
  memset(DSP->EFREG, 0, 2 * 16);

  // Main program loop. Iterates through each instruction step.
  for (step = 0; step < DSP->LastStep; ++step) {
    // Get a pointer to the current 128-bit instruction (4x 16-bit words).
    UINT16 *IPtr = DSP->MPRO + step * 4;

    // Decode the instruction fields from the 16-bit words.
    // Instruction Word 0 (IPtr[0])
    UINT32 TRA = (IPtr[0] >> 8) & 0x7F; // TEMP Read Address
    UINT32 TWT = (IPtr[0] >> 7) & 0x01; // TEMP Write Trigger
    UINT32 TWA = (IPtr[0] >> 0) & 0x7F; // TEMP Write Address

    // Instruction Word 1 (IPtr[1])
    UINT32 XSEL = (IPtr[1] >> 15) & 0x01; // X Operand Selector
    UINT32 YSEL = (IPtr[1] >> 13) & 0x03; // Y Operand Selector
    UINT32 IRA = (IPtr[1] >> 6) & 0x3F;   // Input Read Address
    UINT32 IWT = (IPtr[1] >> 5) & 0x01;   // Input Write Trigger
    UINT32 IWA = (IPtr[1] >> 0) & 0x1F;   // Input Write Address

    // Instruction Word 2 (IPtr[2])
    UINT32 TABLE =
        (IPtr[2] >> 15) & 0x01;          // Table select for address calculation
    UINT32 MWT = (IPtr[2] >> 14) & 0x01; // Main Memory Write Trigger
    UINT32 MRD = (IPtr[2] >> 13) & 0x01; // Main Memory Read Trigger
    UINT32 EWT = (IPtr[2] >> 12) & 0x01; // Effect Memory Write Trigger
    UINT32 EWA = (IPtr[2] >> 8) & 0x0F;  // Effect Memory Write Address
    UINT32 ADRL = (IPtr[2] >> 7) & 0x01; // Address Register Load Trigger
    UINT32 FRCL = (IPtr[2] >> 6) & 0x01; // FRC Register Load Trigger
    UINT32 SHIFT = (IPtr[2] >> 4) & 0x03; // Shifter Control
    UINT32 YRL = (IPtr[2] >> 3) & 0x01;   // Y Register Load Trigger
    UINT32 NEGB = (IPtr[2] >> 2) & 0x01;  // Negate B operand
    UINT32 ZERO = (IPtr[2] >> 1) & 0x01;  // Zero B operand
    UINT32 BSEL = (IPtr[2] >> 0) & 0x01;  // B Operand Selector (ACC or TEMP)

    // Instruction Word 3 (IPtr[3])
    UINT32 NOFL =
        (IPtr[3] >> 15) & 1; // No Floating-Point conversion (direct read/write)
    UINT32 COEF = (IPtr[3] >> 9) & 0x3f; // Coefficient Index
    UINT32 MASA = (IPtr[3] >> 2) & 0x1f; // Main Memory Address Source
    UINT32 ADREB = (IPtr[3] >> 1) & 0x1; // Add Address Register to address
    UINT32 NXADR = (IPtr[3] >> 0) & 0x1; // Next Address (increment address)

    INT64 v;

    // --- EXECUTION STAGE ---

    // 1. INPUTS RW (Input Read/Write)
    assert(IRA < 0x32);
    if (IRA <= 0x1f)
      INPUTS = DSP->MEMS[IRA]; // Read from MEMS
    else if (IRA <= 0x2F)
      INPUTS = DSP->MIXS[IRA - 0x20]
               << 4; // Read from MIXS (20-bit, so left-shifted)
    else if (IRA <= 0x31)
      INPUTS = 0; // Unused addresses

    INPUTS <<= 8;
    INPUTS >>= 8; // Sign-extend the 24-bit value

    if (IWT) {
      // Write to MEMS. The value is taken from a memory read from the previous
      // step.
      DSP->MEMS[IWA] = MEMVAL;
      // If we just wrote to the address we are reading from, update INPUTS
      // immediately.
      if (IRA == IWA)
        INPUTS = MEMVAL;
    }

    // 2. Operand Selection
    // Select operand B
    if (!ZERO) {
      if (BSEL)
        B = ACC; // B = Accumulator
      else {
        B = DSP->TEMP[(TRA + DSP->DEC) & 0x7F]; // B = TEMP memory location
        B <<= 8;
        B >>= 8; // Sign-extend
      }
      if (NEGB)
        B = 0 - B; // Negate B
    } else
      B = 0; // B = 0

    // Select operand X
    if (XSEL)
      X = INPUTS; // X = INPUTS register
    else {
      X = DSP->TEMP[(TRA + DSP->DEC) & 0x7F]; // X = TEMP memory location
      X <<= 8;
      X >>= 8; // Sign-extend
    }

    // Select operand Y
    if (YSEL == 0)
      Y = FRC_REG; // Y = FRC register
    else if (YSEL == 1)
      Y = DSP->COEF[COEF] >> 3; // Y = selected coefficient
    else if (YSEL == 2)
      Y = (Y_REG >> 11) & 0x1FFF; // Y = part of Y_REG
    else if (YSEL == 3)
      Y = (Y_REG >> 4) & 0x0FFF; // Y = another part of Y_REG

    if (YRL)
      Y_REG = INPUTS; // Load Y_REG from INPUTS

    // 3. Shifter
    if (SHIFT == 0) {
      SHIFTED = ACC;
      // Clamp the 26-bit ACC to a 24-bit range
      if (SHIFTED > 0x007FFFFF)
        SHIFTED = 0x007FFFFF;
      if (SHIFTED < (-0x00800000))
        SHIFTED = -0x00800000;
    } else if (SHIFT == 1) {
      SHIFTED = ACC * 2;
      // Clamp the shifted value
      if (SHIFTED > 0x007FFFFF)
        SHIFTED = 0x007FFFFF;
      if (SHIFTED < (-0x00800000))
        SHIFTED = -0x00800000;
    } else if (SHIFT == 2) {
      SHIFTED = ACC * 2;
      SHIFTED <<= 8;
      SHIFTED >>= 8; // Shift left by 1 and sign-extend to 24 bits
    } else if (SHIFT == 3) {
      SHIFTED = ACC;
      SHIFTED <<= 8;
      SHIFTED >>= 8; // Shift left by 0 and sign-extend to 24 bits
    }

    // 4. Accumulator (Multiplication-Accumulation)
    Y <<= 19;
    Y >>= 19; // Sign-extend the 13-bit Y operand to 32 bits
    // Perform the multiplication and right-shift by 12 (fixed-point arithmetic)
    v = (((INT64)X * (INT64)Y) >> 12);
    ACC = (int)v + B; // Add B to the product and store in ACC

    // 5. Data Writes
    // Write to TEMP memory
    if (TWT)
      DSP->TEMP[(TWA + DSP->DEC) & 0x7F] = SHIFTED;

    // Load FRC_REG
    if (FRCL) {
      if (SHIFT == 3)
        FRC_REG = SHIFTED & 0x0FFF; // 12-bit
      else
        FRC_REG = (SHIFTED >> 11) & 0x1FFF; // 13-bit
    }

    // 6. Main Memory Access (SCSPRAM)
    if (MRD || MWT) {
      // Calculate the address for SCSPRAM
      ADDR = DSP->MADRS[MASA];
      if (!TABLE)
        ADDR += DSP->DEC;
      if (ADREB)
        ADDR += ADRS_REG & 0x0FFF;
      if (NXADR)
        ADDR++;
      // Apply ring buffer mask
      if (!TABLE)
        ADDR &= DSP->RBL - 1;
      else
        ADDR &= 0xFFFF;

      ADDR += DSP->RBP << 12; // Add the Ring Buffer Base Pointer offset

      // Read from SCSPRAM
      if (MRD && (step & 1)) // Memory access seems to be limited to odd steps
      {
        if (NOFL)
          MEMVAL = DSP->SCSPRAM[ADDR]
                   << 8; // Direct read, no floating-point conversion
        else
          MEMVAL = UNPACK(DSP->SCSPRAM[ADDR]); // Read and UNPACK
      }

      // Write to SCSPRAM
      if (MWT && (step & 1)) {
        if (NOFL)
          DSP->SCSPRAM[ADDR] = SHIFTED >> 8; // Direct write
        else
          DSP->SCSPRAM[ADDR] = PACK(SHIFTED); // PACK and write
      }
    }

    // Load ADRS_REG
    if (ADRL) {
      if (SHIFT == 3)
        ADRS_REG = (SHIFTED >> 12) & 0xFFF;
      else
        ADRS_REG = (INPUTS >> 16);
    }

    // Write to Effect Registers
    if (EWT)
      DSP->EFREG[EWA] +=
          SHIFTED >> 8; // Add the shifted value to the effect register
  }

  // End of step:
  --DSP->DEC;                   // Decrement the global address decrementer
  memset(DSP->MIXS, 0, 4 * 16); // Clear the input mixers for the next step
}

// A structure to hold the decoded instruction fields for clarity.
typedef struct {
  uint32_t tra;
  uint32_t twt;
  uint32_t twa;
  uint32_t xsel;
  uint32_t ysel;
  uint32_t ira;
  uint32_t iwt;
  uint32_t iwa;
  uint32_t table;
  uint32_t mwt;
  uint32_t mrd;
  uint32_t ewt;
  uint32_t ewa;
  uint32_t adrl;
  uint32_t frcl;
  uint32_t shift;
  uint32_t yrl;
  uint32_t negb;
  uint32_t zero;
  uint32_t bsel;
  uint32_t nofl;
  uint32_t coef;
  uint32_t masa;
  uint32_t adreb;
  uint32_t nxadr;
} dsp_instruction_t;

// Function to print a human-readable disassembly of a DSP instruction.
// This function mimics the decoding logic from SCSPDSP_Step.
void SCSPDSP_Disassemble(uint16_t *IPtr, int step_index) {
  dsp_instruction_t instr;

  // Decode instruction fields from the four 16-bit words
  instr.tra = (IPtr[0] >> 8) & 0x7F; // TEMP Read Address
  instr.twt = (IPtr[0] >> 7) & 0x01; // TEMP Write Trigger
  instr.twa = (IPtr[0] >> 0) & 0x7F; // TEMP Write Address

  instr.xsel = (IPtr[1] >> 15) & 0x01; // X Operand Selector
  instr.ysel = (IPtr[1] >> 13) & 0x03; // Y Operand Selector
  instr.ira = (IPtr[1] >> 6) & 0x3F;   // Input Read Address
  instr.iwt = (IPtr[1] >> 5) & 0x01;   // Input Write Trigger
  instr.iwa = (IPtr[1] >> 0) & 0x1F;   // Input Write Address

  instr.table = (IPtr[2] >> 15) & 0x01; // Table Select for address calc
  instr.mwt = (IPtr[2] >> 14) & 0x01;   // Main Memory Write Trigger
  instr.mrd = (IPtr[2] >> 13) & 0x01;   // Main Memory Read Trigger
  instr.ewt = (IPtr[2] >> 12) & 0x01;   // Effect Memory Write Trigger
  instr.ewa = (IPtr[2] >> 8) & 0x0F;    // Effect Memory Write Address
  instr.adrl = (IPtr[2] >> 7) & 0x01;   // Address Register Load Trigger
  instr.frcl = (IPtr[2] >> 6) & 0x01;   // FRC Register Load Trigger
  instr.shift = (IPtr[2] >> 4) & 0x03;  // Shifter Control
  instr.yrl = (IPtr[2] >> 3) & 0x01;    // Y Register Load Trigger
  instr.negb = (IPtr[2] >> 2) & 0x01;   // Negate B operand
  instr.zero = (IPtr[2] >> 1) & 0x01;   // Zero B operand
  instr.bsel = (IPtr[2] >> 0) & 0x01;   // B Operand Selector

  instr.nofl = (IPtr[3] >> 15) & 1;   // No Floating-Point conversion
  instr.coef = (IPtr[3] >> 9) & 0x3F; // Coefficient Index
  instr.masa = (IPtr[3] >> 2) & 0x1F; // Main Memory Address Source
  instr.adreb = (IPtr[3] >> 1) & 0x1; // Add Address Register to address
  instr.nxadr = (IPtr[3] >> 0) & 0x1; // Next Address

  // Print the decoded fields in a structured format
  printf("DSP Instruction at MPRO[%d]:\n", step_index);
  printf("  RAW: 0x%04X 0x%04X 0x%04X 0x%04X\n", IPtr[0], IPtr[1], IPtr[2],
         IPtr[3]);
  printf("  --- Memory and Data Movement ---\n");
  printf("  TEMP READ:    TRA=0x%02X\n", instr.tra);
  if (instr.twt) {
    printf("  TEMP WRITE:   TWT=1, TWA=0x%02X\n", instr.twa);
  }
  printf("  INPUT READ:   IRA=0x%02X (%s)\n", instr.ira,
         (instr.ira <= 0x1F) ? "MEMS" : "MIXS");
  if (instr.iwt) {
    printf("  INPUT WRITE:  IWT=1, IWA=0x%02X\n", instr.iwa);
  }
  if (instr.mrd || instr.mwt) {
    printf("  MAIN MEMORY:  %s%s\n", instr.mrd ? "READ " : "",
           instr.mwt ? "WRITE" : "");
    printf("    Addr Calc:  MASA=0x%02X, ADREB=%d, NXADR=%d, TABLE=%d\n",
           instr.masa, instr.adreb, instr.nxadr, instr.table);
    if (instr.nofl) {
      printf("    Data Type:  NOFL (Direct 16-bit)\n");
    }
  }
  if (instr.ewt) {
    printf("  EFFECT WRITE: EWT=1, EWA=0x%02X\n", instr.ewa);
  }

  printf("  --- Registers and ALU Operations ---\n");
  printf("  Y REGISTER:   %s, YSEL=%d (%s)\n", instr.yrl ? "LOAD" : "NO LOAD",
         instr.ysel,
         instr.ysel == 0   ? "FRC_REG"
         : instr.ysel == 1 ? "COEF"
         : instr.ysel == 2 ? "Y_REG[13:0]"
                           : "Y_REG[11:0]");
  printf("  FRC REGISTER: %s\n", instr.frcl ? "LOAD" : "NO LOAD");
  printf("  ADDR REG:     %s\n", instr.adrl ? "LOAD" : "NO LOAD");
  printf("  B OPERAND:    %s, BSEL=%d (%s) %s\n", instr.zero ? "ZERO" : "SET",
         instr.bsel, instr.bsel ? "ACC" : "TEMP", instr.negb ? "NEGATE" : "");
  printf("  X OPERAND:    XSEL=%d (%s)\n", instr.xsel,
         instr.xsel ? "INPUTS" : "TEMP");
  printf("  SHIFT OP:     SHIFT=%d (%s)\n", instr.shift,
         instr.shift == 0   ? "SATURATE"
         : instr.shift == 1 ? "SHIFT_LEFT_1_SATURATE"
         : instr.shift == 2 ? "SHIFT_LEFT_1"
                            : "NO_SHIFT");

  printf("\n");
}

void SCSPDSP_DisassembleProgram(struct _SCSPDSP *DSP) {
  int step;
  for (step = 0; step < 128; ++step) {
    SCSPDSP_Disassemble(DSP->MPRO + step * 4, step);
  }
}

/**
 * @brief Loads a DSP program from a .EXC file into the DSP program memory (MPRO).
 *
 * This function reads a text-based .EXC file, parses the "PROG" section,
 * and populates the DSP's MPRO array with the 16-bit instruction words.
 * The .EXC file format is expected to have lines like:
 * [ProgramRAM Address]:[data]
 * where [data] consists of four 4-hex-digit blocks separated by spaces (e.g., "00:xxxx xxxx xxxx xxxx").
 *
 * @param DSP A pointer to the SCSPDSP structure to load the program into.
 * @param filename The path to the .EXC file.
 * @return 0 on success, -1 on failure (e.g., file not found, format error).
 */
int SCSPDSP_LoadProgramFromEXL(struct _SCSPDSP *DSP, const char *filename) {
    FILE *file = NULL;
    char line[256]; // Buffer to read each line
    int in_prog_section = 0; // Flag to indicate if we are in the #PROG section
    int line_num = 0;

    // Open the .EXC file for reading
    file = fopen(filename, "r");
    if (file == NULL) {
        fprintf(stderr, "Error: Could not open file '%s'\n", filename);
        return -1;
    }

    // Read the file line by line
    while (fgets(line, sizeof(line), file) != NULL) {
        line_num++;

        // Remove trailing newline character if present
        line[strcspn(line, "\n")] = 0;

        // Check for section markers
        if (strcmp(line, "PROG") == 0) {
            in_prog_section = 1;
            continue; // Move to the next line
        }
        if (strcmp(line, "COEF") == 0 || strcmp(line, "ADRS") == 0 || strcmp(line, "END") == 0) {
            // If we encounter another section marker or END, and we were in PROG, stop parsing PROG.
            // If we encounter it before PROG, it's just skipping.
            if (in_prog_section) {
                break;
            }
            continue; // Move to the next line
        }

        // If we are in the PROG section, parse the instruction line
        if (in_prog_section) {
            unsigned int address;
            unsigned int word0, word1, word2, word3;
            int parsed_items;

            // Attempt to parse the line: "XX:YYYY YYYY YYYY YYYY"
            parsed_items = sscanf(line, "%x:%x %x %x %x", 
                                  &address, &word0, &word1, &word2, &word3);

            if (parsed_items == 5) {
                // Check if the address is within the valid range for MPRO
                if (address >= 0 && address < 128) {
                    DSP->MPRO[address * 4 + 0] = (UINT16)word0;
                    DSP->MPRO[address * 4 + 1] = (UINT16)word1;
                    DSP->MPRO[address * 4 + 2] = (UINT16)word2;
                    DSP->MPRO[address * 4 + 3] = (UINT16)word3;
                } else {
                    fprintf(stderr, "Warning: Line %d: Program address 0x%X out of bounds (0-127). Skipping.\n", line_num, address);
                }
            } else {
                // If parsing failed, it might be a comment or an empty line, or a malformed instruction line.
                // Ignore comments (lines starting with ';') or empty lines.
                // The dAsms manual says comments start with apostrophe (').
                if (line[0] == '\'' || line[0] == '\0' || line[0] == '\r') {
                    continue; // Skip comments and empty lines
                }
                fprintf(stderr, "Error: Line %d: Malformed PROG instruction line: '%s'. Expected 'ADDR:WORD0 WORD1 WORD2 WORD3'.\n", line_num, line);
                fclose(file);
                return -1;
            }
        }
    }

    fclose(file);
    return 0; // Success
}

int main() {
  // initialize DSP
  struct _SCSPDSP* DSP = malloc(sizeof(struct _SCSPDSP));
  // read DSP code block
  SCSPDSP_LoadProgramFromEXL(DSP, "samples/SAMPLE.EXL");
  // run disassembly
  SCSPDSP_DisassembleProgram(DSP);
}
