# Advanced-Computer-Architecture-Lab-5
Labs #5: Pipe-lining the processor

It turns out potential customers are unhappy with the low performance of our processor. Although the DMA is a unique
feature, they say our baseline performance is far below our competitors.
To avoid losing the customer, management concluded that the processor should be pipelined, hoping to reduce CPI from the
current 6 cycles per instruction to 1.
Further, it was estimated that the single SRAM may be a performance bottleneck due to many structural hazards, and
therefore the new processor will follow the Harvard architecture, containing two single ported SRAMs, each the same size
as the original: one for instructions, and one for data.
You have been in charge with the design of the new micro-architecture. You'll model it in the low level simulator and verify
it against the same high level ISS from lab #1. 
To compete well, the processor should contain the following features:
1. Pipelining.
2. Branch predictor.
3. DMA support.

**SP pipelined micro architecture**
In contrast to labs 2 and 4, we'll now use a micro-architecture with two single ported SRAMs: srami for instructions, and
sramd for data. The original control states will now be pipeline stages, active simultaneously:
F0: FETCH0: Issues read command to memory to fetch the current instruction from address PC.
F1: FETCH1: Samples memory output to the inst register.
D0: DEC0: Decode the instruction register into its fields: opcode, dst, src0, src1, and immediate (sign extended).
D1: DEC1: Prepares the ALU operands.
E0: EXEC0: Executes ALU and LD operations.
E1: EXEC1: Write backs ALU and memory (ST).

Registers (basic set, you may add additional as required):

The architectural registers and cycle_counter will remain as before:
r2 – r7: [31:0]: 6 32 bit registers, holding the ISA general registers. Different from Computer Structure course, a value
 written to a register in clock cycle t, will be available only in clock cycle t+1. This affects hazard analysis.
cycle_counter[31:0]: 32 bit cycle counter.

For the micro architecture registers, we will have multiple copies, where each register will now contain a prefix, depending
on its pipeline stage:

fetch0_active, fetch1_active, dec0_active, dec1_active, exec0_active, exec1_active: 1 bit active signals.
fetch0_pc, fetch1_pc, dec0_pc, dec1_pc, exec0_pc, exec1_pc: [15:0] 16 bit program counter.
dec0_inst, dec1_inst, exec0_inst, exec1_inst [31:0] 32 bit instruction.
dec1_opcode, exec0_opcode, exec1_opcode [4:0]: 5 bit opcode.
dec1_dst, exec0_dst, exec1_dst[2:0]: 3 bit destination register index.
dec1_src0, exec0_src0, exec1_src0 [2:0]: 3 bit source #0 register index.
dec1_src1, exec0_src1, exec1_src1[2:0]: 3 bit source #1 register index.
dec1_immediate exec0_immediate, exec1_immediate [31:0]: sign extended immediate.
exec0_alu0, exec1_alu0 [31:0]: 32 bit alu operand #0.
exec0_alu1, exec1_alu0 [31:0]: 32 bit alu operand #1.
exec1_aluout[31:0]: 32 bit alu output

*Part of Advanced-Computer-Architecture-Lab in TAU. Assignment instructions were copied - All rights reserved to Tel Aviv University.
