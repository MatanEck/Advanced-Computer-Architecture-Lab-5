#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <ctype.h>
#include "llsim.h"


#define sp_printf(a...)						\
	do {							\
		llsim_printf("sp: clock %d: ", llsim->clock);	\
		llsim_printf(a);				\
	} while (0)
/***************************************************************

    Submitted by:
    Matan Eckhaus Moyal
    Bar Kachlon

****************************************************************/
// our section: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// DMA states //
#define DMA_STATE_IDLE     0
#define DMA_STATE_FETCH    1
#define DMA_STATE_EXEC     2

int nr_simulated_instructions = 0;
FILE* inst_trace_fp = NULL, * cycle_trace_fp = NULL;
int dma_begin = 0;
int memory_status = 0;//
int branch_lookup_table[1024]; //Branch History Table for branch prediction
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

typedef struct sp_registers_s {
    // 6 32 bit registers (r[0], r[1] don't exist)
    int r[8];

    // 32 bit cycle counter
    int cycle_counter;

    // fetch0
    int fetch0_active; // 1 bit
    int fetch0_pc; // 16 bits

    // fetch1
    int fetch1_active; // 1 bit
    int fetch1_pc; // 16 bits

    // dec0
    int dec0_active; // 1 bit
    int dec0_pc; // 16 bits
    int dec0_inst; // 32 bits

    // dec1
    int dec1_active; // 1 bit
    int dec1_pc; // 16 bits
    int dec1_inst; // 32 bits
    int dec1_opcode; // 5 bits
    int dec1_src0; // 3 bits
    int dec1_src1; // 3 bits
    int dec1_dst; // 3 bits
    int dec1_immediate; // 32 bits

    // exec0
    int exec0_active; // 1 bit
    int exec0_pc; // 16 bits
    int exec0_inst; // 32 bits
    int exec0_opcode; // 5 bits
    int exec0_src0; // 3 bits
    int exec0_src1; // 3 bits
    int exec0_dst; // 3 bits
    int exec0_immediate; // 32 bits
    int exec0_alu0; // 32 bits
    int exec0_alu1; // 32 bits

    // exec1
    int exec1_active; // 1 bit
    int exec1_pc; // 16 bits
    int exec1_inst; // 32 bits
    int exec1_opcode; // 5 bits
    int exec1_src0; // 3 bits
    int exec1_src1; // 3 bits
    int exec1_dst; // 3 bits
    int exec1_immediate; // 32 bits
    int exec1_alu0; // 32 bits
    int exec1_alu1; // 32 bits
    int exec1_aluout;
	// our section: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // DMA //
    int dma_state; // dma_state indicates the current DMA state(waiting/idle,Read,Write) 
    int dma_length;//  The DMA Data length 
    int dma_status;// Busy Flag for DMA (1 or 0)
    int dma_src;// DMA source address - 32 bit 
    int dma_dst;  //DMA Destination address.
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

} sp_registers_t;

/*
 * Master structure
 */
typedef struct sp_s {
    // local srams
#define SP_SRAM_HEIGHT	64 * 1024
    llsim_memory_t* srami, * sramd;

    unsigned int memory_image[SP_SRAM_HEIGHT];
    int memory_image_size;

    int start;

    sp_registers_t* spro, * sprn;
} sp_t;

static void sp_reset(sp_t* sp)
{
    sp_registers_t* sprn = sp->sprn;

    memset(sprn, 0, sizeof(*sprn));
}

/*
 * opcodes
 */
#define ADD 0
#define SUB 1
#define LSF 2
#define RSF 3
#define AND 4
#define OR  5
#define XOR 6
#define LHI 7
#define LD 8
#define ST 9
#define JLT 16
#define JLE 17
#define JEQ 18
#define JNE 19
#define JIN 20
#define NOP 23// Hazard Stop DMA 
#define HLT 24
#define DMA 25//Initiate DMA process
#define POL 26// Poll DMA process status

static char opcode_name[32][4] = { "ADD", "SUB", "LSF", "RSF", "AND", "OR", "XOR", "LHI",
                 "LD", "ST", "U", "U", "U", "U", "U", "U",
                 "JLT", "JLE", "JEQ", "JNE", "JIN", "U", "U", "U",
                 "HLT", "DMA", "POL", "NOP", "U", "U", "U", "U" };

// our section: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void print_to_trace(sp_t* sp, int extracted_line_number);
void dma_ctl(sp_t* sp);// DMA
void init_lookup_table();
void stall_pipline_to_avoid_hazard(sp_t* sp);
int check_input(int dst, int src0, int src1);
void execute_alu(sp_t* sp, int alu0, int alu1);
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
static void dump_sram(sp_t* sp, char* name, llsim_memory_t* sram)
{
    FILE* fp;
    int i;

    fp = fopen(name, "w");
    if (fp == NULL) {
        printf("couldn't open file %s\n", name);
        exit(1);
    }
    for (i = 0; i < SP_SRAM_HEIGHT; i++)
        fprintf(fp, "%08x\n", llsim_mem_extract(sram, i, 31, 0));
    fclose(fp);
}

static void sp_ctl(sp_t* sp)
{
    sp_registers_t* spro = sp->spro;
    sp_registers_t* sprn = sp->sprn;
    int i;

    fprintf(cycle_trace_fp, "cycle %d\n", spro->cycle_counter);
    fprintf(cycle_trace_fp, "cycle_counter %08x\n", spro->cycle_counter);
    for (i = 2; i <= 7; i++)
        fprintf(cycle_trace_fp, "r%d %08x\n", i, spro->r[i]);

    fprintf(cycle_trace_fp, "fetch0_active %08x\n", spro->fetch0_active);
    fprintf(cycle_trace_fp, "fetch0_pc %08x\n", spro->fetch0_pc);

    fprintf(cycle_trace_fp, "fetch1_active %08x\n", spro->fetch1_active);
    fprintf(cycle_trace_fp, "fetch1_pc %08x\n", spro->fetch1_pc);

    fprintf(cycle_trace_fp, "dec0_active %08x\n", spro->dec0_active);
    fprintf(cycle_trace_fp, "dec0_pc %08x\n", spro->dec0_pc);
    fprintf(cycle_trace_fp, "dec0_inst %08x\n", spro->dec0_inst); // 32 bits

    fprintf(cycle_trace_fp, "dec1_active %08x\n", spro->dec1_active);
    fprintf(cycle_trace_fp, "dec1_pc %08x\n", spro->dec1_pc); // 16 bits
    fprintf(cycle_trace_fp, "dec1_inst %08x\n", spro->dec1_inst); // 32 bits
    fprintf(cycle_trace_fp, "dec1_opcode %08x\n", spro->dec1_opcode); // 5 bits
    fprintf(cycle_trace_fp, "dec1_src0 %08x\n", spro->dec1_src0); // 3 bits
    fprintf(cycle_trace_fp, "dec1_src1 %08x\n", spro->dec1_src1); // 3 bits
    fprintf(cycle_trace_fp, "dec1_dst %08x\n", spro->dec1_dst); // 3 bits
    fprintf(cycle_trace_fp, "dec1_immediate %08x\n", spro->dec1_immediate); // 32 bits

    fprintf(cycle_trace_fp, "exec0_active %08x\n", spro->exec0_active);
    fprintf(cycle_trace_fp, "exec0_pc %08x\n", spro->exec0_pc); // 16 bits
    fprintf(cycle_trace_fp, "exec0_inst %08x\n", spro->exec0_inst); // 32 bits
    fprintf(cycle_trace_fp, "exec0_opcode %08x\n", spro->exec0_opcode); // 5 bits
    fprintf(cycle_trace_fp, "exec0_src0 %08x\n", spro->exec0_src0); // 3 bits
    fprintf(cycle_trace_fp, "exec0_src1 %08x\n", spro->exec0_src1); // 3 bits
    fprintf(cycle_trace_fp, "exec0_dst %08x\n", spro->exec0_dst); // 3 bits
    fprintf(cycle_trace_fp, "exec0_immediate %08x\n", spro->exec0_immediate); // 32 bits
    fprintf(cycle_trace_fp, "exec0_alu0 %08x\n", spro->exec0_alu0); // 32 bits
    fprintf(cycle_trace_fp, "exec0_alu1 %08x\n", spro->exec0_alu1); // 32 bits

    fprintf(cycle_trace_fp, "exec1_active %08x\n", spro->exec1_active);
    fprintf(cycle_trace_fp, "exec1_pc %08x\n", spro->exec1_pc); // 16 bits
    fprintf(cycle_trace_fp, "exec1_inst %08x\n", spro->exec1_inst); // 32 bits
    fprintf(cycle_trace_fp, "exec1_opcode %08x\n", spro->exec1_opcode); // 5 bits
    fprintf(cycle_trace_fp, "exec1_src0 %08x\n", spro->exec1_src0); // 3 bits
    fprintf(cycle_trace_fp, "exec1_src1 %08x\n", spro->exec1_src1); // 3 bits
    fprintf(cycle_trace_fp, "exec1_dst %08x\n", spro->exec1_dst); // 3 bits
    fprintf(cycle_trace_fp, "exec1_immediate %08x\n", spro->exec1_immediate); // 32 bits
    fprintf(cycle_trace_fp, "exec1_alu0 %08x\n", spro->exec1_alu0); // 32 bits
    fprintf(cycle_trace_fp, "exec1_alu1 %08x\n", spro->exec1_alu1); // 32 bits
    fprintf(cycle_trace_fp, "exec1_aluout %08x\n", spro->exec1_aluout);

    fprintf(cycle_trace_fp, "dma_src %08x\n", spro->dma_src);
    fprintf(cycle_trace_fp, "dma_state %08x\n", spro->dma_state);
    fprintf(cycle_trace_fp, "dma_len %08x\n", spro->dma_length);
    fprintf(cycle_trace_fp, "dma_dest %08x\n", spro->dma_dst);
    fprintf(cycle_trace_fp, "dma_status %08x\n", spro->dma_status);
    fprintf(cycle_trace_fp, "\n");

    sp_printf("cycle_counter %08x\n", spro->cycle_counter);
    sp_printf("r2 %08x, r3 %08x\n", spro->r[2], spro->r[3]);
    sp_printf("r4 %08x, r5 %08x, r6 %08x, r7 %08x\n", spro->r[4], spro->r[5], spro->r[6], spro->r[7]);
    sp_printf("fetch0_active %d, fetch1_active %d, dec0_active %d, dec1_active %d, exec0_active %d, exec1_active %d\n", spro->fetch0_active, spro->fetch1_active, spro->dec0_active, spro->dec1_active, spro->exec0_active, spro->exec1_active);
    sp_printf("fetch0_pc %d, fetch1_pc %d, dec0_pc %d, dec1_pc %d, exec0_pc %d, exec1_pc %d\n", spro->fetch0_pc, spro->fetch1_pc, spro->dec0_pc, spro->dec1_pc, spro->exec0_pc, spro->exec1_pc);

    sprn->cycle_counter = spro->cycle_counter + 1;

    if (sp->start)
        sprn->fetch0_active = 1;
	// our section: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // fetch0
    sprn->fetch1_active = 0;
    if (spro->fetch0_active) {//read command and update the pc.
        llsim_mem_read(sp->srami, spro->fetch0_pc);
        if ((sp->spro->fetch0_pc + 1) == (0xFFFF))//check for overflow.
            sp->sprn->fetch0_pc = 0;
        else
            sp->sprn->fetch0_pc = (sp->spro->fetch0_pc + 1) & (0x0000FFFF);
        if (branch_lookup_table[spro->fetch0_pc] != -1) {//if jump is predicted. 
            sprn->fetch0_active = 1;
            sprn->fetch0_pc = branch_lookup_table[spro->fetch0_pc];
        }
        sprn->fetch1_pc = spro->fetch0_pc;
        sp->sprn->fetch1_active = 1;
    }
    else // fetch0 wasn't active
        sp->sprn->fetch1_active = 0;
    // fetch1
    if (spro->fetch1_active) {//if fetch1 is active, extract instruction from memory.
        sprn->dec0_inst = llsim_mem_extract(sp->srami, sp->spro->fetch1_pc, 31, 0);
        sprn->dec0_pc = sp->spro->fetch1_pc;
        sp->sprn->dec0_active = 1;
    }
    else // fetch1 wasn't active
        sp->sprn->dec0_active = 0;
    // dec0
    int opcode, imm;
    int	src0, dst, src1;
    if (spro->dec0_active) {
        imm = spro->dec0_inst & 0x0000ffff;
        src0 = (spro->dec0_inst >> 19) & 0x7;
        src1 = (spro->dec0_inst >> 16) & 0x7;
        dst = (spro->dec0_inst >> 22) & 0x7;
        opcode = (spro->dec0_inst >> 25) & 0x1F; //masking and shifting for opcode
        if (spro->dec1_opcode == ST && opcode == LD && spro->dec1_active) {//stall command to avoid hazard.
            sprn->dec0_pc = spro->dec0_pc;
            sprn->fetch0_pc = spro->fetch1_pc;
            sprn->dec0_inst = spro->dec0_inst;
            sprn->fetch0_active = spro->fetch1_active;
            sprn->dec0_active = spro->dec0_active;
            sprn->fetch1_active = 0;
            sprn->dec1_active = 0;
        }
        else {
            sprn->dec1_pc = spro->dec0_pc;
            sprn->dec1_opcode = opcode;
            sprn->dec1_immediate = imm;
            sprn->dec1_immediate <<= 20;
            sprn->dec1_immediate >>= 20;
            sprn->dec1_dst = dst;
            sprn->dec1_src0 = src0;
            sprn->dec1_src1 = src1;
            sprn->dec1_inst = spro->dec0_inst;
            sp->sprn->dec1_active = 1;
        }
    }
    else {
        sp->sprn->dec1_active = 0;
    }
    // dec1
    int flag0 = 0;
    int flag1 = 0;
    if (spro->dec1_active) {
        if ((spro->dec1_src0 != 1) && (spro->dec1_src0 != 0) && ((sp->spro->exec0_opcode == 8) && (sp->spro->exec0_active) && (sp->spro->exec0_dst == spro->dec1_src0)))
            stall_pipline_to_avoid_hazard(sp);
        else if (spro->dec1_opcode == LHI && spro->r[spro->dec1_dst] == 0)
            stall_pipline_to_avoid_hazard(sp);
        else if ((spro->dec1_src1 != 1) && (spro->dec1_src1 != 0) && ((sp->spro->exec0_opcode == 8) && (sp->spro->exec0_active) && (sp->spro->exec0_dst == spro->dec1_src1)))
            stall_pipline_to_avoid_hazard(sp);
        else {
            if (sp->spro->dec1_src0 == 1) {//src 0
                sprn->r[1] = spro->dec1_immediate;
                sprn->exec0_alu0 = spro->dec1_immediate;
                flag0 = 1;
            }
            else if (spro->dec1_src0 == 0) {
                sprn->exec0_alu0 = 0;
                flag0 = 1;
            }
            else if ((((sp->spro->exec1_opcode >= ADD) && (sp->spro->exec1_opcode <= LHI)) || sp->spro->exec1_opcode == POL) && (sp->spro->exec1_active) && (sp->spro->exec1_dst == spro->dec1_src0)) {//reg data hazard detect
                sprn->exec0_alu0 = spro->exec1_aluout;
                flag0 = 1;
            }
            else if ((spro->exec1_active && spro->dec1_src0 == 7 && ((spro->exec1_opcode >= JLT && spro->exec1_opcode <= JNE) && spro->exec1_aluout == 1)) || (spro->exec1_opcode == JIN)) {//control hazard detect
                sprn->exec0_alu0 = spro->exec1_pc;
                flag0 = 1;
            }
            else if ((sp->spro->exec1_opcode == LD) && (sp->spro->exec1_active) && (sp->spro->exec1_dst == spro->dec1_src0)) {//mem hazard detect
                sprn->exec0_alu0 = llsim_mem_extract_dataout(sp->sramd, 31, 0);
                flag0 = 1;
            }
            else {
                sprn->exec0_alu0 = spro->r[spro->dec1_src0];//no hazard detected
                flag0 = 1;
            }
            if (spro->dec1_src1 == 1) {//src 1
                sprn->r[1] = spro->dec1_immediate;
                sprn->exec0_alu1 = spro->dec1_immediate;
                flag1 = 1;
            }
            else if (spro->dec1_src1 == 0) {
                sprn->exec0_alu1 = 0;
                flag1 = 1;
            }
            else if ((((sp->spro->exec1_opcode >= ADD) && (sp->spro->exec1_opcode <= LHI)) || sp->spro->exec1_opcode == POL) && (sp->spro->exec1_active) && (sp->spro->exec1_dst == spro->dec1_src1)) {//reg data hazard detect
                sprn->exec0_alu1 = spro->exec1_aluout;
                flag1 = 1;
            }
            else if ((spro->exec1_active && spro->dec1_src1 == 7 && ((spro->exec1_opcode >= JLT && spro->exec1_opcode <= JNE) && spro->exec1_aluout == 1)) || (spro->exec1_opcode == JIN)) {//control hazard detect
                sprn->exec0_alu1 = sp->spro->exec1_pc;
                flag1 = 1;
            }
            else if ((sp->spro->exec1_opcode == LD) && (sp->spro->exec1_active) && (sp->spro->exec1_dst == spro->dec1_src1)) {//mem hazard detect
                sprn->exec0_alu1 = llsim_mem_extract_dataout(sp->sramd, 31, 0);
                flag1 = 1;
            }
            else {
                sprn->exec0_alu1 = spro->r[spro->dec1_src1];
                flag1 = 1;
            }
            if (flag1 && flag0) {
                if (spro->dec1_opcode == LHI) {
                    sprn->exec0_alu0 = spro->r[spro->dec1_dst];
                    sprn->exec0_alu1 = spro->dec1_immediate;
                }
                sprn->exec0_pc = spro->dec1_pc;
                sprn->exec0_opcode = spro->dec1_opcode;
                sprn->exec0_inst = spro->dec1_inst;
                sprn->exec0_immediate = spro->dec1_immediate;
                sprn->exec0_dst = spro->dec1_dst;
                sprn->exec0_src0 = spro->dec1_src0;
                sprn->exec0_src1 = spro->dec1_src1;
                sp->sprn->exec0_active = 1;
            }
        }
    }
    else {
        sprn->exec0_active = 0;
    }
    // exec0
    int alu0 = sp->spro->exec0_alu0, alu1 = sp->spro->exec0_alu1;
    if (spro->exec0_active) {
        if (spro->exec0_opcode == NOP)
		{   //Data Hazard caused Stall
            sprn->exec1_active = 0;
            sprn->exec1_pc = spro->exec1_pc;
            sprn->exec1_opcode = spro->exec1_opcode;
            sprn->exec1_dst = spro->exec1_dst;
            sprn->exec1_src0 = spro->exec1_src0;
            sprn->exec1_src1 = spro->exec1_src1;
            sprn->exec1_inst = spro->exec1_inst;
            sprn->exec1_alu0 = spro->exec1_alu0;
            sprn->exec1_alu1 = spro->exec1_alu1;
            sprn->exec1_immediate = spro->exec1_immediate;
        }
     
		else {
            if ((spro->exec0_src0 != 1) && (spro->exec0_src0 != 0))
            {
                if ((spro->exec1_active && spro->exec0_src0 == 7 && ((spro->exec1_opcode >= JLT && spro->exec1_opcode <= JNE) && spro->exec1_aluout == 1)) || (spro->exec1_opcode == JIN)) {//control hazard detect
                    alu0 = spro->exec1_pc;
                }
                else if ((((sp->spro->exec1_opcode >= ADD) && (sp->spro->exec1_opcode <= LHI)) || sp->spro->exec1_opcode == POL) && (sp->spro->exec1_active) && (sp->spro->exec1_dst == spro->exec0_src0)) {//reg data hazard detect
                    alu0 = spro->exec1_aluout;
                }
            }
            if ((spro->exec0_src1 != 1) && (spro->exec0_src1 != 0))
            {
                if ((spro->exec1_active && spro->exec0_src1 == 7 && ((spro->exec1_opcode >= JLT && spro->exec1_opcode <= JNE) && spro->exec1_aluout == 1)) || (spro->exec1_opcode == JIN)) {//control hazard detect
                    alu1 = spro->exec1_pc;
                }
                else if ((((sp->spro->exec1_opcode >= ADD) && (sp->spro->exec1_opcode <= LHI)) || sp->spro->exec1_opcode == POL) && (sp->spro->exec1_active) && (sp->spro->exec1_dst == spro->exec0_src1)) {//reg data hazard detect
                    alu1 = spro->exec1_aluout;
                }
            }
            execute_alu(sp, alu0, alu1);//ALU execution
				if (spro->exec0_opcode == POL) {//POL op
				
					sprn->exec1_aluout = ((spro->exec1_opcode == DMA && spro->exec1_active == 1) || spro->dma_status == 1);
            }
				if (spro->exec0_opcode == LD) {//Load operation
					llsim_mem_read(sp->sramd, alu1); // alu1 now has src1 
					}
                int empty_pipeline = 0, temp_pc = 0;
			if ((spro->exec0_opcode == JIN)|| (spro->exec0_opcode >= JLT && spro->exec0_opcode <= JNE)) 
            {
                if (spro->exec0_opcode >= JLT && spro->exec0_opcode <= JNE) {//if the opcode belongs to a conditional jump (not jin)
                    if (sprn->exec1_aluout == 1) {
                        temp_pc = spro->exec0_immediate & (0x0000FFFF);
                        branch_lookup_table[spro->exec0_pc] = spro->exec0_immediate & (0x0000FFFF);
                    }
                    else {
                        temp_pc = sp->spro->exec0_pc + 1;
                        branch_lookup_table[spro->exec0_pc] = -1;
                    }
                }
                else { //non conditional jump
                    sprn->r[7] = spro->exec0_pc;
                    temp_pc = spro->exec0_alu0 & (0x0000ffff);
                }
                if (spro->dec1_active) {
                    if (spro->dec1_pc != temp_pc) {
                        empty_pipeline = 1;
                    }
                }
                else if (spro->dec0_active) {
                    if (spro->dec0_pc != temp_pc) {
                        empty_pipeline = 1;
                    }
                }
                else if (spro->fetch1_active) {
                    if (spro->fetch1_pc != temp_pc) {
                        empty_pipeline = 1;
                    }
                }
                else if (spro->fetch0_active) {
                    if (spro->fetch0_pc != temp_pc) {
                        empty_pipeline = 1;
                    }
                }
                if (empty_pipeline) {//Pipeline is empty
                    sprn->fetch0_pc = temp_pc;
                    sprn->fetch0_active = 1;
                    sprn->fetch1_active = 0;
                    sprn->dec0_active = 0;
                    sprn->dec1_active = 0;
                    sprn->exec0_active = 0;
                    sprn->exec1_active = 0;
                }
            }
            sprn->exec1_pc = spro->exec0_pc;//Setting exec1 parameters anyway
			sprn->exec1_opcode = spro->exec0_opcode;
            sprn->exec1_immediate = spro->exec0_immediate;
		    sprn->exec1_src0 = spro->exec0_src0;
            sprn->exec1_src1 = spro->exec0_src1;
            sprn->exec1_inst = spro->exec0_inst;
			sprn->exec1_dst = spro->exec0_dst;

			sprn->exec1_alu1 = alu1;//feed in alu
            sprn->exec1_alu0 = alu0;
            sprn->exec1_active = 1;//activate next stage
        }
    }
    else {sprn->exec1_active = 0;//exec0 wasn't activated in the first place, no need to activate next
    }

    // exec1: Write backs ALU and memory (ST). On HLT goes to IDLE, otherwise loops back to FETCH0.
    int extracted_line_number;
    if (spro->exec1_active) {
        if (spro->exec1_opcode == HLT) {//stops the program
            print_to_trace(sp, 0);//writing last to trace
            llsim_stop();
            dump_sram(sp, "srami_out.txt", sp->srami);
            dump_sram(sp, "sramd_out.txt", sp->sramd);
        }
        else {

            if (spro->exec1_opcode == 9) {//9 is SAVE operation (ST)
                llsim_mem_set_datain(sp->sramd, sp->spro->exec1_alu0, 31, 0); // alu0 has the write value 
                 //writing into adresss (correct writing adress should be in alu1)
                llsim_mem_write(sp->sramd, spro->exec1_alu1);
                print_to_trace(sp, 0);//write to trace
            }
			else if (sp->spro->exec1_opcode == 8) {// 8 is LOAD operation (LD)
                extracted_line_number = llsim_mem_extract(sp->sramd, spro->exec1_alu1, 31, 0); // alu1 has src1 val, extracting line num 
                sprn->r[spro->exec1_dst] = extracted_line_number;//saving line num into register
                print_to_trace(sp, extracted_line_number);//write to trace
            }
            else if ((spro->exec1_opcode == JIN) || (spro->exec1_opcode >= 16 && spro->exec1_opcode <= 19)) {//20 is unconditional jump operation (JIN) 
                if ((spro->exec1_opcode >= 16 && spro->exec1_opcode <= 19)) {
                    if (sprn->exec1_aluout == 1) {
                        sprn->r[7] = spro->exec1_pc;
                    }
                }
                print_to_trace(sp, 0);//write to trace 
            }
            else if (spro->exec1_opcode == POL) {//26 is POL DMA
                sprn->r[spro->exec1_dst] = spro->exec1_aluout;
            }
            else if (spro->exec1_opcode == DMA) {//25 DMA
            	sprn->dma_dst = spro->r[spro->exec1_dst];
            	sprn->dma_src = spro->exec1_alu0;
            	sprn->dma_length = spro->exec1_alu1;
            	print_to_trace(sp, 0);//write to trace
            }
            else {
                print_to_trace(sp, 0);//shouln't get here 
            }
            if (((spro->exec1_opcode >= ADD) && (spro->exec1_opcode <= LHI)) || (spro->exec1_opcode == POL)) {
                sprn->r[spro->exec1_dst] = spro->exec1_aluout;
            }
            nr_simulated_instructions++;
        }

        if (!dma_begin && spro->exec1_opcode == DMA) {
            dma_begin = 1;
        }

        if ((sprn->dec1_opcode == ST)||(sprn->dec1_opcode==LD)||(sprn->exec0_opcode==LD)||
            (sprn->exec0_opcode == ST) || (sprn->exec1_opcode == LD)||(sprn->exec1_opcode == ST)) {
            memory_status = 0;//memory is involved in some stage of the pipeline, so memory status is 0
        }
        else {
            memory_status = 1;//no memory action involved/memory action is done. therefore set memory status to 1
        }     
        dma_ctl(sp);//use dma
    }
}
// end of our section ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  

static void sp_run(llsim_unit_t* unit)
{
    sp_t* sp = (sp_t*)unit->private;
    //	sp_registers_t *spro = sp->spro;
//	sp_registers_t *sprn = sp->sprn;

//	llsim_printf("-------------------------\n");

    if (llsim->reset) {
        sp_reset(sp);
        return;
    }

    sp->srami->read = 0;
    sp->srami->write = 0;
    sp->sramd->read = 0;
    sp->sramd->write = 0;

    sp_ctl(sp);

}

static void sp_generate_sram_memory_image(sp_t* sp, char* program_name)
{
    FILE* fp;
    int addr, i;

    fp = fopen(program_name, "r");
    if (fp == NULL) {
        printf("couldn't open file %s\n", program_name);
        exit(1);
    }
    addr = 0;
    while (addr < SP_SRAM_HEIGHT) {
        fscanf(fp, "%08x\n", &sp->memory_image[addr]);
        //              printf("addr %x: %08x\n", addr, sp->memory_image[addr]);
        addr++;
        if (feof(fp))
            break;
    }
    sp->memory_image_size = addr;

    fprintf(inst_trace_fp, "program %s loaded, %d lines\n", program_name, addr);

    for (i = 0; i < sp->memory_image_size; i++) {
        llsim_mem_inject(sp->srami, i, sp->memory_image[i], 31, 0);
        llsim_mem_inject(sp->sramd, i, sp->memory_image[i], 31, 0);
    }
}

void sp_init(char* program_name)
{
    llsim_unit_t* llsim_sp_unit;
    llsim_unit_registers_t* llsim_ur;
    sp_t* sp;

    llsim_printf("initializing sp unit\n");

    inst_trace_fp = fopen("inst_trace.txt", "w");
    if (inst_trace_fp == NULL) {
        printf("couldn't open file inst_trace.txt\n");
        exit(1);
    }

    cycle_trace_fp = fopen("cycle_trace.txt", "w");
    if (cycle_trace_fp == NULL) {
        printf("couldn't open file cycle_trace.txt\n");
        exit(1);
    }

    llsim_sp_unit = llsim_register_unit("sp", sp_run);
    llsim_ur = llsim_allocate_registers(llsim_sp_unit, "sp_registers", sizeof(sp_registers_t));
    sp = llsim_malloc(sizeof(sp_t));
    llsim_sp_unit->private = sp;
    sp->spro = llsim_ur->old;
    sp->sprn = llsim_ur->new;

    sp->srami = llsim_allocate_memory(llsim_sp_unit, "srami", 32, SP_SRAM_HEIGHT, 0);
    sp->sramd = llsim_allocate_memory(llsim_sp_unit, "sramd", 32, SP_SRAM_HEIGHT, 0);
    sp_generate_sram_memory_image(sp, program_name);
    init_lookup_table(branch_lookup_table);

    sp->start = 1;
}
// c2v_translate_end

// our section: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void print_to_trace(sp_t* sp, int extracted_line_number) {
    int LD_reg_addr = 0, ST_mem_addr = 0, curr_pc;
    fprintf(inst_trace_fp, "--- instruction %i (%04x) @ PC %i (%04x) -----------------------------------------------------------\n", nr_simulated_instructions, nr_simulated_instructions, sp->spro->exec1_pc, sp->spro->exec1_pc);

    fprintf(inst_trace_fp, "pc = %04d, inst = %08x, opcode = %i (%s), dst = %i, src0 = %i, src1 = %i, immediate = %08x\n", sp->spro->exec1_pc, sp->spro->exec1_inst, sp->spro->exec1_opcode, opcode_name[sp->spro->exec1_opcode], sp->spro->exec1_dst, sp->spro->exec1_src0, sp->spro->exec1_src1, sp->spro->exec1_immediate);
    fprintf(inst_trace_fp, "r[0] = 00000000 r[1] = %08x r[2] = %08x r[3] = %08x \n", sp->spro->exec1_immediate, sp->spro->r[2], sp->spro->r[3]);

    fprintf(inst_trace_fp, "r[4] = %08x r[5] = %08x r[6] = %08x r[7] = %08x \n\n", sp->spro->r[4], sp->spro->r[5], sp->spro->r[6], sp->spro->r[7]);

    if (sp->spro->exec1_opcode <= LHI) {
        fprintf(inst_trace_fp, ">>>> EXEC: R[%i] = %i %s %i <<<<\n\n", sp->spro->exec1_dst, sp->spro->exec1_alu0, opcode_name[sp->spro->exec1_opcode], sp->spro->exec1_alu1);
    }
    else if (sp->spro->exec1_opcode == LD)
    {
        if (sp->spro->exec1_src1 == 1) {
            LD_reg_addr = sp->spro->exec1_immediate;
        }
        else
            LD_reg_addr = sp->spro->r[sp->spro->exec1_src1];
        fprintf(inst_trace_fp, ">>>> EXEC: R[%i] = MEM[%i] = %08x <<<<\n\n", sp->spro->exec1_dst, LD_reg_addr, extracted_line_number);
    }
    else if (sp->spro->exec1_opcode == ST)
    {
        if (sp->spro->exec1_src1 == 1) {
            ST_mem_addr = sp->spro->exec1_immediate;
        }
        else
            ST_mem_addr = sp->spro->r[sp->spro->exec1_src1];
        fprintf(inst_trace_fp, ">>>> EXEC: MEM[%i] = R[%i] = %08x <<<<\n\n", ST_mem_addr, sp->spro->exec1_src0, sp->spro->r[sp->spro->exec1_src0]);
    }
    else if (sp->spro->exec1_opcode == JLT || sp->spro->exec1_opcode == JLE || sp->spro->exec1_opcode == JEQ || sp->spro->exec1_opcode == JNE || sp->spro->exec1_opcode == JIN)
    {
        if (sp->spro->exec1_aluout) {
            curr_pc = sp->spro->exec1_immediate;
        }
        else
            curr_pc = sp->spro->exec1_pc + 1;

        fprintf(inst_trace_fp, ">>>> EXEC: %s %i, %i, %i <<<<\n\n", opcode_name[sp->spro->exec1_opcode], sp->spro->r[sp->spro->exec1_src0], sp->spro->r[sp->spro->exec1_src1], curr_pc);
    }
    else if (sp->spro->exec1_opcode == HLT)
    {
        fprintf(inst_trace_fp, ">>>> EXEC: HALT at PC %04x<<<<\n", sp->spro->exec1_pc);
        fprintf(inst_trace_fp, "sim finished at pc %i, %i instructions", sp->spro->exec1_pc, nr_simulated_instructions + 1);
    }
    if (sp->spro->exec1_opcode == DMA) // DMA copy instruction
        fprintf(inst_trace_fp, ">>>> EXEC: DMA %d %d %d <<<<\n\n", sp->sprn->dma_dst, sp->sprn->dma_src, sp->sprn->dma_length);
    if (sp->spro->exec1_opcode == POL) // DMA poll instruction
        fprintf(inst_trace_fp, ">>>> EXEC: POL R[%d]=%d <<<<\n\n", sp->spro->exec1_dst, sp->spro->dma_status);
}

void execute_alu(sp_t* sp, int alu0, int alu1) {
	
    sp_registers_t* spro = sp->spro, * sprn = sp->sprn;
    if (spro->exec0_dst > 7 || spro->exec0_dst < 0 || spro->exec0_src0 > 7 || spro->exec0_src0 < 0 || spro->exec0_src1 > 7 || spro->exec0_src1 < 0) {
        printf("Error: wrong input instruction\n");
    }
    switch (sp->spro->exec0_opcode) {
    case ADD:
        sprn->exec1_aluout = alu0+alu1;
        break;
    case SUB:
        sprn->exec1_aluout = alu0-alu1;
        break;
    case LSF:
        sprn->exec1_aluout = alu0<<alu1;
        break;
    case RSF:
        sprn->exec1_aluout = alu0>>alu1;
        break;
    case AND:
        sprn->exec1_aluout = alu0&alu1;
        break;
    case OR:
        sprn->exec1_aluout = alu0|alu1;
        break;
    case XOR:
        sprn->exec1_aluout = alu0^alu1;
        break;
    case LHI:
        sprn->exec1_aluout = (alu1 << 16) | (alu0 & 0x0000ffff);
        break;
    case LD://No ALU operation for LD
        break;
    case ST: //No ALU operation for ST
        break;
    case DMA://No ALU operation for DMA
        break;
    case POL://No ALU operation for POL
        break;
    case JLT:
        if (alu0 < alu1) {
            sprn->exec1_aluout = 1;
        }
        else {
            sprn->exec1_aluout = 0;
        }
        break;
    case JLE:
        if (alu0 <= alu1) {
            sprn->exec1_aluout = 1;
        }
        else {
            sprn->exec1_aluout = 0;
        }
        break;
    case JEQ:
        if (alu0 == alu1) {
            sprn->exec1_aluout = 1;
        }
        else {
            sprn->exec1_aluout = 0;
        }
        break;
    case JNE:
        if (alu0 != alu1) {
            sprn->exec1_aluout = 1;
        }
        else {
            sprn->exec1_aluout = 0;
        }
        break;
    case JIN: // No ALU operation for JIN
        break;
    case NOP: // No ALU operation for NOP
        break;
    case HLT: // No ALU operation for HLT
        break;
    default:
        printf("invalid opcode error. \n");// Case of invalid/unknown opcode goes to default
    }
}


void dma_ctl(sp_t* sp) {
    sp_registers_t* spro = sp->spro, * sprn = sp->sprn;
	int mem = 0;
	//DMA can be in idle, fetch and exec mode
    if(spro->dma_state==DMA_STATE_IDLE) {  //DMA is in idle, so contiue instruction raise dma state
        if (memory_status && dma_begin) {
            sprn->dma_status = 1;
			sprn->dma_state = DMA_STATE_FETCH; }
        else {
            sprn->dma_state = DMA_STATE_IDLE;//DMA already working or memory isn't available yet,break
        }
    }
    else if(spro->dma_state==DMA_STATE_FETCH) {// fetch instruction
        if (memory_status == 0) {//memory not available yet (some memory related instruction is still a problem, break)
            sprn->dma_state = DMA_STATE_IDLE;
        }
        else {
            llsim_mem_read(sp->sramd, spro->dma_src);//do the DMA fetch 
            sprn->dma_state = DMA_STATE_EXEC;//move on to execute state and break
        }
    }
    else if(spro->dma_state==DMA_STATE_EXEC) {
        if (memory_status == 0) {//memory unavailable yet go back to idle and break
            sprn->dma_state = DMA_STATE_IDLE;
        }
		else{
        mem = llsim_mem_extract(sp->sramd, spro->dma_src, 31, 0);//DMA execution
        llsim_mem_set_datain(sp->sramd, mem, 31, 0);//data in
        llsim_mem_write(sp->sramd, spro->dma_dst);//mem write
		
		sprn->dma_dst = spro->dma_dst + 1;
        sprn->dma_src = spro->dma_src + 1;

        sprn->dma_length = spro->dma_length - 1;

        if (spro->dma_length == 1) {//last 
		    dma_begin = 0;
            sprn->dma_state = DMA_STATE_IDLE;
            sprn->dma_status = 0;//DMA not busy anymore after last action
            sprn->dma_length = 0;//initialize
		}

        else if (memory_status == 1) {//mem is good to go, go on to fetch
            sprn->dma_state = DMA_STATE_FETCH;
        }
        else if (memory_status == 0) {//go back to idle, (when mem is 0)
            sprn->dma_state = DMA_STATE_IDLE;
        }
    }
    }
}

void init_lookup_table() {
	for (int i = 0; i < 1024; ++i) {
		branch_lookup_table[i] = -1;
	}
}

void stall_pipline_to_avoid_hazard(sp_t* sp) {
    sp_registers_t* spro = sp->spro, * sprn = sp->sprn;
    sprn->fetch0_pc = spro->fetch0_pc;
    sprn->fetch0_active = spro->fetch0_active;

    sprn->fetch1_pc = spro->fetch1_pc;
    sprn->fetch1_active = spro->fetch1_active;

    sprn->dec0_pc = spro->dec0_pc;
    sprn->dec0_active = spro->dec0_active;
    sprn->dec0_inst = spro->dec0_inst;

    sprn->dec1_pc = spro->dec1_pc;
    sprn->dec1_active = spro->dec1_active;
    sprn->dec1_inst = spro->dec1_inst;
    sprn->dec1_opcode = spro->dec1_opcode;
    sprn->dec1_immediate = spro->dec1_immediate;
    sprn->dec1_dst = spro->dec1_dst;
    sprn->dec1_src0 = spro->dec1_src0;
    sprn->dec1_src1 = spro->dec1_src1;

    sprn->exec0_pc = 0;
    sprn->exec0_inst = 0;
    sprn->exec0_opcode = 23;
    sprn->exec0_dst = 0;
    sprn->exec0_src0 = 0;
    sprn->exec0_src1 = 0;
    sprn->exec0_immediate = 0;
    sprn->exec0_active = 1;
    sprn->exec1_active = 0;
}