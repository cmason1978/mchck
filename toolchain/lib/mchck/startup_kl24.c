/**
 * Freescale K20 ISR table and startup code.
 */

#include <stdint.h>
#include <mchck-cdefs.h>

#ifndef STACK_SIZE
#define STACK_SIZE 0x200
#endif

__attribute__ ((__section__(".co_stack")))
__attribute__ ((__used__))
static uint32_t sys_stack[STACK_SIZE / 4];

/**
 * What follows is some macro magic to populate the
 * ISR vector table and to declare weak symbols for the handlers.
 *
 * We start by defining the macros VH and V, which by themselves
 * just call the (yet undefined) macro V_handler().
 *
 * V_handler will then be defined separately for each use of the
 * vector list.
 *
 * V_reserved is just used to properly skip the reserved entries
 * in the vector table.
 */

typedef void (isr_handler_t)(void);

isr_handler_t Default_Handler __attribute__((__weak__, __alias__("__Default_Handler")));
isr_handler_t Default_Reset_Handler;

isr_handler_t HardFault_Handler __attribute__((__weak__, __alias__("__HardFault_Handler")));

#define VH(num, handler, default)                               \
	V_handler(num, handler, _CONCAT(handler, _Handler), default)
#define V(num, x)                               \
	VH(num, x, Default_Handler)

/**
 * Declare the weak symbols.  By default they will be aliased
 * to Default_Handler, but the default handler can be specified
 * by using VH() instead of V().
 */

#define V_handler(num, name, handler, default)                          \
	isr_handler_t handler __attribute__((__weak__, __alias__(#default)));	\
	isr_handler_t _CONCAT(name, _IRQHandler) __attribute__((__weak__, __alias__(_STR(handler))));
#include "vecs.h"
#undef V_handler

/**
 * Define the vector table.  We simply fill in all (weak) vector symbols
 * and the occasional `0' for the reserved entries.
 */

__attribute__ ((__section__(".isr_vector"), __used__))
isr_handler_t * const isr_vectors[] =
{
	(isr_handler_t *)&sys_stack[sizeof(sys_stack)/sizeof(*sys_stack)],
#define V_handler(num, name, handler, default)	[num] = handler,
#include "vecs.h"
#undef V_handler
};

#undef V
#undef VH


static void
__Default_Handler(void)
{
	__asm("BKPT");
	for (;;)
		/* NOTHING */;
}

/**
 * The following variables are only used for their addresses;
 * their symbols are defined by the linker script.
 *
 * They are used to delimit various sections the process image:
 * _sidata marks where the flash copy of the .data section starts.
 * _sdata and _edata delimit the RAM addresses of the .data section.
 * _sbss and _ebss delimit the RAM BSS section in the same way.
 */

#include <mchck.h>

void main(void);

void
Default_Reset_Handler(void)
{
	/* disable watchdog */
	*(uint32_t*)0x40048100 = 0;
	
#ifdef EXTERNAL_XTAL
        OSC_CR = OSC_CR_SC16P_MASK;
        MCG.c2.raw = ((struct MCG_C2_t){
                        .range0 = MCG_RANGE_VERYHIGH,
                                .erefs0 = MCG_EREF_OSC
                                }).raw;
        MCG.c1.raw = ((struct MCG_C1_t){
                        .clks = MCG_CLKS_EXTERNAL,
                                .frdiv = 4, /* log2(EXTERNAL_XTAL) - 20 */
                                .irefs = 0
                                }).raw;

        while (!MCG.s.oscinit0)
                /* NOTHING */;
        while (MCG.s.clkst != MCG_CLKST_EXTERNAL)
                /* NOTHING */;

        MCG.c5.raw = ((struct MCG_C5_t){
                        .prdiv0 = ((EXTERNAL_XTAL / 2000000L) - 1),
                                .pllclken0 = 1
                                }).raw;
        MCG.c6.raw = ((struct MCG_C6_t){
                        .vdiv0 = 0,
                        .plls = 1
                                }).raw;

        while (!MCG.s.pllst)
                /* NOTHING */;
        while (!MCG.s.lock0)
                /* NOTHING */;

        MCG.c1.clks = MCG_CLKS_FLLPLL;

        while (MCG.s.clkst != MCG_CLKST_PLL)
                /* NOTHING */;

        SIM.sopt2.pllfllsel = SIM_PLLFLLSEL_PLL;
#else
        /* FLL at 48MHz */
//         MCG.c4.raw = ((struct MCG_C4_t){
//                         .drst_drs = MCG_DRST_DRS_MID,
//                         .dmx32 = 1
//                 }).raw;
//         SIM.sopt2.pllfllsel = SIM_PLLFLLSEL_FLL;
        
		/* SIM->CLKDIV1: OUTDIV1=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,OUTDIV4=2,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
		SIM.clkdiv1.outdiv4 = 2;

		// MCG.c3
		//*(uint8_t*)0x40064002 = 0;

		MCG.c4.fctrim = 7;

		/* Switch to FEI Mode */
		/* MCG->C1: CLKS=0,FRDIV=0,IREFS=1,IRCLKEN=1,IREFSTEN=0 */
		MCG.c1.irefs=1;
		MCG.c1.irclken=1;

		/* MCG_C2: LOCRE0=0,??=0,RANGE0=0,HGO0=0,EREFS0=0,LP=0,IRCS=0 */
		MCG.c2.raw = 0u;
		/* MCG->C4: DMX32=0,DRST_DRS=1 */
		MCG.c4.drst_drs = MCG_DRST_DRS_MID;
		MCG.c4.dmx32 = 1;

		/* OSC0->CR: ERCLKEN=1,??=0,EREFSTEN=0,??=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
		*(uint8_t*)0x40065000 = (uint8_t)0x80U;

		/* MCG->C5: ??=0,PLLCLKEN0=0,PLLSTEN0=0,PRDIV0=0 */
		MCG.c5.raw = 0u;

		/* MCG->C6: LOLIE0=0,PLLS=0,CME0=0,VDIV0=0 */
		MCG.c6.raw = 0u;

		while((MCG.s.irefst) == 0x00U) { /* Check that the source of the FLL reference clock is the internal reference clock. */
		}
		while(MCG.s.clkst) {    /* Wait until output of the FLL is selected */
		}
        
#endif

	memcpy(&_sdata, &_sidata, (uintptr_t)&_edata - (uintptr_t)&_sdata);
	memset(&_sbss, 0, (uintptr_t)&_ebss - (uintptr_t)&_sbss);

	main();
}


// see http://mcuoneclipse.com/2012/11/24/debugging-hard-faults-on-arm-cortex-m/
/**
* HardFaultHandler_C:
* This is called from the HardFault_HandlerAsm with a pointer the Fault stack
* as the parameter. We can then read the values from the stack and place them
* into local variables for ease of reading.
* We then read the various Fault Status and Address Registers to help decode
* cause of the fault.
* The function ends with a BKPT instruction to force control back into the debugger
*/
void HardFault_HandlerC(unsigned long *hardfault_args){
    volatile unsigned long stacked_r0 __attribute__((unused));
    volatile unsigned long stacked_r1 __attribute__((unused));
    volatile unsigned long stacked_r2 __attribute__((unused));
    volatile unsigned long stacked_r3 __attribute__((unused));
    volatile unsigned long stacked_r12 __attribute__((unused));
    volatile unsigned long stacked_lr __attribute__((unused));
    volatile unsigned long stacked_pc __attribute__((unused));
    volatile unsigned long stacked_psr __attribute__((unused));
    volatile unsigned long _CFSR __attribute__((unused));
    volatile unsigned long _HFSR __attribute__((unused));
    volatile unsigned long _DFSR __attribute__((unused));
    volatile unsigned long _AFSR __attribute__((unused));
    volatile unsigned long _BFAR __attribute__((unused));
    volatile unsigned long _MMAR __attribute__((unused));

    stacked_r0 = ((unsigned long)hardfault_args[0]) ;
    stacked_r1 = ((unsigned long)hardfault_args[1]) ;
    stacked_r2 = ((unsigned long)hardfault_args[2]) ;
    stacked_r3 = ((unsigned long)hardfault_args[3]) ;
    stacked_r12 = ((unsigned long)hardfault_args[4]) ;
    stacked_lr = ((unsigned long)hardfault_args[5]) ;
    stacked_pc = ((unsigned long)hardfault_args[6]) ;
    stacked_psr = ((unsigned long)hardfault_args[7]) ;

    // Configurable Fault Status Register
    // Consists of MMSR, BFSR and UFSR
    _CFSR = (*((volatile unsigned long *)(0xE000ED28))) ;

    // Hard Fault Status Register
    _HFSR = (*((volatile unsigned long *)(0xE000ED2C))) ;

    // Debug Fault Status Register
    _DFSR = (*((volatile unsigned long *)(0xE000ED30))) ;

    // Auxiliary Fault Status Register
    _AFSR = (*((volatile unsigned long *)(0xE000ED3C))) ;

    // Read the Fault Address Registers. These may not contain valid values.
    // Check BFARVALID/MMARVALID to see if they are valid values
    // MemManage Fault Address Register
    _MMAR = (*((volatile unsigned long *)(0xE000ED34))) ;
    // Bus Fault Address Register
    _BFAR = (*((volatile unsigned long *)(0xE000ED38))) ;

    __asm("BKPT #0\n") ; // Break into the debugger

}

/**
* HardFault_HandlerAsm:
* Alternative Hard Fault handler to help debug the reason for a fault.
* To use, edit the vector table to reference this function in the HardFault vector
* This code is suitable for Cortex-M3 and Cortex-M0 cores
*/

// Use the 'naked' attribute so that C stacking is not used.
__attribute__((naked))
void __HardFault_Handler(void){
/*
 * Get the appropriate stack pointer, depending on our mode,
 * and use it as the parameter to the C handler. This function
 * will never return
 */

    __asm(  ".syntax unified\n"
            "MOVS   R0, #4  \n"
            "MOV    R1, LR  \n"
            "TST    R0, R1  \n"
            "BEQ    _MSP    \n"
            "MRS    R0, PSP \n"
            "B      HardFault_HandlerC      \n"
            "_MSP:  \n"
            "MRS    R0, MSP \n"
            "B      HardFault_HandlerC      \n"
            ".syntax divided\n") ;
}


