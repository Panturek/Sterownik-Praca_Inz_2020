
/* AUTO-GENERATED by gen_isr_tables.py, do not edit! */

#include <toolchain.h>
#include <linker/sections.h>
#include <sw_isr_table.h>
#include <arch/cpu.h>

#if defined(CONFIG_GEN_SW_ISR_TABLE) && defined(CONFIG_GEN_IRQ_VECTOR_TABLE)
#define ISR_WRAPPER ((u32_t)&_isr_wrapper)
#else
#define ISR_WRAPPER NULL
#endif

u32_t __irq_vector_table _irq_vector_table[82] = {
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
	ISR_WRAPPER,
};
struct _isr_table_entry __sw_isr_table _sw_isr_table[82] = {
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x20002a34, (void *)0x80077b7},
	{(void *)0x20002a34, (void *)0x80077cb},
	{(void *)0x20002a34, (void *)0x80077d5},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x20002a34, (void *)0x8007771},
	{(void *)0x20002a34, (void *)0x800777b},
	{(void *)0x20002a34, (void *)0x8007785},
	{(void *)0x20002a34, (void *)0x800778f},
	{(void *)0x20002a34, (void *)0x8007799},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x20002ae8, (void *)0x8007d9d},
	{(void *)0x20002ae8, (void *)0x8007927},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x20002ae8, (void *)0x8007da9},
	{(void *)0x20002a34, (void *)0x80077a3},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x20002a40, (void *)0x80085e1},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x20002a34, (void *)0x80077ad},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x20002a34, (void *)0x80077c1},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
	{(void *)0x0, (void *)&z_irq_spurious},
};
