void *memset(void *, int, size_t);
void *memcpy(void *, const void *, size_t);
int memcmp(const void *, const void *, size_t);
void *memchr(const void *addr, int val, size_t len);
size_t strlen(const char *str);

void sys_reset(void);
void __attribute__((noreturn)) sys_yield_for_frogs(void);

void crit_enter(void);
void crit_exit(void);
int crit_active(void);

void __attribute__((noreturn)) panic(const char *reason);

void int_enable(size_t intno);
void int_disable(size_t intno);

#if TARGET_FAMILY == MKL24

// Use Bit Manipulation Engine (BME) for Bitband access.
// See http://cache.freescale.com/files/32bit/doc/app_note/AN4757.pdf

#define GPIO_ALIAS_OFFSET 0x000F0000L

#define BME_OPCODE_BITFIELD 4

//macro used to generate hardcoded bit field insert address
#define BME_BITFIELD_INSERT(ADDR,bit,width) (*(volatile uint32_t *) \
	(((uint32_t)ADDR) \
 	| (BME_OPCODE_BITFIELD <<26) \
 	| ((bit & 0x1F)<<23) | ((width-1) & 0xF)<<19))

static inline volatile uint32_t *
bitband_bitp(volatile void *addr, size_t bit)
{
        return &BME_BITFIELD_INSERT(addr-GPIO_ALIAS_OFFSET,bit, 1);
}
#else
static inline volatile uint32_t *
bitband_bitp(volatile void *addr, size_t bit)
{
        return ((volatile void *)(0x42000000 + ((uintptr_t)addr - 0x40000000) * 32 + 4 * bit));
}

#endif

#define BITBAND_BIT(var, bit)	(*bitband_bitp(&(var), (bit)))

#define KKASSERT(cond)                          \
        if (!(cond)) {                          \
                panic(_STR(cond));              \
        }
