Porting notes on this example
#############################


Notes on porting from BSP to Zephyr
***********************************

1. Search keywords below for relevant modifications

   - __ZEPHYR__
   - CONFIG_SOC_FAMILY_NUMAKER

2. Must enable FPU (CONFIG_FPU and friends) additionally to match BSP
   performance on inference pre-process/post-process
3. Configure MPU region for tensor arena in zephyr way rather than
   direct control
4. Change ethosu_flush_dcache/ethosu_invalidate_dcache to match zephyr
   condition
5. Place tensor arena at .noinit.tensor_arena section
6. Profile/PMU

   (1) ethosu_driver is encapsulated in ethos-u zephyr driver and is
       invisible from app. Acquire it by invoking ethosu_reserve_driver
       and ethosu_release_driver in pair transiently
   (2) SysTick is exclusively used by zephyr kernel. Use zephyr kernel
       timing api instead

7. DMIC/LPPDMA

   (1) Allocate LPPDMA descriptor at DTCM to spare cache coherency
       handling on dcache on
   (2) Set up LPPDMA interrupt in IRQ_CONNECT/irq_enable
   (3) Add sample update semaphore to avoid CPU lock by main thread

8. ml-embedded-evaluation-kit

   (1) Change BufAttributes.hpp for model related code/data placement

       a. Place model at .rodata.tflm_model section
       b. Place tensor arena at .noinit.tflm_arena section
       c. Place input feature map at .rodata.tflm_input section
       d. Place labels at .rodata.tflm_labels section

   (2) Redirect ml-embedded-evaluation-kit logging to zephyr way
       NOTE: Zephyr logging doesn't open interface for disabling newline
       print, and this redirect has drawback of duplicate newline print.

9. ml-embedded-evaluation-kit/kws

   (1) Use MicroNetKwsMfcc instead of KwsMfcc
   (2) KwsPreProcess has API update on passing inference index
   (3) KwsPostProcess has API update on passing useSoftmax
   (4) Tweak KwsPostProcess to enable use or not of softmax
    
10. Use shell instead of unsupported getchar
    Add shell commands "kws <subcommand>" for kws record control

11. Adjust memory size

    (1) Enlarge main stack size (CONFIG_MAIN_STACK_SIZE)
        NOTE: MPU fault may imply stack overflow.
    (2) Enable system heap (CONFIG_HEAP_MEM_POOL_SIZE)
        This is used for k_malloc in ethos-u driver overrides
        ethosu_mutex_create and ethosu_semaphore_create
