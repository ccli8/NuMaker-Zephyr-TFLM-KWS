/*
 * Copyright 2022 Arm Limited and/or its affiliates <open-source-office@arm.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(hyperram), okay)

  .hyperram.data : SUBALIGN(4)
  {
    __hyperram_start = ALIGN(4);
    __hyperram_data_start = ALIGN(4);
    *(.hyperram.data)
    *(".hyperram.data.*")
    __hyperram_data_end = ALIGN(4);
  } > HYPERRAM AT> FLASH
  __hyperram_data_load_start = LOADADDR(.hyperram.data);

  .hyperram.bss (NOLOAD) : SUBALIGN(4)
  {
    __hyperram_bss_start = ALIGN(4);
    *(.hyperram.bss)
    *(".hyperram.bss.*")
    __hyperram_bss_end = ALIGN(4);
  } > HYPERRAM

  .hyperram.noinit (NOLOAD) : SUBALIGN(4)
  {
    __hyperram_noinit_start = ALIGN(4);
    *(.hyperram.noinit)
    *(".hyperram.noinit.*")
    __hyperram_noinit_end = ALIGN(4);
  } > HYPERRAM

  __hyperram_end = ALIGN(4);

#endif
