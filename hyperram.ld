/*
 * Copyright 2022 Arm Limited and/or its affiliates <open-source-office@arm.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(hyperram), okay)

  .hyperram.data : ALIGN(_region_min_align)
  {
    __hyperram_start = ALIGN(_region_min_align);
    __hyperram_data_start = .;
    *(.hyperram.data)
    *(".hyperram.data.*")
    __hyperram_data_end = ALIGN(_region_min_align);
  } > HYPERRAM AT> FLASH
  __hyperram_data_load_start = LOADADDR(.hyperram.data);

  .hyperram.bss (NOLOAD) : ALIGN(_region_min_align)
  {
    __hyperram_bss_start = ALIGN(_region_min_align);
    *(.hyperram.bss)
    *(".hyperram.bss.*")
    __hyperram_bss_end = ALIGN(_region_min_align);
  } > HYPERRAM

  .hyperram.noinit (NOLOAD) : ALIGN(_region_min_align)
  {
    __hyperram_noinit_start = ALIGN(_region_min_align);
    *(.hyperram.noinit)
    *(".hyperram.noinit.*")
    __hyperram_noinit_end = ALIGN(_region_min_align);
  } > HYPERRAM

  __hyperram_end = ALIGN(_region_min_align);

#endif
