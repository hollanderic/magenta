/* libunwind - a platform-independent unwind library
   Copyright (C) 2010, 2011 by FERMI NATIONAL ACCELERATOR LABORATORY
   Copyright (C) 2014 CERN and Aalto University
        Contributed by Filip Nyback

This file is part of libunwind.

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.  */

#include "unwind_i.h"
#include "dwarf_i.h"

HIDDEN void
tdep_stash_frame (struct dwarf_cursor *d, struct dwarf_reg_state *rs)
{
  struct cursor *c = (struct cursor *) dwarf_to_cursor (d);
  unw_tdep_frame_t *f = &c->frame_info;

  Debug (4, "ip=0x%x cfa=0x%x type %d cfa [where=%d val=%d] cfaoff=%d"
         " ra=0x%x r7 [where=%d val=%d @0x%x] lr [where=%d val=%d @0x%x] "
         "sp [where=%d val=%d @0x%x]\n",
         (int) d->ip, (int) d->cfa, f->frame_type,
         rs->reg[DWARF_CFA_REG_COLUMN].where,
         (int) rs->reg[DWARF_CFA_REG_COLUMN].val,
         (int) rs->reg[DWARF_CFA_OFF_COLUMN].val,
         (int) DWARF_GET_LOC(d->loc[d->ret_addr_column]),
         rs->reg[R7].where, (int) rs->reg[R7].val, (int) DWARF_GET_LOC(d->loc[R7]),
         rs->reg[LR].where, (int) rs->reg[LR].val, (int) DWARF_GET_LOC(d->loc[LR]),
         rs->reg[SP].where, (int) rs->reg[SP].val, (int) DWARF_GET_LOC(d->loc[SP]));

  /* A standard frame is defined as:
      - CFA is register-relative offset off R7 or SP;
      - Return address is saved in LR;
      - R7 is unsaved or saved at CFA+offset, offset != -1;
      - LR is unsaved or saved at CFA+offset, offset != -1;
      - SP is unsaved or saved at CFA+offset, offset != -1.  */
  if (f->frame_type == UNW_ARM_FRAME_OTHER
      && (rs->reg[DWARF_CFA_REG_COLUMN].where == DWARF_WHERE_REG)
      && (rs->reg[DWARF_CFA_REG_COLUMN].val == R7
          || rs->reg[DWARF_CFA_REG_COLUMN].val == SP)
      && labs(rs->reg[DWARF_CFA_OFF_COLUMN].val) < (1 << 29)
      && d->ret_addr_column == LR
      && (rs->reg[R7].where == DWARF_WHERE_UNDEF
          || rs->reg[R7].where == DWARF_WHERE_SAME
          || (rs->reg[R7].where == DWARF_WHERE_CFAREL
              && labs(rs->reg[R7].val) < (1 << 29)
              && rs->reg[R7].val+1 != 0))
      && (rs->reg[LR].where == DWARF_WHERE_UNDEF
          || rs->reg[LR].where == DWARF_WHERE_SAME
          || (rs->reg[LR].where == DWARF_WHERE_CFAREL
              && labs(rs->reg[LR].val) < (1 << 29)
              && rs->reg[LR].val+1 != 0))
      && (rs->reg[SP].where == DWARF_WHERE_UNDEF
          || rs->reg[SP].where == DWARF_WHERE_SAME
          || (rs->reg[SP].where == DWARF_WHERE_CFAREL
              && labs(rs->reg[SP].val) < (1 << 29)
              && rs->reg[SP].val+1 != 0)))
  {
    /* Save information for a standard frame. */
    f->frame_type = UNW_ARM_FRAME_STANDARD;
    f->cfa_reg_sp = (rs->reg[DWARF_CFA_REG_COLUMN].val == SP);
    f->cfa_reg_offset = rs->reg[DWARF_CFA_OFF_COLUMN].val;
    if (rs->reg[R7].where == DWARF_WHERE_CFAREL)
      f->fp_cfa_offset = rs->reg[R7].val;
    if (rs->reg[LR].where == DWARF_WHERE_CFAREL)
      f->lr_cfa_offset = rs->reg[LR].val;
    if (rs->reg[SP].where == DWARF_WHERE_CFAREL)
      f->sp_cfa_offset = rs->reg[SP].val;
    Debug (4, " standard frame\n");
  }
  else
    Debug (4, " unusual frame\n");
}

