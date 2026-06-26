library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.vhsnunzip_utils_pkg.all;
use work.vhsnunzip_int_pkg.all;

-- Speculative dual-issue command generator stage 1.
--
-- Slot 0 is exactly the single-issue copy splitter (vhsnunzip_cmd_gen_1): it
-- takes the lead element and emits one partial command per cycle, splitting
-- copies longer than C_BYTES (or partial-overlap copies) into chunks over
-- multiple cycles. Slot 1 is a conservative accelerator: in the cycle the lead
-- element retires its final (single) chunk, if the next element is also a
-- single-chunk copy it is emitted as a second partial command the same cycle.
--
-- The flattened output (c1_0 then c1_1 where present) is identical to the
-- single-issue partial-command sequence, which the standalone testbench checks
-- against c1.tv.
entity vhsnunzip_cmd_gen_1_dual is
  port (
    clk         : in  std_logic;
    reset       : in  std_logic;

    -- Dual element information stream input.
    el          : in  dual_element_stream;
    el_ready    : out std_logic;

    -- Dual partial command stream output.
    c1          : out dual_partial_command_stream;
    c1_ready    : in  std_logic
  );
end vhsnunzip_cmd_gen_1_dual;

architecture behavior of vhsnunzip_cmd_gen_1_dual is

  -- Whether an element produces a single partial command in cmd_gen_1, i.e. its
  -- copy does not need splitting: no copy, or a copy that fits one C_BYTES chunk
  -- and does not hit the partial-overlap (cp_len >= cp_off, non-rle) split.
  function simple_copy(e : element_stream) return boolean is
  begin
    if e.cp_val = '0' then
      return true;
    elsif e.cp_len >= C_BYTES then
      return false;
    elsif e.cp_off <= 1 then
      return true;
    elsif e.cp_len < e.cp_off then
      return true;
    else
      return false;
    end if;
  end function;

  -- The single partial command for a simple element (slot 1). Mirrors the
  -- single-issue cmd_gen_1 output for an element that retires in one chunk.
  function simple_c1(e : element_stream) return partial_command_stream is
    variable c : partial_command_stream;
  begin
    c.valid  := '1';
    c.cp_off := e.cp_off;
    if e.cp_val = '1' then
      c.cp_len := signed(resize(e.cp_len, C_CNT));
    else
      c.cp_len := (others => '1');     -- -1, no copy
    end if;
    if e.cp_off <= 1 then
      c.cp_rle := '1';
    else
      c.cp_rle := '0';
    end if;
    c.li_val := e.li_val;
    c.li_off := e.li_off;
    c.li_len := e.li_len;
    c.ld_pop := e.ld_pop;
    c.last   := e.last;
    return c;
  end function;

begin
  proc: process (clk) is

    -- Element holding registers: e0 is the lead (slot 0, may be mid-split); e1
    -- is the lookahead (slot 1 candidate).
    variable e0     : element_stream := ELEMENT_STREAM_INIT;
    variable e1     : element_stream := ELEMENT_STREAM_INIT;

    -- Remaining copy length of e0, diminished-one; sign bit = inverted validity.
    variable cp_rem : signed(6 downto 0) := (others => '1');

    -- Output holding register.
    variable c1h    : dual_partial_command_stream := DUAL_PARTIAL_COMMAND_STREAM_INIT;

    variable chunk  : signed(C_CNT-1 downto 0);
    variable adv0   : boolean;

  begin
    if rising_edge(clk) then

      -- Invalidate the output register if it was shifted out.
      if c1_ready = '1' then
        c1h.c1_0.valid := '0';
        c1h.c1_1.valid := '0';
      end if;

      -- Latch a fresh dual transfer when both element slots are free.
      if e0.valid = '0' and e1.valid = '0' then
        e0 := el.el0;
        e1 := el.el1;
        if el.el0.valid = '1' and el.el0.cp_val = '1' then
          cp_rem := signed(resize(el.el0.cp_len, 7));
        else
          cp_rem := (others => '1');
        end if;
      end if;

      -- Process slot 0 when we have a lead element and the output is free.
      if e0.valid = '1' and c1h.c1_0.valid = '0' then
        c1h.c1_0.valid  := '1';
        c1h.c1_0.cp_off := e0.cp_off;

        -- Determine the chunk length for this cycle (cmd_gen_1 logic).
        if cp_rem < C_BYTES then
          chunk := cp_rem(C_CNT-1 downto 0);
        else
          chunk := to_signed(C_BYTES-1, C_CNT);
        end if;

        if e0.cp_off <= 1 then
          c1h.c1_0.cp_rle := '1';
        else
          if unsigned(chunk(C_IDX-1 downto 0)) >= e0.cp_off and chunk(C_IDX) = '0' then
            chunk(C_IDX-1 downto 0) := signed(resize(e0.cp_off(C_IDX downto 0) - 1, C_IDX));
            e0.cp_off(C_IDX downto 0) := e0.cp_off(C_IDX-1 downto 0) & "0";
          end if;
          c1h.c1_0.cp_rle := '0';
        end if;

        c1h.c1_0.cp_len := chunk;
        cp_rem := cp_rem - (resize(chunk, 7) + 1);
        adv0 := cp_rem(6) = '1';

        if adv0 then
          c1h.c1_0.li_val := e0.li_val;
          c1h.c1_0.ld_pop := e0.ld_pop;
          c1h.c1_0.last   := e0.last;
        else
          c1h.c1_0.li_val := '0';
          c1h.c1_0.ld_pop := '0';
          c1h.c1_0.last   := '0';
        end if;
        c1h.c1_0.li_off := e0.li_off;
        c1h.c1_0.li_len := e0.li_len;

        -- Slot 1 / element advance.
        if adv0 then
          if e1.valid = '1' and simple_copy(e1) then
            -- Co-issue the next element as a single partial command.
            c1h.c1_1 := simple_c1(e1);
            e0.valid := '0';
            e1.valid := '0';
          else
            -- Cannot pair; shift the lookahead into the lead slot.
            c1h.c1_1.valid := '0';
            e0 := e1;
            if e1.valid = '1' and e1.cp_val = '1' then
              cp_rem := signed(resize(e1.cp_len, 7));
            else
              cp_rem := (others => '1');
            end if;
            e1.valid := '0';
          end if;
        else
          c1h.c1_1.valid := '0';
        end if;

      end if;

      -- Handle reset.
      if reset = '1' then
        e0.valid := '0';
        e1.valid := '0';
        c1h.c1_0.valid := '0';
        c1h.c1_1.valid := '0';
        cp_rem := (others => '1');
      end if;

      -- Assign outputs.
      if e0.valid = '0' and e1.valid = '0' then
        el_ready <= '1';
      else
        el_ready <= '0';
      end if;
      c1 <= c1h;

    end if;
  end process;
end behavior;
