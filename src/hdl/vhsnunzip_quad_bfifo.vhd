-- Block RAM stream FIFOs of the quad toplevel (32 entries), replacing the
-- SRL FIFO (vhsnunzip_fifo).

-- Input FIFO (first-word fall-through). The read address runs one entry
-- ahead, so the RAM output register holds the head entry. The RAM read takes
-- a cycle: an entry can be read from the second cycle after it was written
-- on (the SRL FIFO: the first), which costs a cycle when the FIFO is empty.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity vhsnunzip_quad_ififo is
  generic (
    W           : natural
  );
  port (
    clk         : in  std_logic;
    reset       : in  std_logic;
    wr_valid    : in  std_logic;
    wr_ready    : out std_logic;
    wr_data     : in  std_logic_vector(W-1 downto 0);
    rd_valid    : out std_logic;
    rd_ready    : in  std_logic;
    rd_data     : out std_logic_vector(W-1 downto 0)
  );
end vhsnunzip_quad_ififo;

architecture behavior of vhsnunzip_quad_ififo is
  type mem_type is array (0 to 31) of std_logic_vector(W-1 downto 0);
  signal mem      : mem_type;
  attribute ram_style : string;
  attribute ram_style of mem : signal is "block";

  -- Level (all entries) and readable level (entries written before the
  -- last clock edge), diminished-one encoded as in vhsnunzip_fifo.
  signal level_s  : unsigned(5 downto 0) := (others => '1');
  signal rlevel   : unsigned(5 downto 0) := (others => '1');
  signal full_s   : std_logic := '0';
  signal wr_d     : std_logic := '0';
  signal wptr     : unsigned(4 downto 0) := (others => '0');
  signal rptr     : unsigned(4 downto 0) := (others => '0');
  signal rptr1    : unsigned(4 downto 0) := "00001";
  signal wr_ena   : std_logic;
  signal rd_ena   : std_logic;
  signal ra       : unsigned(4 downto 0);
begin

  wr_ready <= not full_s;
  wr_ena <= wr_valid and not full_s;
  rd_valid <= not rlevel(5);
  rd_ena <= rd_ready and not rlevel(5);
  ra <= rptr1 when rd_ena = '1' else rptr;

  reg_proc: process (clk) is
    variable level_v : unsigned(5 downto 0);
    variable rlevel_v : unsigned(5 downto 0);
  begin
    if rising_edge(clk) then
      level_v := level_s;
      rlevel_v := rlevel;
      if rd_ena = '1' then
        level_v := level_v - 1;
        rlevel_v := rlevel_v - 1;
      end if;
      if wr_ena = '1' then
        level_v := level_v + 1;
      end if;
      if wr_d = '1' then
        rlevel_v := rlevel_v + 1;
      end if;
      level_s <= level_v;
      rlevel <= rlevel_v;
      wr_d <= wr_ena;
      if level_v = 31 then
        full_s <= '1';
      else
        full_s <= '0';
      end if;

      if wr_ena = '1' then
        mem(to_integer(wptr)) <= wr_data;
        wptr <= wptr + 1;
      end if;
      rd_data <= mem(to_integer(ra));
      if rd_ena = '1' then
        rptr <= rptr1;
        rptr1 <= rptr1 + 1;
      end if;

      if reset = '1' then
        level_s <= (others => '1');
        rlevel <= (others => '1');
        full_s <= '0';
        wr_d <= '0';
        wptr <= (others => '0');
        rptr <= (others => '0');
        rptr1 <= "00001";
      end if;
    end if;
  end process;

end behavior;

-- Output FIFO (first-word fall-through) with an early write: the producer
-- hands over the data of an entry (pw_valid, pw_data) one cycle before it
-- is pushed (wr_valid), i.e. the RAM is written from the D side of the
-- producer's output register. The read address runs one entry ahead, so the
-- RAM output register always holds the head entry. Overflow must be
-- prevented by the producer (level).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity vhsnunzip_quad_ofifo is
  generic (
    W           : natural
  );
  port (
    clk         : in  std_logic;
    reset       : in  std_logic;
    pw_valid    : in  std_logic;
    pw_data     : in  std_logic_vector(W-1 downto 0);
    wr_valid    : in  std_logic;
    rd_valid    : out std_logic;
    rd_ready    : in  std_logic;
    rd_data     : out std_logic_vector(W-1 downto 0);
    level       : out unsigned(5 downto 0)
  );
end vhsnunzip_quad_ofifo;

architecture behavior of vhsnunzip_quad_ofifo is
  type mem_type is array (0 to 31) of std_logic_vector(W-1 downto 0);
  signal mem      : mem_type;
  attribute ram_style : string;
  attribute ram_style of mem : signal is "block";

  signal level_s  : unsigned(5 downto 0) := (others => '1');
  signal wptr     : unsigned(4 downto 0) := (others => '0');
  signal rptr     : unsigned(4 downto 0) := (others => '0');
  signal rptr1    : unsigned(4 downto 0) := "00001";
  signal rd_ena   : std_logic;
  signal ra       : unsigned(4 downto 0);
begin

  rd_valid <= not level_s(5);
  rd_ena <= rd_ready and not level_s(5);
  ra <= rptr1 when rd_ena = '1' else rptr;
  level <= level_s;

  reg_proc: process (clk) is
    variable level_v : unsigned(5 downto 0);
  begin
    if rising_edge(clk) then
      level_v := level_s;
      if rd_ena = '1' then
        level_v := level_v - 1;
      end if;
      if wr_valid = '1' then
        level_v := level_v + 1;
      end if;
      level_s <= level_v;

      if pw_valid = '1' then
        mem(to_integer(wptr)) <= pw_data;
        wptr <= wptr + 1;
      end if;
      rd_data <= mem(to_integer(ra));
      if rd_ena = '1' then
        rptr <= rptr1;
        rptr1 <= rptr1 + 1;
      end if;

      if reset = '1' then
        level_s <= (others => '1');
        wptr <= (others => '0');
        rptr <= (others => '0');
        rptr1 <= "00001";
      end if;
    end if;
  end process;

  -- pragma translate_off
  chk_proc: process (clk) is
  begin
    if rising_edge(clk) then
      assert not (wr_valid = '1' and rd_ena = '0' and level_s = 31)
        report "output FIFO overflow" severity failure;
    end if;
  end process;
  -- pragma translate_on

end behavior;
