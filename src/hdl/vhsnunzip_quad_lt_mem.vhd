library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.vhsnunzip_int_pkg.all;
use work.vhsnunzip_quad_pkg.all;

-- Long-term history for the quad datapath: 64 KiB as an even and an odd bank
-- of 1024 lines of 32 bytes, using both ports of every bank for reads. Port B
-- always reads; port A reads when the group former grants it, and otherwise
-- writes the next completed output line from the bank's write queue. When a
-- queue fills up, wq_resv tells the group former to leave port A free.
entity vhsnunzip_quad_lt_mem is
  port (
    clk         : in  std_logic;
    reset       : in  std_logic;

    -- Reads (0 = even B, 1 = even A, 2 = odd B, 3 = odd A); the data is valid
    -- four cycles later.
    rd_v        : in  std_logic_vector(3 downto 0);
    rd_a        : in  lt_addr_array(0 to 3);
    rdat        : out byte_matrix(0 to 3);

    -- Completed output lines.
    wr_valid    : in  std_logic;
    wr_line     : in  unsigned(10 downto 0);
    wr_data     : in  byte_array(0 to QB-1);

    wq_resv     : out std_logic_vector(1 downto 0)
  );
end vhsnunzip_quad_lt_mem;

architecture behavior of vhsnunzip_quad_lt_mem is
  constant WQ_LOG2 : natural := 4;
  type wq_mem_type is array (0 to 2**WQ_LOG2-1) of std_logic_vector(265 downto 0);
  type uv_array is array (natural range <>) of std_logic_vector(255 downto 0);
begin

  bank_gen: for b in 0 to 1 generate
    signal wq      : wq_mem_type;
    signal wptr    : unsigned(WQ_LOG2 downto 0) := (others => '0');
    signal rptr    : unsigned(WQ_LOG2 downto 0) := (others => '0');
    signal cnt     : unsigned(WQ_LOG2 downto 0);
    signal head    : std_logic_vector(265 downto 0);
    signal drain   : std_logic;
    signal a_en    : std_logic;
    signal a_addr  : unsigned(9 downto 0);
    signal din     : std_logic_vector(255 downto 0);
    signal a_dout  : std_logic_vector(255 downto 0);
    signal b_dout  : std_logic_vector(255 downto 0);
  begin

    cnt <= wptr - rptr;
    head <= wq(to_integer(rptr(WQ_LOG2-1 downto 0)));
    drain <= '1' when cnt /= 0 and rd_v(2*b+1) = '0' else '0';

    wq_proc: process (clk) is
      variable wd : std_logic_vector(265 downto 0);
    begin
      if rising_edge(clk) then
        if wr_valid = '1' and wr_line(0) = to_unsigned(b, 1)(0) then
          for i in 0 to QB-1 loop
            wd(8*i+7 downto 8*i) := wr_data(i);
          end loop;
          wd(265 downto 256) := std_logic_vector(wr_line(10 downto 1));
          wq(to_integer(wptr(WQ_LOG2-1 downto 0))) <= wd;
          wptr <= wptr + 1;
        end if;
        if drain = '1' then
          rptr <= rptr + 1;
        end if;
        if cnt >= 8 then
          wq_resv(b) <= '1';
        else
          wq_resv(b) <= '0';
        end if;
        if reset = '1' then
          wptr <= (others => '0');
          rptr <= (others => '0');
          wq_resv(b) <= '0';
        end if;
      end if;
    end process;

    a_en <= rd_v(2*b+1) or drain;
    a_addr <= rd_a(2*b+1) when rd_v(2*b+1) = '1' else unsigned(head(265 downto 256));

    ram_inst: entity work.vhsnunzip_quad_uram
      port map (
        clk     => clk,
        a_en    => a_en,
        a_we    => drain,
        a_addr  => a_addr,
        a_din   => head(255 downto 0),
        a_dout  => a_dout,
        b_en    => rd_v(2*b),
        b_addr  => rd_a(2*b),
        b_dout  => b_dout
      );

    out_proc: process (a_dout, b_dout) is
    begin
      for i in 0 to QB-1 loop
        rdat(2*b)(i) <= b_dout(8*i+7 downto 8*i);
        rdat(2*b+1)(i) <= a_dout(8*i+7 downto 8*i);
      end loop;
    end process;

  end generate;

end behavior;
