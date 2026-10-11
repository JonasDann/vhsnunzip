library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.vhsnunzip_int_pkg.all;
use work.vhsnunzip_quad_pkg.all;

-- Streaming toplevel of the 32-byte, 4-slot (quad, R2-lite) Snappy
-- decompressor. Same stream conventions as vhsnunzip_unbuffered, with
-- 32-byte lines: co_cnt = 0 means 32 valid bytes; de_cnt is the number of
-- valid bytes (0..32) and the last transfer of a chunk may be empty.
entity vhsnunzip_quad is
  port (
    clk         : in  std_logic;
    reset       : in  std_logic;

    co_valid    : in  std_logic;
    co_ready    : out std_logic;
    co_data     : in  std_logic_vector(QB*8-1 downto 0);
    co_cnt      : in  std_logic_vector(QI-1 downto 0);
    co_last     : in  std_logic;

    de_valid    : out std_logic;
    de_ready    : in  std_logic;
    de_dvalid   : out std_logic;
    de_data     : out std_logic_vector(QB*8-1 downto 0);
    de_cnt      : out std_logic_vector(QI downto 0);
    de_last     : out std_logic
  );
end vhsnunzip_quad;

architecture behavior of vhsnunzip_quad is

  signal co_bytes   : byte_array(0 to QB-1);
  signal co_ctrl    : std_logic_vector(QI downto 0);
  signal cs_valid   : std_logic;
  signal cs_ready   : std_logic;
  signal cs_data    : byte_array(0 to QB-1);
  signal cs_ctrl    : std_logic_vector(QI downto 0);

  signal lb_we      : std_logic;
  signal lb_line    : unsigned(QI-1 downto 0);
  signal lb_data    : byte_array(0 to QB-1);
  signal lb_wpos    : unsigned(19 downto 0);
  signal cons_pos   : unsigned(15 downto 0);

  signal el         : qel_array(0 to QK-1);
  signal el_cnt     : unsigned(2 downto 0);
  signal el_space   : std_logic;

  signal wq_resv    : std_logic_vector(1 downto 0);
  signal stall      : std_logic := '0';
  signal gr         : qgroup_type;
  signal lt_rdat    : byte_matrix(0 to 3);

  signal ln_valid   : std_logic;
  signal ln_data    : byte_array(0 to QB-1);
  signal ln_line    : unsigned(10 downto 0);

  signal out_valid  : std_logic;
  signal out_data   : byte_array(0 to QB-1);
  signal out_cnt    : unsigned(QI downto 0);
  signal out_last   : std_logic;
  signal out_ctrl   : std_logic_vector(QI+1 downto 0);
  signal de_bytes   : byte_array(0 to QB-1);
  signal de_ctrl    : std_logic_vector(QI+1 downto 0);
  signal de_level   : unsigned(5 downto 0);

begin

  co_proc: process (co_data, co_cnt, co_last) is
  begin
    for i in 0 to QB-1 loop
      co_bytes(i) <= co_data(8*i+7 downto 8*i);
    end loop;
    co_ctrl(0) <= co_last;
    co_ctrl(QI downto 1) <= std_logic_vector(unsigned(co_cnt) - 1);
  end process;

  co_fifo_inst: vhsnunzip_fifo
    generic map (
      DATA_WIDTH  => QB,
      CTRL_WIDTH  => QI+1
    )
    port map (
      clk         => clk,
      reset       => reset,
      wr_valid    => co_valid,
      wr_ready    => co_ready,
      wr_data     => co_bytes,
      wr_ctrl     => co_ctrl,
      rd_valid    => cs_valid,
      rd_ready    => cs_ready,
      rd_data     => cs_data,
      rd_ctrl     => cs_ctrl
    );

  decoder_inst: vhsnunzip_quad_decoder
    port map (
      clk         => clk,
      reset       => reset,
      cs_valid    => cs_valid,
      cs_ready    => cs_ready,
      cs_data     => cs_data,
      cs_last     => cs_ctrl(0),
      cs_endi     => unsigned(cs_ctrl(QI downto 1)),
      lb_we       => lb_we,
      lb_line     => lb_line,
      lb_data     => lb_data,
      lb_wpos     => lb_wpos,
      cons_pos    => cons_pos,
      el          => el,
      el_cnt      => el_cnt,
      el_space    => el_space
    );

  group_inst: vhsnunzip_quad_group
    port map (
      clk         => clk,
      reset       => reset,
      el          => el,
      el_cnt      => el_cnt,
      el_space    => el_space,
      lb_wpos     => lb_wpos,
      wq_resv     => wq_resv,
      stall       => stall,
      gr          => gr
    );

  datapath_inst: vhsnunzip_quad_datapath
    port map (
      clk         => clk,
      reset       => reset,
      gr          => gr,
      lb_we       => lb_we,
      lb_line     => lb_line,
      lb_data     => lb_data,
      cons_pos    => cons_pos,
      lt_rdat     => lt_rdat,
      ln_valid    => ln_valid,
      ln_data     => ln_data,
      ln_line     => ln_line,
      out_valid   => out_valid,
      out_data    => out_data,
      out_cnt     => out_cnt,
      out_last    => out_last
    );

  lt_mem_inst: vhsnunzip_quad_lt_mem
    port map (
      clk         => clk,
      reset       => reset,
      rd_v        => gr.rd_v,
      rd_a        => gr.rd_a,
      rdat        => lt_rdat,
      wr_valid    => ln_valid,
      wr_line     => ln_line,
      wr_data     => ln_data,
      wq_resv     => wq_resv
    );

  out_ctrl(0) <= out_last;
  out_ctrl(QI+1 downto 1) <= std_logic_vector(out_cnt);

  de_fifo_inst: vhsnunzip_fifo
    generic map (
      DATA_WIDTH  => QB,
      CTRL_WIDTH  => QI+2
    )
    port map (
      clk         => clk,
      reset       => reset,
      wr_valid    => out_valid,
      wr_data     => out_data,
      wr_ctrl     => out_ctrl,
      rd_valid    => de_valid,
      rd_ready    => de_ready,
      rd_data     => de_bytes,
      rd_ctrl     => de_ctrl,
      level       => de_level
    );

  -- Stop issuing groups while the output FIFO holds 18 or more lines (the
  -- level output is the number of entries minus one); up to ~13 more lines
  -- can still be in flight.
  stall_proc: process (clk) is
  begin
    if rising_edge(clk) then
      if de_level(5) = '0' and de_level >= 17 then
        stall <= '1';
      else
        stall <= '0';
      end if;
      if reset = '1' then
        stall <= '1';
      end if;
    end if;
  end process;

  de_proc: process (de_bytes, de_ctrl) is
  begin
    for i in 0 to QB-1 loop
      de_data(8*i+7 downto 8*i) <= de_bytes(i);
    end loop;
    de_cnt <= de_ctrl(QI+1 downto 1);
    if unsigned(de_ctrl(QI+1 downto 1)) /= 0 then
      de_dvalid <= '1';
    else
      de_dvalid <= '0';
    end if;
    de_last <= de_ctrl(0);
  end process;

end behavior;
