--
-- Author: Erick S. Dias
-- Last update: 26/03/25 

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

entity clk_div is
    generic(
        clk_div_max : integer := 16
    );
    port(
        clk, rst : in std_logic;
        clk_out  : out std_logic
    );
end entity;

architecture main of clk_div is
    constant clk_div_width  : integer := integer(ceil(log2(real(clk_div_max))));
    signal clk_div_cnt      : unsigned(clk_div_width - 1 downto 0) := (others => '0');
    signal clk_reg          : std_logic := '0';

begin
    process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                clk_div_cnt <= (others => '0');
                clk_reg <= '0';
            elsif clk_div_cnt = clk_div_max - 1 then
                clk_div_cnt <= (others => '0');
                clk_reg <= not clk_reg;
            else
                clk_div_cnt <= clk_div_cnt + 1;
            end if;
        end if;
    end process;
    
    clk_out <= clk_reg;

end architecture;
