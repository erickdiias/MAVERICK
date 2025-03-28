--
-- Description 
-- Author: Erick S. Dias
-- last update: 27/03/25

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity A429_Tx is
    generic(
        clk_freq : integer := 100_000_000;
        tranmssion_speed : integer := 100_000;
        frame_label : std_logic_vector(7 downto 0) := X"4F"; -- 01001111 (79)
        sdi : std_logic_vector(1 downto 0) := "00"; -- 00-All Systems, 01-System1, 10-System2, 11-System3
        ssm : std_logic_vector(1 downto 0) := "00" -- Normal
    );
    port(
        clk, rst : in std_logic;
        data_in : in std_logic_vector(18 downto 0) -- Form discret
    );
end entity;

architecture main of A429_Tx is
    constant clk_div: natural := clk_freq / tranmssion_speed;
begin
end architecture;
