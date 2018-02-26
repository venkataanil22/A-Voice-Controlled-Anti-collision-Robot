----------------------------------------------------------------------------------
----------------------------------------------------------------------------
-- Author:  Albert Fazakas
--          Copyright 2014 Digilent, Inc.
----------------------------------------------------------------------------
-- 
-- Create Date:    15:00:45 03/04/2014 
-- Design Name: 
-- Module Name:    AccelerometerCtl - Behavioral 
-- Project Name: 
-- Target Devices: 
-- Tool versions: 
-- Description: 
--       This is the main module for the the Nexys4 onboard ADXL362 accelerometer.
--    The module consists of two components, AXDX362Ctrl and AccelArithmetics. The first one
--    configures the ADXL362 accelerometer and continuously reads X, Y, Z acceleration data and
--    temperature data in 12-bit two's complement format. 
--       The data read is sent to the AccelArithmetics module that formats X and Y acceleration 
--    data to be displayed on the VGA screen in a 512 X 512 pixel area. Therefore the X and Y 
--    acceleration data will be scaled and limited to -1g: 0, 0g: 255, 1g: 511.
--       The AccelArithmetics module also determines the acceleration  magnitude using the
--    SQRT (X^2 + Y^2 + Z^2) formula. The magnitude value is also displayed on the VGA screen.
--    To perform SQRT calculation a Logicore component is used.
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: used for ECE 544 Lunar Rover Final Project to calculate speed 
--
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity Accel_top is
   port(
    clk_i          	: in  std_logic;
    rstn_i         	: in  std_logic;

    -- SPI Interface signals for the ADXL362 accelerometer
    sclk           	: out STD_LOGIC;
    mosi           	: out STD_LOGIC;
    miso           	: in STD_LOGIC;
    ss             	: out STD_LOGIC;
	  
	--accelerometer data out
	ACCEL_X    		: out STD_LOGIC_VECTOR (8 downto 0);
	ACCEL_Y    		: out STD_LOGIC_VECTOR (8 downto 0);
    ACCEL_Z    		: out STD_LOGIC_VECTOR (8 downto 0)
   );
end Accel_top;

architecture Behavioral of Accel_top is 

component AccelerometerCtl is
generic 
(
   SYSCLK_FREQUENCY_HZ : integer := 100000000;
   SCLK_FREQUENCY_HZ   : integer := 1000000;
   NUM_READS_AVG       : integer := 16;
   UPDATE_FREQUENCY_HZ : integer := 1000
);
port
(
 SYSCLK     : in STD_LOGIC; -- System Clock
 RESET      : in STD_LOGIC; -- Reset button on the Nexys4 board is active low

 -- SPI interface Signals
 SCLK       : out STD_LOGIC;
 MOSI       : out STD_LOGIC;
 MISO       : in STD_LOGIC;
 SS         : out STD_LOGIC;
 
-- Accelerometer data signals
 ACCEL_X_OUT    : out STD_LOGIC_VECTOR (8 downto 0);
 ACCEL_Y_OUT    : out STD_LOGIC_VECTOR (8 downto 0);
 ACCEL_Z_OUT    : out STD_LOGIC_VECTOR (8 downto 0);
 ACCEL_TMP_OUT  : out STD_LOGIC_VECTOR (11 downto 0)
);
end component;

----------------------------------------------------------------------------------
-- Signal Declarations
----------------------------------------------------------------------------------  
-- Inverted input reset signal
signal rst        : std_logic;
-- Reset signal conditioned by the PLL lock
signal reset      : std_logic;
signal resetn     : std_logic;
--signal locked     : std_logic;


-- ADXL362 Accelerometer data signals
signal ACCEL_TMP  : STD_LOGIC_VECTOR (11 downto 0);


begin

   -- The Reset Button on the Nexys4 board is active-low,
   -- however many components need an active-high reset
--   rst <= not rstn_i;

   -- Assign reset signals conditioned by the PLL lock
--   reset <= rst or (not locked);
   -- active-low version of the reset signal
--   resetn <= not reset;

----------------------------------------------------------------------------------
-- Accelerometer Controller
----------------------------------------------------------------------------------
   Inst_AccelerometerCtl: AccelerometerCtl
   generic map
   (
        SYSCLK_FREQUENCY_HZ   => 100000000,
        SCLK_FREQUENCY_HZ     => 100000,
        NUM_READS_AVG         => 16,
        UPDATE_FREQUENCY_HZ   => 1000
   )
   port map
   (
       SYSCLK     => clk_i,
       RESET      => reset, 
       -- Spi interface Signals
       SCLK       => sclk,
       MOSI       => mosi,
       MISO       => miso,
       SS         => ss,
     
      -- Accelerometer data signals
       ACCEL_X_OUT   => ACCEL_X,
       ACCEL_Y_OUT   => ACCEL_Y,
       ACCEL_Z_OUT   => ACCEL_Z
   );
end Behavioral;






