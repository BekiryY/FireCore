//Copyright (C)2014-2026 GOWIN Semiconductor Corporation.
//All rights reserved.
//File Title: Timing Constraints file
//Tool Version: V1.9.11.03 (64-bit) 
//Created Time: 2026-05-19 09:44:40
create_clock -name sys_clk -period 37.037 -waveform {0 18.518} [get_ports {sys_clk}] -add
create_clock -name vga_clk -period 39.683 -waveform {0 19.841} [get_pins {u_clkdiv5/CLKOUT}] -add
create_clock -name ddr_user_clk -period 10 -waveform {0 5} [get_nets {main_ram_gw3_top_i4_ddr_user_clk}] -add
create_clock -name ddr_mem_clk -period 2.5 -waveform {0 1.25} [get_nets {gw3_top/i4/clk_x4i}] -add
create_clock -name hdmi_clk -period 7.937 -waveform {0 3.969} [get_nets {u_Gowin_rPLL_VGA_hdmi_serial_clk}] -add
set_clock_groups -asynchronous -group [get_clocks {sys_clk}] -group [get_clocks {vga_clk}] -group [get_clocks {ddr_mem_clk}] -group [get_clocks {ddr_user_clk}] -group [get_clocks {hdmi_clk}]

report_high_fanout_nets -max_nets 40
//report_max_frequency -mod_ins {SoC}
//report_max_frequency -mod_ins {u_soc_cpu}
//report_max_frequency -mod_ins {cpu_register_controller}
//report_max_frequency -mod_ins {dcache_inst}
//report_max_frequency -mod_ins {icache_inst}
//report_max_frequency -mod_ins {debug_uart}
//report_max_frequency -mod_ins {main_ram}
//report_max_frequency -mod_ins {vram}
//report_max_frequency -mod_ins {u_hdmi}
//report_max_frequency -mod_ins {spi_inst}
//report_max_frequency -mod_ins {spi_inst}
