 
// HDMI_Top.v
// Wraps VGA_Controller and connects to Gowin DVI TX IP.
// No manual TMDS encoding needed — IP handles everything.
// Clocks from Main.v:
//   pix_clk      = 25.2 MHz  (vga_clk, from clkdiv5)
//   tmds_clk     = 126 MHz   (hdmi_serial_clk, from rpll_126, 5x pixel clock)

module HDMI_Top (
    input  wire        pix_clk,        // 25.2 MHz pixel clock
    input  wire        tmds_clk,       // 126 MHz serial clock (5x pix_clk)
    input  wire        sys_rst_n,

    // VRAM interface (pass-through to VGA_Controller)
    output wire [13:0] vram_read_addr,
    output wire        vram_read_en,
    input  wire [31:0] vram_read_data,  

    // V-sync for IO register 0x4040_0000
    output wire        vsync_out,

    // HDMI differential outputs
    output wire        tmds_clk_p,
    output wire        tmds_clk_n,
    output wire [2:0]  tmds_data_p,
    output wire [2:0]  tmds_data_n
);

// -------------------------------------------------------
// VGA Controller
// -------------------------------------------------------
wire h_sync, v_sync;
wire vga_de;
wire [2:0] red_3;
wire [2:0] green_3;
wire [1:0] blue_2;

assign vsync_out = v_sync;

VGA_Controller u_vga (
    .sys_clk        (pix_clk),
    .sys_rst_n      (sys_rst_n),
    .vram_read_addr (vram_read_addr),
    .vram_read_en   (vram_read_en),
    .vram_read_data (vram_read_data),
    .h_sync_i       (h_sync),
    .v_sync_i       (v_sync),
    .de             (vga_de),
    .RED            (red_3),
    .GREEN          (green_3),
    .BLUE           (blue_2)
);

// -------------------------------------------------------
// RGB332 -> RGB888 expansion
// -------------------------------------------------------
wire [7:0] red_8   = {red_3,   red_3,   red_3[2:1]};
wire [7:0] green_8 = {green_3, green_3, green_3[2:1]};
wire [7:0] blue_8  = {blue_2,  blue_2,  blue_2,  blue_2};

// -------------------------------------------------------
// Brightness boost: ~1.25x with saturation at 255
// -------------------------------------------------------
wire [8:0] r_boost = {1'b0, red_8}   + {3'b0, red_8[7:2]};
wire [8:0] g_boost = {1'b0, green_8} + {3'b0, green_8[7:2]};
wire [8:0] b_boost = {1'b0, blue_8}  + {3'b0, blue_8[7:2]};

wire [7:0] red_final   = r_boost[8] ? 8'hFF : r_boost[7:0];
wire [7:0] green_final = g_boost[8] ? 8'hFF : g_boost[7:0];
wire [7:0] blue_final  = b_boost[8] ? 8'hFF : b_boost[7:0];

// -------------------------------------------------------
// Data enable comes directly from VGA controller
// (eliminates synchronization mismatch with RGB data)
// -------------------------------------------------------
wire de = vga_de;

// -------------------------------------------------------
// Gowin DVI TX IP
// -------------------------------------------------------
DVI_TX_Top u_dvi_tx (
    .I_rst_n       (sys_rst_n),
    .I_serial_clk  (tmds_clk),    // 126 MHz
    .I_rgb_clk     (pix_clk),     // 25.2 MHz
    .I_rgb_vs      (v_sync),
    .I_rgb_hs      (h_sync),
    .I_rgb_de      (de),
    .I_rgb_r       (red_final),
    .I_rgb_g       (green_final),
    .I_rgb_b       (blue_final),
    .O_tmds_clk_p  (tmds_clk_p),
    .O_tmds_clk_n  (tmds_clk_n),
    .O_tmds_data_p (tmds_data_p),
    .O_tmds_data_n (tmds_data_n)
);

endmodule