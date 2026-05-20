module SoC(
    input sys_clk,
    input sys_rst_n,
    input MISO,
//    input PS2_DATA_I,
//    input PS2_CLK_I,
//    output PS2_DATA_DEBUG,
//    output PS2_CLK_DEBUG,
    input RX,
    output TX,
    output wire MOSI,
    output wire SPI_CS,
    output wire SPI_CLK,
    output wire debug_cs,
    output wire debug_clk,
    output wire debug_mosi,
    output wire debug_miso,

    output [13:0] ddr_addr,
    output [2:0]  ddr_bank,
    output        ddr_cs,

    output        ddr_ras,
    output        ddr_cas,
    output        ddr_we,
    output        ddr_ck,
    output        ddr_ck_n,
    output        ddr_cke,
    output        ddr_odt,
    output        ddr_reset_n,
    output [1:0]  ddr_dm,
    inout  [15:0] ddr_dq,
    inout  [1:0]  ddr_dqs,
    inout  [1:0]  ddr_dqs_n,

    // HDMI TMDS outputs (replaces VGA)
    output wire        tmds_clk_p,
    output wire        tmds_clk_n,
    output wire [2:0]  tmds_data_p,
    output wire [2:0]  tmds_data_n
) /* synthesis syn_netlist_hierarchy = 0 */;

// Flatten hierarchical netlist (Gowin SUG 5.12 syn_netlist_hierarchy). Splitting CPU/caches preserves RTL
// but can hurt QoR versus one flat module; this nudges synth back toward pre-split timing.

assign debug_cs = SPI_CS;
assign debug_clk = SPI_CLK;
assign debug_mosi = MOSI;
assign debug_miso = MISO;

assign PS2_DATA_DEBUG = ~PS2_DATA_I;
assign PS2_CLK_DEBUG = ~PS2_CLK_I;

// ==============================================================================
// 1. HARDWARE ARBITER & MULTIPLEXER WIRES
// ==============================================================================
wire [2:0]   ddr_cmd;
wire         ddr_cmd_en;
wire [26:0]  user_addr;
wire [127:0] ddr_wr_data;
wire         ddr_wr_data_en;
wire         ddr_wr_data_end;
wire [15:0]  ddr_wr_data_mask;
wire [7:0]   uart_data;
wire         uart_wr_en;

reg boot_done = 0;
wire cpu_done;

wire [2:0]   cpu_ddr_cmd;
wire         cpu_ddr_cmd_en;
wire [26:0]  cpu_user_addr;
wire [127:0] cpu_ddr_wr_data;
wire         cpu_ddr_wr_data_en;
wire         cpu_ddr_wr_data_end;
wire [15:0]  cpu_ddr_wr_data_mask;
wire [7:0]   cpu_uart_data;
wire         cpu_uart_wr_en;

wire cpu_active = (boot_done == 1 && cpu_done == 0);

assign ddr_cmd          = cpu_active ? cpu_ddr_cmd          : boot_ddr_cmd;
assign ddr_cmd_en       = cpu_active ? cpu_ddr_cmd_en       : boot_ddr_cmd_en;
assign user_addr        = cpu_active ? cpu_user_addr        : boot_user_addr;
assign ddr_wr_data      = cpu_active ? cpu_ddr_wr_data      : boot_ddr_wr_data;
assign ddr_wr_data_en   = cpu_active ? cpu_ddr_wr_data_en   : boot_ddr_wr_data_en;
assign ddr_wr_data_end  = cpu_active ? cpu_ddr_wr_data_end  : boot_ddr_wr_data_end;
assign ddr_wr_data_mask = cpu_active ? cpu_ddr_wr_data_mask : boot_ddr_wr_data_mask;
assign uart_data        = cpu_active ? cpu_uart_data        : boot_uart_data;
assign uart_wr_en       = cpu_active ? cpu_uart_wr_en       : boot_uart_wr_en;

// CPU IO READS (doomgeneric uyumlu)
wire [31:0]  ps2_data;        // IO 0x4010_0000 — live tuş bitmask
wire [31:0]  ps2_key_event;   // IO 0x4010_0004 — son tuş event {press[31], doomkey[7:0]}
wire         ps2_key_ev_valid; // KEY_EVENT geçerliliği (1-saat pulse)

// ==============================================================================
// 2. DDR3 MEMORY INTERFACE
// ==============================================================================
wire ddr_memory_clk;
wire ddr_pll_lock;
wire ddr_user_clk;
wire ddr_logic_rst;
wire ddr_calib_complete;
wire ddr_cmd_ready;
wire ddr_wr_data_rdy;
wire [127:0] ddr_rd_data;
wire ddr_rd_data_valid;
wire ddr_rd_data_end;

wire lock_270;
wire clk_270;

rpll_270 rpll_270_inst(
        .clkout(clk_270), //output clkout
        .lock(lock_270), //output lock
        .clkin(sys_clk) //input clkin
);

Gowin_rPLL r_clock(
    .clkout(ddr_memory_clk),
    .lock(ddr_pll_lock),
    .reset(~sys_rst_n), 
    .clkin(clk_270)
);

DDR3_Memory_Interface_Top main_ram(
    .clk(sys_clk),
    .memory_clk(ddr_memory_clk),
    .pll_lock(ddr_pll_lock & lock_270),
    .rst_n(sys_rst_n),
    .clk_out(ddr_user_clk),
    .ddr_rst(ddr_logic_rst),
    .init_calib_complete(ddr_calib_complete),
    .app_burst_number(6'b000000),
    .cmd_ready(ddr_cmd_ready),
    .cmd(ddr_cmd),
    .cmd_en(ddr_cmd_en),
    .addr({1'b0, user_addr}),
    .wr_data_rdy(ddr_wr_data_rdy),
    .wr_data(ddr_wr_data),
    .wr_data_en(ddr_wr_data_en),
    .wr_data_end(ddr_wr_data_end),
    .wr_data_mask(ddr_wr_data_mask),
    .rd_data(ddr_rd_data),
    .rd_data_valid(ddr_rd_data_valid),
    .rd_data_end(ddr_rd_data_end),
    .sr_req(1'b0),
    .ref_req(1'b0),
    .sr_ack(),
    .ref_ack(),
    .burst(1'b0),
    .O_ddr_addr(ddr_addr),
    .O_ddr_ba(ddr_bank),
    .O_ddr_cs_n(ddr_cs),
    .O_ddr_ras_n(ddr_ras),
    .O_ddr_cas_n(ddr_cas),
    .O_ddr_we_n(ddr_we),
    .O_ddr_clk(ddr_ck),
    .O_ddr_clk_n(ddr_ck_n),
    .O_ddr_cke(ddr_cke),
    .O_ddr_odt(ddr_odt),
    .O_ddr_reset_n(ddr_reset_n),
    .O_ddr_dqm(ddr_dm),
    .IO_ddr_dq(ddr_dq),
    .IO_ddr_dqs(ddr_dqs),
    .IO_ddr_dqs_n(ddr_dqs_n)
);

// ==============================================================================
// 3. UART CONTROLLER
// ==============================================================================
wire uart_write_done;

// key_catcher.py'den gelen klavye verisi (UART RX)
wire       uart_key_ready;   // 1-saat pulse: yeni byte geldi
wire [7:0] uart_key_data;    // gelen byte değeri

UART_Controller #(
    .BAUD_RATE(1000000),
    .CLOCK_FREQ(100000000)
) debug_uart (
    .sys_clk(ddr_user_clk),
    .sys_rst_n(sys_rst_n),
    .write_enable(uart_wr_en),
    .data_to_send(uart_data),
    .RX(RX),
    .TX(TX),
    .write_done(uart_write_done),
    .read_done(uart_key_ready),   // ← artık bağlı!
    .data_readed(uart_key_data)   // ← artık bağlı!
);

// ==============================================================================
// 4. SPI CONTROLLER
// ==============================================================================
reg spi_read_enable = 0;
reg [7:0] spi_cmd = 8'h03;
reg [23:0] spi_address = 24'h10_00_00;
reg [7:0] spi_total_bits = 160;
wire [127:0] spi_128bit_data;
wire spi_done;

SPI_Controller spi_inst (
    .sys_clk(ddr_user_clk),
    .sys_rst_n(sys_rst_n),
    .MISO(MISO),
    .read_enable(spi_read_enable),
    .command(spi_cmd),
    .address(spi_address),
    .total_bits(spi_total_bits),
    .data_readed(spi_128bit_data),
    .MOSI(MOSI),
    .CS(SPI_CS),
    .CLK(SPI_CLK),
    .done(spi_done)
);

// ==============================================================================
// 5. DMA BOOTLOADER STATE MACHINE
// ==============================================================================
localparam B_BOOT_DELAY      = 4'd0;
localparam B_WAIT_CALIB      = 4'd1;
localparam B_UART_SEND       = 4'd2;
localparam B_UART_WAIT_DONE  = 4'd3;
localparam B_SPI_START       = 4'd4;
localparam B_SPI_WAIT        = 4'd5;
localparam B_DDR_WAIT        = 4'd6;
localparam B_DDR_WRITE       = 4'd7;
localparam B_CS_HOLD         = 4'd8;
localparam B_CHECK_PROGRESS  = 4'd9;
localparam B_HALT            = 4'd10;

reg [3:0]  b_state = B_BOOT_DELAY;
reg [3:0]  b_return_state = B_BOOT_DELAY;
reg [24:0] timer = 0;
reg [1:0]  boot_byte_idx = 0;
reg [31:0] boot_uart_msg = 0;
reg [17:0] progress_counter = 0;

reg [2:0]   boot_ddr_cmd = 0;
reg         boot_ddr_cmd_en = 0;
reg [26:0]  boot_user_addr = 0;
reg [127:0] boot_ddr_wr_data = 0;
reg         boot_ddr_wr_data_en = 0;
reg         boot_ddr_wr_data_end = 0;
reg [7:0]   boot_uart_data = 0;
reg         boot_uart_wr_en = 0;

wire [15:0] boot_ddr_wr_data_mask = 16'h0000;

always @(posedge ddr_user_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        b_state <= B_BOOT_DELAY;
        spi_read_enable <= 0;
        boot_uart_wr_en <= 0;
        spi_address <= 24'h10_00_00;
        boot_user_addr <= 27'd0;
        boot_ddr_cmd_en <= 0;
        boot_ddr_wr_data_en <= 0;
        boot_ddr_wr_data_end <= 0;
        timer <= 0;
        boot_byte_idx <= 0;
        progress_counter <= 0;
        boot_done <= 0;
    end else if (!boot_done) begin
        case (b_state)
            B_BOOT_DELAY: begin
                if (timer >= 1_000_000) begin
                    timer <= 0;
                    boot_uart_msg <= 32'hDE_AD_BE_EF;
                    b_return_state <= B_WAIT_CALIB;
                    b_state <= B_UART_SEND;
                end else begin
                    timer <= timer + 25'd1;
                end
            end

            B_WAIT_CALIB: begin
                if (ddr_calib_complete) begin
                    boot_uart_msg <= 32'h44_44_52_33;
                    b_return_state <= B_SPI_START;
                    b_state <= B_UART_SEND;
                end
            end

            B_UART_SEND: begin
                case (boot_byte_idx)
                    2'd0: boot_uart_data <= boot_uart_msg[31:24];
                    2'd1: boot_uart_data <= boot_uart_msg[23:16];
                    2'd2: boot_uart_data <= boot_uart_msg[15:8];
                    2'd3: boot_uart_data <= boot_uart_msg[7:0];
                endcase
                boot_uart_wr_en <= 1;
                b_state <= B_UART_WAIT_DONE;
            end

            B_UART_WAIT_DONE: begin
                if (uart_write_done) begin
                    boot_uart_wr_en <= 0;
                    if (boot_byte_idx == 3) begin
                        boot_byte_idx <= 0;
                        b_state <= b_return_state;
                    end else begin
                        boot_byte_idx <= boot_byte_idx + 2'd1;
                        b_state <= B_UART_SEND;
                    end
                end
            end

            B_SPI_START: begin
                spi_cmd <= 8'h03;
                spi_total_bits <= 160;
                spi_read_enable <= 1;
                b_state <= B_SPI_WAIT;
            end

            B_SPI_WAIT: begin
                if (spi_done) begin
                    spi_read_enable <= 0;
                    b_state <= B_DDR_WAIT;
                end
            end

            B_DDR_WAIT: begin
                if (ddr_cmd_ready && ddr_wr_data_rdy) begin
                    boot_ddr_cmd <= 3'b000;
                    boot_ddr_cmd_en <= 1;
                    boot_ddr_wr_data <= spi_128bit_data;
                    boot_ddr_wr_data_en <= 1;
                    boot_ddr_wr_data_end <= 1;
                    b_state <= B_DDR_WRITE;
                end
            end

            B_DDR_WRITE: begin
                boot_ddr_cmd_en <= 0;
                boot_ddr_wr_data_en <= 0;
                boot_ddr_wr_data_end <= 0;
                spi_address <= spi_address + 24'd16;
                boot_user_addr <= boot_user_addr + 27'd8;
                progress_counter <= progress_counter + 18'd16;
                timer <= 0;
                b_state <= B_CS_HOLD;
            end

            B_CS_HOLD: begin
                if (timer >= 50) begin
                    b_state <= B_CHECK_PROGRESS;
                end else begin
                    timer <= timer + 25'd1;
                end
            end

            B_CHECK_PROGRESS: begin
                if (spi_address >= 24'h80_00_00) begin
                    boot_uart_msg <= 32'h44_4F_4E_45;
                    b_return_state <= B_HALT;
                    b_state <= B_UART_SEND;
                end
                else if (progress_counter >= 102400) begin
                    progress_counter <= 0;
                    boot_uart_msg <= 32'h2B_2B_2B_2B;
                    b_return_state <= B_SPI_START;
                    b_state <= B_UART_SEND;
                end
                else begin
                    b_state <= B_SPI_START;
                end
            end

            B_HALT: begin
                boot_done <= 1;
                b_state <= B_HALT;
            end

            default: b_state <= B_BOOT_DELAY;
        endcase
    end
    // --- CTRL+ESC SOFT REBOOT TRIGGER FOR BOOTLOADER ---
    else if (cpu_done && ps2_data[4] && ps2_data[7]) begin
        b_state <= B_BOOT_DELAY;
        spi_read_enable <= 0;
        boot_uart_wr_en <= 0;
        spi_address <= 24'h10_00_00;
        boot_user_addr <= 27'd0;
        boot_ddr_cmd_en <= 0;
        boot_ddr_wr_data_en <= 0;
        boot_ddr_wr_data_end <= 0;
        timer <= 0;
        boot_byte_idx <= 0;
        progress_counter <= 0;
        boot_done <= 0;
    end
end

// ==============================================================================
// Keyboard Controller (PS/2)
// ==============================================================================
uart2PS2 u_uart2PS2 (
    .sys_clk(ddr_user_clk),
    .sys_rst_n(sys_rst_n),
    .PS2_D_I(PS2_DATA_I),
    .PS2_CLK_I(PS2_CLK_I),
    .uart_key_ready(uart_key_ready),  // PC klavyesi: key_catcher.py → UART
    .uart_key_data(uart_key_data),    // PC klavyesi: gelen byte
    .DOOM_KEYS(ps2_data),
    .KEY_EVENT(ps2_key_event),
    .KEY_EV_VALID(ps2_key_ev_valid)
);

// ==============================================================================
// HDMI Controller (replaces VGA)
// ==============================================================================

// Clock generation: Gowin_rPLL_VGA -> 126 MHz serial clock, CLKDIV/5 -> 25.2 MHz pixel clock
wire hdmi_serial_clk;
wire vga_clk;
wire vga_lock;

Gowin_rPLL_VGA u_Gowin_rPLL_VGA (
    .clkin(sys_clk),
    .lock(vga_lock),
    .reset(~sys_rst_n),
    .clkout(hdmi_serial_clk)
);

CLKDIV u_clkdiv5 (
    .HCLKIN (hdmi_serial_clk),
    .CLKOUT (vga_clk),
    .RESETN (sys_rst_n),
    .CALIB  (1'b1)
);
defparam u_clkdiv5.DIV_MODE = "5";
defparam u_clkdiv5.GSREN    = "false";

wire hdmi_rst_n = sys_rst_n & vga_lock;

wire [13:0] vga_read_addr;
wire        vga_read_en;
wire [31:0] active_display_data;
wire        hdmi_vsync;

// 2-stage CDC: hdmi_vsync (vga_clk domain) → ddr_user_clk domain
// Prevents metastability when the CPU polls 0x40400000 for vsync
reg hdmi_vsync_d1 = 1, hdmi_vsync_d2 = 1;
always @(posedge ddr_user_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        hdmi_vsync_d1 <= 1;
        hdmi_vsync_d2 <= 1;
    end else begin
        hdmi_vsync_d1 <= hdmi_vsync;
        hdmi_vsync_d2 <= hdmi_vsync_d1;
    end
end

// Legacy ordering: VRAM + display mux adjacent to hdmi_vsync CDC, then HDMI last.
// (Same topology as flat SoC; helps keep read-data path LUTs clustered near SDRAM pixel clock.)

// ==============================================================================
// CPU subsystem (timer, core, I/D caches)
// ==============================================================================
wire [13:0] vram_addr_out;
wire [31:0] vram_data_out;
wire        vram_write_en;

SoC_CPU u_soc_cpu (
    .clk                 (ddr_user_clk),
    .rst_n               (sys_rst_n),
    .boot_done           (boot_done),
    .ps2_data            (ps2_data),
    .ps2_key_event       (ps2_key_event),
    .hdmi_vsync_sync     (hdmi_vsync_d2),
    .uart_write_done     (uart_write_done),
    .ddr_cmd_ready       (ddr_cmd_ready),
    .ddr_wr_data_rdy     (ddr_wr_data_rdy),
    .ddr_rd_data         (ddr_rd_data),
    .ddr_rd_data_valid   (ddr_rd_data_valid),
    .cpu_done            (cpu_done),
    .vram_addr_out       (vram_addr_out),
    .vram_data_out       (vram_data_out),
    .vram_write_en       (vram_write_en),
    .cpu_ddr_cmd         (cpu_ddr_cmd),
    .cpu_ddr_cmd_en      (cpu_ddr_cmd_en),
    .cpu_user_addr       (cpu_user_addr),
    .cpu_ddr_wr_data     (cpu_ddr_wr_data),
    .cpu_ddr_wr_data_en  (cpu_ddr_wr_data_en),
    .cpu_ddr_wr_data_end (cpu_ddr_wr_data_end),
    .cpu_ddr_wr_data_mask(cpu_ddr_wr_data_mask),
    .cpu_uart_data       (cpu_uart_data),
    .cpu_uart_wr_en      (cpu_uart_wr_en)
);

// ==============================================================================
// VRAM (Main Display) & CRASH MULTIPLEXER (Algorithmic BSOD)
// ==============================================================================
wire [31:0] main_vram_read_data;

Gowin_SDPB_VRAM vram(
    .clka(ddr_user_clk),
    .cea(vram_write_en),
    .reseta(~sys_rst_n),
    .ada(vram_addr_out),
    .din(vram_data_out),

    .clkb(vga_clk),
    .ceb(vga_read_en),
    .resetb(~sys_rst_n),
    .oce(1'b0),
    .adb(vga_read_addr),
    .dout(main_vram_read_data)
);

// If CPU halts, override the VRAM data and output Pure Blue (0x03030303)
assign active_display_data = cpu_done ? 32'h03_03_03_03 : main_vram_read_data;

HDMI_Top u_hdmi (
    .pix_clk       (vga_clk),
    .tmds_clk      (hdmi_serial_clk),
    .sys_rst_n     (hdmi_rst_n),
    .vram_read_addr(vga_read_addr),
    .vram_read_en  (vga_read_en),
    .vram_read_data(active_display_data),
    .vsync_out     (hdmi_vsync),
    .tmds_clk_p    (tmds_clk_p),
    .tmds_clk_n    (tmds_clk_n),
    .tmds_data_p   (tmds_data_p),
    .tmds_data_n   (tmds_data_n)
);

endmodule
