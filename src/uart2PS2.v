// =============================================================================
// PS2_Controller.v  — GoWin Tang Primer 20K  DOOM FPGA
// =============================================================================
//
// doomgeneric.h arayüzüne göre tasarlandı:
//   DG_GetKey(int* pressed, unsigned char* doomKey)
//   → Hardware IO 0x4010_0000 = DOOM_KEYS bitmask  (live state)
//   → Hardware IO 0x4010_0004 = KEY_EVENT           (en son event)
//     [31]    = 1:press / 0:release
//     [7:0]   = doomkeys.h değeri (KEY_UPARROW=0xAD, KEY_FIRE=0xA3 vb.)
//
// doomkeys.h sabit değerleri:
//   KEY_RIGHTARROW = 0xAE    KEY_LEFTARROW  = 0xAC
//   KEY_UPARROW    = 0xAD    KEY_DOWNARROW  = 0xAF
//   KEY_FIRE(Ctrl) = 0xA3    KEY_USE(Space) = 0xA2
//   KEY_RSHIFT     = 0xB6    KEY_ESCAPE     = 27
//   KEY_ENTER      = 13      KEY_TAB        = 9
//   KEY_RALT/LALT  = 0xB8    KEY_F1..F10    = 0xBB..0xC4
//
// DOOM_KEYS bit atamaları (mevcut C koduyla uyumlu):
//   [0]=İleri(Up)  [1]=Geri(Down)  [2]=SolDön(Left)  [3]=SağDön(Right)
//   [4]=Ateş(Ctrl) [5]=Kullan(Space) [6]=Koş(Shift)  [7]=Esc
//   [8]=Enter      [9]=Tab          [10]=Sol(strafeL) [11]=Sağ(strafeR)
//   [12]=Alt        [13]=F1 [14]=F2 [15]=F3 [16]=F4 [17]=F5
//   [18]=F6 [19]=F7 [20]=F8 [21]=F9 [22]=F10
//   [23]=+ [24]=-
// =============================================================================
module uart2PS2(
    input  wire        sys_clk,      // ~100 MHz (ddr_user_clk)
    input  wire        sys_rst_n,
    input  wire        PS2_D_I,
    input  wire        PS2_CLK_I,
    // --- UART klavye girişi (key_catcher.py → COM6) ---
    input  wire        uart_key_ready,  // UART_Controller.read_done
    input  wire [7:0]  uart_key_data,   // UART_Controller.data_readed
    output reg  [31:0] DOOM_KEYS    = 32'd0,  // IO 0x4010_0000
    output reg  [31:0] KEY_EVENT    = 32'd0,  // IO 0x4010_0004
    output reg         KEY_EV_VALID = 1'b0    // 1-saat pulse
);

// ---------------------------------------------------------------------------
// 1. Level-shift (donanım open-collector tersleyici)
// ---------------------------------------------------------------------------
wire ps2_d_raw   = ~PS2_D_I;
wire ps2_clk_raw = ~PS2_CLK_I;

// ---------------------------------------------------------------------------
// 1b. 1 ms tick üreteci  (100 MHz → 1 kHz)
// ---------------------------------------------------------------------------
reg [16:0] ms_div  = 0;
reg        ms_tick = 0;

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        ms_div  <= 0;
        ms_tick <= 0;
    end else begin
        ms_tick <= 0;
        if (ms_div >= 17'd99_999) begin
            ms_div  <= 0;
            ms_tick <= 1;
        end else begin
            ms_div <= ms_div + 17'd1;
        end
    end
end

// ---------------------------------------------------------------------------
// 2. Debounce filtresi (~320 ns, 32 sample)
// ---------------------------------------------------------------------------
reg [4:0] clk_filt = 0, dat_filt = 0;
reg ps2_clk_c = 1, ps2_dat_c = 1;

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        clk_filt <= 0; dat_filt <= 0;
        ps2_clk_c <= 1; ps2_dat_c <= 1;
    end else begin
        if (ps2_clk_raw) begin
            if (clk_filt != 5'h1F) clk_filt <= clk_filt + 5'd1;
            else ps2_clk_c <= 1;
        end else begin
            if (clk_filt != 5'h00) clk_filt <= clk_filt - 5'd1;
            else ps2_clk_c <= 0;
        end
        if (ps2_d_raw) begin
            if (dat_filt != 5'h1F) dat_filt <= dat_filt + 5'd1;
            else ps2_dat_c <= 1;
        end else begin
            if (dat_filt != 5'h00) dat_filt <= dat_filt - 5'd1;
            else ps2_dat_c <= 0;
        end
    end
end

// ---------------------------------------------------------------------------
// 3. Falling-edge detect
// ---------------------------------------------------------------------------
reg  ps2_clk_prev = 1;
wire clk_fall     = (~ps2_clk_c & ps2_clk_prev);

// ---------------------------------------------------------------------------
// 4. Timeout watchdog (~2 ms)
// ---------------------------------------------------------------------------
reg [17:0] timeout_cnt = 0;
wire       timed_out   = (timeout_cnt >= 18'd200_000);

// ---------------------------------------------------------------------------
// 5. PS/2 seri alıcı
// ---------------------------------------------------------------------------
reg [10:0] shift_reg  = 0;
reg [3:0]  bit_cnt    = 0;
reg [7:0]  ps2_byte   = 0;
reg        byte_rdy   = 0;

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        ps2_clk_prev <= 1; shift_reg <= 0; bit_cnt <= 0;
        ps2_byte <= 0; byte_rdy <= 0; timeout_cnt <= 0;
    end else begin
        ps2_clk_prev <= ps2_clk_c;
        byte_rdy     <= 0;

        if (bit_cnt > 0) begin
            timeout_cnt <= timeout_cnt + 18'd1;
            if (timed_out) begin bit_cnt <= 0; timeout_cnt <= 0; end
        end else begin
            timeout_cnt <= 0;
        end

        if (clk_fall) begin
            timeout_cnt <= 0;
            if (bit_cnt == 0) begin
                if (!ps2_dat_c) begin
                    shift_reg[0] <= 0;
                    bit_cnt <= 1;
                end
            end else begin
                shift_reg[bit_cnt] <= ps2_dat_c;
                if (bit_cnt == 10) begin
                    if (shift_reg[0] == 0 && ps2_dat_c == 1) begin
                        ps2_byte <= shift_reg[8:1];
                        byte_rdy <= 1;
                    end
                    bit_cnt <= 0;
                end else begin
                    bit_cnt <= bit_cnt + 4'd1;
                end
            end
        end
    end
end

// ---------------------------------------------------------------------------
// 6. Protokol dekoder
//    E0 prefix → is_ext=1, F0 → release, sonra scan kodu
// ---------------------------------------------------------------------------
localparam S_IDLE = 2'd0;
localparam S_EXT  = 2'd1;   // E0 alındı
localparam S_REL  = 2'd2;   // F0 alındı (release gelecek)

reg [1:0] dec_st = S_IDLE;
reg       is_ext = 0;        // Son prefix E0 mıydı?

// ---------------------------------------------------------------------------
// 7. PS/2 scan kodu → {DOOM_KEYS_bit[4:0], doomkey_value[7:0]} dönüştürücü
//
//    doomkeys.h sabitlerine birebir uygun
// ---------------------------------------------------------------------------
// doomkey değerleri (doomkeys.h)
localparam [7:0] DK_UP    = 8'hAD;  // KEY_UPARROW    → ileri
localparam [7:0] DK_DOWN  = 8'hAF;  // KEY_DOWNARROW  → geri
localparam [7:0] DK_LEFT  = 8'hAC;  // KEY_LEFTARROW  → sola dön
localparam [7:0] DK_RIGHT = 8'hAE;  // KEY_RIGHTARROW → sağa dön
localparam [7:0] DK_FIRE  = 8'hA3;  // KEY_FIRE       → Ctrl
localparam [7:0] DK_USE   = 8'hA2;  // KEY_USE        → Space
localparam [7:0] DK_SHIFT = 8'hB6;  // KEY_RSHIFT     → Shift
localparam [7:0] DK_ESC   = 8'd27;  // KEY_ESCAPE
localparam [7:0] DK_ENTER = 8'd13;  // KEY_ENTER
localparam [7:0] DK_TAB   = 8'd9;   // KEY_TAB
localparam [7:0] DK_ALT   = 8'hB8;  // KEY_LALT/RALT
localparam [7:0] DK_F1    = 8'hBB;  // KEY_F1  = 0x80+0x3B
localparam [7:0] DK_F2    = 8'hBC;
localparam [7:0] DK_F3    = 8'hBD;
localparam [7:0] DK_F4    = 8'hBE;
localparam [7:0] DK_F5    = 8'hBF;
localparam [7:0] DK_F6    = 8'hC0;
localparam [7:0] DK_F7    = 8'hC1;
localparam [7:0] DK_F8    = 8'hC2;
localparam [7:0] DK_F9    = 8'hC3;
localparam [7:0] DK_F10   = 8'hC4;
localparam [7:0] DK_PLUS  = 8'h3D;  // KEY_EQUALS (+)
localparam [7:0] DK_MINUS = 8'h2D;  // KEY_MINUS  (-)
localparam [7:0] DK_NONE  = 8'h00;  // tanımsız

// Dönüştürücü fonksiyon: {bit[4:0], doomkey[7:0]} = 13-bit
// bit=5'h1F → tanımsız (görmezden gel)
function [12:0] scan2doom;
    input       ext;
    input [7:0] sc;
    begin
        case ({ext, sc})
            // ---- Normal (non-extended) tuşlar ----
            {1'b0, 8'h1D}: scan2doom = {5'd0,  DK_UP};    // W → ileri
            {1'b0, 8'h1B}: scan2doom = {5'd1,  DK_DOWN};  // S → geri
            {1'b0, 8'h1C}: scan2doom = {5'd2,  DK_LEFT};  // A → sol strafe
            {1'b0, 8'h23}: scan2doom = {5'd3,  DK_RIGHT}; // D → sağ strafe
            {1'b0, 8'h14}: scan2doom = {5'd4,  DK_FIRE};  // L-Ctrl
            {1'b0, 8'h29}: scan2doom = {5'd5,  DK_USE};   // Space
            {1'b0, 8'h12}: scan2doom = {5'd6,  DK_SHIFT}; // L-Shift
            {1'b0, 8'h59}: scan2doom = {5'd6,  DK_SHIFT}; // R-Shift
            {1'b0, 8'h76}: scan2doom = {5'd7,  DK_ESC};   // Esc
            {1'b0, 8'h5A}: scan2doom = {5'd8,  DK_ENTER}; // Enter
            {1'b0, 8'h0D}: scan2doom = {5'd9,  DK_TAB};   // Tab
            {1'b0, 8'h11}: scan2doom = {5'd12, DK_ALT};   // L-Alt
            {1'b0, 8'h05}: scan2doom = {5'd13, DK_F1};    // F1
            {1'b0, 8'h06}: scan2doom = {5'd14, DK_F2};    // F2
            {1'b0, 8'h04}: scan2doom = {5'd15, DK_F3};    // F3
            {1'b0, 8'h0C}: scan2doom = {5'd16, DK_F4};    // F4
            {1'b0, 8'h03}: scan2doom = {5'd17, DK_F5};    // F5
            {1'b0, 8'h0B}: scan2doom = {5'd18, DK_F6};    // F6
            {1'b0, 8'h83}: scan2doom = {5'd19, DK_F7};    // F7
            {1'b0, 8'h0A}: scan2doom = {5'd20, DK_F8};    // F8
            {1'b0, 8'h01}: scan2doom = {5'd21, DK_F9};    // F9
            {1'b0, 8'h09}: scan2doom = {5'd22, DK_F10};   // F10
            {1'b0, 8'h55}: scan2doom = {5'd23, DK_PLUS};  // =
            {1'b0, 8'h4E}: scan2doom = {5'd24, DK_MINUS}; // -
            // ---- Extended (E0) tuşlar — OK tuşları ----
            {1'b1, 8'h75}: scan2doom = {5'd0,  DK_UP};    // ↑ → ileri (bit 0, aynı W gibi)
            {1'b1, 8'h72}: scan2doom = {5'd1,  DK_DOWN};  // ↓ → geri
            {1'b1, 8'h6B}: scan2doom = {5'd2,  DK_LEFT};  // ← → sola dön
            {1'b1, 8'h74}: scan2doom = {5'd3,  DK_RIGHT}; // → → sağa dön
            {1'b1, 8'h14}: scan2doom = {5'd4,  DK_FIRE};  // R-Ctrl → ateş
            {1'b1, 8'h11}: scan2doom = {5'd12, DK_ALT};   // R-Alt
            {1'b1, 8'h5A}: scan2doom = {5'd8,  DK_ENTER}; // Numpad Enter
            default:       scan2doom = {5'h1F, DK_NONE};  // tanımsız
        endcase
    end
endfunction

// ---------------------------------------------------------------------------
// 9. UART bit_index → doomkeys.h değeri dönüştürücü
//    (key_catcher.py bit indekslerini Doom'un beklediği key code'larına çevirir)
// ---------------------------------------------------------------------------
function [7:0] bit2doomkey;
    input [4:0] bit_idx;
    begin
        case (bit_idx)
            5'd0:  bit2doomkey = 8'hAD; // KEY_UPARROW
            5'd1:  bit2doomkey = 8'hAF; // KEY_DOWNARROW
            5'd2:  bit2doomkey = 8'hAC; // KEY_LEFTARROW
            5'd3:  bit2doomkey = 8'hAE; // KEY_RIGHTARROW
            5'd4:  bit2doomkey = 8'hA3; // KEY_FIRE (Ctrl)
            5'd5:  bit2doomkey = 8'hA2; // KEY_USE  (Space)
            5'd6:  bit2doomkey = 8'hB6; // KEY_RSHIFT
            5'd7:  bit2doomkey = 8'd27; // KEY_ESCAPE
            5'd8:  bit2doomkey = 8'd13; // KEY_ENTER
            5'd9:  bit2doomkey = 8'hB8; // KEY_ALT
            5'd10: bit2doomkey = 8'hA0; // KEY_STRAFE_L
            5'd11: bit2doomkey = 8'hA1; // KEY_STRAFE_R
            5'd12: bit2doomkey = 8'd49; // '1'
            5'd13: bit2doomkey = 8'd50; // '2'
            5'd14: bit2doomkey = 8'd51; // '3'
            5'd15: bit2doomkey = 8'd52; // '4'
            5'd16: bit2doomkey = 8'd53; // '5'
            5'd17: bit2doomkey = 8'd54; // '6'
            5'd18: bit2doomkey = 8'd55; // '7'
            5'd19: bit2doomkey = 8'd9;  // KEY_TAB
            5'd20: bit2doomkey = 8'hBB; // KEY_F1
            5'd21: bit2doomkey = 8'hBC; // KEY_F2
            5'd22: bit2doomkey = 8'hBD; // KEY_F3
            5'd23: bit2doomkey = 8'hBF; // KEY_F5
            5'd24: bit2doomkey = 8'hC0; // KEY_F6
            5'd25: bit2doomkey = 8'hC1; // KEY_F7
            5'd26: bit2doomkey = 8'hC2; // KEY_F8
            5'd27: bit2doomkey = 8'hC3; // KEY_F9
            5'd28: bit2doomkey = 8'hC4; // KEY_F10
            5'd31: bit2doomkey = 8'h2D; // KEY_MINUS
            default: bit2doomkey = 8'h00;
        endcase
    end
endfunction

// ---------------------------------------------------------------------------
// 8. Birleşik dekoder — PS/2 + UART (tek always bloğu — çift sürücü hatası önlenir)
// ---------------------------------------------------------------------------
reg [31:0] phys_keys = 32'd0;   // anlık fiziksel tuş durumu (hold extension öncesi)
reg uart_press_flag = 0;
reg uart_byte0_seen = 0;

// Rising-edge detector for uart_key_ready (defensive: treats it as a pulse
// even if the upstream UART somehow produces a level signal)
reg uart_key_ready_prev = 0;
wire uart_key_pulse = uart_key_ready & ~uart_key_ready_prev;

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        dec_st          <= S_IDLE;
        is_ext          <= 0;
        phys_keys       <= 32'd0;
        KEY_EVENT       <= 32'd0;
        KEY_EV_VALID    <= 0;
        uart_press_flag <= 0;
        uart_byte0_seen <= 0;
        uart_key_ready_prev <= 0;
    end else begin
        KEY_EV_VALID <= 0;  // her saat sıfırla, pulse olarak çalışsın
        uart_key_ready_prev <= uart_key_ready;  // edge detector update

        // ── PS/2 Dekoderi ──────────────────────────────────────────────
        if (byte_rdy) begin
            case (dec_st)
                S_IDLE: begin
                    if (ps2_byte == 8'hE0) begin
                        is_ext <= 1;
                        dec_st <= S_EXT;
                    end else if (ps2_byte == 8'hF0) begin
                        is_ext <= 0;
                        dec_st <= S_REL;
                    end else begin
                        begin : press_n
                            reg [12:0] kd;
                            kd = scan2doom(1'b0, ps2_byte);
                            if (kd[12:8] != 5'h1F) begin
                                phys_keys[kd[12:8]] <= 1'b1;
                                KEY_EVENT           <= {1'b1, 23'd0, kd[7:0]};
                                KEY_EV_VALID        <= 1;
                            end
                        end
                        is_ext <= 0;
                        dec_st <= S_IDLE;
                    end
                end
                S_EXT: begin
                    if (ps2_byte == 8'hF0) begin
                        dec_st <= S_REL;
                    end else begin
                        begin : press_e
                            reg [12:0] kd;
                            kd = scan2doom(1'b1, ps2_byte);
                            if (kd[12:8] != 5'h1F) begin
                                phys_keys[kd[12:8]] <= 1'b1;
                                KEY_EVENT           <= {1'b1, 23'd0, kd[7:0]};
                                KEY_EV_VALID        <= 1;
                            end
                        end
                        is_ext <= 0;
                        dec_st <= S_IDLE;
                    end
                end
                S_REL: begin
                    begin : release_k
                        reg [12:0] kd;
                        kd = scan2doom(is_ext, ps2_byte);
                        if (kd[12:8] != 5'h1F) begin
                            phys_keys[kd[12:8]] <= 1'b0;
                            KEY_EVENT           <= {1'b0, 23'd0, kd[7:0]};
                            KEY_EV_VALID        <= 1;
                        end
                    end
                    is_ext <= 0;
                    dec_st <= S_IDLE;
                end
                default: begin
                    dec_st <= S_IDLE;
                    is_ext <= 0;
                end
            endcase
        end

        // ── UART Dekoderi (key_catcher.py: [0x01/0x00][bit_index]) ────
        // uart_key_pulse is a 1-cycle rising-edge strobe — processed exactly once
        if (uart_key_pulse) begin
            if (!uart_byte0_seen) begin
                if (uart_key_data == 8'h01 || uart_key_data == 8'h00) begin
                    uart_press_flag <= (uart_key_data == 8'h01);
                    uart_byte0_seen <= 1;
                end
            end else begin
                uart_byte0_seen <= 0;
                if (uart_key_data <= 8'd31) begin
                    phys_keys[uart_key_data[4:0]] <= uart_press_flag;
                    KEY_EVENT    <= {uart_press_flag, 23'd0,
                                     bit2doomkey(uart_key_data[4:0])};
                    KEY_EV_VALID <= 1;
                end
            end
        end

    end
end

// ---------------------------------------------------------------------------
// 10. Hold extension — fiziksel bırakma sonrası 150 ms DOOM_KEYS'i yüksek tutar
//     Böylece hızlı tap (press+release < 1 frame) CPU tarafından yakalanır.
// ---------------------------------------------------------------------------
reg [7:0] hold_cnt [0:31];
integer   hold_i;

initial begin
    for (hold_i = 0; hold_i < 32; hold_i = hold_i + 1)
        hold_cnt[hold_i] = 8'd0;
end

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        DOOM_KEYS <= 32'd0;
        for (hold_i = 0; hold_i < 32; hold_i = hold_i + 1)
            hold_cnt[hold_i] <= 8'd0;
    end else begin
        for (hold_i = 0; hold_i < 32; hold_i = hold_i + 1) begin
            if (phys_keys[hold_i]) begin
                // tuş basılı: çıkışı 1 yap ve sayacı doldur
                DOOM_KEYS[hold_i]   <= 1'b1;
                hold_cnt[hold_i]    <= 8'd150;
            end else begin
                // tuş bırakıldı: sayaç sıfırlanana kadar çıkışı 1 tut
                if (ms_tick && hold_cnt[hold_i] != 8'd0)
                    hold_cnt[hold_i] <= hold_cnt[hold_i] - 8'd1;
                if (hold_cnt[hold_i] == 8'd0)
                    DOOM_KEYS[hold_i] <= 1'b0;
            end
        end
    end
end

endmodule