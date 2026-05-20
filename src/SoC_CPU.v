// SoC CPU: hardware timer, RV32I state machine, instruction/data caches.
module SoC_CPU (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        boot_done,
    input  wire [31:0] ps2_data,
    input  wire [31:0] ps2_key_event,
    input  wire        hdmi_vsync_sync,
    input  wire        uart_write_done,
    input  wire        ddr_cmd_ready,
    input  wire        ddr_wr_data_rdy,
    input  wire [127:0] ddr_rd_data,
    input  wire        ddr_rd_data_valid,
    output reg         cpu_done,
    output reg [13:0]  vram_addr_out,
    output reg [31:0]  vram_data_out,
    output reg         vram_write_en,
    output reg [2:0]   cpu_ddr_cmd,
    output reg         cpu_ddr_cmd_en,
    output reg [26:0]  cpu_user_addr,
    output reg [127:0] cpu_ddr_wr_data,
    output reg         cpu_ddr_wr_data_en,
    output reg         cpu_ddr_wr_data_end,
    output reg [15:0]  cpu_ddr_wr_data_mask,
    output reg [7:0]   cpu_uart_data,
    output reg         cpu_uart_wr_en
);

// ==============================================================================
// INSTRUCTION CACHE (ICache)
// ==============================================================================
wire        icache_hit;
wire [31:0] icache_data;
wire        icache_valid;
reg         icache_fill_en   = 0;
reg [31:0]  icache_fill_addr = 0;
reg [127:0] icache_fill_data = 0;
reg         icache_req       = 0;
reg [31:0]  program_counter  = 0;

ICache icache_inst (
    .clk       (clk),
    .rst_n     (rst_n),
    .cpu_addr  (program_counter),
    .cpu_req   (icache_req),
    .cpu_data  (icache_data),
    .cpu_valid (icache_valid),
    .cache_hit (icache_hit),
    .fill_addr (icache_fill_addr),
    .fill_data (icache_fill_data),
    .fill_en   (icache_fill_en)
);

// ==============================================================================
// SYSTEM HARDWARE TIMER (1ms resolution)
// ==============================================================================
reg [31:0] ms_counter = 0;
reg [16:0] tick_counter = 0;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        ms_counter <= 32'd0;
        tick_counter <= 17'd0;
    end else if (boot_done && !cpu_done) begin
        if (tick_counter >= 17'd99_999) begin
            tick_counter <= 17'd0;
            ms_counter <= ms_counter + 1;
        end else begin
            tick_counter <= tick_counter + 17'd1;
        end
    end
    // --- CTRL+ESC SOFT REBOOT TRIGGER FOR TIMER ---
    else if (cpu_done && ps2_data[4] && ps2_data[7]) begin
        ms_counter <= 32'd0;
        tick_counter <= 17'd0;
    end
end

// ==============================================================================
// 6. CPU REGISTERS
// ==============================================================================
reg [4:0] read1_addr = 0;
wire [31:0] read1_data;

reg [4:0] read2_addr = 0;
wire [31:0] read2_data;

reg [4:0] write_addr = 0;
reg [31:0] write_data = 0;
reg [31:0] read1_data_reg = 0;
reg [31:0] read2_data_reg = 0;
// --- Pre-computed flags (registered in C_REG_FETCH_WAIT to break critical paths) ---
reg         rs2_is_zero   = 0;  // read2_data == 0 (division by zero)
reg         branch_eq     = 0;  // rs1 == rs2
reg         branch_lt     = 0;  // $signed(rs1) < $signed(rs2)
reg         branch_ltu    = 0;  // rs1 < rs2 (unsigned)
reg write_enable = 0;

CPU_Registers cpu_register_controller(
    .clk(clk),
    .read1_addr(read1_addr),
    .read1_data(read1_data),
    .read2_addr(read2_addr),
    .read2_data(read2_data),
    .write_addr(write_addr),
    .write_data(write_data),
    .write_enable(write_enable)
);

// ==============================================================================
// 7. COMPUTER STATE MACHINE (Now 6-bit for Prefetch States)
// ==============================================================================
localparam C_UART_SEND         = 6'd0;
localparam C_UART_WAIT_DONE    = 6'd1;
localparam C_DDR_WAIT_READ     = 6'd2;
localparam C_DDR_READ          = 6'd3;
localparam C_DDR_WAIT_DATA     = 6'd4;
localparam C_LOAD_INSTR        = 6'd5;
localparam C_DECODE            = 6'd6;

localparam C_EXEC_R_TYPE       = 6'd7;
localparam C_EXEC_I_TYPE       = 6'd8;
localparam C_EXEC_LOAD         = 6'd9;
localparam C_EXEC_STORE        = 6'd10;
localparam C_EXEC_BRANCH       = 6'd11;
localparam C_EXEC_JAL          = 6'd12;
localparam C_EXEC_JALR         = 6'd13;
localparam C_EXEC_LUI          = 6'd14;
localparam C_EXEC_AUIPC        = 6'd15;

localparam C_EXECUTE           = 6'd16;
localparam C_FINISH_DATA_READ  = 6'd17;
localparam C_DDR_WAIT_WRITE    = 6'd18;
localparam C_DDR_WRITE         = 6'd19;
localparam C_IO_READ           = 6'd20;
localparam C_IO_WRITE          = 6'd21;

localparam C_HALT              = 6'd22;
localparam C_HALT_SETUP        = 6'd23;
localparam C_HALT_FETCH        = 6'd24;
localparam C_HALT_NEXT         = 6'd25;
localparam C_HALT_FOREVER      = 6'd26;

localparam C_DEBUG_PRINT_PC    = 6'd27;
localparam C_DEBUG_PRINT_INSTR = 6'd28;

localparam C_CHECK_ILLEGAL_R   = 6'd29;
localparam C_REG_FETCH_WAIT    = 6'd30;

// --- NEW PREFETCH STATES ---
localparam C_PREFETCH_ISSUE    = 6'd31;
localparam C_PREFETCH_CLEANUP  = 6'd32;

// --- RV32M Extension States ---
localparam C_EXEC_MUL          = 6'd33; // MUL / MULH / MULHU / MULHSU (1 cycle, DSP)
localparam C_DIV_SETUP         = 6'd34; // DIV / DIVU / REM / REMU setup
localparam C_DIV_EXEC          = 6'd35; // 33-cycle iterative divider
localparam C_DIV_FINISH        = 6'd36; // write result

// --- Instruction Cache State ---
localparam C_ICACHE_WAIT       = 6'd37; // 1-cycle BSRAM read latency
localparam C_DCACHE_WAIT       = 6'd38; // 1-cycle BSRAM read latency

reg [5:0]  state = C_DDR_WAIT_READ;
reg [5:0]  return_state = C_DDR_WAIT_READ;
reg [5:0]  pending_exec_state = 0;

// UART burst / halt dump sequencing (internal)
reg [1:0]   cpu_byte_idx = 0;
reg [31:0]  cpu_uart_msg = 0;

// Sub-Word Memory Alignment Wires
reg [31:0]  raw_word = 0;
reg [3:0]   base_mask = 0;
reg [31:0]  active_payload = 0;

// program_counter: defined above next to ICache
reg [127:0] memory_read_reg = 0; // Now only used for Data Loads
reg [31:0]  first_instr = 0;
reg [31:0]  second_instr = 0;
reg [31:0]  third_instr = 0;
reg [31:0]  fourth_instr = 0;

// --- DDR3 TRANSACTION QUEUE & BACKGROUND CATCHER ---
reg [1:0]  rq_head = 0;
reg [1:0]  rq_tail = 0;
reg [3:0]  rq_is_data = 0; // 1 = Data Load, 0 = Instruction/Prefetch
reg [26:0] rq_addr_0=0, rq_addr_1=0, rq_addr_2=0, rq_addr_3=0;

reg [127:0] pf_read_reg = 0;
reg [26:0]  pf_ready_addr = 27'h7FFFFFF; // Initializes to invalid address
reg         pf_valid = 0;

reg [127:0] dmem_read_reg = 0;
reg         dmem_valid = 0;
// ----------------------------------------------------

// --- RV32M Divider Registers ---
// 64-bit working register: [63:32] = partial remainder, [31:0] = dividend/quotient shift reg
reg [63:0]  div_working    = 0;
reg [31:0]  div_divisor_r  = 0;  // absolute value of divisor
reg [5:0]   div_bit        = 0;  // iteration counter 0..31
reg         div_is_signed  = 0;
reg         div_is_rem     = 0;
reg         div_neg_result = 0;  // quotient needs negation
reg         div_neg_rem    = 0;  // remainder needs negation
// 64-bit multiplier accumulator (DSP blocks)
reg [63:0]  mul_result_r   = 0;

// --- Divider combinational step wires (restoring algorithm) ---
// Each cycle: shift PR left by 1, bring in MSB of dividend (div_working[31])
wire [32:0] div_pr_shifted = {div_working[62:32], div_working[31]};
wire [32:0] div_pr_sub     = div_pr_shifted - {1'b0, div_divisor_r};
wire        div_pr_lt_d    = !div_pr_sub[32]; // 1 = subtract succeeded

// Internal CPU decoding wires
reg [31:0]  current_instr = 0;
reg [6:0]   opcode = 0;
reg [2:0]   funct3 = 0;
reg [6:0]   funct7 = 0;

wire [31:0] imm_i = {{20{current_instr[31]}}, current_instr[31:20]};
wire [31:0] imm_s = {{20{current_instr[31]}}, current_instr[31:25], current_instr[11:7]};
wire [31:0] imm_b = {{20{current_instr[31]}}, current_instr[7], current_instr[30:25], current_instr[11:8], 1'b0};
wire [31:0] imm_j = {{12{current_instr[31]}}, current_instr[19:12], current_instr[20], current_instr[30:21], 1'b0};
wire [31:0] imm_u = {current_instr[31:12], 12'b0};

reg         is_instruction_fetch = 1;
reg [31:0]  data_addr = 0;
reg [31:0]  cpu_store_data = 0;

reg [4:0]   dump_reg_idx = 0;

wire        dcache_hit;
wire [31:0] dcache_data;
wire        dcache_valid;
reg         dcache_fill_en   = 0;
reg [31:0]  dcache_fill_addr = 0;
reg [127:0] dcache_fill_data = 0;
reg         dcache_req       = 0;
reg         dcache_inv_en    = 0;
reg [31:0]  dcache_inv_addr  = 0;

DCache dcache_inst (
    .clk       (clk),
    .rst_n     (rst_n),
    .cpu_addr  (data_addr),
    .cpu_req   (dcache_req),
    .cpu_data  (dcache_data),
    .cpu_valid (dcache_valid),
    .cache_hit (dcache_hit),
    .fill_addr (dcache_fill_addr),
    .fill_data (dcache_fill_data),
    .fill_en   (dcache_fill_en),
    .inv_addr  (dcache_inv_addr),
    .inv_en    (dcache_inv_en)
);
//====================================================================================
//                                   CPU BEGIN
//====================================================================================

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= C_DDR_WAIT_READ;
        cpu_ddr_cmd_en <= 0;
        cpu_ddr_wr_data_en <= 0;
        cpu_ddr_wr_data_end <= 0;
        cpu_ddr_wr_data_mask <= 16'h0000;
        cpu_uart_wr_en <= 0;
        cpu_user_addr <= 27'd0;
        program_counter <= 32'd0;
        memory_read_reg <= 0;
        cpu_byte_idx <= 0;
        cpu_done <= 0;
        is_instruction_fetch <= 1;
        data_addr <= 32'd0;
        cpu_store_data <= 32'd0;
        vram_write_en <= 0;
        dump_reg_idx <= 5'd0;
        pending_exec_state <= 0;

        // Reset Queue
        rq_head <= 0; rq_tail <= 0; rq_is_data <= 0;
        pf_valid <= 0; dmem_valid <= 0; pf_ready_addr <= 27'h7FFFFFF;

    end else if (boot_done && !cpu_done) begin

        // ========================================================
        // GLOBAL SNOOPING QUEUE (Background Data Catcher)
        // ========================================================
        if (ddr_rd_data_valid) begin
            if (rq_is_data[rq_tail]) begin
                // It was a Data Load Request
                dmem_read_reg <= ddr_rd_data;
                dmem_valid    <= 1;
                
                // â”€â”€â”€ DCache doldur â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
                dcache_fill_en   <= 1;
                dcache_fill_addr <= (rq_tail == 2'd0) ? {rq_addr_0, 4'b0} :
                                    (rq_tail == 2'd1) ? {rq_addr_1, 4'b0} :
                                    (rq_tail == 2'd2) ? {rq_addr_2, 4'b0} :
                                                        {rq_addr_3, 4'b0};
                dcache_fill_data <= ddr_rd_data;
                // â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
            end else begin
                // It was an Instruction Prefetch Request
                pf_read_reg   <= ddr_rd_data;
                pf_valid      <= 1;
                // Save the exact address this block belongs to
                if (rq_tail == 2'd0) pf_ready_addr <= rq_addr_0;
                else if (rq_tail == 2'd1) pf_ready_addr <= rq_addr_1;
                else if (rq_tail == 2'd2) pf_ready_addr <= rq_addr_2;
                else pf_ready_addr <= rq_addr_3;

                // â”€â”€â”€ ICache doldur â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
                icache_fill_en   <= 1;
                icache_fill_addr <= (rq_tail == 2'd0) ? {rq_addr_0, 4'b0} :
                                    (rq_tail == 2'd1) ? {rq_addr_1, 4'b0} :
                                    (rq_tail == 2'd2) ? {rq_addr_2, 4'b0} :
                                                        {rq_addr_3, 4'b0};
                icache_fill_data <= ddr_rd_data;
                // â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
            end
            rq_tail <= rq_tail + 2'd1; // Advance the FIFO
        end else begin
            icache_fill_en <= 0; // pulse geniÅŸliÄŸi sadece 1 saat
            dcache_fill_en <= 0;
        end
        // ========================================================

        case (state)
            C_UART_SEND: begin
                case (cpu_byte_idx)
                    2'd0: cpu_uart_data <= cpu_uart_msg[31:24];
                    2'd1: cpu_uart_data <= cpu_uart_msg[23:16];
                    2'd2: cpu_uart_data <= cpu_uart_msg[15:8];
                    2'd3: cpu_uart_data <= cpu_uart_msg[7:0];
                endcase
                cpu_uart_wr_en <= 1;
                state <= C_UART_WAIT_DONE;
            end

            C_UART_WAIT_DONE: begin
                if (uart_write_done) begin
                    cpu_uart_wr_en <= 0;
                    if (cpu_byte_idx == 3) begin
                        cpu_byte_idx <= 0;
                        state <= return_state;
                    end else begin
                        cpu_byte_idx <= cpu_byte_idx + 2'd1;
                        state <= C_UART_SEND;
                    end
                end
            end

            C_DDR_WAIT_READ: begin
                write_enable  <= 0;

                if (is_instruction_fetch == 0 && data_addr >= 32'h40000000) begin
                    icache_req <= 0;
                    dcache_req <= 0;
                    state <= C_IO_READ;
                end
                else if (is_instruction_fetch == 1'b1) begin
                    icache_req <= 1;
                    dcache_req <= 0;
                    state <= C_ICACHE_WAIT; // Pipeline ICache
                end
                else begin
                    icache_req <= 0;
                    dcache_req <= 1;
                    state <= C_DCACHE_WAIT; // Pipeline DCache
                end
            end

            C_DDR_READ: begin
                cpu_ddr_cmd_en <= 0;
                state <= C_DDR_WAIT_DATA;
            end

            C_DDR_WAIT_DATA: begin
                // We wait for the Global Catcher to flag that data arrived
                if (is_instruction_fetch) begin
                    if (pf_valid) begin
                        // Check if it's the data we actually want (Protects against stray branch prefetches)
                        if (pf_ready_addr == {program_counter[27:4], 3'b000}) begin
                            pf_valid <= 0;
                            first_instr  <= {pf_read_reg[103:96], pf_read_reg[111:104], pf_read_reg[119:112], pf_read_reg[127:120]};
                            second_instr <= {pf_read_reg[71:64],  pf_read_reg[79:72],   pf_read_reg[87:80],   pf_read_reg[95:88]};
                            third_instr  <= {pf_read_reg[39:32],  pf_read_reg[47:40],   pf_read_reg[55:48],   pf_read_reg[63:56]};
                            fourth_instr <= {pf_read_reg[7:0],    pf_read_reg[15:8],    pf_read_reg[23:16],   pf_read_reg[31:24]};
                            state <= C_LOAD_INSTR;
                        end else begin
                            pf_valid <= 0; // Throw away stale branch prefetch
                        end
                    end
                end else begin
                    // It was a Data Load
                    if (dmem_valid) begin
                        dmem_valid <= 0;
                        memory_read_reg <= dmem_read_reg;
                        state <= C_FINISH_DATA_READ;
                    end
                end
            end

            C_LOAD_INSTR: begin
                write_enable <= 0;
                case (program_counter[3:2])
                    2'b00: current_instr <= first_instr;
                    2'b01: current_instr <= second_instr;
                    2'b10: current_instr <= third_instr;
                    2'b11: current_instr <= fourth_instr;
                endcase
                state <= C_DECODE;
            end

            // ICache hit: BSRAM 1 saat okuma gecikmesi bekleniyor
            C_ICACHE_WAIT: begin
                write_enable <= 0;
                icache_req   <= 0;
                if (icache_valid) begin
                    current_instr <= icache_data;
                    state         <= C_DECODE;
                end
                else if (pf_valid && pf_ready_addr == {program_counter[27:4], 3'b000}) begin
                    pf_valid    <= 0;
                    first_instr  <= {pf_read_reg[103:96], pf_read_reg[111:104], pf_read_reg[119:112], pf_read_reg[127:120]};
                    second_instr <= {pf_read_reg[71:64],  pf_read_reg[79:72],   pf_read_reg[87:80],   pf_read_reg[95:88]};
                    third_instr  <= {pf_read_reg[39:32],  pf_read_reg[47:40],   pf_read_reg[55:48],   pf_read_reg[63:56]};
                    fourth_instr <= {pf_read_reg[7:0],    pf_read_reg[15:8],    pf_read_reg[23:16],   pf_read_reg[31:24]};
                    state <= C_LOAD_INSTR;
                end
                else if (ddr_cmd_ready) begin
                    cpu_ddr_cmd      <= 3'b001;
                    cpu_ddr_cmd_en   <= 1;
                    cpu_user_addr    <= {program_counter[27:4], 3'b000};
                    rq_is_data[rq_head] <= 0;
                    case (rq_head)
                        2'd0: rq_addr_0 <= {program_counter[27:4], 3'b000};
                        2'd1: rq_addr_1 <= {program_counter[27:4], 3'b000};
                        2'd2: rq_addr_2 <= {program_counter[27:4], 3'b000};
                        2'd3: rq_addr_3 <= {program_counter[27:4], 3'b000};
                    endcase
                    rq_head <= rq_head + 2'd1;
                    state   <= C_DDR_READ;
                end
            end

            // DCache hit: 1 saat gecikme
            C_DCACHE_WAIT: begin
                write_enable <= 0;
                dcache_req   <= 0;
                if (dcache_valid) begin
                    raw_word = dcache_data;
                    case (funct3)
                        3'b000:
                            case (data_addr[1:0])
                                2'b00: write_data <= {{24{raw_word[7]}},  raw_word[7:0]};
                                2'b01: write_data <= {{24{raw_word[15]}}, raw_word[15:8]};
                                2'b10: write_data <= {{24{raw_word[23]}}, raw_word[23:16]};
                                2'b11: write_data <= {{24{raw_word[31]}}, raw_word[31:24]};
                            endcase
                        3'b100:
                            case (data_addr[1:0])
                                2'b00: write_data <= {24'd0, raw_word[7:0]};
                                2'b01: write_data <= {24'd0, raw_word[15:8]};
                                2'b10: write_data <= {24'd0, raw_word[23:16]};
                                2'b11: write_data <= {24'd0, raw_word[31:24]};
                            endcase
                        3'b001:
                            if (data_addr[1] == 1'b0) write_data <= {{16{raw_word[15]}}, raw_word[15:0]};
                            else                      write_data <= {{16{raw_word[31]}}, raw_word[31:16]};
                        3'b101:
                            if (data_addr[1] == 1'b0) write_data <= {16'd0, raw_word[15:0]};
                            else                      write_data <= {16'd0, raw_word[31:16]};
                        3'b010:
                            write_data <= raw_word;
                        default:
                            write_data <= raw_word;
                    endcase
                    write_enable <= 1;
                    state <= return_state;
                end
                else if (ddr_cmd_ready) begin
                    cpu_ddr_cmd    <= 3'b001;
                    cpu_ddr_cmd_en <= 1;
                    cpu_user_addr  <= {data_addr[27:4], 3'b000};
                    rq_is_data[rq_head] <= 1;
                    case (rq_head)
                        2'd0: rq_addr_0 <= {data_addr[27:4], 3'b000};
                        2'd1: rq_addr_1 <= {data_addr[27:4], 3'b000};
                        2'd2: rq_addr_2 <= {data_addr[27:4], 3'b000};
                        2'd3: rq_addr_3 <= {data_addr[27:4], 3'b000};
                    endcase
                    rq_head <= rq_head + 2'd1;
                    state   <= C_DDR_READ;
                end
            end

            C_DEBUG_PRINT_PC: begin
                cpu_uart_msg <= program_counter;
                cpu_byte_idx <= 0;
                return_state <= C_DEBUG_PRINT_INSTR;
                state <= C_UART_SEND;
            end

            C_DEBUG_PRINT_INSTR: begin
                cpu_uart_msg <= current_instr;
                cpu_byte_idx <= 0;
                return_state <= C_DECODE;
                state <= C_UART_SEND;
            end

            C_DECODE: begin
                opcode <= current_instr[6:0];
                funct3 <= current_instr[14:12];
                funct7 <= current_instr[31:25];

                read1_addr <= current_instr[19:15];
                read2_addr <= current_instr[24:20];
                write_addr <= current_instr[11:7];

                is_instruction_fetch <= 0;
                return_state <= C_EXECUTE;

                case (current_instr[6:0])
                    7'b0110011: begin pending_exec_state <= C_CHECK_ILLEGAL_R; state <= C_REG_FETCH_WAIT; end

                    7'b0010011: begin pending_exec_state <= C_EXEC_I_TYPE; state <= C_REG_FETCH_WAIT; end
                    7'b0000011: begin pending_exec_state <= C_EXEC_LOAD;   state <= C_REG_FETCH_WAIT; end
                    7'b0100011: begin pending_exec_state <= C_EXEC_STORE;  state <= C_REG_FETCH_WAIT; end
                    7'b1100011: begin pending_exec_state <= C_EXEC_BRANCH; state <= C_REG_FETCH_WAIT; end
                    7'b1100111: begin pending_exec_state <= C_EXEC_JALR;   state <= C_REG_FETCH_WAIT; end

                    7'b1101111: state <= C_EXEC_JAL;
                    7'b0110111: state <= C_EXEC_LUI;
                    7'b0010111: state <= C_EXEC_AUIPC;

                    7'b0001111: state <= C_EXECUTE;
                    7'b1110011: state <= C_EXECUTE;

                    default:    state <= C_HALT;
                endcase
            end

            C_CHECK_ILLEGAL_R: begin
                if (funct7 == 7'b0000001) begin
                    // RV32M: MUL needs reg values, route through fetch wait
                    case (funct3)
                        3'b000,
                        3'b001,
                        3'b010,
                        3'b011: begin
                            // MUL variants â€” registers already loaded (C_REG_FETCH_WAIT ran)
                            state <= C_EXEC_MUL;
                        end
                        default: begin
                            // DIV/REM variants
                            state <= C_DIV_SETUP;
                        end
                    endcase
                end else begin
                    state <= C_EXEC_R_TYPE;
                end
            end

            C_REG_FETCH_WAIT: begin
                read1_data_reg <= read1_data;
                read2_data_reg <= read2_data;
                // Pre-compute comparison flags to cut critical paths
                rs2_is_zero  <= (read2_data == 32'd0);
                branch_eq    <= (read1_data == read2_data);
                branch_lt    <= ($signed(read1_data) < $signed(read2_data));
                branch_ltu   <= (read1_data < read2_data);
                state <= pending_exec_state;
            end

            // ========================================================
            // RV32M â€” MULTIPLICATION  (1 cycle, uses Gowin DSP48 blocks)
            // ========================================================
            C_EXEC_MUL: begin
                write_enable <= 1;
                case (funct3)
                    3'b000: begin // MUL â€” lower 32 bits of rs1 * rs2
                        mul_result_r <= $signed({{32{read1_data_reg[31]}}, read1_data_reg}) *
                                        $signed({{32{read2_data_reg[31]}}, read2_data_reg});
                        write_data <= (read1_data_reg * read2_data_reg); // synth uses DSP
                    end
                    3'b001: begin // MULH â€” upper 32 bits, signed Ã— signed
                        mul_result_r <= $signed({{32{read1_data_reg[31]}}, read1_data_reg}) *
                                        $signed({{32{read2_data_reg[31]}}, read2_data_reg});
                        write_data <= ($signed({{32{read1_data_reg[31]}}, read1_data_reg}) *
                                       $signed({{32{read2_data_reg[31]}}, read2_data_reg})) >> 32;
                    end
                    3'b010: begin // MULHSU â€” upper 32 bits, signed Ã— unsigned
                        mul_result_r <= $signed({{32{read1_data_reg[31]}}, read1_data_reg}) *
                                        {32'd0, read2_data_reg};
                        write_data <= ($signed({{32{read1_data_reg[31]}}, read1_data_reg}) *
                                       {32'd0, read2_data_reg}) >> 32;
                    end
                    3'b011: begin // MULHU â€” upper 32 bits, unsigned Ã— unsigned
                        mul_result_r <= {32'd0, read1_data_reg} * {32'd0, read2_data_reg};
                        write_data <= ({32'd0, read1_data_reg} * {32'd0, read2_data_reg}) >> 32;
                    end
                    default: write_data <= 32'd0;
                endcase
                state <= return_state;
            end

            // ========================================================
            // RV32M â€” DIVISION / REMAINDER  (DISABLED â€” returns spec stub)
            // DIV/DIVU  â†’ 0xFFFFFFFF  (all-ones, same as divide-by-zero)
            // REM/REMU  â†’ 0x00000000
            // Result is written in 1 clock; no iterative loop.
            // ========================================================
            C_DIV_SETUP: begin
                write_enable <= 1;
                // funct3[1] = 1 for REM/REMU, 0 for DIV/DIVU
                write_data   <= funct3[1] ? 32'h0000_0000 : 32'hFFFF_FFFF;
                state        <= return_state;
            end

            // Dead states â€” should never be reached; go back to fetch cleanly
            C_DIV_EXEC: begin
                state <= return_state;
            end

            C_DIV_FINISH: begin
                write_enable <= 1;
                write_data   <= 32'h0000_0000;
                state        <= return_state;
            end

            C_EXEC_R_TYPE: begin
                write_enable <= 1;
                case (funct3)
                    3'b000: begin
                        if (funct7 == 7'b0100000) write_data <= read1_data_reg - read2_data_reg;
                        else write_data <= read1_data_reg + read2_data_reg;
                    end
                    3'b001: write_data <= read1_data_reg << read2_data_reg[4:0];
                    3'b010: write_data <= ($signed(read1_data_reg) < $signed(read2_data_reg)) ? 32'd1 : 32'd0;
                    3'b011: write_data <= (read1_data_reg < read2_data_reg) ? 32'd1 : 32'd0;
                    3'b100: write_data <= read1_data_reg ^ read2_data_reg;
                    3'b101: begin
                        if (funct7 == 7'b0100000)
                            write_data <= $signed(read1_data_reg) >>> read2_data_reg[4:0];
                        else
                            write_data <= read1_data_reg >> read2_data_reg[4:0];
                    end
                    3'b110: write_data <= read1_data_reg | read2_data_reg;
                    3'b111: write_data <= read1_data_reg & read2_data_reg;
                endcase
                state <= return_state;
            end

            C_EXEC_I_TYPE: begin
                write_enable <= 1;
                case (funct3)
                    3'b000: write_data <= read1_data_reg + imm_i;
                    3'b001: write_data <= read1_data_reg << imm_i[4:0];
                    3'b010: write_data <= ($signed(read1_data_reg) < $signed(imm_i)) ? 32'd1 : 32'd0;
                    3'b011: write_data <= (read1_data_reg < imm_i) ? 32'd1 : 32'd0;
                    3'b100: write_data <= read1_data_reg ^ imm_i;
                    3'b101: begin
                        if (current_instr[30] == 1'b1)
                            write_data <= $signed(read1_data_reg) >>> imm_i[4:0];
                        else
                            write_data <= read1_data_reg >> imm_i[4:0];
                    end
                    3'b110: write_data <= read1_data_reg | imm_i;
                    3'b111: write_data <= read1_data_reg & imm_i;
                endcase
                state <= return_state;
            end

            C_EXEC_LOAD: begin
                data_addr <= read1_data_reg + imm_i;
                is_instruction_fetch <= 0;
                state <= C_DDR_WAIT_READ;
            end

            C_EXEC_STORE: begin
                data_addr <= read1_data_reg + imm_s;
                cpu_store_data <= read2_data_reg;
                state <= C_DDR_WAIT_WRITE;
            end

            C_EXEC_BRANCH: begin
                state <= return_state;
                case (funct3)
                    3'b000: if (branch_eq) begin
                        program_counter <= program_counter + imm_b;
                        is_instruction_fetch <= 1;
                        state <= C_DDR_WAIT_READ;
                    end
                    3'b001: if (!branch_eq) begin
                        program_counter <= program_counter + imm_b;
                        is_instruction_fetch <= 1;
                        state <= C_DDR_WAIT_READ;
                    end
                    3'b100: if (branch_lt) begin
                        program_counter <= program_counter + imm_b;
                        is_instruction_fetch <= 1;
                        state <= C_DDR_WAIT_READ;
                    end
                    3'b101: if (!branch_lt) begin
                        program_counter <= program_counter + imm_b;
                        is_instruction_fetch <= 1;
                        state <= C_DDR_WAIT_READ;
                    end
                    3'b110: if (branch_ltu) begin
                        program_counter <= program_counter + imm_b;
                        is_instruction_fetch <= 1;
                        state <= C_DDR_WAIT_READ;
                    end
                    3'b111: if (!branch_ltu) begin
                        program_counter <= program_counter + imm_b;
                        is_instruction_fetch <= 1;
                        state <= C_DDR_WAIT_READ;
                    end
                endcase
            end

            C_EXEC_JAL: begin
                write_data <= program_counter + 32'd4;
                write_enable <= 1;
                program_counter <= program_counter + imm_j;
                is_instruction_fetch <= 1;
                state <= C_DDR_WAIT_READ;
            end

            C_EXEC_JALR: begin
                write_data <= program_counter + 32'd4;
                write_enable <= 1;
                program_counter <= (read1_data_reg + imm_i) & 32'hFFFF_FFFE;
                is_instruction_fetch <= 1;
                state <= C_DDR_WAIT_READ;
            end

            C_EXEC_LUI: begin
                write_data <= imm_u;
                write_enable <= 1;
                state <= C_EXECUTE;
            end

            C_EXEC_AUIPC: begin
                write_data <= program_counter + imm_u;
                write_enable <= 1;
                state <= C_EXECUTE;
            end

            // ========================================================
            // I/O OPERATIONS
            // ========================================================
            C_IO_READ: begin
                if (data_addr == 32'h4010_0000) begin
                    // doomgeneric DG_GetKey: live key-state bitmask
                    write_data <= ps2_data;
                end else if (data_addr == 32'h4010_0004) begin
                    // doomgeneric DG_GetKey: last key event {press[31], doomkey[7:0]}
                    // C kodu bu adresi KEY_EV_VALID pulse'dan sonra okur
                    write_data <= ps2_key_event;
                end else if (data_addr == 32'h4030_0000) begin
                    // doomgeneric DG_GetTicksMs
                    write_data <= ms_counter;
                end else if (data_addr == 32'h4040_0000) begin
                    // doomgeneric DG_DrawFrame vsync bekleme (CDC sync'd)
                    write_data <= {31'd0, hdmi_vsync_sync};
                end else begin
                    write_data <= 32'd0;
                end
                write_enable <= 1;
                state <= C_EXECUTE;
            end

            C_IO_WRITE: begin
                if (data_addr >= 32'h4000_0000 && data_addr <= 32'h4000_FFFC) begin
                    vram_addr_out <= data_addr[15:2];
                    vram_data_out <= cpu_store_data;
                    vram_write_en <= 1;
                    state <= C_EXECUTE;
                end
                else if (data_addr == 32'h4020_0000) begin
                    cpu_uart_msg <= cpu_store_data;
                    cpu_byte_idx <= 0;
                    return_state <= C_EXECUTE;
                    state <= C_UART_SEND;
                end
                else begin
                    state <= C_EXECUTE;
                end
            end

            // ========================================================
            // THE WRAP-UP STATE & PREFETCH TRIGGER
            // ========================================================
            C_EXECUTE: begin
                write_enable <= 0;
                vram_write_en <= 0;

                program_counter <= program_counter + 32'd4;
                is_instruction_fetch <= 1;

                if (program_counter[3:2] == 2'b00) begin
                    state <= C_PREFETCH_ISSUE;
                end else if (program_counter[3:2] == 2'b11) begin
                    state <= C_DDR_WAIT_READ;
                end else begin
                    state <= C_LOAD_INSTR;
                end
            end

            // --- THE BACKGROUND PREFETCHER STATES ---
            C_PREFETCH_ISSUE: begin
                if (ddr_cmd_ready) begin
                    cpu_ddr_cmd <= 3'b001;
                    cpu_ddr_cmd_en <= 1;
                    cpu_user_addr <= {program_counter[27:4], 3'b000} + 27'd8;
                    rq_is_data[rq_head] <= 0;
                    case (rq_head)
                        2'd0: rq_addr_0 <= {program_counter[27:4], 3'b000} + 27'd8;
                        2'd1: rq_addr_1 <= {program_counter[27:4], 3'b000} + 27'd8;
                        2'd2: rq_addr_2 <= {program_counter[27:4], 3'b000} + 27'd8;
                        2'd3: rq_addr_3 <= {program_counter[27:4], 3'b000} + 27'd8;
                    endcase
                    rq_head <= rq_head + 2'd1;
                    state <= C_PREFETCH_CLEANUP;
                end
            end

            C_PREFETCH_CLEANUP: begin
                cpu_ddr_cmd_en <= 0;
                state <= C_LOAD_INSTR;
            end

            // ========================================================
            // LOAD STATE (BYTE/HALFWORD/WORD)
            // ========================================================
            C_FINISH_DATA_READ: begin
                case (data_addr[3:2])
                    2'b00: raw_word = {memory_read_reg[103:96], memory_read_reg[111:104], memory_read_reg[119:112], memory_read_reg[127:120]};
                    2'b01: raw_word = {memory_read_reg[71:64],  memory_read_reg[79:72],   memory_read_reg[87:80],   memory_read_reg[95:88]};
                    2'b10: raw_word = {memory_read_reg[39:32],  memory_read_reg[47:40],   memory_read_reg[55:48],   memory_read_reg[63:56]};
                    2'b11: raw_word = {memory_read_reg[7:0],    memory_read_reg[15:8],    memory_read_reg[23:16],   memory_read_reg[31:24]};
                endcase

                case (funct3)
                    3'b000:
                        case (data_addr[1:0])
                            2'b00: write_data <= {{24{raw_word[7]}},  raw_word[7:0]};
                            2'b01: write_data <= {{24{raw_word[15]}}, raw_word[15:8]};
                            2'b10: write_data <= {{24{raw_word[23]}}, raw_word[23:16]};
                            2'b11: write_data <= {{24{raw_word[31]}}, raw_word[31:24]};
                        endcase
                    3'b100:
                        case (data_addr[1:0])
                            2'b00: write_data <= {24'd0, raw_word[7:0]};
                            2'b01: write_data <= {24'd0, raw_word[15:8]};
                            2'b10: write_data <= {24'd0, raw_word[23:16]};
                            2'b11: write_data <= {24'd0, raw_word[31:24]};
                        endcase
                    3'b001:
                        if (data_addr[1] == 1'b0) write_data <= {{16{raw_word[15]}}, raw_word[15:0]};
                        else                      write_data <= {{16{raw_word[31]}}, raw_word[31:16]};
                    3'b101:
                        if (data_addr[1] == 1'b0) write_data <= {16'd0, raw_word[15:0]};
                        else                      write_data <= {16'd0, raw_word[31:16]};
                    3'b010:
                        write_data <= raw_word;
                    default:
                        write_data <= raw_word;
                endcase

                write_enable <= 1;
                state <= C_EXECUTE;
            end

            // ========================================================
            // STORE STATE (BYTE/HALFWORD/WORD MASKS)
            // ========================================================
            C_DDR_WAIT_WRITE: begin
                if (data_addr >= 32'h40000000) begin
                    state <= C_IO_WRITE;
                end
                else if (ddr_cmd_ready && ddr_wr_data_rdy) begin
                    cpu_ddr_cmd <= 3'b000;
                    cpu_ddr_cmd_en <= 1;
                    cpu_user_addr <= {data_addr[27:4], 3'b000};

                    case (funct3[1:0])
                        2'b00: active_payload = {4{cpu_store_data[7:0]}};
                        2'b01: active_payload = {2{cpu_store_data[7:0], cpu_store_data[15:8]}};
                        2'b10: active_payload = {cpu_store_data[7:0], cpu_store_data[15:8], cpu_store_data[23:16], cpu_store_data[31:24]};
                        default: active_payload = {cpu_store_data[7:0], cpu_store_data[15:8], cpu_store_data[23:16], cpu_store_data[31:24]};
                    endcase

                    cpu_ddr_wr_data <= {4{active_payload}};

                    case (funct3[1:0])
                        2'b00:
                            case (data_addr[1:0])
                                2'b00: base_mask = 4'b0111;
                                2'b01: base_mask = 4'b1011;
                                2'b10: base_mask = 4'b1101;
                                2'b11: base_mask = 4'b1110;
                            endcase
                        2'b01:
                            if (data_addr[1] == 1'b0) base_mask = 4'b0011;
                            else                      base_mask = 4'b1100;
                        2'b10:
                            base_mask = 4'b0000;
                        default:
                            base_mask = 4'b0000;
                    endcase

                    case (data_addr[3:2])
                        2'b00: cpu_ddr_wr_data_mask <= {base_mask, 12'hFFF};
                        2'b01: cpu_ddr_wr_data_mask <= {4'hF, base_mask, 8'hFF};
                        2'b10: cpu_ddr_wr_data_mask <= {8'hFF, base_mask, 4'hF};
                        2'b11: cpu_ddr_wr_data_mask <= {12'hFFF, base_mask};
                    endcase

                    cpu_ddr_wr_data_en <= 1;
                    cpu_ddr_wr_data_end <= 1;
                    dcache_inv_en <= 1;
                    dcache_inv_addr <= {data_addr[27:4], 3'b000};
                    state <= C_DDR_WRITE;
                end
            end

            C_DDR_WRITE: begin
                cpu_ddr_cmd_en <= 0;
                cpu_ddr_wr_data_en <= 0;
                cpu_ddr_wr_data_end <= 0;
                dcache_inv_en <= 0;
                state <= C_EXECUTE;
            end

            // ========================================================
            // CRASH / END OF PROGRAM DUMP
            // ========================================================
            C_HALT: begin
                dump_reg_idx <= 5'd0;
                state <= C_HALT_SETUP;
            end

            C_HALT_SETUP: begin
                read1_addr <= dump_reg_idx;
                state <= C_HALT_FETCH;
            end

            C_HALT_FETCH: begin
                cpu_uart_msg <= read1_data;
                cpu_byte_idx <= 0;
                return_state <= C_HALT_NEXT;
                state <= C_UART_SEND;
            end

            C_HALT_NEXT: begin
                if (dump_reg_idx == 5'd31) begin
                    state <= C_HALT_FOREVER;
                end else begin
                    dump_reg_idx <= dump_reg_idx + 5'd1;
                    state <= C_HALT_SETUP;
                end
            end

            C_HALT_FOREVER: begin
                cpu_done <= 1;
                state <= C_HALT_FOREVER;
            end

            default: state <= C_DDR_WAIT_READ;
        endcase
    end
    // --- CTRL+ESC SOFT REBOOT TRIGGER FOR CPU ---
    else if (cpu_done && ps2_data[4] && ps2_data[7]) begin
        state <= C_DDR_WAIT_READ;
        cpu_ddr_cmd_en <= 0;
        cpu_ddr_wr_data_en <= 0;
        cpu_ddr_wr_data_end <= 0;
        cpu_ddr_wr_data_mask <= 16'h0000;
        cpu_uart_wr_en <= 0;
        cpu_user_addr <= 27'd0;
        program_counter <= 32'd0;
        memory_read_reg <= 0;
        cpu_byte_idx <= 0;
        is_instruction_fetch <= 1;
        data_addr <= 32'd0;
        cpu_store_data <= 32'd0;
        vram_write_en <= 0;
        dump_reg_idx <= 5'd0;
        pending_exec_state <= 0;
        cpu_done <= 0;

        rq_head <= 0; rq_tail <= 0; rq_is_data <= 0;
        pf_valid <= 0; dmem_valid <= 0; pf_ready_addr <= 27'h7FFFFFF;
    end
end
endmodule

