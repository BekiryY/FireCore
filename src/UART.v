module UART_Controller
#(
    parameter BAUD_RATE = 115200,
    parameter CLOCK_FREQ = 100000000
)(
    input sys_clk,
    input sys_rst_n,
    input write_enable,
    input [7:0] data_to_send,
    input RX,
    output reg TX = 1,
    output reg write_done = 0,
    output reg read_done = 0,
    output reg [7:0] data_readed
);

localparam counter_up_limit = CLOCK_FREQ / BAUD_RATE;

reg [31:0] clock_counter = 0;
reg [3:0] current_bit = 0; // supports 0-9 (start + 8 data + stop)
reg write_status = 0;
reg write_status_prev = 0;

reg current_RX_bit = 1;
reg prev_RX_bit = 1;
reg [31:0] clock_counter_RX = 0;
reg [3:0] current_bit_RX = 0; // supports 0-9 (start + 8 data + stop)
reg read_status = 0;
reg read_status_prev = 0;
reg read_done_set = 0;  // internal: 1 when stop bit just sampled OK



always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        TX <= 1;
        clock_counter <= 0;
        current_bit <= 0;
        write_status <= 0;
        write_status_prev <= 0;
        write_done <= 0;
        read_done <= 0;
        data_readed <= 8'b11111111;
        current_RX_bit <= 1;
        prev_RX_bit <= 1;
        clock_counter_RX <= 0;
        current_bit_RX <= 0; // supports 0-9 (start + 8 data + stop)
        read_status <= 0;
        read_status_prev <= 0;

//----------For TX-------------------------------------------------------------
    end else begin
        if (write_enable && !write_status && !write_status_prev) begin
            write_status <= 1;
        end else if (!write_enable) begin
            write_done <= 0;
            write_status <= 0;
            write_status_prev <= 0;
        end

        if (clock_counter >= counter_up_limit) begin
            clock_counter <= 0;

            if (write_status && !write_status_prev) begin
                TX <= 0; // Start bit
                write_status_prev <= 1;
            end
            else if (write_status) begin
                if (current_bit < 8) begin
                    TX <= data_to_send[current_bit];
                    current_bit <= current_bit + 4'd1;
                end
                else if (current_bit == 8) begin
                    TX <= 1; // Stop bit
                    current_bit <= current_bit + 4'd1;
                end
                else begin
                    current_bit <= 0;
                    write_status <= 0;
                    write_done <= 1;
                end
            end

        end else begin
            clock_counter <= clock_counter + 1;
        end

//--------------For RX--------------------------------------------------------------------------------------------
        // 2-stage synchronizer for RX to prevent metastability
        current_RX_bit <= RX;
        prev_RX_bit <= current_RX_bit;

        // read_done is a 1-cycle pulse: set it one cycle, then clear it
        read_done <= read_done_set;
        read_done_set <= 0;

        if (!read_status && !read_status_prev) begin
            // Idle state, look for falling edge of start bit
            if (current_RX_bit == 0 && prev_RX_bit == 1) begin
                read_status <= 1;
                clock_counter_RX <= 0;
                current_bit_RX <= 0;
                data_readed <= 8'b1111_1111;
            end
        end else if (read_status) begin
            clock_counter_RX <= clock_counter_RX + 1;
            
            // Phase 1: Wait half a baud period to center the sample point
            if (!read_status_prev && (clock_counter_RX >= (counter_up_limit / 2))) begin
                read_status_prev <= 1;
                clock_counter_RX <= 0; // Reset counter to count full bits now
            end
            
            // Phase 2: Sample every full baud period
            if (read_status_prev && (clock_counter_RX >= counter_up_limit)) begin
                clock_counter_RX <= 0;
                
                if (current_bit_RX < 8) begin
                    // Shift in data using the synchronized RX signal
                    data_readed[current_bit_RX] <= prev_RX_bit;             
                    current_bit_RX <= current_bit_RX + 4'd1;
                end else begin
                    // Stop bit logic
                    if (prev_RX_bit == 1) begin // Stop bit should be high
                        read_done_set <= 1;  // Will generate a 1-cycle pulse on read_done
                    end else begin
                        // Framing error — discard
                        data_readed <= 8'b1111_1111;
                    end
                    read_status <= 0;
                    read_status_prev <= 0;
                end
            end
        end
    end
end
endmodule
