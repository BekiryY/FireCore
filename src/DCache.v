// =============================================================================
// DATA CACHE (DCache)
// Direct-Mapped 4KB Read-Only Cache (Write-Through / Invalidate on Write)
// =============================================================================
module DCache (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [31:0] cpu_addr,
    input  wire        cpu_req,
    output reg  [31:0] cpu_data,
    output reg         cpu_valid,
    output wire        cache_hit,
    input  wire [31:0]  fill_addr,
    input  wire [127:0] fill_data,
    input  wire         fill_en,
    input  wire [31:0]  inv_addr,
    input  wire         inv_en
);

(* ram_style = "block" *) reg [127:0] data_mem [0:255];
(* ram_style = "block" *) reg [31:0]  tag_mem  [0:255];   // [20]=valid, [19:0]=tag

wire [7:0]  idx      = cpu_addr[11:4];
wire [19:0] req_tag  = cpu_addr[31:12];

reg [127:0] read_line;
reg [31:0]  tag_out;
reg [19:0]  req_tag_reg;
reg [1:0]   word_sel;
reg         cache_hit_reg;

always @(posedge clk) begin
    if (cpu_req) begin
        read_line     <= data_mem[idx];
        tag_out       <= tag_mem[idx];
        req_tag_reg   <= req_tag;
        word_sel      <= cpu_addr[3:2];
        cache_hit_reg <= 0;   // Reset: yeni istek gelince valid'i kapat
        cpu_valid     <= 0;
    end else begin
        // BSRAM verisi yerlestikten 1 saat sonra hit karari
        cache_hit_reg <= tag_out[20] & (tag_out[19:0] == req_tag_reg);
        cpu_valid     <= cache_hit_reg;
        // Kayitli data cikisi
        case (word_sel)
            2'd0: cpu_data <= read_line[31:0];
            2'd1: cpu_data <= read_line[63:32];
            2'd2: cpu_data <= read_line[95:64];
            2'd3: cpu_data <= read_line[127:96];
        endcase
    end
end

assign cache_hit = cache_hit_reg;

always @(posedge clk) begin
    if (fill_en) begin
        data_mem[fill_addr[11:4]] <= fill_data;
    end
end

wire [7:0]  tag_wr_addr = fill_en ? fill_addr[11:4] : inv_addr[11:4];
wire [31:0] tag_wr_data = fill_en ? {11'd0, 1'b1, fill_addr[31:12]} : 32'd0;

always @(posedge clk) begin
    if (fill_en || inv_en) begin
        tag_mem[tag_wr_addr] <= tag_wr_data;
    end
end

integer ci2;
initial begin
    for (ci2 = 0; ci2 < 256; ci2 = ci2 + 1) begin
        tag_mem[ci2]  = 32'd0;
        data_mem[ci2] = 128'd0;
    end
end

endmodule
