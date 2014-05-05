//Instantiation of Top (Reference Chip) with Rekall and Phy
module topWithRekall
(
 //Signals used to instantiate Top
  input         clk_ref,
  input         reset,
  output        io_host_clk,
  output        io_host_clk_edge,
  output        io_host_in_ready,
  input         io_host_in_valid,
  input [15:0]  io_host_in_bits,
  input         io_host_out_ready,
  output        io_host_out_valid,
  output [15:0] io_host_out_bits,
  output        io_host_debug_stats_pcr,
  input         io_mem_backup_en,
  output        io_in_mem_ready,
  input         io_in_mem_valid,
  input         io_out_mem_ready,
  output        io_out_mem_valid,
  input         io_init,

 //Micron DRAM interface
  inout         CLK,
  inout         CLKB,
  inout [1:0]   CKE,
  inout [1:0]   CSB,
  inout [9:0]   CA,
  inout [3:0]   DM,
  inout [31:0]  DQ,
  inout [3:0]   DQS,
  inout [3:0]   DQSB,

 //Timing regfile interface
  input         timing_rf_valid,
  output        timing_rf_ready,
  input [63:0]  timing_rf_bits_addr,
  input [63:0]  timing_rf_bits_data
);

  wire  dfi_clk;
  wire [1:0] dfi_cke;
  wire [1:0] dfi_cs_n;
  wire [19:0] dfi_address;
  wire [63:0] dfi_wrdata;
  wire [3:0] dfi_wrdata_en;
  wire [7:0] dfi_wrdata_mask;
  wire [63:0] dfi_rddata;
  wire       dfi_rddata_valid;
  wire [3:0] dfi_rddata_en;

  parameter CPU_ADDR_BITS = 26;
  parameter REK_ADDR_BITS = 24;
  parameter TAG_BITS = 5;
  parameter DATA_BITS = 128;
  parameter CMD_BITS = CPU_ADDR_BITS + TAG_BITS + 1;
   
 //Processor ioMem interface to Async Queues
  wire   io_mem_req_cmd_ready_top;
  wire  io_mem_req_cmd_valid_top;
  wire [CPU_ADDR_BITS-1:0] io_mem_req_cmd_bits_addr_top;
  wire [TAG_BITS-1:0] io_mem_req_cmd_bits_tag_top;
  wire      io_mem_req_cmd_bits_rw_top;
 
  wire     io_mem_req_data_ready_top;
  wire       io_mem_req_data_valid_top;
  wire [DATA_BITS-1:0] io_mem_req_data_bits_data_top;
 
  wire        io_mem_resp_ready_top;
  wire       io_mem_resp_valid_top;
  wire [DATA_BITS-1:0] io_mem_resp_bits_data_top;
  wire [TAG_BITS-1:0]    io_mem_resp_bits_tag_top;

  //Rekall ioMem interface to Async Queues
  wire   io_mem_req_cmd_ready_rek;
  wire  io_mem_req_cmd_valid_rek;
  wire [CPU_ADDR_BITS-1:0] io_mem_req_cmd_bits_addr_rek;
  wire [TAG_BITS-1:0] io_mem_req_cmd_bits_tag_rek;
  wire      io_mem_req_cmd_bits_rw_rek;
 
  wire     io_mem_req_data_ready_rek;
  wire       io_mem_req_data_valid_rek;
  wire [DATA_BITS-1:0] io_mem_req_data_bits_data_rek;
 
  wire        io_mem_resp_ready_rek;
  wire       io_mem_resp_valid_rek;
  wire [DATA_BITS-1:0] io_mem_resp_bits_data_rek;
  wire [TAG_BITS-1:0]    io_mem_resp_bits_tag_rek;

  //  Instantiate the processor

  Top referenceChip
  (
   .clk(clk_ref),
   .reset(reset),
   .io_host_clk(io_host_clk),
   .io_host_clk_edge(io_host_clk_edge),
   .io_host_in_ready(io_host_in_ready),
   .io_host_in_valid(io_host_in_valid),
   .io_host_in_bits(io_host_in_bits),
   .io_host_out_ready(io_host_out_ready),
   .io_host_out_valid(io_host_out_valid),
   .io_host_out_bits(io_host_out_bits),
   .io_host_debug_stats_pcr(io_host_debug_stats_pcr),
   .io_mem_req_cmd_ready(io_mem_req_cmd_ready_top),
   .io_mem_req_cmd_valid(io_mem_req_cmd_valid_top),
   .io_mem_req_cmd_bits_addr(io_mem_req_cmd_bits_addr_top),
   .io_mem_req_cmd_bits_tag(io_mem_req_cmd_bits_tag_top),
   .io_mem_req_cmd_bits_rw(io_mem_req_cmd_bits_rw_top),
   .io_mem_req_data_ready(io_mem_req_data_ready_top),
   .io_mem_req_data_valid(io_mem_req_data_valid_top),
   .io_mem_req_data_bits_data(io_mem_req_data_bits_data_top),
   .io_mem_resp_ready(io_mem_resp_ready_top),
   .io_mem_resp_valid(io_mem_resp_valid_top),
   .io_mem_resp_bits_data(io_mem_resp_bits_data_top),
   .io_mem_resp_bits_tag(io_mem_resp_bits_tag_top),
   .io_mem_backup_en(io_mem_backup_en),
   .io_in_mem_ready(io_in_mem_ready),
   .io_in_mem_valid(io_in_mem_valid),
   .io_out_mem_ready(io_out_mem_ready),
   .io_out_mem_valid(io_out_mem_valid),
   .io_init(io_init)
  );

  //wires for the TOP to Rekall Command Async Queue
  wire [CMD_BITS-1:0] top_to_rek_enq_cmd;
  wire [CMD_BITS-1:0] top_to_rek_deq_cmd;
  assign top_to_rek_enq_cmd = { io_mem_req_cmd_bits_addr_top, io_mem_req_cmd_bits_tag_top, io_mem_req_cmd_bits_rw_top };
  assign io_mem_req_cmd_bits_addr_rek = top_to_rek_deq_cmd[CMD_BITS-1:TAG_BITS+1];
  assign io_mem_req_cmd_bits_tag_rek = top_to_rek_deq_cmd[TAG_BITS:1];
  assign io_mem_req_cmd_bits_rw_rek = top_to_rek_deq_cmd[0];

  //wires for the Rekall to Top Response Async Queue
  wire [DATA_BITS+TAG_BITS-1:0] rek_to_top_enq_resp;
  wire [DATA_BITS+TAG_BITS-1:0] rek_to_top_deq_resp; 
  assign rek_to_top_enq_resp = { io_mem_resp_bits_tag_rek, io_mem_resp_bits_data_rek };
  assign io_mem_resp_bits_tag_top = rek_to_top_deq_resp[DATA_BITS+TAG_BITS-1:DATA_BITS];
  assign io_mem_resp_bits_data_top = rek_to_top_deq_resp[DATA_BITS-1:0];
 
  wire         clk_sdram;
  assign clk_sdram = CLK;

  async_fifo1 
  #(
    .DSIZE(CMD_BITS)
  ) topToRek_cmd
  (
    .deq_bits(top_to_rek_deq_cmd),
    .deq_valid(io_mem_req_cmd_valid_rek),
    .enq_bits(top_to_rek_enq_cmd),
    .enq_ready(io_mem_req_cmd_ready_top),
    .enq_valid(io_mem_req_cmd_valid_top),
    .enq_clk(clk_ref),
    .enq_rst_n(reset),
    .deq_ready(io_mem_req_cmd_ready_rek),
    .deq_clk(clk_sdram),
    .deq_rst_n(reset)
   );

  async_fifo1
  #(
    .DSIZE(DATA_BITS)
  ) topToRek_data
  (
    .deq_bits(io_mem_req_data_bits_data_rek),
    .deq_valid(io_mem_req_data_valid_rek),
    .enq_bits(io_mem_req_data_bits_data_top),
    .enq_ready(io_mem_req_data_ready_top),
    .enq_valid(io_mem_req_data_valid_top),
    .enq_clk(clk_ref),
    .enq_rst_n(reset),
    .deq_ready(io_mem_req_data_ready_rek),
    .deq_clk(clk_sdram),
    .deq_rst_n(reset)
  );

  async_fifo1
  #(
    .DSIZE(DATA_BITS+TAG_BITS)
  ) rekToTop_resp
  (
    .deq_bits(rek_to_top_deq_resp),
    .deq_valid(io_mem_resp_valid_top),
    .enq_bits(rek_to_top_enq_resp),
    .enq_ready(io_mem_resp_ready_rek),
    .enq_valid(io_mem_resp_valid_rek),
    .enq_clk(clk_sdram),
    .enq_rst_n(reset),
    .deq_ready(io_mem_resp_ready_top),
    .deq_clk(clk_ref),
    .deq_rst_n(reset)
  );

  rekallRekall memAccessScheduler
  (
   .clk(clk_sdram),
   .reset(reset),
   //Async Queue IoMem interface
   .io_req_ready(io_mem_req_cmd_ready_rek),
   .io_req_valid(io_mem_req_cmd_valid_rek),
   //Drop the top 2 bits from the CPU since it maps to a larger address space
   //than we have in our 4Gb DRAM
   .io_req_bits_addr(io_mem_req_cmd_bits_addr_rek[REK_ADDR_BITS-1:0]),
   .io_req_bits_RW(io_mem_req_cmd_bits_rw_rek),
   .io_req_bits_tag(io_mem_req_cmd_bits_tag_rek),
   .io_req_data_ready(io_mem_req_data_ready_rek),
   .io_req_data_valid(io_mem_req_data_valid_rek),
   .io_req_data_bits(io_mem_req_data_bits_data_rek),
   .io_resp_ready(io_mem_resp_ready_rek),
   .io_resp_valid(io_mem_resp_valid_rek),
   .io_resp_bits_tag(io_mem_resp_bits_tag_rek),
   .io_resp_bits_data(io_mem_resp_bits_data_rek),
   // DFI interface to PHY
   .io_dfi_clk(dfi_clk),
   .io_dfi_cs_n(dfi_cs_n),
   .io_dfi_address(dfi_address),
   .io_dfi_wrdata(dfi_wrdata),
   .io_dfi_wrdata_en(dfi_wrdata_en),
   .io_dfi_wrdata_mask(dfi_wrdata_mask),
   .io_dfi_rddata_en(dfi_rddata_en),
   .io_dfi_cke(dfi_cke),
   .io_dfi_rddata (dfi_rddata),
   .io_dfi_rddata_valid(dfi_rddata_valid),
   //Timing Constraint Regfile Control
   .io_timing_rf_valid(timing_rf_valid),
   .io_timing_rf_ready(timing_rf_ready),
   .io_timing_rf_bits_addr(timing_rf_bits_addr),
   .io_timing_rf_bits_data(timing_rf_bits_data)
  );


  LPDDR2PHY phy
  (
    .reset(reset),
    .CLKB(CLKB),
    .CLK(CLK),
    .CSB(CSB),
    .CA(CA),
    .DQS(DQS),
    .DM(DM),
    .DQ(DQ),
    .DQSB(DQSB),
    .CKE(CKE),
    .dfi_cs_n(dfi_cs_n),
    .dfi_address(dfi_address),
    .dfi_wrdata(dfi_wrdata),
    .dfi_wrdata_en(dfi_wrdata_en),
    .dfi_wrdata_mask(dfi_wrdata_mask),
    .dfi_rddata_en(dfi_rddata_en),
    .dfi_rddata(dfi_rddata),
    .dfi_rddata_valid(dfi_rddata_valid),
    .dfi_cke(dfi_cke)
  );
   
endmodule