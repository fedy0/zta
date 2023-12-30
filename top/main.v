//---------------------------------------------------------------------------
// Copyright [2014] [Ztachip Technologies Inc]
//
// Author: Vuong Nguyen
// Modified/Ported to ULX3S with SDRAM Controller by Ifediora E C
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except IN compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to IN writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//----------------------------------------------------------------------------
//
// This is the reference design to show how to build a SOC with ztachip as 
// accelerator for vision/AI workload
//
// The example here is based on ULX3S board. However, it is generic
// and can be adapted to other FPGA or ASIC platform.
// When ported to other FPGA/ASIC, the components to be replaced in this example
// are SDRAM-Controller. Other components including ztachip are 
// generic HDL implementation.
//
// Processor used here is RISCV based on VexRiscv implementation
//
// Components are interconnected over AXI bus (memory-map/stream/peripheral).
//
//    1) DDR memory controller is connected via 64-bit AXI memory-mapped bus.
//    2) SoC CORE_BASE has RISCV ZTACHIP which has 2 AXI bus
//       * one to receive tensor instructions from RISCV
//       * one to initiate memory DMA transfer
//    3) VGA receives screen data from DDR memory over AXI streaming bus.
//       * DDR Memory blocks are managed by Xilinx VDMA within AXI crossbar.
//    4) CAMERA sends captured data to DDR memory over AXI streaming bus.
//       * DDR Memory blocks are managed by Xilinx VDMA within AXI crossbar. 
//    5) GPIO is connected via AXI Advanced Peripheral Bus.
//
//  +-----------+
//  +           + AXI-MM     +--------------+
//  +           +----------->+    SDRAM     +--> DDR3 Bus
//  +           +            +  Controller  +
//  +           +            +--------------+
//  +           +
//  +           + AXI-Stream +--------------+
//  +    SoC    +------------+     VGA      +--> VGA signals
//  + CORE_BASE +            +--------------+
//  +           +
//  +           + AXI-Stream +--------------+
//  +           +------------+    Camera    +--> OV7670 I2C/Signals               
//  +           +            +--------------+
//  +           +
//  +           + AXI-APB    +--------------+
//  +           +------------+     GPIO     +--> LED/BUTTON
//  +           +            +--------------+
//  +-----------+
//

//--------------------------------------------------------------------------
//                  TOP COMPONENT DECLARATION
//                  SIGNAL/PIN ASSIGNMENTS
//--------------------------------------------------------------------------
                              
module main(

   // External memory data bus is 64-bit wide
   // This should match with exmem_data_width_c defined in HW/src/config.vhd
   `define EXMEM_DATA_WIDTH 64

   // External memory data bus is 32-bit wide
   // This should match with exmem_data_width_c defined in HW/src/config.vhd
   //`define EXMEM_DATA_WIDTH 32

   // RISCV is booted using Xilinx built-in JTAG
   `define RISCV_MODE "RISCV_XILINX_BSCAN2_JTAG" 

   // RISCV is booted using an external JTAG adapter
   // `define RISCV_MODE "RISCV_JTAG" 

   // RISCV is running in simulation mode.
   // `define RISCV_MODE "RISCV_SIM" 
     
   // Reference clock/external reset
   
   input          sys_resetn,
   input          sys_clock,
   
   //  SDRAM signals 
   output          sdram_clk,
   output          sdram_cke,
   output          sdram_csn,
   output          sdram_rasn,
   output          sdram_casn,
   output          sdram_wen,
   output [  1:0]  sdram_dqm,
   output [ 12:0]  sdram_a,
   output [  1:0]  sdram_ba,
   inout  [ 15:0]  sdram_d,
   
   // UART signals
   output         ULX3S_TX,
   input          ULX3S_RX,
   
   // GPIO signals
   output [3:0]   led,
   input [3:0]    btn,
   
   // VGA signals
   output         VGA_HS_O,
   output         VGA_VS_O,
   output [3:0]   VGA_R,
   output [3:0]   VGA_B,
   output [3:0]   VGA_G,
   
   // CAMERA signals
   output         CAMERA_SCL,
   input          CAMERA_VS,
   input          CAMERA_PCLK,
   input [7:0]    CAMERA_D,
   output         CAMERA_RESET,
   inout          CAMERA_SDR,
   input          CAMERA_RS,
   output         CAMERA_MCLK,
   output         CAMERA_PWDN      
   );

   wire [ 15:0]        sdram_data_in_w;
   wire [ 15:0]        sdram_data_out_w;
   wire                sdram_data_out_en_w;
   
   wire               SDRAM_clk;
   wire [31:0]        SDRAM_araddr;
   wire [1:0]         SDRAM_arburst;
   wire [7:0]         SDRAM_arlen;
   wire               SDRAM_arready;
   wire [2:0]         SDRAM_arsize;
   wire               SDRAM_arvalid;
   wire [31:0]        SDRAM_awaddr;
   wire [1:0]         SDRAM_awburst;
   wire [7:0]         SDRAM_awlen;
   wire               SDRAM_awready;
   wire [2:0]         SDRAM_awsize;
   wire               SDRAM_awvalid;
   wire               SDRAM_bready;
   wire [1:0]         SDRAM_bresp;
   wire               SDRAM_bvalid;
   wire               SDRAM_rlast;
   wire               SDRAM_rready;
   wire [1:0]         SDRAM_rresp;
   wire               SDRAM_rvalid;
   wire               SDRAM_wlast;
   wire               SDRAM_wready;
   wire               SDRAM_wvalid;

   wire [`EXMEM_DATA_WIDTH-1:0] SDRAM_rdata;
   wire [`EXMEM_DATA_WIDTH-1:0] SDRAM_wdata;
   wire [`EXMEM_DATA_WIDTH/8-1:0] SDRAM_wstrb;
      
   wire [31:0]        VIDEO_tdata;
   wire               VIDEO_tlast;
   wire               VIDEO_tready;
   wire               VIDEO_tvalid;

   wire [31:0]        camera_tdata;
   wire               camera_tlast;
   wire               camera_tready;
   wire [0:0]         camera_tuser;
   wire               camera_tvalid;

   soc_base #(.RISCV(`RISCV_MODE)) soc_base_inst (

      .clk_main(clk_main),
      .clk_x2_main(clk_x2_main),
      .clk_reset(1), // Dont need reset for FPGA design. Register already initialized after programming.

      // With this example, JTAG is using Xilinx built-in JTAG. So no external 
      // JTAG adapter is required
      // To boot with an external JTAG, TMS/TMO/TDO/TCLK need to be routed
      // to GPIO pins that in turn connect to an external JTAG adapter
      // To enable booting using external JTAG, set RISCV_MODE="RISCV_JTAG" above

      .TMS(0),
      .TDI(0),
      .TDO(),
      .TCK(0),

      .led(led),
      .pushbutton(btn),

      .UART_TXD(ULX3S_TX),
      .UART_RXD(ULX3S_RX),

      .VIDEO_clk(clk_vga),  
      .VIDEO_tdata(VIDEO_tdata),
      .VIDEO_tready(VIDEO_tready),
      .VIDEO_tvalid(VIDEO_tvalid),
      .VIDEO_tlast(VIDEO_tlast),

      .camera_clk(CAMERA_PCLK),
      .camera_tdata(camera_tdata),
      .camera_tlast(camera_tlast),
      .camera_tready(camera_tready),
      .camera_tvalid(camera_tvalid),

      .SDRAM_clk(SDRAM_clk),
      .SDRAM_reset(1), // Dont need reset for FPGA design. Register already intialized after programming. 
      .SDRAM_araddr(SDRAM_araddr),
      .SDRAM_arburst(SDRAM_arburst),
      .SDRAM_arlen(SDRAM_arlen),
      .SDRAM_arready(SDRAM_arready),
      .SDRAM_arsize(SDRAM_arsize),
      .SDRAM_arvalid(SDRAM_arvalid),
      .SDRAM_awaddr(SDRAM_awaddr),
      .SDRAM_awburst(SDRAM_awburst),
      .SDRAM_awlen(SDRAM_awlen),
      .SDRAM_awready(SDRAM_awready),
      .SDRAM_awsize(SDRAM_awsize),
      .SDRAM_awvalid(SDRAM_awvalid),
      .SDRAM_bready(SDRAM_bready),
      .SDRAM_bresp(SDRAM_bresp),
      .SDRAM_bvalid(SDRAM_bvalid),
      .SDRAM_rdata(SDRAM_rdata),
      .SDRAM_rlast(SDRAM_rlast),
      .SDRAM_rready(SDRAM_rready),
      .SDRAM_rresp(SDRAM_rresp),
      .SDRAM_rvalid(SDRAM_rvalid),
      .SDRAM_wdata(SDRAM_wdata),
      .SDRAM_wlast(SDRAM_wlast),
      .SDRAM_wready(SDRAM_wready),
      .SDRAM_wstrb(SDRAM_wstrb),
      .SDRAM_wvalid(SDRAM_wvalid)
   );

   //---------------------------
   // SDRAM Memory controller
   //---------------------------

sdram_axi sdram_axi_inst
(
    .clk_i(SDRAM_clk),  // or sys_clock ?
    .rst_i(sys_resetn),

    // AXI port
    // Inputs
    // .s_axi_awsize(SDRAM_awsize),
    // .s_axi_arsize(SDRAM_arsize),
    .inport_awvalid_i(SDRAM_awvalid),
    .inport_awaddr_i(SDRAM_awaddr),
    .inport_awid_i(0),
    .inport_awlen_i(SDRAM_awlen),
    .inport_awburst_i(SDRAM_awburst),
    .inport_wvalid_i(SDRAM_wvalid),
    .inport_wdata_i(SDRAM_wdata),
    .inport_wstrb_i(SDRAM_wstrb),
    .inport_wlast_i(SDRAM_wlast),
    .inport_bready_i(SDRAM_bready),
    .inport_arvalid_i(SDRAM_arvalid),
    .inport_araddr_i(SDRAM_araddr),
    .inport_arid_i(0),
    .inport_arlen_i(SDRAM_arlen),
    .inport_arburst_i(SDRAM_arburst),
    .inport_rready_i(SDRAM_rready),
    // Outputs
    .inport_awready_o(SDRAM_awready),
    .inport_wready_o(SDRAM_wready),
    .inport_bvalid_o(SDRAM_bvalid),
    .inport_bresp_o(SDRAM_bresp),
    .inport_bid_o(),
    .inport_arready_o(SDRAM_arready),
    .inport_rvalid_o(SDRAM_rvalid),
    .inport_rdata_o(SDRAM_rdata),
    .inport_rresp_o(SDRAM_rresp),
    .inport_rid_o(),
    .inport_rlast_o(SDRAM_rlast),

    // SDRAM Interface
    .sdram_clk_o(sdram_clk),
    .sdram_cke_o(sdram_cke),
    .sdram_cs_o(sdram_csn),
    .sdram_ras_o(sdram_rasn),
    .sdram_cas_o(sdram_casn),
    .sdram_we_o(sdram_wen),
    .sdram_dqm_o(sdram_dqm),
    .sdram_addr_o(sdram_a),
    .sdram_ba_o(sdram_ba),
    .sdram_data_input_i(sdram_data_in_w),
    .sdram_data_output_o(sdram_data_out_w),
    .sdram_data_out_en_o(sdram_data_out_en_w)
);

   //-----------
   // VGA
   //-----------
   
   vga vga_inst(
      .clk_in(clk_vga),
      .tdata_in(VIDEO_tdata),
      .tready_out(VIDEO_tready),
      .tvalid_in(VIDEO_tvalid),
      .tlast_in(VIDEO_tlast),
      .VGA_HS_O_out(VGA_HS_O),
      .VGA_VS_O_out(VGA_VS_O),
      .VGA_R_out(VGA_R),
      .VGA_B_out(VGA_B),
      .VGA_G_out(VGA_G)
   );

   //------------------------
   // Camera
   //------------------------
   
   camera camera_inst(
      .clk_in(clk_camera),
      .SIOC(CAMERA_SCL),
      .SIOD(CAMERA_SDR),
      .RESET(CAMERA_RESET),
      .PWDN(CAMERA_PWDN),
      .XCLK(CAMERA_MCLK),  
      .CAMERA_PCLK(CAMERA_PCLK),
      .CAMERA_D(CAMERA_D),
      .CAMERA_VS(CAMERA_VS),
      .CAMERA_RS(CAMERA_RS),
      .tdata_out(camera_tdata),
      .tlast_out(camera_tlast),
      .tready_in(camera_tready),
      .tuser_out(),
      .tvalid_out(camera_tvalid)
   );

   // ------------------
   // Clock synthesizer
   // -------------------

   clk_wiz_0 clk_wiz_inst(
      .clk_out1(clk_vga),
      .clk_out2(clk_mig_ref),
      .clk_out3(clk_mig_sysclk),
      .clk_out4(clk_camera),
      .clk_out5(clk_main),
      .clk_out6(clk_x2_main),
      .resetn(sys_resetn),
      .locked(),
      .clk_in1(sys_clock));

endmodule
