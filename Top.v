`define MNONE 2'b00
`define MREAD 2'b01
`define MWRITE 2'b10

`define LED0 7'b1000000
`define LED1 7'b1111001
`define LED2 7'b0100100
`define LED3 7'b0110000
`define LED4 7'b0011001
`define LED5 7'b0010010
`define LED6 7'b0000010
`define LED7 7'b1111000
`define LED8 7'b0000000
`define LED9 7'b0010000
`define LEDA 7'b0001000
`define LEDb 7'b0000011
`define LEDC 7'b1000110
`define LEDd 7'b0100001
`define LEDE 7'b0000110
`define LEDF 7'b0001110

module top(KEY,SW,LEDR,HEX0,HEX1,HEX2,HEX3,HEX4,HEX5);
                  
    input [3:0] KEY;
    input [9:0] SW;
    output [9:0] LEDR;
    output [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;

    wire [1:0] mem_cmd;
    wire [8:0] mem_addr;
    wire [15:0] datapath_out, dout, din, write_data;
    wire [7:0] read_address, write_address;
    wire andIn_9, msel;
    wire and_out, write;
    wire clk, reset;
    wire N, V, Z;
    wire write_EqualOut;

    wire [15:0] read_data;

    //Instantiate RAM
    RAM MEM(clk,read_address,write_address,write,din,dout);

    //Instantiate CPU module
    cpu CPU(clk,reset,read_data,datapath_out,N,V,Z,
            mem_cmd, mem_addr, write_data);

    //Equality comparator 8
    assign msel = (1'b0 == mem_addr[8]);

    //Equality comparator 9
    assign andIn_9 = (`MREAD == mem_cmd);

    //And Gate for tri-state driver
    assign and_out = msel & andIn_9;
    
    //Tri-state driver
    assign read_data = and_out ? dout : {16{1'bz}};

    assign clk = ~KEY[0];
    assign reset = ~KEY[1];

    assign read_address = mem_addr [7:0];
    assign write_address = mem_addr [7:0];

    //Equality comparator for WRITE
    assign write_EqualOut = (`MWRITE == mem_cmd);

    //And Gate for write
    assign write = write_EqualOut & msel;
    
    assign din = write_data;

    //Switch take in section
    wire SW_cmd_eq = (`MREAD == mem_cmd);
    wire SW_addr_eq = (9'h140 == mem_addr);
    wire SW_in_eq = (SW_cmd_eq & SW_addr_eq);
    assign read_data[15:8] = SW_in_eq ? 8'h00 : {8{1'bz}};
    assign read_data[7:0] = SW_in_eq ? SW[7:0] : {8{1'bz}};
    
    //LEDR write out section
    wire LED_cmd_eq = (`MWRITE == mem_cmd);
    wire LED_addr_eq = (9'h100 == mem_addr);
    wire LED_load = (LED_addr_eq & LED_cmd_eq);
    vDFF1 #(8) load_LED (clk, LED_load, write_data[7:0], LEDR[7:0]);

    // fill in sseg to display 4-bits in hexidecimal 0,1,2...9,A,B,C,D,E,F
    sseg H0(LEDR [3:0],   HEX0);
    sseg H1(LEDR [7:4],   HEX1);
    assign HEX2 = 7'b1111111;
    assign HEX3 = 7'b1111111;
    assign HEX4 = 7'b1111111;
    assign HEX5 = 7'b1111111;

endmodule



module RAM(clk,read_address,write_address,write,din,dout);
  parameter data_width = 16; 
  parameter addr_width = 8;
  parameter filename = "data.txt";

  input clk;
  input [addr_width-1:0] read_address, write_address;
  input write;
  input [data_width-1:0] din;
  output [data_width-1:0] dout;
  reg [data_width-1:0] dout;

  reg [data_width-1:0] mem [2**addr_width-1:0];

  initial $readmemb(filename, mem);

  always @ (posedge clk) begin
    if (write)
      mem[write_address] <= din;
    dout <= mem[read_address]; // dout doesn't get din in this clock cycle 
                               // (this is due to Verilog non-blocking assignment "<=")
  end 
endmodule


// Module for displaying digits on DE1-Soc board
module sseg(in,segs);
  input [3:0] in;
  output reg [6:0] segs;

  always @(*) begin
    case (in)
      4'b0000: segs = `LED0;
      4'b0001: segs = `LED1;
      4'b0010: segs = `LED2;
      4'b0011: segs = `LED3;
      4'b0100: segs = `LED4;
      4'b0101: segs = `LED5;
      4'b0110: segs = `LED6;
      4'b0111: segs = `LED7;
      4'b1000: segs = `LED8;
      4'b1001: segs = `LED9;
      4'b1010: segs = `LEDA;
      4'b1011: segs = `LEDb;
      4'b1100: segs = `LEDC;
      4'b1101: segs = `LEDd;
      4'b1110: segs = `LEDE;
      4'b1111: segs = `LEDF;
      default: segs = `LED0;
    endcase
  end
endmodule