`define SA 5'b01101
`define SB 5'b01110
`define SC 5'b01111

`define S0 5'b00000
`define S1 5'b00001
`define S2 5'b00010
`define S3 5'b00011
`define S4 5'b00100
`define S5 5'b00101
`define S6 5'b00110
`define S7 5'b00111
`define S8 5'b01000
`define S9 5'b01001
`define S10 5'b01010
`define S11 5'b01011
`define S12 5'b01100
`define S13 5'b10000
`define S14 5'b10001
`define S15 5'b10010
`define S16 5'b10011
`define S17 5'b10100
`define S18 5'b10101
`define S19 5'b10110
`define S20 5'b10111
`define S21 5'b11000
`define S_wait_LDR 5'b11001
`define S_wait_STR 5'b11010

`define MNONE 2'b00
`define MREAD 2'b01
`define MWRITE 2'b10

module cpu(clk,reset,read_data,datapath_out,N,V,Z,
            mem_cmd, mem_addr, write_data);
    input clk, reset;
    input [15:0] read_data;

    output [15:0] datapath_out;
    output N, V, Z;
    output [1:0] mem_cmd;
    output [8:0] mem_addr;
    output [15:0] write_data;
    
    wire [15:0] instruction_out;

    wire [2:0] nsel;
    wire [2:0] opcode;
    wire [1:0] op, ALUop, shift;
    wire [15:0] mdata, sximm5, sximm8;
    wire [2:0] readnum, writenum;
    wire [7:0] PC_DP;
    wire [3:0] vsel;
    wire loada, loadb, loadc, loads, asel, bsel, write;
    wire load_ir, load_pc, reset_pc, addr_sel, load_addr;
    wire [8:0] next_pc, PC, addr_0;
    
    wire [8:0] next_pc_add;

    assign mdata = read_data;
    assign write_data = datapath_out;
    
    //Instantiate Instruction Register
    vDFF1 Instruction_reg (clk, load_ir, read_data, instruction_out);

    //Instantiate Instruction Decoder
    Instruction_DEC Decoder (instruction_out, nsel, opcode, op, ALUop, sximm5, sximm8, shift, readnum, writenum);

    //Instantiate State Machine
    State_machine State (clk, reset, opcode, op, nsel, vsel, loada, loadb, asel, bsel, loadc, loads, write, 
                         load_ir, load_pc, reset_pc, addr_sel, mem_cmd, load_addr);

    //Instantiate Datapath
    datapath DP (clk, readnum, vsel, loada, loadb, shift, asel, bsel, ALUop, 
                loadc, loads, writenum, write, Z, datapath_out,
                mdata, sximm8, sximm5, PC_DP, N, V, opcode);

    assign next_pc_add = PC + 1'b1;

    //PC MUX
    assign next_pc = reset_pc ? {9{1'b0}} : next_pc_add;
    
    //Instantiate VDFF for Program Counter
    vDFF1 #(9) Program_Counter (clk, load_pc, next_pc, PC);

    //Address selector MUX
    assign mem_addr = addr_sel ? PC : addr_0;

    //Instantiate VDFF for Data Address
    vDFF1 #(9) Data_Address (clk, load_addr, datapath_out[8:0], addr_0);

endmodule


// Module for Instruction Decoder
module Instruction_DEC(data_in, nsel, 
                        opcode, op, ALUop, sximm5, sximm8, shift, readnum, writenum);
    input [15:0] data_in;
    input [2:0] nsel;

    output [2:0] opcode;
    output [1:0] op, ALUop, shift;
    output [15:0] sximm5, sximm8;
    output [2:0] readnum, writenum;

    wire [2:0] Rn, Rd, Rm;
    wire [4:0] imm5;
    wire [7:0] imm8;
    reg [2:0] mux_out;
    
    assign opcode = data_in [15:13];
    assign op = data_in [12:11];
    assign ALUop = data_in [12:11];
    assign shift = data_in [4:3];
    
    assign Rn = data_in [10:8];
    assign Rd = data_in [7:5];
    assign Rm = data_in [2:0];

    assign imm5 = data_in [4:0];
    assign imm8 = data_in [7:0];

    //Sign Extend
    assign sximm5 = imm5[4] ? {{11{1'b1}}, imm5} : {{11{1'b0}}, imm5};
    assign sximm8 = imm8[7] ? {{8{1'b1}}, imm8} : {{8{1'b0}}, imm8};

    //Always block for multiplexer
    always @(*) begin
        case(nsel)
            3'b001: mux_out = Rn;
            3'b010: mux_out = Rd;
            3'b100: mux_out = Rm;
            default: mux_out = 3'bxxx;
        endcase
    end

    assign readnum = mux_out;
    assign writenum = mux_out;

endmodule


// Moduel for State Machine
module State_machine (clk, reset, opcode, op, 
                    nsel, vsel, loada, loadb, asel, bsel, loadc, loads, write, 
                    load_ir, load_pc, reset_pc, addr_sel, mem_cmd, load_addr);
    
    input clk, reset;
    input [1:0] op;
    input [2:0] opcode;
    
    output reg [2:0] nsel;
    output reg [3:0] vsel;
    output reg loada, loadb, asel, bsel, loadc, loads, write;
    output reg load_ir, load_pc, reset_pc, addr_sel, load_addr;
    output reg [1:0] mem_cmd;

    reg [4:0] present_state;
    
    //Always block for states
    always @(posedge clk) begin
        if (reset == 1'b1) begin
			present_state = `SA;
		end 
        
        else begin
            case(present_state)
                `SA:    present_state = `SB;
                
                `SB:    present_state = `SC;

                `SC:    present_state = `S0;
                
                `S0:    present_state = `S1;
                
                `S1:    if(opcode == 3'b110 && op == 2'b10) begin
                            present_state = `S2;
                        end
                        else if(opcode == 3'b110 && op == 2'b00) begin
                            present_state = `S3;
                        end
                        else if(opcode == 3'b101 && op !== 2'b11) begin
                            present_state = `S4;
                        end
                        else if(opcode == 3'b101 && op == 2'b11) begin
                            present_state = `S5;
                        end
                        else if(opcode == 3'b011 && op == 2'b00) begin
                            present_state = `S13;
                        end
                        else if(opcode == 3'b100 && op == 2'b00) begin
                            present_state = `S17;
                        end
                        else if(opcode == 3'b111 && op == 2'b00) begin
                            present_state = `S21;
                        end
                        else begin
                            present_state = 4'bxxxx;
                        end

                `S2:    present_state = `SB;

                `S3:    present_state = `S6;

                `S4:    present_state = `S7;

                `S5:    present_state = `S8;

                `S6:    present_state = `S12;

                `S7:    if(op == 2'b00) begin
                            present_state = `S9;
                        end
                        else if(op == 2'b01) begin
                            present_state = `S10;
                        end
                        else if(op == 2'b10) begin
                            present_state = `S11;
                        end
                        else begin
                            present_state = 4'bxxxx;
                        end
                
                `S8:    present_state = `S12;
                
                `S9:    present_state = `S12;

                `S10:   present_state = `SB;

                `S11:   present_state = `S12;
                            
                `S12:   present_state = `SB;

                `S13:   present_state = `S14;

                `S14:   present_state = `S15;

                `S15:   present_state = `S_wait_LDR;

                `S_wait_LDR: present_state = `S16;

                `S16:   present_state = `SB;

                `S17:   present_state = `S18;

                `S18:   present_state = `S19;

                `S19:   present_state = `S20;

                `S20:   present_state = `S_wait_STR;

                `S_wait_STR: present_state = `SB;

                `S21:   present_state = `S21;

                default:    present_state = 5'bxxxxx;
                
            endcase
        end
    end

    //Always block for output
    always @(*) begin
        case(present_state)
            `SA:begin
                    reset_pc = 1;
                    load_pc = 1;
                    mem_cmd = `MNONE;
                    addr_sel = 0;
                    load_ir = 0;
                    load_addr = 0;

                    loada = 1'b0;
                    loadb = 1'b0;
                    loadc = 1'b0;
                    loads = 1'b0;
                    write = 1'b0;
                    asel = 1'b0;
                    bsel = 1'b0;
                    vsel = 4'b0;
                    nsel = 3'b0;
                end  

            `SB:begin
                    addr_sel = 1;
                    mem_cmd = `MREAD;

                    reset_pc = 0;
                    load_pc = 0;
                    load_ir = 0;
                    load_addr = 0;
                    loada = 1'b0;
                    loadb = 1'b0;
                    loadc = 1'b0;
                    loads = 1'b0;
                    write = 1'b0;
                    asel = 1'b0;
                    bsel = 1'b0;
                    vsel = 4'b0;
                    nsel = 3'b0;
                end

            `SC:begin
                    addr_sel = 1;
                    load_ir = 1;
                    mem_cmd = `MREAD;

                    reset_pc = 0;
                    load_pc = 0;
                    load_addr = 0;
                    loada = 1'b0;
                    loadb = 1'b0;
                    loadc = 1'b0;
                    loads = 1'b0;
                    write = 1'b0;
                    asel = 1'b0;
                    bsel = 1'b0;
                    vsel = 4'b0;
                    nsel = 3'b0;
                end

            `S0:begin
                    load_pc = 1;
                    mem_cmd = `MNONE;
                    addr_sel = 0;
                    load_ir = 0;

                    reset_pc = 0;
                    load_addr = 0;
                    loada = 1'b0;
                    loadb = 1'b0;
                    loadc = 1'b0;
                    loads = 1'b0;
                    write = 1'b0;
                    asel = 1'b0;
                    bsel = 1'b0;
                    vsel = 4'b0;
                    nsel = 3'b0;
                end
            `S1:begin
                    asel = 1'b0;
                    bsel = 1'b0;
                    vsel = 4'b0;
                    nsel = 3'b0;
                    loada = 1'b0;
                    loadb = 1'b0;
                    loadc = 1'b0;
                    loads = 1'b0;
                    write = 1'b0;
                    
                    load_pc = 0;
                    reset_pc = 0;
                    mem_cmd = `MNONE;
                    addr_sel = 0;
                    load_ir = 0;
                    load_addr = 0;
                end
            `S2:begin
                    vsel = 4'b0010;
                    nsel = 3'b001;
                    write = 1'b1;

                    asel = 1'b0;
                    bsel = 1'b0;
                    loada = 1'b0;
                    loadb = 1'b0;
                    loadc = 1'b0;
                    loads = 1'b0;

                    load_pc = 0;
                    reset_pc = 0;
                    mem_cmd = `MNONE;
                    addr_sel = 0;
                    load_ir = 0;
                    load_addr = 0;
                end
            `S3:begin
                    nsel = 3'b100;
                    write = 1'b0;
                    loadb = 1'b1;

                    asel = 1'b0;
                    bsel = 1'b0;
                    vsel = 4'b0;
                    loada = 1'b0;
                    loadc = 1'b0;
                    loads = 1'b0;

                    load_pc = 0;
                    reset_pc = 0;
                    mem_cmd = `MNONE;
                    addr_sel = 0;
                    load_ir = 0;
                    load_addr = 0;
                end
            `S4:begin
                    nsel = 3'b001;
                    write = 1'b0;
                    loada = 1'b1;

                    asel = 1'b0;
                    bsel = 1'b0;
                    vsel = 4'b0;
                    loadb = 1'b0;
                    loadc = 1'b0;
                    loads = 1'b0;

                    load_pc = 0;
                    reset_pc = 0;
                    mem_cmd = `MNONE;
                    addr_sel = 0;
                    load_ir = 0;
                    load_addr = 0;
                end
            `S5:begin
                    nsel = 3'b100;
                    write = 1'b0;
                    loadb = 1'b1;

                    asel = 1'b0;
                    bsel = 1'b0;
                    vsel = 4'b0;
                    loada = 1'b0;
                    loadc = 1'b0;
                    loads = 1'b0;

                    load_pc = 0;
                    reset_pc = 0;
                    mem_cmd = `MNONE;
                    addr_sel = 0;
                    load_ir = 0;
                    load_addr = 0;
                end
            `S6:begin
                    asel = 1'b1;
                    bsel = 1'b0;
                    loadb = 1'b0;
                    loadc = 1'b1;
                    loads = 1'b1;

                    vsel = 4'b0;
                    nsel = 3'b0;
                    loada = 1'b0;
                    write = 1'b0;

                    load_pc = 0;
                    reset_pc = 0;
                    mem_cmd = `MNONE;
                    addr_sel = 0;
                    load_ir = 0;
                    load_addr = 0;
                end
            `S7:begin
                    nsel = 3'b100;
                    write = 1'b0;
                    loada = 1'b0;
                    loadb = 1'b1;

                    asel = 1'b0;
                    bsel = 1'b0;
                    vsel = 4'b0;
                    loadc = 1'b0;
                    loads = 1'b0;

                    load_pc = 0;
                    reset_pc = 0;
                    mem_cmd = `MNONE;
                    addr_sel = 0;
                    load_ir = 0;
                    load_addr = 0;
                end
            `S8:begin
                    asel = 1'b0;
                    bsel = 1'b0;
                    loadb = 1'b0;
                    loadc = 1'b1;
                    loads = 1'b1;

                    vsel = 4'b0;
                    nsel = 3'b0;
                    loada = 1'b0;
                    write = 1'b0;

                    load_pc = 0;
                    reset_pc = 0;
                    mem_cmd = `MNONE;
                    addr_sel = 0;
                    load_ir = 0;
                    load_addr = 0;
                end
            `S9:begin
                    asel = 1'b0;
                    bsel = 1'b0;
                    loadb = 1'b0;
                    loadc = 1'b1;
                    loads = 1'b1;

                    vsel = 4'b0;
                    nsel = 3'b0;
                    loada = 1'b0;
                    write = 1'b0;

                    load_pc = 0;
                    reset_pc = 0;
                    mem_cmd = `MNONE;
                    addr_sel = 0;
                    load_ir = 0;
                    load_addr = 0;
                end
            `S10:begin
                    asel = 1'b0;
                    bsel = 1'b0;
                    loadb = 1'b0;
                    loadc = 1'b0;
                    loads = 1'b1;

                    vsel = 4'b0;
                    nsel = 3'b0;
                    loada = 1'b0;
                    write = 1'b0;

                    load_pc = 0;
                    reset_pc = 0;
                    mem_cmd = `MNONE;
                    addr_sel = 0;
                    load_ir = 0;
                    load_addr = 0;
                end
            `S11:begin
                    asel = 1'b0;
                    bsel = 1'b0;
                    loadb = 1'b0;
                    loadc = 1'b1;
                    loads = 1'b1;

                    vsel = 4'b0;
                    nsel = 3'b0;
                    loada = 1'b0;
                    write = 1'b0;

                    load_pc = 0;
                    reset_pc = 0;
                    mem_cmd = `MNONE;
                    addr_sel = 0;
                    load_ir = 0;
                    load_addr = 0;
                end
            `S12:begin
                    vsel = 4'b1000;
                    nsel = 3'b010;
                    write = 1'b1;

                    asel = 1'b0;
                    bsel = 1'b0;
                    loada = 1'b0;
                    loadb = 1'b0;
                    loadc = 1'b0;
                    loads = 1'b0;

                    load_pc = 0;
                    reset_pc = 0;
                    mem_cmd = `MNONE;
                    addr_sel = 0;
                    load_ir = 0;
                    load_addr = 0;
                end
            `S13:begin
                    nsel = 3'b001;
                    write = 1'b0;
                    loada = 1'b1;

                    asel = 1'b0;
                    bsel = 1'b0;
                    vsel = 4'b0;
                    loadb = 1'b0;
                    loadc = 1'b0;
                    loads = 1'b0;
                    
                    load_pc = 0;
                    reset_pc = 0;
                    mem_cmd = `MNONE;
                    addr_sel = 0;
                    load_ir = 0;
                    load_addr = 0;
                end
            `S14:begin
                    asel = 1'b0;
                    bsel = 1'b1;
                    loadc = 1'b1;
                    loads = 1'b1;
                    loada = 1'b0;

                    nsel = 3'b000;
                    write = 1'b0;
                    vsel = 4'b0;
                    loadb = 1'b0;
                    load_pc = 0;
                    reset_pc = 0;
                    mem_cmd = `MNONE;
                    addr_sel = 0;
                    load_ir = 0;
                    load_addr = 0;
                end
            `S15:begin
                    load_addr = 1;
                    mem_cmd = `MREAD;

                    asel = 1'b0;
                    bsel = 1'b0;
                    vsel = 4'b0;
                    nsel = 3'b0;
                    loada = 1'b0;
                    loadb = 1'b0;
                    loadc = 1'b0;
                    loads = 1'b0;
                    write = 1'b0;
                    
                    load_pc = 0;
                    reset_pc = 0;
                    addr_sel = 0;
                    load_ir = 0;
                end
            `S_wait_LDR:begin
                    load_addr = 0;
                    mem_cmd = `MREAD;

                    asel = 1'b0;
                    bsel = 1'b0;
                    vsel = 4'b0;
                    nsel = 3'b0;
                    loada = 1'b0;
                    loadb = 1'b0;
                    loadc = 1'b0;
                    loads = 1'b0;
                    write = 1'b0;
                    
                    load_pc = 0;
                    reset_pc = 0;
                    addr_sel = 0;
                    load_ir = 0;
                end
            `S16:begin
                    nsel = 3'b010;
                    vsel = 4'b0001;
                    write = 1;
                    mem_cmd = `MREAD;

                    asel = 1'b0;
                    bsel = 1'b0;
                    loada = 1'b0;
                    loadb = 1'b0;
                    loadc = 1'b0;
                    loads = 1'b0;
                    
                    load_addr = 0;
                    load_pc = 0;
                    reset_pc = 0;
                    addr_sel = 0;
                    load_ir = 0;
                end
            `S17:begin
                    nsel = 3'b001;
                    write = 1'b0;
                    loada = 1'b1;

                    asel = 1'b0;
                    bsel = 1'b0;
                    vsel = 4'b0;
                    loadb = 1'b0;
                    loadc = 1'b0;
                    loads = 1'b0;
                    
                    load_pc = 0;
                    reset_pc = 0;
                    mem_cmd = `MNONE;
                    addr_sel = 0;
                    load_ir = 0;
                    load_addr = 0;
                end
            `S18:begin
                    asel = 1'b0;
                    bsel = 1'b1;
                    loadc = 1'b1;
                    loads = 1'b1;
                    loada = 1'b0;

                    nsel = 3'b000;
                    write = 1'b0;
                    vsel = 4'b0;
                    loadb = 1'b0;
                    load_pc = 0;
                    reset_pc = 0;
                    mem_cmd = `MNONE;
                    addr_sel = 0;
                    load_ir = 0;
                    load_addr = 0;
                end
            `S19:begin
                    load_addr = 1;
                    nsel = 3'b010;
                    vsel = 4'b0001;
                    write = 0;
                    loadb = 1;

                    asel = 1'b0;
                    bsel = 1'b0;
                    loada = 1'b0;
                    loadc = 1'b0;
                    loads = 1'b0;
                    
                    load_pc = 0;
                    reset_pc = 0;
                    mem_cmd = `MNONE;
                    addr_sel = 0;
                    load_ir = 0;
                end
            `S20:begin
                    asel = 1;
                    bsel = 0;
                    loadc = 1;
                    loads = 1;
                    loadb = 0;
                    load_addr = 0;
                    mem_cmd = `MNONE;

                    vsel = 4'b0;
                    nsel = 3'b0;
                    loada = 1'b0;
                    write = 1'b0;
                    load_pc = 0;
                    reset_pc = 0;
                    addr_sel = 0;
                    load_ir = 0;
                end
            `S_wait_STR:begin
                    
                    mem_cmd = `MWRITE;

                    asel = 1'b0;
                    bsel = 1'b0;
                    vsel = 4'b0;
                    nsel = 3'b0;
                    loada = 1'b0;
                    loadb = 1'b0;
                    loadc = 1'b0;
                    loads = 1'b0;
                    write = 1'b0;

                    load_addr = 0;
                    load_pc = 0;
                    reset_pc = 0;
                    addr_sel = 0;
                    load_ir = 0;
                end
            `S21:begin
                    load_pc = 0;

                    asel = 1'b0;
                    bsel = 1'b0;
                    vsel = 4'b0;
                    nsel = 3'b0;
                    loada = 1'b0;
                    loadb = 1'b0;
                    loadc = 1'b0;
                    loads = 1'b0;
                    write = 1'b0;
                    
                    reset_pc = 0;
                    mem_cmd = `MNONE;
                    addr_sel = 0;
                    load_ir = 0;
                    load_addr = 0;
                end
            default:begin
                        nsel = 3'bx;
                        vsel = 4'bx;
                        loada = 1'bx;
                        loadb = 1'bx;
                        asel = 1'bx;
                        bsel = 1'bx;
                        loadc = 1'bx;
                        loads = 1'bx;
                        write = 1'bx;
                    
                        load_pc = 1'bx;
                        reset_pc = 1'bx;
                        mem_cmd = 2'bxx;
                        addr_sel = 1'bx;
                        load_ir = 1'bx;
                        load_addr = 1'bx;
                    end
        endcase
    end
endmodule