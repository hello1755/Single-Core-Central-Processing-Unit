
module CPU(

				clk,
			  rst_n,
  
		   IO_stall,

         awid_m_inf,
       awaddr_m_inf,
       awsize_m_inf,
      awburst_m_inf,
        awlen_m_inf,
      awvalid_m_inf,
      awready_m_inf,
                    
        wdata_m_inf,
        wlast_m_inf,
       wvalid_m_inf,
       wready_m_inf,
                    
          bid_m_inf,
        bresp_m_inf,
       bvalid_m_inf,
       bready_m_inf,
                    
         arid_m_inf,
       araddr_m_inf,
        arlen_m_inf,
       arsize_m_inf,
      arburst_m_inf,
      arvalid_m_inf,
                    
      arready_m_inf, 
          rid_m_inf,
        rdata_m_inf,
        rresp_m_inf,
        rlast_m_inf,
       rvalid_m_inf,
       rready_m_inf 

);

input  wire clk, rst_n;
output reg  IO_stall;

parameter ID_WIDTH = 4 , ADDR_WIDTH = 32, DATA_WIDTH = 16, DRAM_NUMBER=2, WRIT_NUMBER=1;
// axi write address channel 
output  wire [WRIT_NUMBER * ID_WIDTH-1:0]        awid_m_inf;
output  reg [WRIT_NUMBER * ADDR_WIDTH-1:0]    awaddr_m_inf;
output  wire [WRIT_NUMBER * 3 -1:0]            awsize_m_inf;
output  wire [WRIT_NUMBER * 2 -1:0]           awburst_m_inf;
output  wire [WRIT_NUMBER * 7 -1:0]             awlen_m_inf;
output  reg [WRIT_NUMBER-1:0]                awvalid_m_inf;
input   wire [WRIT_NUMBER-1:0]                awready_m_inf;
// axi write data channel 
output  wire [WRIT_NUMBER * DATA_WIDTH-1:0]     wdata_m_inf;
output  wire [WRIT_NUMBER-1:0]                  wlast_m_inf;
output  reg [WRIT_NUMBER-1:0]                 wvalid_m_inf;
input   wire [WRIT_NUMBER-1:0]                 wready_m_inf;
// axi write response channel
input   wire [WRIT_NUMBER * ID_WIDTH-1:0]         bid_m_inf;
input   wire [WRIT_NUMBER * 2 -1:0]             bresp_m_inf;
input   wire [WRIT_NUMBER-1:0]             	   bvalid_m_inf;
output  reg [WRIT_NUMBER-1:0]                 bready_m_inf;
// -----------------------------
// axi read address channel 
output  wire [DRAM_NUMBER * ID_WIDTH-1:0]       arid_m_inf;
output  wire [DRAM_NUMBER * ADDR_WIDTH-1:0]   araddr_m_inf;
output  wire [DRAM_NUMBER * 7 -1:0]            arlen_m_inf;
output  wire [DRAM_NUMBER * 3 -1:0]           arsize_m_inf;
output  wire [DRAM_NUMBER * 2 -1:0]          arburst_m_inf;
output  wire [DRAM_NUMBER-1:0]               arvalid_m_inf;
input   wire [DRAM_NUMBER-1:0]               arready_m_inf;
// -----------------------------
// axi read data channel 
input   wire [DRAM_NUMBER * ID_WIDTH-1:0]         rid_m_inf;
input   wire [DRAM_NUMBER * DATA_WIDTH-1:0]     rdata_m_inf;
input   wire [DRAM_NUMBER * 2 -1:0]             rresp_m_inf;
input   wire [DRAM_NUMBER-1:0]                  rlast_m_inf;
input   wire [DRAM_NUMBER-1:0]                 rvalid_m_inf;
output  wire [DRAM_NUMBER-1:0]                 rready_m_inf;
// -----------------------------

// registers of CPU
reg signed [15:0] core_r0 , core_r1 , core_r2 , core_r3 ;
reg signed [15:0] core_r4 , core_r5 , core_r6 , core_r7 ;
reg signed [15:0] core_r8 , core_r9 , core_r10, core_r11;
reg signed [15:0] core_r12, core_r13, core_r14, core_r15;


//####################################################
//               instruction information
//####################################################

wire [15:0] inst, data_from_memory;
wire func              = inst[0];
wire [3:0] rs_name     = inst[12:9];
wire [3:0] rt_name     = inst[8:5];
wire [3:0] rd_name     = inst[4:1];
wire [2:0] op_code     = inst[15:13];
wire signed [4:0] imme  = inst[4:0];
wire [3:0] coeff_a     = inst[12:9];
wire [8:0] coeff_b     = inst[8:0];


//####################################################
//               control signal
//####################################################
wire inst_end;          
wire r_data_end;     
reg w_data_end;    
wire det_ready;    

reg signed [11:0] PC;
reg signed [15:0] rs_data, rt_data, rd_data;
wire signed [11:0] data_addr;


parameter IDLE = 0, IF = 1, ID = 2, EXE = 3, LOAD = 4, STORE = 5;
reg [2:0] c_state, n_state;
always @(posedge clk, negedge rst_n) begin
    if (!rst_n) c_state <= IDLE;
    else        c_state <= n_state;
end

always @* begin
    case (c_state) 
        IDLE:                           n_state = IF;
        IF:begin
            if (inst_end)               n_state = ID;
            else                        n_state = IF;
        end
        ID:                             n_state = EXE;
        EXE:begin
            case (op_code)
                3'b000, 3'b001, 3'b100: n_state = IF;
                3'b010:                 n_state = LOAD;
                3'b011:                 n_state = STORE;
                3'b111: begin
                    if(det_ready)       n_state = IF;
                    else                n_state = EXE;
                end
                default:                n_state = IF;
            endcase
        end
        LOAD: begin
            if(r_data_end)              n_state = IF;
            else                        n_state = LOAD;
        end
        STORE: begin
            if(w_data_end)              n_state = IF;
            else                        n_state = STORE;
        end
        default:                        n_state = IDLE;
    endcase
end

wire EXE_reg = (c_state == EXE);
wire IF_reg = (n_state == IF);
wire WB_reg = (IF_reg && EXE_reg && (op_code == 3'b000 || op_code == 3'b001));
wire IF_no_reg = (c_state != IF);
wire STOR_reg = (n_state == STORE);
wire LOAD_reg = (n_state == LOAD);
always @(posedge clk, negedge rst_n) begin
    if (!rst_n) rs_data <= 0 ;
    else begin
        if (n_state == ID) begin
            case(rs_name)
                0:  rs_data <= core_r0;  1:  rs_data <= core_r1;  2:  rs_data <= core_r2;  3:  rs_data <= core_r3; 
                4:  rs_data <= core_r4;  5:  rs_data <= core_r5;  6:  rs_data <= core_r6;  7:  rs_data <= core_r7; 
                8:  rs_data <= core_r8;  9:  rs_data <= core_r9;  10: rs_data <= core_r10; 11: rs_data <= core_r11; 
                12: rs_data <= core_r12; 13: rs_data <= core_r13; 14: rs_data <= core_r14; 15: rs_data <= core_r15; 
            endcase
        end
    end
end

always @(posedge clk, negedge rst_n) begin
    if (!rst_n) rt_data <= 0 ;
    else begin
        if (n_state == ID) begin
            case(rt_name)
                0:  rt_data <= core_r0;  1:  rt_data <= core_r1;  2:  rt_data <= core_r2;  3:  rt_data <= core_r3;
                4:  rt_data <= core_r4;  5:  rt_data <= core_r5;  6:  rt_data <= core_r6;  7:  rt_data <= core_r7;
                8:  rt_data <= core_r8;  9:  rt_data <= core_r9;  10: rt_data <= core_r10; 11: rt_data <= core_r11; 
                12: rt_data <= core_r12; 13: rt_data <= core_r13; 14: rt_data <= core_r14; 15: rt_data <= core_r15; 
            endcase
        end 
    end
end

reg [4:0] det_cnt;
always @(posedge clk, negedge rst_n) begin
    if (!rst_n) PC <= 0;
    else begin
        if(EXE_reg) begin
            if(op_code == 3'b100) begin
                if (rs_data == rt_data) PC <= PC + 2 + (imme << 1);
                else                    PC <= PC + 2;
            end
            else if(&op_code)begin
                if(det_cnt == 10)       PC <= PC + 2;
                else                    PC <= PC;
            end
            else PC <= PC + 2;
        end
    end
end


always @* begin
    case ({op_code[0], func})
        2'b00: rd_data <= rs_data + rt_data; 
        2'b01: rd_data <= rs_data - rt_data; 
        2'b10: rd_data <= rs_data < rt_data; 
        2'b11: rd_data <= rs_data * rt_data;
    endcase
end


////////   det ///////////////////////////////////

always @(posedge clk, negedge rst_n)begin 
    if(!rst_n)  det_cnt <= 0;
    else begin
        if(EXE_reg && &op_code) det_cnt <= det_cnt + 1;
        else                    det_cnt <= 0;
    end
end



reg signed [32:0] a1, a2;
reg signed [50:0] reg1, reg2, reg3, reg4;
reg signed [15:0] mult_1_1, mult_1_2, mult_1_3, mult_1_4, mult_1_5, mult_1_6;
reg signed [15:0] mult_2_1, mult_2_2, mult_2_3, mult_2_4, mult_2_5, mult_2_6;

always @(posedge clk, negedge rst_n)begin
    if(!rst_n) begin
        mult_1_1 <= 0;
        mult_1_2 <= 0;
        mult_1_3 <= 0;
        mult_1_4 <= 0;
        mult_1_5 <= 0;
        mult_1_6 <= 0;
    end
    else begin
        case(det_cnt)
            0: begin
                mult_1_1 <= core_r10;
                mult_1_2 <= core_r15;
                mult_1_3 <= core_r11;
                mult_1_4 <= core_r14;
                mult_1_5 <= 0;
                mult_1_6 <= 0;
            end
            1: begin
                mult_1_1 <= core_r11;
                mult_1_2 <= core_r13;
                mult_1_3 <= core_r9;
                mult_1_4 <= core_r15;
                mult_1_5 <= core_r5;
                mult_1_6 <= core_r1;
            end
            2: begin
                mult_1_1 <= core_r9;
                mult_1_2 <= core_r14;
                mult_1_3 <= core_r10;
                mult_1_4 <= core_r13;
                mult_1_5 <= core_r6;
                mult_1_6 <= core_r2;
            end
            3: begin
                mult_1_1 <= 0;
                mult_1_2 <= 0;
                mult_1_3 <= 0;
                mult_1_4 <= 0;
                mult_1_5 <= core_r7;
                mult_1_6 <= core_r3;
            end
            default: begin
                mult_1_1 <= 0;
                mult_1_2 <= 0;
                mult_1_3 <= 0;
                mult_1_4 <= 0;
                mult_1_5 <= 0;
            end
        endcase
    end
end

reg signed [32:0] tempp1, tempp2;
always @(posedge clk, negedge rst_n) begin
    if(!rst_n) a1 <= 0;
    else       a1 <= mult_1_1 * mult_1_2 - mult_1_3 * mult_1_4;
end

reg signed [48:0] temp1;
always @(posedge clk) begin
    temp1 <= a1 * mult_1_5;
end

always @(posedge clk, negedge rst_n) begin
    if(!rst_n) reg1 <= 0;
    else begin
        if(c_state == EXE) reg1 <= temp1 + reg1;
        else               reg1 <= 0;
    end       
end

reg signed [48:0] temp2;
always @(posedge clk) begin
    temp2 <= a1 * mult_1_6;
end

always @(posedge clk, negedge rst_n) begin
    if(!rst_n) reg2 <= 0;
    else begin
        if(EXE_reg) reg2 <= temp2 + reg2;
        else        reg2 <= 0;
    end   
end



always @(posedge clk, negedge rst_n)begin
    if(!rst_n)begin
        mult_2_1 <= 0;
        mult_2_2 <= 0;
        mult_2_3 <= 0;
        mult_2_4 <= 0;
        mult_2_5 <= 0;
        mult_2_6 <= 0;
    end
    else begin
        case(det_cnt)
            0: begin
                mult_2_1 <= core_r1;
                mult_2_2 <= core_r6;
                mult_2_3 <= core_r2;
                mult_2_4 <= core_r5;
                mult_2_5 <= 0;
                mult_2_6 <= 0;
            end
            1: begin
                mult_2_1 <= core_r2;
                mult_2_2 <= core_r7;
                mult_2_3 <= core_r3;
                mult_2_4 <= core_r6;
                mult_2_5 <= core_r15;
                mult_2_6 <= core_r11;
            end
            2: begin
                mult_2_1 <= core_r3;
                mult_2_2 <= core_r5;
                mult_2_3 <= core_r1;
                mult_2_4 <= core_r7;
                mult_2_5 <= core_r13;
                mult_2_6 <= core_r9;
            end
            3: begin
                mult_2_1 <= 0;
                mult_2_2 <= 0;
                mult_2_3 <= 0;
                mult_2_4 <= 0;
                mult_2_5 <= core_r14;
                mult_2_6 <= core_r10;
            end
            default: begin
                mult_2_1 <= 0;
                mult_2_2 <= 0;
                mult_2_3 <= 0;
                mult_2_4 <= 0;
                mult_2_5 <= 0;
                mult_2_6 <= 0;
            end
        endcase
    end
end

always @(posedge clk, negedge rst_n) begin
    if(!rst_n) a2 <= 0;
    else       a2 <= mult_2_1 * mult_2_2 - mult_2_3 * mult_2_4;
end

reg signed [48:0] temp3;
always @(posedge clk) begin
    temp3 <= a2 * mult_2_5;
end

always @(posedge clk, negedge rst_n) begin
    if(!rst_n) reg3 <= 0;
    else begin
        if(EXE_reg) reg3 <= a2 * mult_2_5 + reg3;
        else        reg3 <= 0;
    end
end

reg signed [48:0] temp4;
always @(posedge clk) begin
    temp4 <= a2 * mult_2_6;
end


always @(posedge clk, negedge rst_n) begin
    if(!rst_n) reg4 <= 0;
    else begin
        if(EXE_reg) reg4 <= temp4 + reg4;
        else        reg4 <= 0;
    end
end

reg signed [15:0] temp_mult1;
reg signed [50:0] temp_mult2;

always @(posedge clk, negedge rst_n)begin
    if(!rst_n)begin
        temp_mult1 <= 0;
        temp_mult2 <= 0;
    end
    else begin
        case(det_cnt)
            6: begin
                temp_mult1 <= core_r0;
                temp_mult2 <= reg1;
            end
            7: begin
                temp_mult1 <= core_r4;
                temp_mult2 <= reg2;
            end
            8: begin
                temp_mult1 <= core_r8;
                temp_mult2 <= reg3;
            end
            9: begin
                temp_mult1 <= core_r12;
                temp_mult2 <= reg4;
            end
            default: begin
                temp_mult1 <= 0;
                temp_mult2 <= 0;
            end
        endcase
    end
end


reg signed [66:0] temp_final, final1, final2, final3, final4;
reg signed [68:0] final_det;
always @(posedge clk) temp_final <= temp_mult1 * temp_mult2;

/*
always @(posedge clk) begin
    case(det_cnt)
        8: final1 <= temp_final;
        9: final2 <= temp_final;
        10: final3 <= temp_final;
    endcase
end


always @(posedge clk, negedge rst_n)begin
    if(!rst_n) final_det <= 0;
    else begin
        if(det_cnt == 11) final_det <= final1 - final2 + final3 - temp_final;
        else              final_det <= final_det;
    end
end
*/
always @(posedge clk, negedge rst_n) begin
    if(!rst_n) final_det <= 0;
    else begin
        if(c_state == EXE)begin
            case(det_cnt)
                8, 10: final_det <= final_det + temp_final;
                9, 11: final_det <= final_det - temp_final;
            endcase
        end
        else final_det <= 0;
    end
    
end


wire signed [9:0] coeff_b_signed = coeff_b;
wire signed [68:0] det_cal_tem = final_det >>> {coeff_a, 1'b0};
wire signed [68:0] det_cal = det_cal_tem + coeff_b_signed;

assign det_ready = (det_cnt == 12);

/////////////////////  end of det //////////////////////////////////////////
always @(posedge clk, negedge rst_n) begin
    if(!rst_n)                                     core_r0 <= 0;
    else begin 
        if(r_data_end && rt_name == 0)             core_r0 <= data_from_memory;
        else if(WB_reg && rd_name == 0)            core_r0 <= rd_data;
        else if(det_ready)begin
            if(det_cal > 32767)                    core_r0 <= 32767;
            else if(det_cal < -32768)              core_r0 <= -32768;
            else                                   core_r0 <= det_cal;
        end  
    end
end

always @(posedge clk, negedge rst_n) begin
    if(!rst_n)                                     core_r1 <= 0;
    else begin
        if(r_data_end && rt_name == 1)             core_r1 <= data_from_memory;
        else if(WB_reg && rd_name == 1)            core_r1 <= rd_data;
    end
end

always @(posedge clk, negedge rst_n) begin
    if(!rst_n)                                     core_r2 <= 0;
    else begin
        if(r_data_end && rt_name == 2)             core_r2 <= data_from_memory;
        else if(WB_reg && rd_name == 2)            core_r2 <= rd_data;
    end
end

always @(posedge clk, negedge rst_n) begin
    if(!rst_n)                                     core_r3 <= 0;
    else begin
        if(r_data_end && rt_name == 3)             core_r3 <= data_from_memory;
        else if(WB_reg && rd_name == 3)            core_r3 <= rd_data;
    end
end

always @(posedge clk, negedge rst_n) begin
    if(!rst_n)                                     core_r4 <= 0;
    else begin
        if(r_data_end && rt_name == 4)             core_r4 <= data_from_memory;
        else if(WB_reg && rd_name == 4)            core_r4 <= rd_data;
    end
end

always @(posedge clk, negedge rst_n) begin
    if(!rst_n)                                     core_r5 <= 0;
    else begin
        if(r_data_end && rt_name == 5)             core_r5 <= data_from_memory;
        else if(WB_reg && rd_name == 5)            core_r5 <= rd_data;
    end
end

always @(posedge clk, negedge rst_n) begin
    if(!rst_n)                                      core_r6 <= 0;
    else begin
        if(r_data_end && rt_name == 6)              core_r6 <= data_from_memory;
        else if(WB_reg && rd_name == 6)             core_r6 <= rd_data;
        else                                        core_r6 <= core_r6;
    end
end

always @(posedge clk, negedge rst_n) begin
    if(!rst_n)                                      core_r7 <= 0;
    else begin
        if(r_data_end && rt_name == 7)              core_r7 <= data_from_memory;
        else if(WB_reg && rd_name == 7)             core_r7 <= rd_data;
        else                                        core_r7 <= core_r7;
    end
end

always @(posedge clk, negedge rst_n) begin
    if(!rst_n)                                      core_r8 <= 0;
    else begin
        if(r_data_end && rt_name == 8)              core_r8 <= data_from_memory;
        else if(WB_reg && rd_name == 8)             core_r8 <= rd_data;
        else                                        core_r8 <= core_r8;
    end
end

always @(posedge clk, negedge rst_n) begin
    if(!rst_n)                                      core_r9 <= 0;
    else begin
        if(r_data_end && rt_name == 9)              core_r9 <= data_from_memory;
        else if(WB_reg && rd_name == 9)             core_r9 <= rd_data;
        else                                        core_r9 <= core_r9;
    end
end

always @(posedge clk, negedge rst_n) begin
    if(!rst_n)                                      core_r10 <= 0;
    else begin
        if(r_data_end && rt_name == 10)             core_r10 <= data_from_memory;
        else if(WB_reg && rd_name == 10)            core_r10 <= rd_data;
        else                                        core_r10 <= core_r10;
    end
end

always @(posedge clk, negedge rst_n) begin
    if(!rst_n)                                      core_r11 <= 0;
    else begin
        if(r_data_end && rt_name == 11)             core_r11 <= data_from_memory;
        else if(WB_reg && rd_name == 11)            core_r11 <= rd_data;
        else                                        core_r11 <= core_r11;
    end
end

always @(posedge clk, negedge rst_n) begin
    if(!rst_n)                                      core_r12 <= 0;
    else begin
        if(r_data_end && rt_name == 12)             core_r12 <= data_from_memory;
        else if(WB_reg && rd_name == 12)            core_r12 <= rd_data;
        else                                        core_r12 <= core_r12;
    end
end

always @(posedge clk, negedge rst_n) begin
    if(!rst_n)                                      core_r13 <= 0;
    else begin
        if(r_data_end && rt_name == 13)             core_r13 <= data_from_memory;
        else if(WB_reg && rd_name == 13)            core_r13 <= rd_data;
        else                                        core_r13 <= core_r13;
    end
end

always @(posedge clk, negedge rst_n) begin
    if(!rst_n)                                      core_r14 <= 0;
    else begin
        if(r_data_end && rt_name == 14)             core_r14 <= data_from_memory;
        else if(WB_reg && rd_name == 14)            core_r14 <= rd_data;
        else                                        core_r14 <= core_r14;
    end
end

always @(posedge clk, negedge rst_n) begin
    if(!rst_n)                                      core_r15 <= 0;
    else begin
        if(r_data_end && rt_name == 15)             core_r15 <= data_from_memory;
        else if(WB_reg && rd_name == 15)            core_r15 <= rd_data;
        else                                        core_r15 <= core_r15;
    end
end

always @(posedge clk, negedge rst_n) begin
    if(!rst_n)                                      IO_stall <= 1;
    else begin
        if(IF_reg && IF_no_reg && c_state != IDLE)  IO_stall <= 0;
        else                                        IO_stall <= 1;
    end
end


//################################################################################################
//               instruction read
//################################################################################################
reg  inst_start;
always @(posedge clk, negedge rst_n) begin
    if(!rst_n) inst_start <= 1'b0;
    else       inst_start <= (IF_no_reg && IF_reg);
end


INST_READ IR(
    // IO
    .clk(clk), .rst_n(rst_n), .inst_start(inst_start), .IR_READ_addr(PC[11:1]), .inst_end(inst_end), .r_inst(inst),

    .arid_m_inf(arid_m_inf[7:4]), .araddr_m_inf(araddr_m_inf[63:32]), .arlen_m_inf(arlen_m_inf[13:7]), 
    .arsize_m_inf(arsize_m_inf[5:3]), .arburst_m_inf(arburst_m_inf[3:2]), .arvalid_m_inf(arvalid_m_inf[1]),  
    .arready_m_inf(arready_m_inf[1]), .rid_m_inf(rid_m_inf[7:4]), .rdata_m_inf(rdata_m_inf[31:16]), .rresp_m_inf(rresp_m_inf[3:2]), 
    .rlast_m_inf(rlast_m_inf[1]), .rvalid_m_inf(rvalid_m_inf[1]), .rready_m_inf(rready_m_inf[1])
);

//################################################################################################
//               data write
//################################################################################################
reg  w_data_start;
always @(posedge clk, negedge rst_n) begin
    if(!rst_n) w_data_start <= 0;
    else       w_data_start <= (EXE_reg && STOR_reg);
end

wire [10:0] addr = data_addr[11:1];
assign awid_m_inf = 0;
assign awlen_m_inf = 0;
assign awsize_m_inf = 1;
assign awburst_m_inf = 1;

assign wdata_m_inf = rt_data;
always @(posedge clk) awaddr_m_inf <= {20'b00000000000000000001, addr, 1'b0};

//==============================================//
//                     FSM                      //
//==============================================//
reg [1:0] w_c_state,w_n_state;
parameter W_IDLE = 0, WAIT_AWREADY = 1, WAIT_WREADY = 2, WAIT_BRESP = 3; 

always @(posedge clk, negedge rst_n) begin
    if(!rst_n) w_c_state <= W_IDLE;
    else       w_c_state <= w_n_state;
end

always @* begin
  	case (w_c_state)
  		W_IDLE: begin
            if(w_data_start)   w_n_state = WAIT_AWREADY;
    		else               w_n_state = W_IDLE;
        end
  		WAIT_AWREADY: begin
            if(awready_m_inf)  w_n_state = WAIT_WREADY;
            else               w_n_state = WAIT_AWREADY;
  		end
  		WAIT_WREADY: begin
            if(wready_m_inf)   w_n_state = WAIT_BRESP;
            else               w_n_state = WAIT_WREADY;
  		end
        WAIT_BRESP: begin
            if(bvalid_m_inf)   w_n_state = W_IDLE;
            else               w_n_state = WAIT_BRESP;
        end
  	endcase
end


wire W_IDLE_reg = (w_n_state == W_IDLE);
wire WAIT_BRESP_reg = (w_c_state == WAIT_BRESP);
wire WAIT_WREADY_reg = (w_c_state == WAIT_WREADY);
always @(posedge clk, negedge rst_n) begin
    if(!rst_n) awvalid_m_inf <= 1'b0;
   else begin
        case(w_c_state) 
            WAIT_AWREADY: begin
                if(!awready_m_inf) awvalid_m_inf <= 1;
                else               awvalid_m_inf <= 0;
            end
        endcase
    end
end


always @(posedge clk, negedge rst_n) begin
    if (!rst_n) bready_m_inf <= 0;
    else begin
        if(WAIT_BRESP_reg) begin
            if(!bvalid_m_inf) bready_m_inf <= 1;
            else              bready_m_inf <= 0;
        end
        else                  bready_m_inf <= 0;
    end  
end

always @(posedge clk, negedge rst_n) begin
    if (!rst_n) wvalid_m_inf <= 0;
    else begin
        if(WAIT_WREADY_reg)  wvalid_m_inf <= 1;
        else                 wvalid_m_inf <= 0;
    end  
end

assign wlast_m_inf = wvalid_m_inf;


always @(posedge clk, negedge rst_n) begin
    if (!rst_n) w_data_end <= 0;
    else begin
        if (WAIT_BRESP_reg && W_IDLE_reg) w_data_end <= 1;
        else                            w_data_end <= 0;
    end
end


//################################################################################################
//               data read
//################################################################################################
reg  r_data_start; 
always @(posedge clk, negedge rst_n) begin
    if (!rst_n) r_data_start <= 0;
    else        r_data_start <= (EXE_reg && (LOAD_reg || STOR_reg));
end

assign data_addr = ((rs_data + imme) << 1);
wire write_back_DRAM = (c_state == STORE);

DATA_READ DR(
    // IO
    .clk(clk), .rst_n(rst_n), .r_data_start(r_data_start), .data_addr(data_addr[11:1]), 
    .rt_data(rt_data), .write_back_DRAM(write_back_DRAM),
    .r_data_end(r_data_end), .r_data(data_from_memory),
    
    .arid_m_inf(arid_m_inf[3:0]), .araddr_m_inf(araddr_m_inf[31:0]), .arlen_m_inf(arlen_m_inf[6:0]),
    .arsize_m_inf(arsize_m_inf[2:0]), .arburst_m_inf(arburst_m_inf[1:0]), .arvalid_m_inf(arvalid_m_inf[0]),  
    .arready_m_inf(arready_m_inf[0]), .rid_m_inf(rid_m_inf[3:0]), .rdata_m_inf(rdata_m_inf[15:0]), .rresp_m_inf(rresp_m_inf[1:0]),
    .rlast_m_inf(rlast_m_inf[0]), .rvalid_m_inf(rvalid_m_inf[0]), .rready_m_inf(rready_m_inf[0])
);

endmodule


module INST_READ(
    // IO
    clk, rst_n, inst_end, IR_READ_addr, r_inst, inst_start,

    // DRAM 
    arid_m_inf, arvalid_m_inf, araddr_m_inf, 
    arready_m_inf, arlen_m_inf, arburst_m_inf, 
    arsize_m_inf, rid_m_inf, rready_m_inf, 
    rvalid_m_inf, rresp_m_inf, rdata_m_inf, rlast_m_inf
);

parameter ID_WIDTH = 4 , ADDR_WIDTH = 32, DATA_WIDTH = 16;


input clk, rst_n, inst_start;
input [10:0] IR_READ_addr; 
output wire inst_end;
output reg [DATA_WIDTH-1:0] r_inst;

output wire [ID_WIDTH-1:0]      arid_m_inf;
output reg [ADDR_WIDTH-1:0]   araddr_m_inf;
output wire [6:0]              arlen_m_inf;
output wire [2:0]             arsize_m_inf;
output wire [1:0]            arburst_m_inf;
output reg                   arvalid_m_inf;
input  wire                  arready_m_inf;
input  wire [ID_WIDTH-1:0]       rid_m_inf;
input  wire [DATA_WIDTH-1:0]   rdata_m_inf;
input  wire [1:0]              rresp_m_inf;
input  wire                    rlast_m_inf;
input  wire                   rvalid_m_inf;
output reg                    rready_m_inf;


//==============================================//
//                  reg / wire                  //
//==============================================//
assign arburst_m_inf = 1;	
assign arsize_m_inf  = 1;
assign arid_m_inf    = 0; 		
assign arlen_m_inf   = 127;


reg [15:0] sram_out;
reg [6:0] sram_addr;
reg [3:0] label;

wire IN_SRAM = (label == IR_READ_addr[10:7]);
always @(posedge clk) araddr_m_inf <= {20'b00000000000000000001, IR_READ_addr[10:7], 8'b00000000};


//==============================================//
//                     FSM                      //
//==============================================//
parameter IDLE = 0, WAIT_ARREADY = 1, WAIT_RVALID = 2, WAIT_SRAM_1 = 3, WAIT_SRAM_2 = 4, OUTPUT = 5, WAIT_SRAM_3 = 6, WAIT_DRAM_1 = 7;;
reg [2:0] c_state, n_state;

always @(posedge clk, negedge rst_n) begin
    if (!rst_n) c_state <= IDLE;
    else        c_state <= n_state;
end

always @* begin
  	case(c_state)
  		IDLE: begin
            if(inst_start) begin
                if(IN_SRAM)                   n_state = WAIT_SRAM_1;
                else                          n_state = WAIT_ARREADY;
            end        
    		else                              n_state = IDLE;
        end
  		WAIT_ARREADY: begin
            if(arready_m_inf)                 n_state = WAIT_RVALID;
            else                              n_state = WAIT_ARREADY;
  		end
        WAIT_RVALID: begin
            if (rvalid_m_inf && rlast_m_inf)  n_state = WAIT_DRAM_1;
            else                              n_state = WAIT_RVALID;
  		end
        WAIT_DRAM_1:                          n_state = OUTPUT;
        WAIT_SRAM_1:                          n_state = WAIT_SRAM_2;
        WAIT_SRAM_2:                          n_state = WAIT_SRAM_3;
        WAIT_SRAM_3:                          n_state = OUTPUT;
        OUTPUT:                               n_state = IDLE;
  		default:                              n_state = IDLE;
  	endcase
end

reg [9:0] RVALID_cnt;
wire SRAM_1_reg = (n_state == WAIT_SRAM_1);
always @(posedge clk, negedge rst_n) begin
    if (!rst_n) sram_addr <= 0;
    else begin
        if(SRAM_1_reg)            sram_addr <= IR_READ_addr[6:0];
        else if(RVALID_cnt > 0)   sram_addr <= sram_addr + 1;
        else if(n_state == IDLE)  sram_addr <= 0;
        else                      sram_addr <= sram_addr;
    end
end



always @(posedge clk, negedge rst_n)begin
    if(!rst_n) RVALID_cnt <= 0 ;
    else begin
        if(c_state == IDLE)             RVALID_cnt <= 0;
        else if(rvalid_m_inf)           RVALID_cnt <= RVALID_cnt + 1;
    end
end

reg [15:0] sram_in;
always @(posedge clk, negedge rst_n)begin
    if(!rst_n) sram_in <= 0;
    else       sram_in <= rdata_m_inf;
    
end

wire WEB = !((c_state == WAIT_RVALID && RVALID_cnt > 0) || (c_state == WAIT_DRAM_1));

SRAM_128x16 INST( .A0(sram_addr[0]),   .A1(sram_addr[1]),   .A2(sram_addr[2]),   .A3(sram_addr[3]), .A4(sram_addr[4]),   
                       .A5(sram_addr[5]),   .A6(sram_addr[6]),  .DO0(sram_out[0]),   .DO1(sram_out[1]),   .DO2(sram_out[2]),   
                       .DO3(sram_out[3]), .DO4(sram_out[4]),   .DO5(sram_out[5]),   .DO6(sram_out[6]),   .DO7(sram_out[7]),
                       .DO8(sram_out[8]),   .DO9(sram_out[9]),   .DO10(sram_out[10]), .DO11(sram_out[11]), .DO12(sram_out[12]), 
                       .DO13(sram_out[13]), .DO14(sram_out[14]), .DO15(sram_out[15]),.DI0(sram_in[0]),    .DI1(sram_in[1]),   
                       .DI2(sram_in[2]),   .DI3(sram_in[3]), .DI4(sram_in[4]),    .DI5(sram_in[5]),   .DI6(sram_in[6]), 
                       .DI7(sram_in[7]),  .DI8(sram_in[8]),    .DI9(sram_in[9]),   .DI10(sram_in[10]), .DI11(sram_in[11]),
                       .DI12(sram_in[12]),  .DI13(sram_in[13]), .DI14(sram_in[14]), .DI15(sram_in[15]),
                       .CK(clk), .WEB(WEB), .OE(1'b1), .CS(1'b1));

always @(posedge clk, negedge rst_n) begin
    if (!rst_n) label <= 4'hf;
    else begin
        if(c_state == WAIT_ARREADY)  label <= IR_READ_addr[10:7];
        else                         label <= label;
    end 
end

always @(posedge clk, negedge rst_n) begin
    if (!rst_n) arvalid_m_inf <= 1'b0;
    else begin
        case (c_state) 
            WAIT_ARREADY: begin
                if(!arready_m_inf) arvalid_m_inf <= 1;
                else               arvalid_m_inf <= 0;
            end
            default:               arvalid_m_inf <= 0;
        endcase
    end
end

always @(posedge clk, negedge rst_n) begin
    if (!rst_n) rready_m_inf <= 1'b0;
    else begin
        case(c_state) 
            WAIT_RVALID: begin
                if(!rlast_m_inf) rready_m_inf <= 1;
                else             rready_m_inf <= 0;
            end
            default:             rready_m_inf <= rready_m_inf;
        endcase
    end
end


reg [15:0] sram_out_reg;
always @(posedge clk, negedge rst_n)begin
    if(!rst_n) sram_out_reg <= 0;
    else       sram_out_reg <= sram_out;
end 


wire tttt = (IR_READ_addr[6:0] == 0 && RVALID_cnt == 0);
always @(posedge clk, negedge rst_n) begin
    if(!rst_n)  r_inst <= 0;
    else begin
        if (c_state == WAIT_SRAM_3)                                                          r_inst <= sram_out_reg;
        else if (c_state == WAIT_RVALID && sram_addr == IR_READ_addr[6:0] && rvalid_m_inf) begin
            if(IR_READ_addr[6:0] == 0 && RVALID_cnt == 1) r_inst <= sram_in;
            else if(IR_READ_addr[6:0] != 0)               r_inst <= sram_in;
        end
        else                                                                                 r_inst <= r_inst;
    end
    
end

assign  inst_end = (c_state == OUTPUT);


endmodule


module DATA_READ(
    // IO
    clk, rst_n, r_data_start, data_addr, rt_data, write_back_DRAM, r_data_end, r_data,
    
    // DRAM 
    arid_m_inf, arvalid_m_inf, araddr_m_inf, arready_m_inf, arlen_m_inf, 
    arburst_m_inf, arsize_m_inf,  rid_m_inf, rready_m_inf, rvalid_m_inf, 
    rresp_m_inf, rdata_m_inf, rlast_m_inf
);

parameter ID_WIDTH = 4 , ADDR_WIDTH = 32, DATA_WIDTH = 16;


input clk, rst_n, r_data_start, write_back_DRAM;
input [10:0] data_addr; 
input [15:0] rt_data;
output reg r_data_end;
output reg [DATA_WIDTH-1:0] r_data;


// DRAM 
output wire [ID_WIDTH-1:0]      arid_m_inf;
output reg [ADDR_WIDTH-1:0]   araddr_m_inf;
output wire [6:0]              arlen_m_inf;
output wire [2:0]             arsize_m_inf;
output wire [1:0]            arburst_m_inf;
output reg                   arvalid_m_inf;
input  wire                  arready_m_inf;
input  wire [ID_WIDTH-1:0]       rid_m_inf;
input  wire [DATA_WIDTH-1:0]   rdata_m_inf;
input  wire [1:0]              rresp_m_inf;
input  wire                    rlast_m_inf;
input  wire                   rvalid_m_inf;
output reg                    rready_m_inf;




assign arid_m_inf    = 0; 	
assign arburst_m_inf = 1;	
assign arsize_m_inf  = 1;	
assign arlen_m_inf   = 127;
//==============================================//
//                     FSM                      //
//==============================================//
parameter IDLE = 0, WAIT_ARREADY = 1, WAIT_RVALID = 2, WAIT_SRAM_1 = 3, WAIT_SRAM_2 = 4, WAIT_SRAM_3 = 5, UPDATE_SRAM = 6, OUTPUT = 7, WAIT_DRAM_1 = 8;
reg [3:0] c_state, n_state;
reg [3:0] label;
wire cache_hit = (label == data_addr[10:7]);
always @(posedge clk, negedge rst_n) begin
    if (!rst_n) c_state <= IDLE;
    else        c_state <= n_state;
end

always @* begin
  	case (c_state)
  		IDLE: begin
            if (r_data_start) begin
                case({cache_hit, write_back_DRAM})
                    2'b00:                   n_state = WAIT_ARREADY; 
                    2'b01:                   n_state = IDLE; 
                    2'b10:                   n_state = WAIT_SRAM_1; 
                    2'b11:                   n_state = UPDATE_SRAM; 
                endcase
            end        
    		else                             n_state = IDLE;
        end
  		WAIT_ARREADY: begin
            if (arready_m_inf)               n_state = WAIT_RVALID;
            else                             n_state = WAIT_ARREADY;
  		end
        WAIT_RVALID: begin
            if (rvalid_m_inf && rlast_m_inf) n_state = WAIT_DRAM_1;
            else                             n_state = WAIT_RVALID;
  		end
        WAIT_DRAM_1:                         n_state = OUTPUT;
        WAIT_SRAM_1:                         n_state = WAIT_SRAM_2;
        WAIT_SRAM_2:                         n_state = WAIT_SRAM_3;
        WAIT_SRAM_3:                         n_state = OUTPUT;
        OUTPUT:                              n_state = IDLE;
        UPDATE_SRAM:                         n_state = IDLE;
  		default:                             n_state = IDLE;
  	endcase
end

//==============================================//
//                   Design                     //
//==============================================//
reg [9:0] RVALID_cnt;
always @(posedge clk, negedge rst_n)begin
    if(!rst_n) RVALID_cnt <= 0 ;
    else begin
        if(c_state == IDLE)             RVALID_cnt <= 0;
        else if(rvalid_m_inf)           RVALID_cnt <= RVALID_cnt + 1;
    end
end

reg [15:0] sram_in;
wire WEB;
reg [15:0] sram_out;
reg [6:0]  sram_addr;
always @(posedge clk, negedge rst_n) begin
    if (!rst_n)     sram_addr <= 0;
    else begin
        if (RVALID_cnt > 0)                                          sram_addr <= sram_addr + 1;
        else if (n_state == WAIT_SRAM_1 || n_state == UPDATE_SRAM)   sram_addr <= data_addr[6:0];
        else if (n_state == IDLE)                                    sram_addr <= 0;
        else                                                        sram_addr <= sram_addr;
    end
end

assign WEB = !((c_state == WAIT_RVALID && RVALID_cnt > 0) || (c_state == WAIT_DRAM_1) || (c_state == UPDATE_SRAM));

always @(posedge clk, negedge rst_n) begin
    if(!rst_n) sram_in <= 0;
    else begin
        if(n_state == UPDATE_SRAM) sram_in <= rt_data;
        else if(c_state == IDLE)   sram_in <= 0;
        else                       sram_in <= rdata_m_inf;
    end
end



SRAM_128x16 DATA( .A0(sram_addr[0]), .A1(sram_addr[1]),   .A2(sram_addr[2]),   .A3(sram_addr[3]), .A4(sram_addr[4]),   
                        .A5(sram_addr[5]), .A6(sram_addr[6]),  .DO0(sram_out[0]),   .DO1(sram_out[1]),   .DO2(sram_out[2]),   
                        .DO3(sram_out[3]), .DO4(sram_out[4]),   .DO5(sram_out[5]),   .DO6(sram_out[6]),   .DO7(sram_out[7]),
                       .DO8(sram_out[8]), .DO9(sram_out[9]),   .DO10(sram_out[10]), .DO11(sram_out[11]), .DO12(sram_out[12]), 
                       .DO13(sram_out[13]), .DO14(sram_out[14]), .DO15(sram_out[15]), .DI0(sram_in[0]),  .DI1(sram_in[1]),   
                       .DI2(sram_in[2]), .DI3(sram_in[3]), .DI4(sram_in[4]),    .DI5(sram_in[5]), .DI6(sram_in[6]), .DI7(sram_in[7]),
                       .DI8(sram_in[8]),  .DI9(sram_in[9]), .DI10(sram_in[10]), .DI11(sram_in[11]),
                       .DI12(sram_in[12]),  .DI13(sram_in[13]), .DI14(sram_in[14]), .DI15(sram_in[15]),
                       .CK(clk), .WEB(WEB), .OE(1'b1), .CS(1'b1));


always @(posedge clk, negedge rst_n) begin
    if (!rst_n) label <= 4'hf;
    else begin
        if(c_state == WAIT_ARREADY)  label <= data_addr[10:7];
    end 
end

always @(posedge clk, negedge rst_n) begin
    if (!rst_n) arvalid_m_inf <= 0;
    else begin
        case(c_state) 
            WAIT_ARREADY: begin
                if(!arready_m_inf) arvalid_m_inf <= 1;
                else               arvalid_m_inf <= 0;
            end
        endcase
    end
end

always @(posedge clk, negedge rst_n) begin
    if(!rst_n) rready_m_inf <= 1'b0;
    else begin
        case(c_state) 
            WAIT_RVALID: begin
                if(!rlast_m_inf) rready_m_inf <= 1;
                else             rready_m_inf <= 0;
            end
            default:             rready_m_inf <= rready_m_inf;
        endcase
    end
end


reg [15:0] sram_out_reg;
always @(posedge clk, negedge rst_n)begin
    if(!rst_n) sram_out_reg <= 0;
    else       sram_out_reg <= sram_out;
end 


always @(posedge clk) begin
    if(c_state == WAIT_SRAM_3)                                                      r_data <= sram_out_reg;
    else if((c_state == WAIT_RVALID || c_state == WAIT_DRAM_1)  && sram_addr == data_addr[6:0]) begin
         r_data <= sram_in;
    end
    else                                                                            r_data <= r_data;
end

always @(posedge clk, negedge rst_n) begin
    if (!rst_n) r_data_end <= 0;
    else        r_data_end <= (n_state == OUTPUT);
end

always @(posedge clk) araddr_m_inf <= {20'b00000000000000000001, data_addr[10:7], 8'b00000000};

endmodule






