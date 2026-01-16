module i2c_master (
  input clk, rst, newd,
  input [6:0] addr,
  input op,   //1-rd, 0-wr
  inout sda,  //SDA is bidirectional because both master and slave must drive it for data and ACK.
  output scl,  //i2c master generates the clock so scl is output 
  input [7:0] din,
  output [7:0] dout,
  output reg busy, ack_err, done
);
  
  reg scl_t = 0;  //temporary storing scl and sda values before sending the ports scl and sda
  reg sda_t = 0;
  
  parameter sys_freq = 40000000; //onboard clock freq= 40MHz
  parameter i2c_freq = 100000;  //freq of i2c = 100k
  
  parameter clk_count4 = (sys_freq/i2c_freq); //400 clock pulse
  parameter clk_count1 = clk_count4/4;  //100 clk pulse  --
  
  
  integer count1 = 0; //variable which will count pulse 0to 99 or 100-199 or 200-299 or 300-399
  reg i2c_clk = 0;
  
  //4x clock
  reg [1:0] pulse = 0;   //divide bit duration inti 4 small paerts -->> 0,1,2,3,0
  
  always@(posedge clk)
    begin
      if(rst) begin
        pulse <= 0;
        count1 <= 0;
      end
      else if (busy == 1'b0) begin  //pulse count starts only after newd 
        pulse <= 0;
        count1 <= 0;
      end
      //reset is low and busy is high --> valid data will come
      else if (count1 == clk_count1 - 1 ) begin //  at count1=99 if condition becomes true , till 0 to 99 clock pulse if condition fails so pulse value is 0 . after 100 to 199 pulse value is 1
        pulse <= 1;
        count1 <= count1 + 1;
      end
      else if (count1 == clk_count1*2 - 1) begin // 199
        pulse <= 2;
        count1 <= count1 +1; 
      end
      else if (count1 == clk_count1*3 - 1) begin //299
        pulse <= 3;
        count1 <= count1 + 1;
      end
      else if (count1 == clk_count1*4 - 1) begin //399
        pulse <= 0;
        count1 <= 0;
      end
      else begin
        count1 <= count1 + 1;
      end        
    end
  

  reg [3:0] bitcount = 0;
  reg [7:0] data_addr = 0, data_tx = 0;
  reg r_ack= 0;
  reg [7:0] rx_data = 0;
  reg sda_en = 0;   //when we want to write the data on sda like make it 1 and when we want to reveive data from slave make it low
  
  
  typedef enum logic [3:0]{idle = 0, start = 1, write_addr = 2, ack_1 = 3, write_data = 4, read_data = 5, stop = 6, ack_2 = 7, master_ack = 8} state_type;
  state_type state = idle;
  
  always@(posedge clk) begin
    if(rst) begin
      bitcount <= 0;
      data_addr <= 0;
      data_tx <= 0;
      scl_t   <= 1;
      sda_t   <= 1;
      state   <= idle;
      busy    <= 0;
      ack_err <= 0;
      done    <= 0;
    end
    
    else 
      
      begin
        case (state)
          
          //idle state 
          idle: 
            begin
              done <= 0;
              if(newd == 1)      //when user have new data first store data in temporary veriable
                begin
                data_addr <= {addr, op};
                data_tx	  <= din;
                busy      <= 1;
                state     <= start;
                ack_err   <= 0;
                end 
              else
                begin
                  data_addr <= 0;
                  data_tx   <= 0;
                  busy      <= 0;
                  state     <= idle;
                  ack_err   <= 0;
                end
            end
          
          
          //start cond: scl stay high, sda goes from 1 to 0
          start:
            begin
              sda_en <= 1;  //send start to slave
              case (pulse)
                0: begin scl_t <= 1'b1; sda_t <= 1'b1; end    //start condition
                1: begin scl_t <= 1'b1; sda_t <= 1'b1; end
                2: begin scl_t <= 1'b1; sda_t <= 1'b0; end
                3: begin scl_t <= 1'b1; sda_t <= 1'b0; end
              endcase
              
              if (count1 == clk_count1*4 - 1) begin
                state <= write_addr;
                scl_t <= 1'b0;       // Immediately after START: The master must begin bit transmission nut i2c rule says, Data can change only when SCL is LOW.So before sending the first address bit, you must pull SCL LOW.
              end
              else
                state <=  start;  //stay in start state untile we complete 400 clock pulse (0 to 399)
            end
          
          
          
          write_addr :
            begin
              sda_en <= 1'b1;   //send addr to slave
              if(bitcount <= 7) 
                begin
                  case (pulse)
                    0: begin scl_t <= 1'b0; sda_t <= 1'b0; end
                    //send msb first
                    1: begin scl_t <= 1'b0; sda_t <= data_addr [7 - bitcount]; end // changing data at negative edge of clk i.e scl is low 
                    //make sure when scl is high data should not changed
                    2: begin scl_t <= 1'b1; end
                    3: begin scl_t <= 1'b1; end
                  endcase
                  
                  if (count1 == clk_count1*4 - 1)
                  begin
                      state <= write_addr; //till all the bitcount is send we will be in write address
                      scl_t <= 1'b0;
                      bitcount <= bitcount + 1;
                  end
                  else
                  begin
                      state <= write_addr;
                  end
                end
              
              else
                begin
                  state <= ack_1;
                  bitcount <=  0;
                  sda_en <= 0;   //ready to receive data from slave so we reselase the line --> sda_en=0
                end
            end
          
          
          
          //waiting ack from slave
          //ack means slave pulls sda low while scl is high   
          ack_1:
            begin
              sda_en <=  0 ; //revceive ack from slave
              case (pulse)
                  0: begin scl_t <= 0; sda_t <= 0;end    //SDA can change only when SCL is LOW
                  1: begin scl_t <= 0; sda_t <= 0;end
                  2: begin scl_t <= 1; sda_t <= 0; r_ack <= 0; end  //recv ack from slave: pulls sda low while scl is high
                  3: begin scl_t <= 1;end
              endcase
              
              if (count1 == clk_count1*4 - 1)
                begin
                  if (r_ack == 1'b0 && data_addr[0] == 1'b0) begin  //if correct ack is coming(r_ack) and op is 0 then write operation
                    state <=  write_data;  
                    sda_t <= 1'b0;   //Gives SDA a safe value before master starts sending data
                    sda_en <= 1'b1;  //write data to slave
                    bitcount <= 0;
                    end
                  else if (r_ack == 1'b0 && data_addr[0] == 1'b1) begin
                    state <= read_data;
                    sda_t <= 1'b1;  //Keeps SDA at idle level in case master drives it by mistake
                    sda_en <= 1'b0; //read data from slave
                    bitcount <= 0;
                  end
                  else  //invalid ack
                    begin
                    state <= stop;
                    sda_en <= 1'b1;  //send stop bit to slave
                    ack_err <= 1'b1;  ///raise ack error
                    end
                end
              
              else
                begin
                  state <= ack_1;
                end
              
            end
          
          
           write_data:
            begin
              sda_en <= 1'b1;
              //write data to slave
              if(bitcount <= 7)
                begin
                  case (pulse)
                    0: begin scl_t <= 0; end
                    1: begin scl_t <= 0; sda_en <= 1; sda_t <= data_tx[7- bitcount]; end  //send msb data  //data is written when scl is low
                    2: begin scl_t <= 1; end 
                    3: begin scl_t <= 1; end
                  endcase
                  
                  if (count1 == clk_count1*4 - 1) 
                  begin
                      state <= write_data;
                      scl_t <= 1'b0;
                      bitcount <= bitcount + 1;
                  end
                  else
                  begin
                      state <= write_data;
                  end
                end
              
              else
                begin
                  state <= ack_2;   //after write operation it goes to ack_2 state
                  bitcount <= 0;
                  sda_en <= 0; //read from slave
                end
            end
          
          
          read_data:
            begin
              sda_en <= 1'b0;  //read from slave
              if(bitcount <= 7)
                begin
                  case (pulse) 
                      0: begin scl_t <= 0; sda_t <= 0; end
                      1: begin scl_t <= 0; sda_t <= 0; end
                      // I²C is serial: one bit is read from SDA on each SCL high, so rx_data acts as a shift register to collect 8 bits into one byte.
                      2: begin scl_t <= 1;  rx_data[7:0] <= (count1 == 200) ? {rx_data[6:0], sda} : rx_data; end   //Data is valid and must be read when SCL is HIGH   //pulse 2 is read from 200 to 299 so count1 == 200 condition is used
                      
                      3: begin scl_t <= 1; end
                  endcase
                  
                  if(count1 == clk_count1*4 - 1)
                    begin
                      state <= read_data;
                      scl_t <= 0;        //SCL LOW  → prepare data SCL HIGH → read data SCL LOW  → move to next bit    So after you read the bit (when SCL was HIGH), you must bring SCL LOW again.
                      bitcount <= bitcount + 1;
                    end
                  else
                    state <= read_data;
                end
              
              else
                begin
                  state <= master_ack;
                  bitcount <= 0;
                  sda_en <= 1;
                end       
            end
          
          
          //send negative ack --> keep sda high
           master_ack : 
                   begin
                      sda_en <= 1'b1;
                      
                                case(pulse)
                                 0: begin scl_t <= 1'b0; sda_t <= 1'b1; end
                                 1: begin scl_t <= 1'b0; sda_t <= 1'b1; end
                                 2: begin scl_t <= 1'b1; sda_t <= 1'b1; end 
                                 3: begin scl_t <= 1'b1; sda_t <= 1'b1; end
                                 endcase
                   
                                if(count1  == clk_count1*4 - 1)
                                  begin
                                      sda_t <= 1'b0;
                                      state <= stop;
                                      sda_en <= 1'b1; ///send stop to slave
                                      
                                  end
                                 else
                                  begin
                                    state <= master_ack;
                                  end
                     
                   end
                 
          
          
          ack_2:
            begin
              sda_en <= 0; //rcev ack from slave
              case(pulse)
                0: begin scl_t <= 0; sda_t <= 0; end
                1: begin scl_t <= 0; sda_t <= 0; end
                2: begin scl_t <= 1; sda_t <= 1; r_ack <= 1'b0; end //rcev ack from slave
                3: begin scl_t <= 1; end
              endcase
                  
                  if(count1 == clk_count1*4 - 1)
                    begin
                      sda_t <= 0;
                      sda_en <= 1;
                      if (r_ack == 1'b0 )
                        begin
                          state <= stop;
                          ack_err <= 1'b0;
                        end
                      else
                        begin
                          state <= stop;
                          ack_err <= 1'b1;  //ack error as rack=1
                        end
                    end
                  else
                    begin
                      state <= ack_2;
                    end
            end
                
          
        stop:
          begin
            sda_en <= 1'b1; //send stop to slave
            case (pulse)
              0: begin scl_t <= 1; sda_t <= 0; end
              1: begin scl_t <= 1; sda_t <= 0; end
              2: begin scl_t <= 1; sda_t <= 1; end
              3: begin scl_t <= 1; sda_t <= 1; end
            endcase
            
            if( count1 == clk_count1*4 - 1) 
              begin
                state <= idle;
                scl_t <= 0;
                busy <= 0;
                sda_en <= 1;  //send start to slave
                done <= 1;
              end
            else
              state <= stop;
           end
        endcase
        end
       end
                assign sda = (sda_en == 1) ? (sda_t == 0) ? 1'b0: 1'b1 : 1'bz;    //sda_en =1 write to slave
                assign scl = scl_t;
                assign dout = rx_data;
  
endmodule

  
