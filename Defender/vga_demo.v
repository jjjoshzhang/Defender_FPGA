`default_nettype none

module vga_demo(
    CLOCK_50, SW, KEY, LEDR,
    HEX5, HEX4, HEX3, HEX2, HEX1, HEX0,
    VGA_R, VGA_G, VGA_B,
    VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_CLK
);
    // specify the number of bits needed for an X (column) pixel coordinate on the VGA display
    parameter nX = 10;
    // specify the number of bits needed for a Y (row) pixel coordinate on the VGA display
    parameter nY = 9;

    // state names for the FSM that controls drawing of objects
    parameter A = 2'b00, B = 2'b01, C = 2'b10, D = 2'b11;

    input  wire CLOCK_50;	
    input  wire [9:0] SW;
    input  wire [3:0] KEY;
    output wire [9:0] LEDR;
 
    output wire [6:0] HEX5, HEX4, HEX3, HEX2, HEX1, HEX0;
    output wire [7:0] VGA_R;
    output wire [7:0] VGA_G;
    output wire [7:0] VGA_B;
    output wire VGA_HS;
    output wire VGA_VS;
    output wire VGA_BLANK_N;
    output wire VGA_SYNC_N;
    output wire VGA_CLK;	
	
    wire fire;

    // missile signals
    wire [nX-1:0] missile_x;
    wire [nY-1:0] missile_y;
    wire [8:0]    missile_color;
    wire          missile_write;
    wire          missile_active;
	 
    // The signals below are used to multiplex the pixels displayed for objects 1 and 2
    wire [nX-1:0] O1_x, O2_x, MUX_x;    // x coordinate multiplexer
    wire [nY-1:0] O1_y, O2_y, MUX_y;    // y coordinate multiplexer
    wire [8:0]    O1_color, O2_color, MUX_color; // color multiplexer
    wire          O1_write, O2_write, MUX_write; // write control multiplexer
    wire          M_write;
    assign M_write = missile_write;

    // NEW: center positions of objects (for missile spawn)
    wire [nX-1:0] O1_center_x, O2_center_x;
    wire [nY-1:0] O1_center_y, O2_center_y;

    wire O1_done, O2_done, done;    // object move completed
    reg  O1_O2;                     // draw object 1 when cleared, object 2 when set
    wire [1:0] O1_dir, O2_dir;      // used to set direction of moving for objects
    reg  [1:0] y_Q, Y_D;            // FSM, used to control drawing of objects

    wire Resetn, KEY1, KEY2;        // Reset, and synchronized versions of KEYs
    
    // No-PS2 but keep the structure for now
    wire ps2_rec = 1'b0;
    wire object_sel = 1'b0;
    reg  Esc, step; 

    //keep sync for keys only
    assign Resetn = KEY[0];
    sync S1 (~KEY[1], Resetn, CLOCK_50, KEY1);
    sync S2 (~KEY[2], Resetn, CLOCK_50, KEY2);

    // KEY[3] -> one-cycle fire pulse
    wire KEY3_sync;
    reg  KEY3_sync_d;
    wire fire_pulse;

    sync S3 (~KEY[3], Resetn, CLOCK_50, KEY3_sync);
    always @(posedge CLOCK_50)
        if (!Resetn)
            KEY3_sync_d <= 1'b0;
        else
            KEY3_sync_d <= KEY3_sync;

    assign fire_pulse = KEY3_sync & ~KEY3_sync_d;  // 1-cycle pulse on press

    // register whether to move object 1 or object 2
    always @(posedge CLOCK_50)
        if (!Resetn | KEY1)
            O1_O2 <= 1'b0;  // select object 1
        else if (KEY2)
            O1_O2 <= 1'b1;  // select object 2
        else if (step)
            O1_O2 <= !object_sel;

    assign O1_dir = 2'b00; 
    assign O2_dir = 2'b00; 

    // small FSM skeleton kept for structure
    always @ (*)
        case (y_Q)
            A:  if (!ps2_rec) Y_D = A;
                else Y_D = B;
            B:  Y_D = C;        // enable scancode register
            C:  Y_D = D;        // send step signal to object
            D:  if (done == 1'b0) Y_D = D;
                else Y_D = A;
            default: Y_D = A;
        endcase

    // FSM outputs
    always @ (*)
    begin
        // default assignments
        Esc = 1'b0; step = 1'b0;
        case (y_Q)
            A:  ;
            B:  Esc = 1'b1;
            C:  step = 1'b1;  
            D:  ;
        endcase
    end

    // FSM state FFs
    always @(posedge CLOCK_50)
        if (!Resetn)
            y_Q <= A;
        else
            y_Q <= Y_D;
				
    assign fire = fire_pulse;

    // missile: NOW SPAWNS FROM SPACECRAFT (O1) LOCATION
    missile M1(
        .Clock     (CLOCK_50),
        .Resetn    (Resetn),
        .fire      (fire_pulse),
        .player_x  (O1_center_x),   // spacecraft center X
        .player_y  (O1_center_y),   // spacecraft center Y
        .VGA_x     (missile_x),
        .VGA_y     (missile_y),
        .VGA_color (missile_color),
        .VGA_write (missile_write),
        .active    (missile_active)
    );

    // instantiate object 1 (spacecraft)
    object O1 (
        .Resetn    (Resetn),
        .Clock     (CLOCK_50),
        .go        (KEY1),
        .ps2_rec   (1'b0),
        .dir       (O1_dir),
        .VGA_x     (O1_x),
        .VGA_y     (O1_y),
        .VGA_color (O1_color),
        .VGA_write (O1_write),
        .done      (O1_done),
        .obj_x     (O1_center_x),   // center position out
        .obj_y     (O1_center_y)
    );

    // instantiate object 2 (alien)
    object O2 (
        .Resetn    (Resetn),
        .Clock     (CLOCK_50),
        .go        (KEY2),
        .ps2_rec   (1'b0),
        .dir       (O2_dir),
        .VGA_x     (O2_x),
        .VGA_y     (O2_y),
        .VGA_color (O2_color),
        .VGA_write (O2_write),
        .done      (O2_done),
        .obj_x     (O2_center_x),
        .obj_y     (O2_center_y)
    );
    defparam O2.XOFFSET  = 160;
    defparam O2.YOFFSET  = 150;
    defparam O2.INIT_FILE = "./MIF/alien_32_32_9.mif";

    assign done = O1_done | O2_done;

    // debug 
    assign LEDR = {6'b0, done, KEY[2], KEY[1], Resetn};

    // choose x, y, color, and write for one of the two objects
    // base selection between O1 and O2
    wire [nX-1:0] base_x   = (!O1_O2) ? O1_x     : O2_x;
    wire [nY-1:0] base_y   = (!O1_O2) ? O1_y     : O2_y;
    wire [8:0]    base_c   = (!O1_O2) ? O1_color : O2_color;
    wire          base_w   = (!O1_O2) ? O1_write : O2_write;

    // missile has top priority
    assign MUX_x     = M_write ? missile_x      : base_x;
    assign MUX_y     = M_write ? missile_y      : base_y;
    assign MUX_color = M_write ? missile_color  : base_c;
    assign MUX_write = M_write ? missile_write  : base_w;

    // Show O1_x, O1_y, O2_x, O2_y nibbles on HEX for debug
    hex7seg H0 (O1_x[3:0],  HEX0);
    hex7seg H1 (O1_x[7:4],  HEX1);
    hex7seg H2 (O1_y[3:0],  HEX2);
    hex7seg H3 (O1_y[7:4],  HEX3);
    hex7seg H4 (O2_x[3:0],  HEX4);
    hex7seg H5 (O2_y[3:0],  HEX5);

    // connect to VGA controller
    vga_adapter VGA (
        .resetn     (Resetn),
        .clock      (CLOCK_50),
        .color      (MUX_color),
        .x          (MUX_x),
        .y          (MUX_y),
        .write      (MUX_write),
        .VGA_R      (VGA_R),
        .VGA_G      (VGA_G),
        .VGA_B      (VGA_B),
        .VGA_HS     (VGA_HS),
        .VGA_VS     (VGA_VS),
        .VGA_BLANK_N(VGA_BLANK_N),
        .VGA_SYNC_N (VGA_SYNC_N),
        .VGA_CLK    (VGA_CLK)
    );
endmodule

// synchronizer, implemented as two FFs in series
module sync(D, Resetn, Clock, Q);
    input  wire D;
    input  wire Resetn, Clock;
    output reg  Q;

    reg Qi; // internal node

    always @(posedge Clock)
        if (Resetn == 0) begin
            Qi <= 1'b0;
            Q  <= 1'b0;
        end
        else begin
            Qi <= D;
            Q  <= Qi;
        end
endmodule

// n-bit register with enable
module regn(R, Resetn, E, Clock, Q);
    parameter n = 8;
    input  wire [n-1:0] R;
    input  wire Resetn, E, Clock;
    output reg  [n-1:0] Q;

    always @(posedge Clock)
        if (!Resetn)
            Q <= 0;
        else if (E)
            Q <= R;
endmodule

// n-bit up/down-counter with reset, load, enable, and direction control
module upDn_count (R, Clock, Resetn, L, E, Dir, Q);
    parameter n = 8;
    input  wire [n-1:0] R;
    input  wire Clock, Resetn, E, L, Dir;
    output reg  [n-1:0] Q;

    always @ (posedge Clock)
        if (Resetn == 0)
            Q <= {n{1'b0}};
        else if (L == 1)
            Q <= R;
        else if (E)
            if (Dir)
                Q <= Q + {{n-1{1'b0}},1'b1};
            else
                Q <= Q - {{n-1{1'b0}},1'b1};
endmodule

// module for missile helpers

module ToggleFF(T, Resetn, Clock, Q);
    input  wire T, Resetn, Clock;
    output reg  Q;

    always @(posedge Clock)
        if (!Resetn)
            Q <= 1'b0;
        else if (T)
            Q <= ~Q;
endmodule

module UpDn_count (R, Clock, Resetn, E, L, UpDn, Q);
    parameter n = 8;
    input  wire [n-1:0] R;
    input  wire Clock, Resetn, E, L, UpDn;
    output reg  [n-1:0] Q;

    always @ (posedge Clock)
        if (Resetn == 0)
            Q <= 0;
        else if (L == 1)
            Q <= R;
        else if (E)
            if (UpDn == 1)
                Q <= Q + 1'b1;
            else
                Q <= Q - 1'b1;
endmodule

module Up_count (Clock, Resetn, Q);
    parameter n = 8;
    input  wire Clock, Resetn;
    output reg  [n-1:0] Q;

    always @ (posedge Clock)
        if (Resetn == 0)
            Q <= 'b0;
        else 
            Q <= Q + 1'b1;
endmodule

module hex7seg (hex, display);
    input  wire [3:0] hex;
    output reg  [6:0] display;

    /*
     *       0  
     *      ---  
     *     |   |
     *    5|   |1
     *     | 6 |
     *      ---  
     *     |   |
     *    4|   |2
     *     |   |
     *      ---  
     *       3  
     */
    always @ (hex)
        case (hex)
            4'h0: display = 7'b1000000;
            4'h1: display = 7'b1111001;
            4'h2: display = 7'b0100100;
            4'h3: display = 7'b0110000;
            4'h4: display = 7'b0011001;
            4'h5: display = 7'b0010010;
            4'h6: display = 7'b0000010;
            4'h7: display = 7'b1111000;
            4'h8: display = 7'b0000000;
            4'h9: display = 7'b0011000;
            4'hA: display = 7'b0001000;
            4'hB: display = 7'b0000011;
            4'hC: display = 7'b1000110;
            4'hD: display = 7'b0100001;
            4'hE: display = 7'b0000110;
            4'hF: display = 7'b0001110;
        endcase
endmodule

// implements a movable object
module object (
    Resetn, Clock, go, ps2_rec, dir,
    VGA_x, VGA_y, VGA_color, VGA_write, done,
    obj_x, obj_y
);
    // specify the number of bits needed for an X (column) pixel coordinate on the VGA display
    parameter nX = 10;
    // specify the number of bits needed for a Y (row) pixel coordinate on the VGA display
    parameter nY = 9;
    // by default, use offsets to center the object on the VGA display
    parameter XOFFSET = 320;
    parameter YOFFSET = 240;
    parameter LEFT = 2'b00 /*'a'*/, RIGHT = 2'b11/*'s'*/, UP = 2'b01/*'w'*/, DOWN = 2'b10/*'z'*/;
    parameter xOBJ = 5, yOBJ = 5;   // object size is 2^xOBJ x 2^yOBJ
    parameter BOX_SIZE_X = 1 << xOBJ;
    parameter BOX_SIZE_Y = 1 << yOBJ;
    parameter Mn = xOBJ + yOBJ; // address lines needed for the object memory
    parameter INIT_FILE = "./MIF/spacecraft_32_32_9.mif";

    // state names for the FSM that draws the object
    parameter A = 3'b000, B = 3'b001, C = 3'b010, D = 3'b011, E = 3'b100,
              F = 3'b101, G = 3'b110, H = 3'b111;
    
    input  wire Resetn, Clock;
    input  wire go;                              // can be used to draw at initial position
    input  wire ps2_rec;                         // PS2 data received
    input  wire [1:0] dir;                       // movement direction
    output wire [nX-1:0] VGA_x;                  // for syncing with object memory
    output wire [nY-1:0] VGA_y;                  // for syncing with object memory
    output wire [8:0]    VGA_color;              // used to draw pixels
    output wire          VGA_write;              // pixel write control
    output reg           done;                   // done drawing cycle

    // NEW: center position outputs (for missile)
    output wire [nX-1:0] obj_x;
    output wire [nY-1:0] obj_y;

    wire [nX-1:0] X, X0;    // starting X location 
    wire [nY-1:0] Y, Y0;    // starting Y location 
    wire [nX-1:0] size_x = BOX_SIZE_X;   // store the X size (must be power of 2)
    wire [nY-1:0] size_y = BOX_SIZE_Y;   // store the Y size
    wire [xOBJ-1:0] XC;                  // used to access object memory
    wire [yOBJ-1:0] YC;                  // used to access object memory
    reg  write, Lxc, Lyc, Exc, Eyc;      // object control signals
    reg  erase;                          // erase/draw object
    wire Right, Left, Up, Down;          // object direction
    reg  Lx, Ly, Ex, Ey;                 // object counter controls
    reg  [2:0] y_Q, Y_D;                 // FSM
    
    wire [8:0] obj_color;    // object pixel colors, read from memory
	
    // object (x,y) location. For x, counter will be enabled when moving L/R, increment
    // for R, decrement for L. For y, counter will be enabled when moving U/D, increment 
    // for D, decrement for U
    assign X0 = XOFFSET;
    assign Y0 = YOFFSET;
    upDn_count UX (X0, Clock, Resetn, Lx, Ex, Right, X);
        defparam UX.n = nX;
    upDn_count UY (Y0, Clock, Resetn, Ly, Ey, Down, Y);
        defparam UY.n = nY;

    // these counter are used to generate (x,y) coordinates to read the object's pixels
    upDn_count U3 ({xOBJ{1'd0}}, Clock, Resetn, Lxc, Exc, 1'b1, XC); // object column counter
        defparam U3.n = xOBJ;
    upDn_count U4 ({yOBJ{1'd0}}, Clock, Resetn, Lyc, Eyc, 1'b1, YC); // object row counter
        defparam U4.n = yOBJ;

    // these signals are used to enable the (x,y) object location counters and to make these 
    // counters increment or decrement
    assign Left  = (dir == LEFT);
    assign Right = (dir == RIGHT);
    assign Up    = (dir == UP);
    assign Down  = (dir == DOWN);

    // FSM state table
    always @ (*)
        case (y_Q)
            A:  Y_D = B;                        // load (x,y) location counters
            B:  if (go) Y_D = F;                // pushbutton KEY pressed to show object
                else if (ps2_rec) Y_D = C;      // PS2 key received to move object
                else Y_D = B;                   // wait
            C:  if (XC != size_x-1) Y_D = C;    // erase row of object
                else Y_D = D;
            D:  if (YC != size_y-1) Y_D = C;    // next row of object to erase
                else Y_D = E;                   // done erase cycle
            E:  Y_D = F;                        // +/- (x,y)
            F:  if (XC != size_x-1) Y_D = F;    // draw row of object
                else Y_D = G;
            G:  if (YC != size_y-1) Y_D = F;    // next row of object to draw
                else Y_D = H;                   // done draw cycle
            H:  if (go) Y_D = H;                // wait for KEY press
                else Y_D = B;
            default: Y_D = A;
        endcase

    // FSM outputs
    always @ (*)
    begin
        // default assignments
        Lx = 1'b0; Ly = 1'b0; Ex = 1'b0; Ey = 1'b0; write = 1'b0; 
        Lxc = 1'b0; Lyc = 1'b0; Exc = 1'b0; Eyc = 1'b0; erase = 1'b0; done = 1'b0;
        case (y_Q)
            A:  begin Lx = 1'b1; Ly = 1'b1; end                   // load (X,Y) counters
            B:  begin Lxc = 1'b1; Lyc = 1'b1; end                 // load (XC,YC) counters
            C:  begin Exc = 1'b1; write = 1'b1; erase = 1'b1; end // enable XC, write pixel
            D:  begin Lxc = 1'b1; Eyc = 1'b1; erase = 1'b1; end   // load XC, enable YC
            // state E is reached after erasing the object. Now, move and draw the object
            E:  begin Ex = Right | Left; Ey = Up | Down; end      // move L/R or U/D
            F:  begin Exc = 1'b1; write = 1'b1; end               // enable XC, write pixel
            G:  begin Lxc = 1'b1; Eyc = 1'b1; end                 // load XC, enable YC
            H:  done = 1'b1;
        endcase
    end

    // FSM state FFs
    always @(posedge Clock)
        if (!Resetn)
            y_Q <= 3'b0;
        else
            y_Q <= Y_D;

    // read a pixel color from the object memory. We can use {YC,XC} because the x dimension
    // of the object memory is a power of 2
    object_mem U6 ({YC,XC}, Clock, obj_color);
        defparam U6.n = 9;
        defparam U6.Mn = xOBJ + yOBJ;
        defparam U6.INIT_FILE = INIT_FILE;

    // compute the (x,y) location of the current pixel to be drawn (or erased).
    regn U7 (X - (size_x >> 1) + XC, Resetn, 1'b1, Clock, VGA_x);
        defparam U7.n = nX;
    regn U8 (Y - (size_y >> 1) + YC, Resetn, 1'b1, Clock, VGA_y);
        defparam U8.n = nY;

    // synchronize write signal with VGA_x, VGA_y, VGA_color 
    regn U9 (write, Resetn, 1'b1, Clock, VGA_write);
        defparam U9.n = 1;

    // use the background color (when erasing), or the object color when drawing
    // (black background is assumed below)
    assign VGA_color = erase ? {9{1'b0}} : obj_color;

    // NEW: expose object's center position for missile
    assign obj_x = X;
    assign obj_y = Y;
endmodule

// implements a horizontally-moving, fixed-color missile (fires from spacecraft position)
module missile(
    Clock, Resetn, fire,
    player_x, player_y,
    VGA_x, VGA_y, VGA_color, VGA_write, active
);
    parameter nX = 10;
    parameter nY = 9;

    input  wire Clock;
    input  wire Resetn;
    input  wire fire;

    // spacecraft position inputs
    input  wire [nX-1:0] player_x;
    input  wire [nY-1:0] player_y;  

    output wire [nX-1:0] VGA_x;
    output wire [nY-1:0] VGA_y;
    output wire [8:0]    VGA_color;
    output wire          VGA_write;
    output wire          active;
    
    parameter ScreenX = 640;
    parameter missile_length    = 25;
    parameter missile_thickness = 3;

    parameter N = 16; // bit width of counter, smaller N -> higher movement speed

    parameter [8:0] RED   = 9'b111000000;
    parameter [8:0] BLACK = 9'b0; 

    parameter [nX-1:0] Max_X = ScreenX;

    reg [nX-1:0] posX; 
    reg [nY-1:0] posY;

    reg [nX-1:0] XC; 
    reg [nY-1:0] YC; 

    reg erase; // 1 for erase;
    reg write; // 1 for writing a pixel;

    wire sync;
    reg [N-1:0] cnt;

    reg [3:0] state;
    reg [3:0] next_state;

    parameter IDLE    = 4'b0000, 
              INITIAL = 4'b0001,
              DRAW    = 4'b0010,
              WAIT    = 4'b0011,
              ERASE   = 4'b0100,
              MOVE    = 4'b0101,
              DONE    = 4'b0110;

    assign active = (state != IDLE) && (state != DONE);

    // counter to control movement speed 
    always @ (posedge Clock) begin 
        if (!Resetn)
            cnt <= 0;
        else 
            cnt <= cnt + 1'b1;
    end
    
    assign sync = (cnt == {N{1'b1}} );

    assign VGA_x     = posX + XC;
    assign VGA_y     = posY + YC;
    assign VGA_color = erase ? BLACK : RED;
    assign VGA_write = write;

    // next-state and output logic
    always @(*) begin
        next_state = state;
        write      = 1'b0;
        erase      = 1'b0;

        case (state)
            IDLE: begin
                if (fire)
                    next_state = INITIAL;
            end

            INITIAL: begin
                next_state = DRAW;
            end

            DRAW: begin 
                write = 1'b1;
                erase = 1'b0;

                if ((XC == missile_length-1) && (YC == missile_thickness-1))
                    next_state = WAIT;
                else
                    next_state = DRAW;
            end 

            WAIT: begin 
                if (sync)
                    next_state = ERASE;
                else
                    next_state = WAIT;
            end 

            ERASE: begin 
                write = 1'b1;
                erase = 1'b1;
                if ((XC == missile_length-1) && (YC == missile_thickness-1))
                    next_state = MOVE;
                else
                    next_state = ERASE;
            end

            MOVE: begin
                if (posX < Max_X)
                    next_state = DRAW; 
                else
                    next_state = DONE;
            end

            DONE: begin 
                if (fire)
                    next_state = INITIAL;
            end

            default: begin 
                next_state = IDLE;
            end
        endcase
    end

    // state & position registers
    always @(posedge Clock) begin
        if (!Resetn) begin
            state <= IDLE;
            posX  <= player_x; // start at spacecraft on reset
            posY  <= player_y;
            XC    <= 0;
            YC    <= 0;
        end
        else begin
            state <= next_state;
            case (state)
                IDLE: begin
                    XC <= 0;
                    YC <= 0;
                    if (fire) begin 
                        // snapshot spacecraft position when firing
                        posX <= player_x;
                        posY <= player_y;
                    end
                end

                INITIAL: begin
                    // ensure missile starts from current spacecraft location
                    posX <= player_x;
                    posY <= player_y;
                    XC   <= 0;
                    YC   <= 0;
                end

                DRAW: begin 
                    if (XC < missile_length-1) begin
                        XC <= XC + 1'b1;
                    end
                    else begin 
                        XC <= 0;
                        if (YC < missile_thickness-1) begin
                            YC <= YC + 1'b1;
                        end
                        else 
                            YC <= 0;
                    end
                end

                ERASE: begin 
                    if (XC < missile_length-1) begin
                        XC <= XC + 1'b1;
                    end
                    else begin
                        XC <= 0;
                        if (YC < missile_thickness-1) begin
                            YC <= YC + 1'b1;
                        end
                        else begin
                            YC <= 0;
                        end
                    end
                end

                MOVE: begin
                    if (posX < Max_X)
                        posX <= posX + 1'b1;
                end

                // WAIT and DONE: no updates needed for posX/posY/XC/YC here
            endcase
        end
    end
endmodule
