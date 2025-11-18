
`default_nettype none

module vga_demo(
    CLOCK_50, SW, KEY, LEDR, PS2_CLK, PS2_DAT,
    HEX5, HEX4, HEX3, HEX2, HEX1, HEX0,
    VGA_R, VGA_G, VGA_B,
    VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_CLK
);
    parameter nX = 10;
    parameter nY = 9;

    input  wire        CLOCK_50;
    input  wire [9:0]  SW;
    input  wire [3:0]  KEY;
    output wire [9:0]  LEDR;
    inout  wire        PS2_CLK, PS2_DAT;
    output wire [6:0]  HEX5, HEX4, HEX3, HEX2, HEX1, HEX0;
    output wire [7:0]  VGA_R;
    output wire [7:0]  VGA_G;
    output wire [7:0]  VGA_B;
    output wire        VGA_HS;
    output wire        VGA_VS;
    output wire        VGA_BLANK_N;
    output wire        VGA_SYNC_N;
    output wire        VGA_CLK;

    wire Resetn;
    assign Resetn = KEY[0];

    wire KEY1_sync, KEY2_sync;
    sync S1 (~KEY[1], Resetn, CLOCK_50, KEY1_sync);
    sync S2 (~KEY[2], Resetn, CLOCK_50, KEY2_sync);

    wire KEY3_sync;
    reg  KEY3_sync_d;
    wire fire_pulse;

    sync S3 (~KEY[3], Resetn, CLOCK_50, KEY3_sync);
    always @(posedge CLOCK_50) begin
        if (!Resetn)
            KEY3_sync_d <= 1'b0;
        else
            KEY3_sync_d <= KEY3_sync;
    end
    assign fire_pulse = KEY3_sync & ~KEY3_sync_d;

    wire [nX-1:0] sc_x;
    wire [nY-1:0] sc_y;
    wire [8:0]    sc_color;
    wire          sc_write;
    wire [nX-1:0] sc_obj_x;
    wire [nY-1:0] sc_obj_y;
    wire [7:0]    sc_scancode;

    spacecraft SC1(
        .Clock        (CLOCK_50),
        .Resetn       (Resetn),
        .go           (KEY1_sync),
        .PS2_CLK      (PS2_CLK),
        .PS2_DAT      (PS2_DAT),
        .VGA_x        (sc_x),
        .VGA_y        (sc_y),
        .VGA_color    (sc_color),
        .VGA_write    (sc_write),
        .obj_x        (sc_obj_x),
        .obj_y        (sc_obj_y),
        .scancode_out (sc_scancode)
    );

    wire [nX-1:0] al_x;
    wire [nY-1:0] al_y;
    wire [8:0]    al_color;
    wire          al_write;
    wire [nX-1:0] al_obj_x;
    wire [nY-1:0] al_obj_y;

    alien AL1(
        .Clock     (CLOCK_50),
        .Resetn    (Resetn),
        .go        (KEY2_sync),
        .VGA_x     (al_x),
        .VGA_y     (al_y),
        .VGA_color (al_color),
        .VGA_write (al_write),
        .obj_x     (al_obj_x),
        .obj_y     (al_obj_y)
    );

    wire [nX-1:0] missile_x;
    wire [nY-1:0] missile_y;
    wire [8:0]    missile_color;
    wire          missile_write;
    wire          missile_active;

    missile M1(
        .Clock     (CLOCK_50),
        .Resetn    (Resetn),
        .fire      (fire_pulse),
        .player_x  (sc_obj_x),
        .player_y  (sc_obj_y),
        .VGA_x     (missile_x),
        .VGA_y     (missile_y),
        .VGA_color (missile_color),
        .VGA_write (missile_write),
        .active    (missile_active)
    );

    wire [nX-1:0] MUX_x;
    wire [nY-1:0] MUX_y;
    wire [8:0]    MUX_color;
    wire          MUX_write;

    assign MUX_x = missile_write ? missile_x :
                   sc_write     ? sc_x      :
                   al_write     ? al_x      :
                   {nX{1'b0}};

    assign MUX_y = missile_write ? missile_y :
                   sc_write     ? sc_y      :
                   al_write     ? al_y      :
                   {nY{1'b0}};

    assign MUX_color = missile_write ? missile_color :
                       sc_write     ? sc_color      :
                       al_write     ? al_color      :
                       9'b0;

    assign MUX_write = missile_write | sc_write | al_write;

    assign LEDR = {2'b00, sc_scancode};

    hex7seg H0 (sc_scancode[3:0], HEX0);
    hex7seg H1 (sc_scancode[7:4], HEX1);
    assign HEX2 = 7'b1111111;
    assign HEX3 = 7'b1111111;
    assign HEX4 = 7'b1111111;
    assign HEX5 = 7'b1111111;

    vga_adapter VGA (
        .resetn      (Resetn),
        .clock       (CLOCK_50),
        .color       (MUX_color),
        .x           (MUX_x),
        .y           (MUX_y),
        .write       (MUX_write),
        .VGA_R       (VGA_R),
        .VGA_G       (VGA_G),
        .VGA_B       (VGA_B),
        .VGA_HS      (VGA_HS),
        .VGA_VS      (VGA_VS),
        .VGA_BLANK_N (VGA_BLANK_N),
        .VGA_SYNC_N  (VGA_SYNC_N),
        .VGA_CLK     (VGA_CLK)
    );
endmodule

module spacecraft(
    Clock, Resetn, go, PS2_CLK, PS2_DAT,
    VGA_x, VGA_y, VGA_color, VGA_write, obj_x, obj_y, scancode_out
);
    parameter nX = 10;
    parameter nY = 9;

    input  wire Clock;
    input  wire Resetn;
    input  wire go;
    inout  wire PS2_CLK;
    inout  wire PS2_DAT;
    output wire [nX-1:0] VGA_x;
    output wire [nY-1:0] VGA_y;
    output wire [8:0]    VGA_color;
    output wire          VGA_write;
    output wire [nX-1:0] obj_x;
    output wire [nY-1:0] obj_y;
    output wire [7:0]    scancode_out;

    wire PS2_CLK_S, PS2_DAT_S;
    sync S0 (PS2_CLK, Resetn, Clock, PS2_CLK_S);
    sync S1 (PS2_DAT, Resetn, Clock, PS2_DAT_S);

    reg prev_ps2_clk;
    wire negedge_ps2_clk;

    always @(posedge Clock)
        prev_ps2_clk <= PS2_CLK_S;

    assign negedge_ps2_clk = prev_ps2_clk & ~PS2_CLK_S;

    reg  [32:0] Serial;
    reg  [3:0]  Packet;

    always @(posedge Clock) begin
        if (!Resetn)
            Serial <= 33'b0;
        else if (negedge_ps2_clk) begin
            Serial[31:0] <= Serial[32:1];
            Serial[32]   <= PS2_DAT_S;
        end
    end

    always @(posedge Clock) begin
        if (!Resetn || Packet == 4'd11)
            Packet <= 4'd0;
        else if (negedge_ps2_clk)
            Packet <= Packet + 4'd1;
    end

    wire ps2_rec;
    assign ps2_rec = (Packet == 4'd11) && (Serial[30:23] == Serial[8:1]);

    wire [7:0] scancode;
    reg  Esc;
    reg  step;
    reg  [1:0] y_Q, Y_D;

    regn USS (Serial[8:1], Resetn, Esc, Clock, scancode);
    assign scancode_out = scancode;

    reg [1:0] dir_reg;
    always @(*) begin
        case (scancode)
            8'h1D: dir_reg = 2'b01;
            8'h1B: dir_reg = 2'b10;
            8'h1C: dir_reg = 2'b00;
            8'h23: dir_reg = 2'b11;
            default: dir_reg = 2'b00;
        endcase
    end
    wire [1:0] dir = dir_reg;

    wire wasd_key;
    assign wasd_key = (scancode == 8'h1D) ||
                      (scancode == 8'h1B) ||
                      (scancode == 8'h1C) ||
                      (scancode == 8'h23);

    localparam S_A = 2'b00;
    localparam S_B = 2'b01;
    localparam S_C = 2'b10;
    localparam S_D = 2'b11;

    wire obj_done;

    always @(*) begin
        case (y_Q)
            S_A: if (!ps2_rec) Y_D = S_A; else Y_D = S_B;
            S_B: Y_D = S_C;
            S_C: Y_D = S_D;
            S_D: if (!obj_done) Y_D = S_D; else Y_D = S_A;
            default: Y_D = S_A;
        endcase
    end

    always @(*) begin
        Esc  = 1'b0;
        step = 1'b0;
        case (y_Q)
            S_A: ;
            S_B: Esc = 1'b1;
            S_C: if (wasd_key) step = 1'b1;
            S_D: ;
        endcase
    end

    always @(posedge Clock) begin
        if (!Resetn)
            y_Q <= S_A;
        else
            y_Q <= Y_D;
    end

    object O1(
        .Resetn    (Resetn),
        .Clock     (Clock),
        .go        (go),
        .ps2_rec   (step),
        .dir       (dir),
        .VGA_x     (VGA_x),
        .VGA_y     (VGA_y),
        .VGA_color (VGA_color),
        .VGA_write (VGA_write),
        .done      (obj_done),
        .obj_x     (obj_x),
        .obj_y     (obj_y)
    );
endmodule

module alien(
    Clock, Resetn, go,
    VGA_x, VGA_y, VGA_color, VGA_write, obj_x, obj_y
);
    parameter nX = 10;
    parameter nY = 9;

    input  wire Clock;
    input  wire Resetn;
    input  wire go;
    output wire [nX-1:0] VGA_x;
    output wire [nY-1:0] VGA_y;
    output wire [8:0]    VGA_color;
    output wire          VGA_write;
    output wire [nX-1:0] obj_x;
    output wire [nY-1:0] obj_y;

    reg [22:0] slow;
    wire       step;

    always @(posedge Clock) begin
        if (!Resetn || go)
            slow <= 23'd0;
        else
            slow <= slow + 23'd1;
    end

    assign step = &slow;

    wire [1:0] dir;
    assign dir = 2'b00;

    wire dummy_done;

    object O2(
        .Resetn    (Resetn),
        .Clock     (Clock),
        .go        (go),
        .ps2_rec   (step),
        .dir       (dir),
        .VGA_x     (VGA_x),
        .VGA_y     (VGA_y),
        .VGA_color (VGA_color),
        .VGA_write (VGA_write),
        .done      (dummy_done),
        .obj_x     (obj_x),
        .obj_y     (obj_y)
    );

    defparam O2.XOFFSET   = 10'd600;
    defparam O2.YOFFSET   = 9'd150;
    defparam O2.xOBJ      = 5;
    defparam O2.yOBJ      = 5;
    defparam O2.INIT_FILE = "./MIF/alien_32_32_9.mif";
endmodule

module object(
    Resetn, Clock, go, ps2_rec, dir,
    VGA_x, VGA_y, VGA_color, VGA_write, done,
    obj_x, obj_y
);
    parameter nX = 10;
    parameter nY = 9;
    parameter XOFFSET = 320;
    parameter YOFFSET = 240;
    parameter LEFT  = 2'b00;
    parameter RIGHT = 2'b11;
    parameter UP    = 2'b01;
    parameter DOWN  = 2'b10;
    parameter xOBJ = 5;
    parameter yOBJ = 5;
    parameter BOX_SIZE_X = 1 << xOBJ;
    parameter BOX_SIZE_Y = 1 << yOBJ;
    parameter Mn = xOBJ + yOBJ;
    parameter INIT_FILE = "./MIF/spacecraft_32_32_9.mif";

    parameter A = 3'b000;
    parameter B = 3'b001;
    parameter C = 3'b010;
    parameter D = 3'b011;
    parameter E = 3'b100;
    parameter F = 3'b101;
    parameter G = 3'b110;
    parameter H = 3'b111;
    
    input  wire Resetn;
    input  wire Clock;
    input  wire go;
    input  wire ps2_rec;
    input  wire [1:0] dir;
    output wire [nX-1:0] VGA_x;
    output wire [nY-1:0] VGA_y;
    output wire [8:0]    VGA_color;
    output wire          VGA_write;
    output reg           done;

    output wire [nX-1:0] obj_x;
    output wire [nY-1:0] obj_y;

    wire [nX-1:0] X;
    wire [nX-1:0] X0;
    wire [nY-1:0] Y;
    wire [nY-1:0] Y0;
    wire [nX-1:0] size_x;
    wire [nY-1:0] size_y;
    wire [xOBJ-1:0] XC;
    wire [yOBJ-1:0] YC;
    reg  write;
    reg  Lxc;
    reg  Lyc;
    reg  Exc;
    reg  Eyc;
    reg  erase;
    wire Right;
    wire Left;
    wire Up;
    wire Down;
    reg  Lx;
    reg  Ly;
    reg  Ex;
    reg  Ey;
    reg  [2:0] y_Q;
    reg  [2:0] Y_D;
    wire [8:0] obj_color;

    assign X0 = XOFFSET;
    assign Y0 = YOFFSET;
    assign size_x = BOX_SIZE_X;
    assign size_y = BOX_SIZE_Y;

    upDn_count UX (X0, Clock, Resetn, Lx, Ex, Right, X);
        defparam UX.n = nX;
    upDn_count UY (Y0, Clock, Resetn, Ly, Ey, Down, Y);
        defparam UY.n = nY;

    upDn_count U3 ({xOBJ{1'd0}}, Clock, Resetn, Lxc, Exc, 1'b1, XC);
        defparam U3.n = xOBJ;
    upDn_count U4 ({yOBJ{1'd0}}, Clock, Resetn, Lyc, Eyc, 1'b1, YC);
        defparam U4.n = yOBJ;

    assign Left  = (dir == LEFT);
    assign Right = (dir == RIGHT);
    assign Up    = (dir == UP);
    assign Down  = (dir == DOWN);

    always @(*) begin
        case (y_Q)
            A:  Y_D = B;
            B:  if (go)       Y_D = F;
                else if (ps2_rec) Y_D = C;
                else          Y_D = B;
            C:  if (XC != size_x-1) Y_D = C;
                else                Y_D = D;
            D:  if (YC != size_y-1) Y_D = C;
                else                Y_D = E;
            E:  Y_D = F;
            F:  if (XC != size_x-1) Y_D = F;
                else                Y_D = G;
            G:  if (YC != size_y-1) Y_D = F;
                else                Y_D = H;
            H:  if (go) Y_D = H;
                else     Y_D = B;
            default: Y_D = A;
        endcase
    end

    always @(*) begin
        Lx = 1'b0;
        Ly = 1'b0;
        Ex = 1'b0;
        Ey = 1'b0;
        write = 1'b0;
        Lxc = 1'b0;
        Lyc = 1'b0;
        Exc = 1'b0;
        Eyc = 1'b0;
        erase = 1'b0;
        done = 1'b0;
        case (y_Q)
            A:  begin Lx = 1'b1; Ly = 1'b1; end
            B:  begin Lxc = 1'b1; Lyc = 1'b1; end
            C:  begin Exc = 1'b1; write = 1'b1; erase = 1'b1; end
            D:  begin Lxc = 1'b1; Eyc = 1'b1; erase = 1'b1; end
            E:  begin Ex = Right | Left; Ey = Up | Down; end
            F:  begin Exc = 1'b1; write = 1'b1; end
            G:  begin Lxc = 1'b1; Eyc = 1'b1; end
            H:  done = 1'b1;
        endcase
    end

    always @(posedge Clock) begin
        if (!Resetn)
            y_Q <= 3'b0;
        else
            y_Q <= Y_D;
    end

    object_mem U6 ({YC,XC}, Clock, obj_color);
        defparam U6.n         = 9;
        defparam U6.Mn        = xOBJ + yOBJ;
        defparam U6.INIT_FILE = INIT_FILE;

    regn U7 (X - (size_x >> 1) + XC, Resetn, 1'b1, Clock, VGA_x);
        defparam U7.n = nX;
    regn U8 (Y - (size_y >> 1) + YC, Resetn, 1'b1, Clock, VGA_y);
        defparam U8.n = nY;
    regn U9 (write, Resetn, 1'b1, Clock, VGA_write);
        defparam U9.n = 1;

    assign VGA_color = erase ? {9{1'b0}} : obj_color;
    assign obj_x = X;
    assign obj_y = Y;
endmodule

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
    parameter N = 16;

    parameter [8:0] RED   = 9'b111000000;
    parameter [8:0] BLACK = 9'b0; 

    parameter [nX-1:0] Max_X = ScreenX;

    reg [nX-1:0] posX; 
    reg [nY-1:0] posY;

    reg [nX-1:0] XC; 
    reg [nY-1:0] YC; 

    reg erase;
    reg write;

    wire sync;
    reg [N-1:0] cnt;

    reg [3:0] state;
    reg [3:0] next_state;

    localparam IDLE    = 4'b0000;
    localparam INITIAL = 4'b0001;
    localparam DRAW    = 4'b0010;
    localparam WAIT    = 4'b0011;
    localparam ERASE   = 4'b0100;
    localparam MOVE    = 4'b0101;
    localparam DONE    = 4'b0110;

    assign active = (state != IDLE) && (state != DONE);

    always @ (posedge Clock) begin 
        if (!Resetn)
            cnt <= 0;
        else 
            cnt <= cnt + 1'b1;
    end
    
    assign sync = (cnt == {N{1'b1}});

    assign VGA_x     = posX + XC;
    assign VGA_y     = posY + YC;
    assign VGA_color = erase ? BLACK : RED;
    assign VGA_write = write;

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

    always @(posedge Clock) begin
        if (!Resetn) begin
            state <= IDLE;
            posX  <= player_x;
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
                        posX <= player_x;
                        posY <= player_y;
                    end
                end
                INITIAL: begin
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
            endcase
        end
    end
endmodule

module sync(D, Resetn, Clock, Q);
    input  wire D;
    input  wire Resetn;
    input  wire Clock;
    output reg  Q;

    reg Qi;

    always @(posedge Clock) begin
        if (!Resetn) begin
            Qi <= 1'b0;
            Q  <= 1'b0;
        end
        else begin
            Qi <= D;
            Q  <= Qi;
        end
    end
endmodule

module regn(R, Resetn, E, Clock, Q);
    parameter n = 8;
    input  wire [n-1:0] R;
    input  wire         Resetn;
    input  wire         E;
    input  wire         Clock;
    output reg  [n-1:0] Q;

    always @(posedge Clock) begin
        if (!Resetn)
            Q <= {n{1'b0}};
        else if (E)
            Q <= R;
    end
endmodule

module upDn_count(R, Clock, Resetn, L, E, Dir, Q);
    parameter n = 8;
    input  wire [n-1:0] R;
    input  wire         Clock;
    input  wire         Resetn;
    input  wire         L;
    input  wire         E;
    input  wire         Dir;
    output reg  [n-1:0] Q;

    always @(posedge Clock) begin
        if (!Resetn)
            Q <= {n{1'b0}};
        else if (L)
            Q <= R;
        else if (E)
            if (Dir)
                Q <= Q + {{n-1{1'b0}},1'b1};
            else
                Q <= Q - {{n-1{1'b0}},1'b1};
    end
endmodule

module hex7seg(hex, display);
    input  wire [3:0] hex;
    output reg  [6:0] display;

    always @(*) begin
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
    end
endmodule

