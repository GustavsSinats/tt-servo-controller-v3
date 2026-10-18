`default_nettype none

module tt_um_gustavssinats_servo_controller_v3 (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,

    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,

    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    // =========================================================
    // CLOCK: 25 MHz
    //
    // Buttons, active high:
    // ui_in[0] = hold to decrease angle
    // ui_in[1] = hold to increase angle
    // ui_in[2] = press to return gradually to centre
    //
    // Servo:
    // uio_out[0] = servo PWM
    //
    // VGA:
    // uo_out[0] = red high bit, unused
    // uo_out[1] = green high bit
    // uo_out[2] = blue high bit
    // uo_out[3] = vertical sync
    // uo_out[4] = red low bit, unused
    // uo_out[5] = green low bit
    // uo_out[6] = blue low bit
    // uo_out[7] = horizontal sync
    //
    // Internal target 0..180 = displayed angle -90..+90.
    // Centre is internal 90, displayed 0.
    //
    // Display shows commanded position, not measured position.
    // =========================================================

    // ---------------------------------------------------------
    // Microsecond clock enable.
    // ---------------------------------------------------------

    reg [4:0] us_divider;
    wire us_tick;

    assign us_tick = (us_divider == 5'd24);

    always @(posedge clk) begin
        if (!rst_n)
            us_divider <= 5'd0;
        else if (us_tick)
            us_divider <= 5'd0;
        else
            us_divider <= us_divider + 5'd1;
    end

    // ---------------------------------------------------------
    // Button synchronization.
    // ---------------------------------------------------------

    reg [2:0] buttons_meta;
    reg [2:0] buttons_sync;

    always @(posedge clk) begin
        if (!rst_n) begin
            buttons_meta <= 3'b000;
            buttons_sync <= 3'b000;
        end else begin
            buttons_meta <= ui_in[2:0];
            buttons_sync <= buttons_meta;
        end
    end

    // ---------------------------------------------------------
    // Button debounce: approximately 10 ms.
    // ---------------------------------------------------------

    reg [2:0] candidate_buttons;
    reg [2:0] debounced_buttons;
    reg [13:0] debounce_count;

    always @(posedge clk) begin
        if (!rst_n || !ena) begin
            candidate_buttons <= 3'b000;
            debounced_buttons <= 3'b000;
            debounce_count    <= 14'd0;

        end else if (us_tick) begin
            if (buttons_sync != candidate_buttons) begin
                candidate_buttons <= buttons_sync;
                debounce_count <= 14'd0;

            end else if (debounce_count < 14'd9999) begin
                debounce_count <= debounce_count + 14'd1;

            end else begin
                debounced_buttons <= candidate_buttons;
            end
        end
    end

    // ---------------------------------------------------------
    // Angle control.
    //
    // One degree every 20 ms.
    // Centre-button rising edge starts automatic return.
    // Direction buttons are ignored during return.
    // ---------------------------------------------------------

    reg [7:0] target_angle;
    reg [14:0] move_count;

    reg returning_to_centre;
    reg centre_button_d;

    wire centre_pressed;

    assign centre_pressed =
        debounced_buttons[2] && !centre_button_d;

    always @(posedge clk) begin
        if (!rst_n || !ena) begin
            target_angle        <= 8'd90;
            move_count          <= 15'd0;
            returning_to_centre <= 1'b0;
            centre_button_d     <= 1'b0;

        end else begin
            centre_button_d <= debounced_buttons[2];

            if (centre_pressed) begin
                move_count <= 15'd0;
                returning_to_centre <= (target_angle != 8'd90);

            end else if (us_tick) begin
                if (returning_to_centre) begin
                    if (target_angle == 8'd90) begin
                        returning_to_centre <= 1'b0;
                        move_count <= 15'd0;

                    end else if (move_count == 15'd19999) begin
                        move_count <= 15'd0;

                        if (target_angle < 8'd90) begin
                            target_angle <= target_angle + 8'd1;

                            if (target_angle == 8'd89)
                                returning_to_centre <= 1'b0;

                        end else begin
                            target_angle <= target_angle - 8'd1;

                            if (target_angle == 8'd91)
                                returning_to_centre <= 1'b0;
                        end

                    end else begin
                        move_count <= move_count + 15'd1;
                    end

                end else if (debounced_buttons[2]) begin
                    // Hold at centre while centre button is held.
                    move_count <= 15'd0;

                end else if (
                    debounced_buttons[0] ^
                    debounced_buttons[1]
                ) begin
                    if (move_count == 15'd19999) begin
                        move_count <= 15'd0;

                        if (debounced_buttons[0]) begin
                            if (target_angle > 8'd0)
                                target_angle <= target_angle - 8'd1;
                        end else begin
                            if (target_angle < 8'd180)
                                target_angle <= target_angle + 8'd1;
                        end

                    end else begin
                        move_count <= move_count + 15'd1;
                    end

                end else begin
                    // No direction, or both directions:
                    // retain target.
                    move_count <= 15'd0;
                end
            end
        end
    end

    // ---------------------------------------------------------
    // Servo pulse calculation.
    //
    // Internal 0   -> 1000 us
    // Internal 90  -> 1500 us
    // Internal 180 -> 2000 us
    // ---------------------------------------------------------

    wire [15:0] pulse_calculated;
    wire [14:0] requested_pulse_us;

    assign pulse_calculated =
        16'd1000 +
        (
            (
                ({8'd0, target_angle} * 16'd50) +
                16'd4
            ) / 16'd9
        );

    assign requested_pulse_us = pulse_calculated[14:0];

    // ---------------------------------------------------------
    // Servo PWM: 20 ms frame.
    // Pulse width changes only between frames.
    // ---------------------------------------------------------

    reg [14:0] frame_us;
    reg [14:0] active_pulse_us;

    wire servo_pwm;

    always @(posedge clk) begin
        if (!rst_n || !ena) begin
            frame_us        <= 15'd0;
            active_pulse_us <= 15'd1500;

        end else if (us_tick) begin
            if (frame_us == 15'd19999) begin
                frame_us <= 15'd0;
                active_pulse_us <= requested_pulse_us;
            end else begin
                frame_us <= frame_us + 15'd1;
            end
        end
    end

    assign servo_pwm =
        rst_n &&
        ena &&
        (frame_us < active_pulse_us);

    // ---------------------------------------------------------
    // VGA timing: 640 x 480 active, 800 x 525 total.
    // ---------------------------------------------------------

    reg [9:0] pixel_x;
    reg [9:0] pixel_y;

    wire hsync;
    wire vsync;
    wire visible;

    always @(posedge clk) begin
        if (!rst_n) begin
            pixel_x <= 10'd0;
            pixel_y <= 10'd0;

        end else if (pixel_x == 10'd799) begin
            pixel_x <= 10'd0;

            if (pixel_y == 10'd524)
                pixel_y <= 10'd0;
            else
                pixel_y <= pixel_y + 10'd1;

        end else begin
            pixel_x <= pixel_x + 10'd1;
        end
    end

    assign hsync = !(
        pixel_x >= 10'd656 &&
        pixel_x < 10'd752
    );

    assign vsync = !(
        pixel_y >= 10'd490 &&
        pixel_y < 10'd492
    );

    assign visible =
        pixel_x < 10'd640 &&
        pixel_y < 10'd480;

    // ---------------------------------------------------------
    // Latch display position during vertical blanking.
    // ---------------------------------------------------------

    reg [7:0] display_angle;

    always @(posedge clk) begin
        if (!rst_n)
            display_angle <= 8'd90;
        else if (
            pixel_x == 10'd0 &&
            pixel_y == 10'd480
        )
            display_angle <= target_angle;
    end

    wire signed [8:0] displayed_degrees;

    assign displayed_degrees =
        $signed({1'b0, display_angle}) - 9'sd90;

    wire negative;
    wire [7:0] magnitude;

    assign negative = display_angle < 8'd90;

    assign magnitude =
        negative ?
        (8'd90 - display_angle) :
        (display_angle - 8'd90);

    // ---------------------------------------------------------
    // Rounded bar.
    // x = 120..519, y = 300..339.
    // ---------------------------------------------------------

    wire in_bar;
    wire [9:0] bar_row;

    reg [4:0] end_inset;

    assign in_bar =
        pixel_x >= 10'd120 &&
        pixel_x < 10'd520 &&
        pixel_y >= 10'd300 &&
        pixel_y < 10'd340;

    assign bar_row = pixel_y - 10'd300;

    always @(*) begin
        case (bar_row)
            10'd0,  10'd39: end_inset = 5'd16;
            10'd1,  10'd38: end_inset = 5'd12;
            10'd2,  10'd37: end_inset = 5'd10;
            10'd3,  10'd36: end_inset = 5'd9;
            10'd4,  10'd35: end_inset = 5'd7;
            10'd5,  10'd34: end_inset = 5'd6;
            10'd6,  10'd33: end_inset = 5'd5;
            10'd7,  10'd32: end_inset = 5'd4;
            10'd8,  10'd31: end_inset = 5'd3;
            10'd9,  10'd30: end_inset = 5'd3;
            10'd10, 10'd29: end_inset = 5'd2;
            10'd11, 10'd28: end_inset = 5'd2;
            10'd12, 10'd27: end_inset = 5'd1;
            10'd13, 10'd26: end_inset = 5'd1;
            10'd14, 10'd25: end_inset = 5'd1;
            default:       end_inset = 5'd0;
        endcase
    end

    wire [9:0] bar_left_edge;
    wire [9:0] bar_right_edge;

    wire bar_shape;
    wire bar_border;
    wire bar_fill;

    assign bar_left_edge =
        10'd120 + {5'd0, end_inset};

    assign bar_right_edge =
        10'd520 - {5'd0, end_inset};

    assign bar_shape =
        in_bar &&
        pixel_x >= bar_left_edge &&
        pixel_x < bar_right_edge;

    assign bar_border =
        bar_shape &&
        (
            pixel_y < 10'd302 ||
            pixel_y >= 10'd338 ||
            pixel_x < (bar_left_edge + 10'd2) ||
            pixel_x >= (bar_right_edge - 10'd2)
        );

    wire [15:0] fill_width;
    wire [15:0] fill_end;

    assign fill_width =
        (
            ({8'd0, display_angle} * 16'd20) +
            16'd4
        ) / 16'd9;

    assign fill_end = 16'd120 + fill_width;

    assign bar_fill =
        bar_shape &&
        !bar_border &&
        ({6'd0, pixel_x} < fill_end);

    // ---------------------------------------------------------
    // Numerical display: -090 through +090.
    // ---------------------------------------------------------

    wire [7:0] tens_value;
    wire [7:0] units_value;

    assign tens_value  = magnitude / 8'd10;
    assign units_value = magnitude % 8'd10;

    // Segment order: {g,f,e,d,c,b,a}.
    function [6:0] digit_segments;
        input [3:0] digit;
        begin
            case (digit)
                4'd0: digit_segments = 7'h3F;
                4'd1: digit_segments = 7'h06;
                4'd2: digit_segments = 7'h5B;
                4'd3: digit_segments = 7'h4F;
                4'd4: digit_segments = 7'h66;
                4'd5: digit_segments = 7'h6D;
                4'd6: digit_segments = 7'h7D;
                4'd7: digit_segments = 7'h07;
                4'd8: digit_segments = 7'h7F;
                4'd9: digit_segments = 7'h6F;
                default: digit_segments = 7'h00;
            endcase
        end
    endfunction

    function digit_pixel;
        input [9:0] x;
        input [9:0] y;
        input [6:0] seg;
        begin
            digit_pixel =
                (seg[0] &&
                 x >= 10'd4 && x < 10'd28 &&
                 y < 10'd4) ||

                (seg[1] &&
                 x >= 10'd28 && x < 10'd32 &&
                 y >= 10'd4 && y < 10'd26) ||

                (seg[2] &&
                 x >= 10'd28 && x < 10'd32 &&
                 y >= 10'd30 && y < 10'd52) ||

                (seg[3] &&
                 x >= 10'd4 && x < 10'd28 &&
                 y >= 10'd52 && y < 10'd56) ||

                (seg[4] &&
                 x < 10'd4 &&
                 y >= 10'd30 && y < 10'd52) ||

                (seg[5] &&
                 x < 10'd4 &&
                 y >= 10'd4 && y < 10'd26) ||

                (seg[6] &&
                 x >= 10'd4 && x < 10'd28 &&
                 y >= 10'd26 && y < 10'd30);
        end
    endfunction

    wire text_row;
    wire digit0_on;
    wire digit1_on;
    wire digit2_on;
    wire sign_on;
    wire degree_on;

    assign text_row =
        pixel_y >= 10'd220 &&
        pixel_y < 10'd276;

    assign digit0_on =
        text_row &&
        pixel_x >= 10'd268 &&
        pixel_x < 10'd300 &&
        digit_pixel(
            pixel_x - 10'd268,
            pixel_y - 10'd220,
            digit_segments(4'd0)
        );

    assign digit1_on =
        text_row &&
        pixel_x >= 10'd308 &&
        pixel_x < 10'd340 &&
        digit_pixel(
            pixel_x - 10'd308,
            pixel_y - 10'd220,
            digit_segments(tens_value[3:0])
        );

    assign digit2_on =
        text_row &&
        pixel_x >= 10'd348 &&
        pixel_x < 10'd380 &&
        digit_pixel(
            pixel_x - 10'd348,
            pixel_y - 10'd220,
            digit_segments(units_value[3:0])
        );

    assign sign_on =
        (
            pixel_x >= 10'd238 &&
            pixel_x < 10'd260 &&
            pixel_y >= 10'd246 &&
            pixel_y < 10'd250
        ) ||
        (
            !negative &&
            pixel_x >= 10'd247 &&
            pixel_x < 10'd251 &&
            pixel_y >= 10'd237 &&
            pixel_y < 10'd259
        );

    assign degree_on =
        pixel_x >= 10'd388 &&
        pixel_x < 10'd400 &&
        pixel_y >= 10'd220 &&
        pixel_y < 10'd232 &&
        (
            pixel_x < 10'd391 ||
            pixel_x >= 10'd397 ||
            pixel_y < 10'd223 ||
            pixel_y >= 10'd229
        );

    // ---------------------------------------------------------
    // Cyan text/border, green fill, black background.
    // Display remains visible when ena is low.
    // ---------------------------------------------------------

    wire text_on;
    wire screen_enable;

    wire green;
    wire blue;

    assign text_on =
        digit0_on ||
        digit1_on ||
        digit2_on ||
        sign_on ||
        degree_on;

    assign screen_enable = visible && rst_n;

    assign green =
        screen_enable &&
        (text_on || bar_border || bar_fill);

    assign blue =
        screen_enable &&
        (text_on || bar_border);

    // ---------------------------------------------------------
    // VGA output mapping.
    // Both red bits are zero: no servo signal in the picture.
    // ---------------------------------------------------------

    assign uo_out[0] = 1'b0;
    assign uo_out[1] = green;
    assign uo_out[2] = blue;
    assign uo_out[3] = vsync;
    assign uo_out[4] = 1'b0;
    assign uo_out[5] = green;
    assign uo_out[6] = blue;
    assign uo_out[7] = hsync;

    // ---------------------------------------------------------
    // Servo PWM on uio_out[0]; remaining uio are inputs.
    // ---------------------------------------------------------

    assign uio_out = {7'b0, servo_pwm};
    assign uio_oe  = 8'b0000_0001;

endmodule
