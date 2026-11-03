// =====================================================================================
// Project 8 - PRE-COMPONENT single-duct thrust rig (branch pre-component-test)
//
// One motor + one prop inside one printed duct, standing on a kitchen scale.
// Air blows UP out of the duct exit, so thrust pushes the rig DOWN onto the scale.
//
//   part = "fit_band"   20-min ring with the duct's throat bore: check the prop clears it
//   part = "stand"      tube cradle (windowed) + arm post; duct drops in inlet-down
//   part = "motor_arm"  arm through the duct's exit slot, motor hangs under the pad
//   part = "open_riser" 40 mm block under the arm for the OPEN-prop baseline run
//   part = "inlet_grille" grille trimmed round: slides up the tube, sits under the ledge
//                       (the exit grille is the normal square one, taped on top of the arm)
//   part = "assembly"   everything in place (needs outputs/test_rig/duct_test_rig.stl)
//
// Do not edit the numbers here: fill config/test_rig_measurements.json and run
//   python src/cad/generate_test_rig.py
// which writes outputs/test_rig/test_rig_dims.scad (included below, overrides defaults)
// and exports every part to outputs/test_rig/*.stl.
// =====================================================================================

$fn = 96;
part = "assembly";

// ---- measured inputs: defaults, overridden by the generated include ----
prop_d         = 130.2;   // prop diameter (2 x shaft centre to furthest tip); 3 x 65.1 mm blades
tip_gap        = 1.5;     // radial blade-tip gap
motor_bolt_min = 16;      // motor hole spacing, adjacent centres (min = max when measured)
motor_bolt_max = 19;
motor_seat_h   = 30;      // motor base -> surface the prop hub sits on
prop_hub_t     = 7;       // prop hub thickness
motor_bell_d   = 34;
motor_center_d = 10;      // clearance for the shaft end / circlip under the motor
inlet_clear    = 150;     // scale surface -> duct inlet plane (>= 1 prop diameter)

include <../outputs/test_rig/test_rig_dims.scad>

// ---- duct constants (identical to src/cad/generate_cad_stl.py) ----
lip_r = 6.5; wall_t = 1.6; duct_len = 50; rotor_depth = 15; diff_half = 4.1;
slot_w = 16; slot_h = 12;

// ---- derived duct geometry (depths measured from the exit plane into the duct) ----
bore      = prop_d + 2 * tip_gap;
rb        = bore / 2;
ro        = rb + lip_r + wall_t;                  // collar (outer) radius
throat_lo = duct_len - rotor_depth;               // 35: rotor plane, diffuser starts below it
throat_hi = duct_len - lip_r;                     // 43.5: bellmouth starts above it
prop_depth = throat_lo + 3;                       // prop plane sits 3 mm into the straight throat

// ---- rig layout (world z = 0 on the scale) ----
arm_h = 12; arm_w = 14; pad_floor = 5;
pad   = motor_bolt_max + 19;
arm_bot_raw = prop_depth - motor_seat_h - prop_hub_t / 2;   // arm underside depth below exit plane
arm_bot  = min(arm_bot_raw, slot_h - 0.5);                  // must stay inside the 12 mm slot
spacer_h = max(0, arm_bot_raw - arm_bot);                   // short motor: drop the pad on a spacer
base_t = 4;
H_in   = inlet_clear;                       // inlet plane
exit_z = H_in + duct_len;
arm_z0 = exit_z - arm_bot;                  // arm underside (world)

tube_ri = ro + 0.3; tube_t = 3; tube_ro = tube_ri + tube_t;
cradle_h = 8; ledge_t = 2; ledge_ri = ro - 1.2;   // ledge only touches the flat rim outside the lip
tube_top = H_in + cradle_h;
post_l = 24; post_w = 16;                  // arm post: radial x tangential
post_r = tube_ro + post_l / 2 - 2;
arm_len = post_r + post_l / 2;
hole_off = 6;
riser_h = 40;

// windows: 3 x pointed (45 deg roof -> no supports), columns at 60/180/300 deg
col_w   = 22;
win_arc = 2 * PI * tube_ro / 3 - col_w;
win_w   = 2 * tube_ro * sin((win_arc / tube_ro) * 180 / PI / 2);
win_zb  = base_t + 6;
win_zt  = H_in - ledge_t - 6;
win_zv  = win_zt - win_w / 2;

module shell(ri, rout, z0, z1)
    translate([0, 0, z0]) difference() {
        cylinder(r = rout, h = z1 - z0);
        translate([0, 0, -1]) cylinder(r = ri, h = z1 - z0 + 2);
    }

module window_cutter()
    translate([tube_ri - 5, 0, 0]) rotate([90, 0, 90])
        linear_extrude(height = tube_t + 10)
            polygon([[-win_w / 2, win_zb], [win_w / 2, win_zb], [win_w / 2, win_zv],
                     [0, win_zt], [-win_w / 2, win_zv]]);

module pilot_holes(z_top, depth) {
    for (dx = [-hole_off, 0, hole_off])
        translate([-post_r + dx, 0, z_top - depth]) cylinder(d = 2.6, h = depth + 1, $fn = 24);
}

module stand() {
    difference() {
        union() {
            shell(tube_ri, tube_ro, 0, tube_top);
            shell(tube_ri, tube_ro + 15, 0, base_t);
            shell(ledge_ri, tube_ri + 0.01, H_in - ledge_t, H_in);
            translate([-post_r - post_l / 2, -post_w / 2, 0]) cube([post_l, post_w, arm_z0]);
        }
        for (a = [0, 120, 240]) rotate([0, 0, a]) window_cutter();
        translate([0, 0, H_in]) cylinder(r = tube_ri, h = 400);
        pilot_holes(arm_z0, 14);
    }
}

module motor_arm_world() {
    difference() {
        union() {
            translate([-arm_len, -arm_w / 2, arm_z0]) cube([arm_len, arm_w, arm_h]);
            translate([-pad / 2, -pad / 2, arm_z0]) cube([pad, pad, arm_h]);
            if (spacer_h > 0)
                translate([0, 0, arm_z0 - spacer_h]) cylinder(d = pad - 4, h = spacer_h + 0.01);
        }
        translate([0, 0, arm_z0 - spacer_h - 1]) cylinder(d = motor_center_d, h = arm_h + spacer_h + 2);
        for (a = [45, 135, 225, 315]) rotate([0, 0, a]) {
            hull() for (s = [motor_bolt_min, motor_bolt_max])
                translate([s / sqrt(2), 0, arm_z0 - spacer_h - 1]) cylinder(d = 3.4, h = arm_h + spacer_h + 2, $fn = 24);
            hull() for (s = [motor_bolt_min, motor_bolt_max])
                translate([s / sqrt(2), 0, arm_z0 + pad_floor]) cylinder(d = 6.5, h = arm_h, $fn = 32);
        }
        for (dx = [-hole_off, hole_off])
            translate([-post_r + dx, 0, arm_z0 - 1]) cylinder(d = 3.4, h = arm_h + 2, $fn = 24);
    }
}

// printed upside down: arm top face on the bed, spacer (if any) points up
module motor_arm_print()
    translate([0, 0, arm_z0 + arm_h]) mirror([0, 0, 1]) motor_arm_world();

// riser in world position (sits on the post top), and its print pose (top face on the bed)
module open_riser_world() {
    difference() {
        union() {
            translate([-post_r - post_l / 2, -post_w / 2, arm_z0]) cube([post_l, post_w, riser_h]);
            for (dx = [-hole_off, hole_off])        // pegs locate in the post's pilot holes
                translate([-post_r + dx, 0, arm_z0 - 4]) cylinder(d = 2.3, h = 4.01, $fn = 24);
        }
        pilot_holes(arm_z0 + riser_h, 14);
        translate([-post_r, 0, arm_z0 - 1]) cylinder(d = 3.4, h = riser_h + 2, $fn = 24);
        translate([-post_r, 0, arm_z0 + 8]) cylinder(d = 6.5, h = riser_h, $fn = 32);
    }
}
module open_riser_print()
    translate([0, 0, arm_z0 + riser_h]) mirror([0, 0, 1]) open_riser_world();

module fit_band() shell(rb, rb + 2, 0, 8);

// square grille trimmed round so it slides up the tube and sits under the ledge (airflow holds it there)
module inlet_grille_round() {
    r = tube_ri - 0.4;
    intersection() {
        import("../outputs/test_rig/grille_test_rig.stl", convexity = 10);
        translate([0, 0, -1]) cylinder(r = r, h = 10, $fn = 192);
    }
    shell(r - 3, r, 0, 4);
}

module assembly() {
    color("lightgray") stand();
    color("orange", 0.85) translate([0, 0, H_in + duct_len]) rotate([180, 0, 0])
        import("../outputs/test_rig/duct_test_rig.stl");
    color("steelblue") motor_arm_world();
    m_base = arm_z0 - spacer_h;
    color("dimgray") translate([0, 0, m_base - motor_seat_h]) cylinder(d = motor_bell_d, h = motor_seat_h);
    color("green", 0.4) translate([0, 0, exit_z - prop_depth - 1]) cylinder(d = prop_d, h = 2);
}

if (part == "fit_band")        fit_band();
else if (part == "inlet_grille") inlet_grille_round();
else if (part == "stand")      stand();
else if (part == "motor_arm")  motor_arm_print();
else if (part == "open_riser") open_riser_print();
else                           assembly();
