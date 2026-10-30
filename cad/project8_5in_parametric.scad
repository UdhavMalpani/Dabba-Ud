// =====================================================================================
// Project 8 - Enclosed Ducted-Fan Quadrotor, 5-inch 6S locked baseline
// Parametric OpenSCAD source (EC4.402, IIIT Hyderabad, Team 4)
//
// Parameters mirror cad/fusion360_parameters_5in.csv and src/cad/generate_cad_stl.py.
// Pick a part with part_to_render, F6 to render, File > Export > STL.
// Box frame (full_assembly): origin = box centre, X forward, Y left (OpenSCAD right-handed), Z up.
// =====================================================================================

/* [Render selector] */
part_to_render = "full_assembly"; // [duct_ring, grille, enclosure_quarter, cross_spine_payload, retractable_leg, full_assembly]

/* [Propeller and duct] */
prop_d            = 127.0;   // Hardware Sync Gate A: caliper-measured prop diameter
tip_clear         = 1.5;     // Stage 0 rings: 1.0 / 1.5 / 2.0
lip_r             = 6.5;     // bellmouth lip radius (0.05 x 130)
wall_t            = 1.6;     // PETG wall
duct_len          = 50.0;    // axial length L
rotor_depth_ratio = 0.30;    // rotor plane 15 mm below inlet
diff_half_angle   = 4.1;     // diffuser half angle (deg)
arm_slot_w        = 16.0;    // carbon arm slot width
arm_slot_h        = 12.0;    // slot height above exit plane
arm_w             = 14.0;    // carbon arm width
arm_t             = 4.0;     // carbon plate thickness
clamp_boss_d      = 7.0;     // duct-to-arm clamp bosses
motor_bolt_sq     = 16.0;    // Emax ECO II 2207: 16 x 16 mm M3 pattern
motor_d           = 28.0;
motor_h           = 20.0;

/* [Box / enclosure] */
motor_offset_a = 97.5;
outer_margin   = 5.9;
box_h          = 100.0;
spine_w        = 45.0;
panel_t        = 0.8;
deck_t         = 1.0;
flange_t       = 1.6;
flange_h       = 10.0;
pocket_clear   = 0.25;
tongue_d       = 0.7;
tongue_h       = 2.8;
tongue_z0      = 3.2;
boss_od        = 6.0;
insert_d       = 4.0;        // M3 heat-set insert pilot
insert_pts     = [[40, 40], [40, 170], [170, 40], [170, 170]];

/* [Grille] */
grille_bar   = 1.0;
grille_pitch = 12.0;
grille_depth = 4.0;
grille_rim   = 3.0;

/* [Payload bay and spine] */
payload_L   = 60.0;
payload_W   = 40.0;
payload_H   = 30.0;
spine_len   = 250.0;
spine_wall  = 1.6;
spine_h     = 30.0;          // X-arm and bay wall height
spine_y_h   = 15.0;          // Y-arm rail height
groove_d    = 0.8;
groove_z0   = 0.9;
groove_h    = 3.2;
cam_win_w   = 20.0;
cam_win_h   = 16.0;
door_t      = 2.0;
hinge_r     = 2.5;

/* [Retractable leg] */
leg_len          = 240.0;
leg_w_root       = 16.0;
leg_w_tip        = 10.0;
leg_t            = 8.0;
foot_len         = 100.0;
foot_w           = 12.0;
horn_boss_d      = 16.0;
horn_boss_h      = 4.0;
horn_hole_d      = 3.0;
leg_deploy_angle = 90;       // 0 = stowed (UP), 90 = deployed (DOWN)

/* [Hidden] */
$fn          = 96;
eps          = 0.01;
bore_d       = prop_d + 2 * tip_clear;
duct_od      = bore_d + 2 * (lip_r + wall_t);
rotor_depth  = duct_len * rotor_depth_ratio;
diff_len     = duct_len - rotor_depth;
exit_d       = bore_d + 2 * diff_len * tan(diff_half_angle);
box_w        = 2 * motor_offset_a + duct_od + 2 * outer_margin;
quarter_w    = box_w / 2;
spine_clear  = 2 * motor_offset_a - duct_od - spine_w;

echo(str("bore_d=", bore_d, " duct_od=", duct_od, " exit_d=", exit_d,
         " box_w=", box_w, " quarter_w=", quarter_w, " spine_clear=", spine_clear));
assert(quarter_w <= 256, "quarter does not fit the Bambu P1S bed");
assert(spine_clear > 0, "ducts collide with the cross spine");

// -------------------------------------------------------------------------------------
// Duct ring: z = 0 exit (on the bed), z = duct_len bellmouth inlet
// -------------------------------------------------------------------------------------
function inner_r(z) = exit_d / 2 - (exit_d / 2 - bore_d / 2) * z / (duct_len - rotor_depth);

function duct_profile_pts() =
    let (rb   = bore_d / 2,
         re   = exit_d / 2,
         ro   = duct_od / 2,
         zrot = duct_len - rotor_depth,
         zcb  = zrot - 0.5,
         zct  = zcb + (ro - (inner_r(zcb) + wall_t)))
    concat(
        [[re, 0], [re + wall_t, 0], [inner_r(zcb) + wall_t, zcb], [ro, zct], [ro, duct_len]],
        [for (a = [90 : 5.625 : 180]) [rb + lip_r + lip_r * cos(a), duct_len - lip_r + lip_r * sin(a)]],
        [[rb, zrot]]
    );

module duct_ring() {
    re = exit_d / 2;
    difference() {
        union() {
            rotate_extrude($fn = 192) polygon(duct_profile_pts());
            // arm clamp bolt bosses either side of the slot (outside the wall)
            for (s = [-1, 1])
                translate([-(re + wall_t + clamp_boss_d / 2 - 1.0), s * (arm_slot_w / 2 + clamp_boss_d / 2), 0])
                    cylinder(d = clamp_boss_d, h = arm_slot_h + 4, $fn = 32);
        }
        // carbon arm slot, facing -X (toward the airframe centre once installed)
        translate([-(duct_od / 2 + 5), -arm_slot_w / 2, -1])
            cube([duct_od / 2 + 5 - (re - 4), arm_slot_w, arm_slot_h + 1]);
        // M3 clamp holes
        for (s = [-1, 1])
            translate([-(re + wall_t + clamp_boss_d / 2 - 1.0), s * (arm_slot_w / 2 + clamp_boss_d / 2), -1])
                cylinder(d = 3.2, h = arm_slot_h + 6, $fn = 24);
    }
}

// Carbon arm end with motor bolt pattern (reference geometry for the assembly)
module carbon_arm(len) {
    difference() {
        translate([-len, -arm_w / 2, 0]) cube([len + motor_bolt_sq / 2 + 4, arm_w, arm_t]);
        for (i = [-1, 1], j = [-1, 1])
            translate([i * motor_bolt_sq / 2, j * motor_bolt_sq / 2, -1]) cylinder(d = 3.2, h = arm_t + 2, $fn = 20);
        translate([0, 0, -1]) cylinder(d = 7, h = arm_t + 2, $fn = 24);
    }
}

// -------------------------------------------------------------------------------------
// Grille: duct_od x duct_od square, 1 mm bars at 12 mm pitch, 3 mm rim
// -------------------------------------------------------------------------------------
module grille() {
    w  = duct_od;
    wi = w - 2 * grille_rim;
    n  = floor((wi / 2) / grille_pitch);
    difference() {
        translate([-w / 2, -w / 2, 0]) cube([w, w, grille_depth]);
        translate([-wi / 2, -wi / 2, -1]) cube([wi, wi, grille_depth + 2]);
    }
    intersection() {
        translate([-wi / 2, -wi / 2, 0]) cube([wi, wi, grille_depth]);
        union() {
            for (k = [-n : n]) {
                translate([k * grille_pitch - grille_bar / 2, -w / 2, 0]) cube([grille_bar, w, grille_depth]);
                translate([-w / 2, k * grille_pitch - grille_bar / 2, 0]) cube([w, grille_bar, grille_depth]);
            }
        }
    }
}

// -------------------------------------------------------------------------------------
// Enclosure quarter (+X,+Y): [0, quarter_w]^2 x [0, box_h]; (0,0) lies on the box centre line.
// Print 2 as-is and 2 mirrored.
// -------------------------------------------------------------------------------------
module enclosure_quarter() {
    qw = quarter_w;
    s2 = spine_w / 2;
    xe = qw - panel_t;
    pr = duct_od / 2 + pocket_clear;
    difference() {
        union() {
            cube([qw, qw, deck_t]);                                          // bottom deck
            translate([0, 0, box_h - deck_t]) cube([qw, qw, deck_t]);        // top deck
            translate([qw - panel_t, 0, 0]) cube([panel_t, qw, box_h]);      // side panel X
            translate([0, qw - panel_t, 0]) cube([qw - panel_t, panel_t, box_h]); // side panel Y
            // spine joint flange rails (bottom + top) along both inner edges
            for (z0 = [deck_t, box_h - deck_t - flange_h]) {
                translate([s2, s2, z0]) cube([xe - s2, flange_t, flange_h]);
                translate([s2, s2 + flange_t, z0]) cube([flange_t, xe - s2 - flange_t, flange_h]);
            }
            // tongues (engage the grooves on the spine walls)
            translate([30, s2 - tongue_d, deck_t + tongue_z0]) cube([spine_len / 2 - 35, tongue_d + eps, tongue_h]);
            translate([s2 - tongue_d, 30, deck_t + tongue_z0]) cube([tongue_d + eps, spine_len / 2 - 35, tongue_h]);
            // M3 heat-set insert bosses (full-height pillars double as ribs)
            for (p = insert_pts) translate([p[0], p[1], 0]) cylinder(d = boss_od, h = box_h, $fn = 32);
        }
        // duct pocket through both decks
        translate([motor_offset_a, motor_offset_a, -1]) cylinder(r = pr, h = box_h + 2, $fn = 192);
        // payload bay door opening (corner of the bottom deck)
        translate([-1, -1, -1]) cube([payload_L / 2 + 1, payload_W / 2 + 1, deck_t + 2]);
        // insert pilot holes through decks and bosses
        for (p = insert_pts) translate([p[0], p[1], -1]) cylinder(d = insert_d, h = box_h + 2, $fn = 24);
        // half of the RunCam side-panel aperture (front panel X, at y = 0 edge)
        translate([qw - panel_t - 1, -1, 3 + spine_h / 2 - cam_win_h / 2])
            cube([panel_t + 2, cam_win_w / 2 + 1, cam_win_h]);
    }
}

// -------------------------------------------------------------------------------------
// Cross spine + 60 x 40 x 30 payload bay; z = 0 at bay floor / door plane
// -------------------------------------------------------------------------------------
module cross_spine_payload() {
    s2 = spine_w / 2;
    L  = spine_len / 2;
    yi = s2 - spine_wall;
    bx = payload_L / 2;
    difference() {
        union() {
            // X-arm side walls (continuous through the centre)
            for (s = [-1, 1])
                translate([-L, (s > 0) ? yi : -s2, 0]) cube([2 * L, spine_wall, spine_h]);
            // Y-arm rails
            for (s = [-1, 1], t = [-1, 1])
                translate([(s > 0) ? yi : -s2, (t > 0) ? s2 : -L, 0]) cube([spine_wall, L - s2, spine_y_h]);
            // servo bulkheads at X-arm ends
            for (s = [-1, 1])
                translate([(s > 0) ? L - spine_wall : -L, -yi, 0]) cube([spine_wall, 2 * yi, spine_h]);
            // payload bay end walls
            for (s = [-1, 1])
                translate([(s > 0) ? bx : -bx - spine_wall, -yi, 0]) cube([spine_wall, 2 * yi, spine_h]);
            // door hinge knuckles on the aft bay wall
            for (t = [-1, 1])
                translate([-bx - spine_wall - hinge_r + 1.0, t * 14 - 4, hinge_r])
                    rotate([-90, 0, 0]) cylinder(r = hinge_r, h = 8, $fn = 24);
        }
        // tongue grooves: X-arm outer faces
        for (s = [-1, 1])
            translate([-L - 1, (s > 0) ? s2 - groove_d : -s2 - 1, groove_z0]) cube([2 * L + 2, groove_d + 1, groove_h]);
        // tongue grooves: Y-arm outer faces (outside the X-arm only)
        for (s = [-1, 1], t = [-1, 1])
            translate([(s > 0) ? s2 - groove_d : -s2 - 1, (t > 0) ? s2 + eps : -L - 1, groove_z0])
                cube([groove_d + 1, L - s2 + 1, groove_h]);
        // RunCam Thumb window in the front bay wall
        translate([bx - 1, -cam_win_w / 2, (spine_h - cam_win_h) / 2]) cube([spine_wall + 2, cam_win_w, cam_win_h]);
        // hinge pin hole
        translate([-bx - spine_wall - hinge_r + 1.0, -L, hinge_r]) rotate([-90, 0, 0]) cylinder(d = 1.8, h = 2 * L, $fn = 16);
    }
}

// Flush hinged payload door (separate print, shown as an outline in the assembly)
module payload_door() {
    difference() {
        translate([-payload_L / 2, -payload_W / 2, 0]) cube([payload_L, payload_W, door_t]);
        translate([payload_L / 2 - 6, -4, -1]) cube([3, 8, door_t + 2]);   // latch slot
    }
}

// -------------------------------------------------------------------------------------
// Retractable leg: pivot at origin, leg along +X, lateral foot along Y, printed flat
// -------------------------------------------------------------------------------------
function leg_outline() =
    concat(
        [[0, -leg_w_root / 2], [leg_len - foot_w, -leg_w_tip / 2],
         [leg_len - foot_w, -foot_len / 2], [leg_len, -foot_len / 2],
         [leg_len, foot_len / 2], [leg_len - foot_w, foot_len / 2],
         [leg_len - foot_w, leg_w_tip / 2], [0, leg_w_root / 2]],
        [for (a = [97.5 : 7.5 : 262.5]) [leg_w_root / 2 * cos(a), leg_w_root / 2 * sin(a)]]
    );

module retractable_leg() {
    difference() {
        union() {
            linear_extrude(height = leg_t) polygon(leg_outline());
            translate([0, 0, leg_t]) cylinder(d = horn_boss_d, h = horn_boss_h, $fn = 64);   // servo horn boss
        }
        translate([0, 0, -1]) cylinder(d = horn_hole_d, h = leg_t + horn_boss_h + 2, $fn = 20);
        // MG90S horn spline/arm recess in the boss top
        translate([0, 0, leg_t + horn_boss_h - 1.5]) hull() {
            cylinder(d = 7, h = 2, $fn = 24);
            translate([12, 0, 0]) cylinder(d = 4, h = 2, $fn = 16);
        }
    }
}

// -------------------------------------------------------------------------------------
// Full assembly (box centre origin, Z up)
// -------------------------------------------------------------------------------------
module place_quarter(sx, sy) {
    if (sx > 0 && sy > 0) children();
    else if (sx < 0 && sy > 0) mirror([1, 0, 0]) children();
    else if (sx < 0 && sy < 0) mirror([1, 0, 0]) mirror([0, 1, 0]) children();
    else mirror([0, 1, 0]) children();
}

module full_assembly() {
    a  = motor_offset_a;
    zx = -duct_len / 2;                   // duct exit plane (-25)
    zf = zx + arm_slot_h - arm_t;         // carbon frame bottom (-17)
    // enclosure quarters
    for (q = [[1, 1], [-1, 1], [-1, -1], [1, -1]])
        color("lightgrey", 0.25) place_quarter(q[0], q[1]) translate([0, 0, -box_h / 2]) enclosure_quarter();
    // ducts, grilles, motors, props, arms
    for (q = [[1, 1], [-1, 1], [-1, -1], [1, -1]]) {
        ang = atan2(-q[1], -q[0]);        // slot faces the centre
        translate([q[0] * a, q[1] * a, zx]) rotate([0, 0, ang - 180]) color("SteelBlue", 0.8) duct_ring();
        translate([q[0] * a, q[1] * a, box_h / 2 - grille_depth]) color("DimGray") grille();
        translate([q[0] * a, q[1] * a, -box_h / 2]) color("DimGray") grille();
        translate([q[0] * a, q[1] * a, zf + arm_t]) color("DarkOrange") cylinder(d = motor_d, h = motor_h);
        translate([q[0] * a, q[1] * a, duct_len / 2 - rotor_depth]) color("Gold", 0.3) cylinder(d = prop_d, h = 1);
        translate([q[0] * a, q[1] * a, zf]) rotate([0, 0, atan2(q[1], q[0])]) color("Black") carbon_arm(a * sqrt(2) - 20);
    }
    color("Black") translate([0, 0, zf + arm_t / 2]) cube([70, 70, arm_t], center = true);
    // spine + payload bay + door
    translate([0, 0, -box_h / 2 + 3]) color("Tan") cross_spine_payload();
    translate([0, 0, -box_h / 2]) color("Tan", 0.6) %payload_door();
    // battery, RunCam, ballast
    color("RoyalBlue", 0.7) translate([0, 0, 4 + 20]) cube([135, 43, 40], center = true);
    color("Red") translate([30, 0, -32]) cube([13, 20, 20], center = true);
    color("Silver") translate([10.3, 3.0, -32]) cube([24, 32, 28], center = true);
    // GPS mast
    color("Black") translate([0, 0, box_h / 2]) cylinder(d = 8, h = 45);
    color("White") translate([0, 0, box_h / 2 + 45]) cube([25, 25, 8], center = true);
    // retractable legs (front leg folds aft, rear leg folds forward; 8 mm nesting stagger)
    L = spine_len / 2;
    color("MediumPurple") translate([L, 12, -54]) rotate([0, -leg_deploy_angle, 0]) rotate([0, 0, 180])
        translate([0, 0, -leg_t / 2]) retractable_leg();
    color("MediumPurple") translate([-L, -12, -62]) rotate([0, leg_deploy_angle, 0])
        translate([0, 0, -leg_t / 2]) retractable_leg();
}

// -------------------------------------------------------------------------------------
// Selector
// -------------------------------------------------------------------------------------
if (part_to_render == "duct_ring")                duct_ring();
else if (part_to_render == "grille")              grille();
else if (part_to_render == "enclosure_quarter")   enclosure_quarter();
else if (part_to_render == "cross_spine_payload") cross_spine_payload();
else if (part_to_render == "retractable_leg")     retractable_leg();
else                                              full_assembly();
