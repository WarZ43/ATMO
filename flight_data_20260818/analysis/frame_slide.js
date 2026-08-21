// Regenerates frame_slide.pptx ("what was actually wrong": two frame errors).
//
//   cd ~/ATMO/flight_data_20260818/analysis
//   node frame_slide.js          # writes frame_slide.pptx here
//
// Styled to match SURF Draft v2: Cambria titles on 1B2A3A, Calibri body on
// 3A4654, brick red C0392B / teal 1C7293 accents, navy table header. Keep this
// in step with the deck if the deck's theme changes.
//
// The 180 deg mocap mount is stated as a premise, not argued: the rig and the
// flight controller visibly face opposite ways on the robot. frame/mount.png
// (the measured 178.6-179.9 deg bar chart) is therefore NOT embedded -- it is
// still generated, and lives in the handoff for anyone who wants the numbers.
//
// Figures it embeds (regenerate with: python analysis/frame_figs.py):
//   frame/roll_feedback.png  policy roll command vs roll angle, + plant sign
//   frame/frames.png         the three body frames and the two errors
const path = require("path");
const pptxgen = require("pptxgenjs");
const HERE = __dirname;

// --- deck theme ----------------------------------------------------------
const NAVY = "1B2A3A";   // titles, table header fill
const INK = "3A4654";    // body text
const MUTE = "7A8794";   // captions
const RED = "C0392B";    // accent / failure
const TEAL = "1C7293";   // accent / section numbers
const GREEN = "1E7A4C";  // "correct"
const RULE = "DDE3EA";   // hairlines
const BAND = "F4F6F9";   // alternating table row
const TITLE_FONT = "Cambria";
const BODY_FONT = "Calibri";

const p = new pptxgen();
p.layout = "LAYOUT_16x9"; // 10 x 5.625 in
const s = p.addSlide();
s.background = { color: "FFFFFF" };

s.addText("Two Frame Errors Flipped the Axes — Not the Mixer", {
  x: 0.5, y: 0.16, w: 9.1, h: 0.5, fontFace: TITLE_FONT, fontSize: 26,
  color: NAVY, bold: true, margin: 0, valign: "middle",
});
s.addText("ATMO 2026-08-18 flight 3 (log_417) · measured from the bags and the deployed runtime", {
  x: 0.5, y: 0.63, w: 9.1, h: 0.24, fontFace: BODY_FONT, fontSize: 11,
  color: MUTE, margin: 0, valign: "middle",
});
s.addShape(p.ShapeType.rect, {
  x: 0.5, y: 0.90, w: 9.1, h: 0.012, fill: { color: RULE }, line: { width: 0 },
});

const head = (t, x, y, w) => s.addText(t, {
  x, y, w, h: 0.26, fontFace: BODY_FONT, fontSize: 13, color: NAVY,
  bold: true, margin: 0, valign: "middle",
});

// --- left, top: the roll loop is the evidence that matters ---------------
head([{ text: "1  ", options: { color: TEAL } },
      { text: "The roll loop was positive feedback" }], 0.5, 1.00, 5.6);
s.addImage({ path: path.join(HERE, "frame/roll_feedback.png"),
  x: 0.5, y: 1.28, w: 5.55, h: 2.33 });

// --- left, bottom: how the two errors compose ----------------------------
head([{ text: "2  ", options: { color: TEAL } },
      { text: "Two wrong frames, one on top of the other" }], 0.5, 3.70, 5.6);
s.addImage({ path: path.join(HERE, "frame/frames.png"),
  x: 0.5, y: 3.98, w: 5.55, h: 1.30 });

// --- right, top: the two defects, stated plainly -------------------------
head("What was wrong", 6.24, 1.00, 3.38);
s.addText([
  { text: "The mocap rig faces backwards. ", options: { bold: true, color: NAVY } },
  { text: "The rigid body and the flight controller point opposite ways on the vehicle, and nothing downstream removes it.",
    options: { color: INK, breakLine: true, paraSpaceAfter: 7 } },
  { text: "The PX4 conversion ran on non-PX4 data. ", options: { bold: true, color: NAVY } },
  { text: "update_px4_state applied NED→ENU and FRD→FLU to bridge odometry that was already z-up world with body-frame twist — right conversion, wrong caller.",
    options: { color: INK } },
], {
  x: 6.24, y: 1.28, w: 3.38, h: 1.55, fontFace: BODY_FONT, fontSize: 10.5,
  margin: 0, valign: "top",
});

// --- right, bottom: the per-axis conclusion ------------------------------
head("What each axis did", 6.24, 2.92, 3.38);
const hcell = (t) => ({ text: t, options: {
  bold: true, color: "FFFFFF", fill: { color: NAVY }, align: "center" } });
const band = (i) => (i % 2 ? { color: BAND } : { color: "FFFFFF" });
const row = (i, axis, d1, d2, result, rcolor, rbold) => ([
  { text: axis, options: { fill: band(i), bold: true } },
  { text: d1[0], options: { fill: band(i), align: "center", color: d1[1] } },
  { text: d2[0], options: { fill: band(i), align: "center", color: d2[1] } },
  { text: result, options: { fill: band(i), color: rcolor, bold: !!rbold } },
]);
s.addTable([
  [hcell("axis"), hcell("mount"), hcell("conv."), hcell("result")],
  row(1, "roll",  ["flip", RED], ["—", MUTE],  "INVERTED — crashed", RED, true),
  row(2, "pitch", ["flip", RED], ["flip", RED], "correct, by accident", GREEN, false),
  row(3, "yaw",   ["—", MUTE],  ["flip", RED], "INVERTED (413 runaway)", RED, true),
], {
  x: 6.24, y: 3.20, w: 3.38, colW: [0.58, 0.60, 0.58, 1.62],
  fontFace: BODY_FONT, fontSize: 9.5, color: INK, rowH: 0.26,
  border: { type: "solid", color: RULE, pt: 0.5 }, valign: "middle",
});
s.addText([
  { text: "Fix both together. ", options: { bold: true, color: NAVY } },
  { text: "Each fix alone still leaves two axes inverted.", options: { color: INK } },
], {
  x: 6.24, y: 4.56, w: 3.38, h: 0.34, fontFace: BODY_FONT, fontSize: 10,
  margin: 0, valign: "top",
});

// --- footer --------------------------------------------------------------
s.addText("analysis/frame_proof.py · frame_figs.py · ANALYSIS_HANDOFF §13", {
  x: 0.5, y: 5.32, w: 5.55, h: 0.22, fontFace: BODY_FONT, fontSize: 8.5,
  color: MUTE, margin: 0,
});

s.addNotes(
  "The crash was attributed to the mixer. It was not the mixer -- the mixer is correct for a standard-X, innie airframe. Two frame errors are, and they hit different axes, which is exactly why the axes disagreed with each other.\n\n" +
  "The mount: the Motive rigid body and the flight controller face opposite ways on the vehicle. You can see it on the robot -- it needs no proof. If asked: measured against the FC's own attitude on three flights it is 179.9 / 179.1 / 178.6 deg, residual 1.0-3.2 deg, and since Motive is z-up and the bridge's to_z_up is a no-op, no code touches the pose, so it can only be the rigid body.\n\n" +
  "The conversion: update_px4_state applies PX4's NED->ENU and FRD->FLU to the mocap bridge's odometry, which is already z-up world with body-frame twist. Right conversion, wrong caller. It also swaps x with y and negates height -- the policy was told a climb was a descent.\n\n" +
  "Composition: training expects Rx(pi); the mount delivers Ry(pi); the conversion adds Rx(pi); the policy received Rz(pi) -- roll and yaw inverted, pitch clean. Roll is flipped by the mount and the conversion never touches roll, so it survived to the rotors and crashed 417: the policy's own roll command climbs +0.15 to +0.37 while roll runs -5 to -100 deg, plant slope -38.1 rad/s^2 per unit. Pitch is flipped by the mount and back by the conversion, which is why every mixer-side test called pitch the clean axis. Yaw is flipped by the conversion alone -- the 413 runaway -- and the 8/18 column flip cancelled it rather than fixing the mixer.\n\n" +
  "So: fix the mount, remove the conversion, and revert the 8/18 yaw column, all in one change. Do NOT deploy the designed roll negation -- fixing the mount already fixes roll. Restrained single-axis bench tests gate the next flight."
);

p.writeFile({ fileName: path.join(HERE, "frame_slide.pptx") })
  .then((f) => console.log("wrote " + f));
