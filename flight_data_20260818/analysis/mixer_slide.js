// Regenerates mixer_slide.pptx (the "why it rolled" deck slide).
//
//   cd ~/ATMO/flight_data_20260818/analysis
//   npm install pptxgenjs        # once
//   node mixer_slide.js          # writes mixer_slide.pptx here
//
// Figures it embeds (regenerate upstream if data changes):
//   log_417/slide_pitch_ok.png   pitch loop working (from bag mocap + replay.csv)
//   log_417/slide_domain.png     roll vs 75 deg trained-domain boundary
//   log_417/slide_scatter.png    command-vs-mocap scatters, in-domain window
const path = require("path");
const pptxgen = require("pptxgenjs");
const HERE = __dirname;
const p = new pptxgen();
p.layout = "LAYOUT_16x9"; // 10 x 5.625 in
const s = p.addSlide();
s.background = { color: "FFFFFF" };

s.addText("Why it rolled: the policy's roll axis was inverted", {
  x: 0.5, y: 0.22, w: 9.0, h: 0.55, fontFace: "Arial", fontSize: 26,
  color: "3C4043", bold: false, margin: 0,
});

// Left: video drop zone
s.addShape(p.ShapeType.roundRect, {
  x: 0.5, y: 0.95, w: 2.95, h: 2.45, rectRadius: 0.06,
  fill: { color: "F1F3F4" }, line: { color: "BDC1C6", width: 1, dashType: "dash" },
});
s.addText([
  { text: "DROP FLIGHT-3 VIDEO HERE", options: { fontSize: 11, color: "80868B", bold: true, breakLine: true } },
  { text: "roll departure, ~1.7 s from spin-up to kill", options: { fontSize: 9, color: "9AA0A6" } },
], { x: 0.5, y: 1.85, w: 2.95, h: 0.7, align: "center", fontFace: "Arial", margin: 0 });

// Middle: the healthy pitch loop; right: roll vs the 75 deg boundary
s.addImage({ path: path.join(HERE, "log_417/slide_pitch_ok.png"),
  x: 3.55, y: 0.95, w: 3.05, h: 2.25 });
s.addImage({ path: path.join(HERE, "log_417/slide_domain.png"),
  x: 6.60, y: 0.95, w: 3.15, h: 2.32 });

// Bottom: scatterplots against the real mocap data + caption
s.addImage({ path: path.join(HERE, "log_417/slide_scatter.png"),
  x: 0.5, y: 3.62, w: 5.9, h: 1.85 });
s.addText([
  { text: "Command vs real mocap (in-domain window): the loop is coherent in the frame the policy saw — the sign fault sits between that frame and the airframe (mixer + obs ingestion, fixed together).", options: { fontSize: 11, color: "3C4043", breakLine: true, paraSpaceAfter: 5 } },
  { text: "Command peaks at 1.40 s — the 75° training termination limit is crossed at 1.41 s. Liftoff at tilt ≈ 29°; tilt rests at the measured 17.6° through the departure.", options: { fontSize: 11, color: "5F6368" } },
], { x: 6.55, y: 3.72, w: 3.2, h: 1.7, fontFace: "Arial", align: "left", margin: 0, valign: "top" });

s.addNotes(
  "KEY BEAT: the roll command peaks at 1.40 s and the vehicle crosses 75 deg at 1.41 s -- 75 deg is exactly the training attitude-termination limit. The policy fought (through an inverted mixer) right up to the edge of every state it had ever seen, and past that edge its inputs saturate (19% of channels at the normalizer rail) and the output stops being a controller. " +
  "The video shows the roll departure. The scatters are the replayed network against real mocap: roll r=+0.94 with the command leading by ~100 ms -- policy and vehicle agreed perfectly about what was happening in the frame the policy saw; they disagreed with reality about which direction it was. " +
  "Root cause: roll+yaw mixer columns inverted sim->hardware, plus the mocap ingestion frame (obs roll = -0.95x FC truth, pitch +1.01x). Yaw was fixed between flights 1 and 2 and the data shows its verdict flip; roll reads inverted in all flights. Mixer flip and obs-frame fix must deploy TOGETHER (each alone leaves an odd inversion count); restrained single-axis tests gate the next flight."
);

p.writeFile({ fileName: path.join(HERE, "mixer_slide.pptx") }).then(() => console.log("written"));
