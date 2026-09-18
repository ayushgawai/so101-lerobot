"""Generate a general SO-101 pick-and-place recording guide PDF (no session progress)."""

from __future__ import annotations

from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.pagesizes import letter
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import inch
from reportlab.platypus import Flowable, Paragraph, SimpleDocTemplate, Spacer, Table, TableStyle

OUT = Path("docs/PickPlace_Recording_Guide.pdf")

# Single accent family: deep slate + one teal
INK = colors.HexColor("#0F172A")
ACCENT = colors.HexColor("#0D9488")
SOFT = colors.HexColor("#F1F5F9")
LINE = colors.HexColor("#E2E8F0")
WHITE = colors.white
MUTED = colors.HexColor("#64748B")


class FlowBox(Flowable):
    def __init__(self, labels: list[str], width: float, height: float = 42):
        super().__init__()
        self.labels = labels
        self.width = width
        self.height = height

    def wrap(self, availWidth, availHeight):
        return self.width, self.height

    def draw(self):
        n = len(self.labels)
        gap = 12
        box_w = (self.width - gap * (n - 1)) / n
        box_h = 30
        y = (self.height - box_h) / 2
        c = self.canv
        for i, label in enumerate(self.labels):
            x = i * (box_w + gap)
            c.setFillColor(ACCENT if i == n - 1 else INK)
            c.roundRect(x, y, box_w, box_h, 4, fill=1, stroke=0)
            c.setFillColor(WHITE)
            c.setFont("Helvetica-Bold", 7)
            c.drawCentredString(x + box_w / 2, y + box_h / 2 - 2, label)
            if i < n - 1:
                mid = y + box_h / 2
                c.setStrokeColor(ACCENT)
                c.setFillColor(ACCENT)
                c.setLineWidth(1.3)
                c.line(x + box_w + 1, mid, x + box_w + gap - 4, mid)
                c.drawCentredString(x + box_w + gap - 2, mid - 3, ">")


def styles():
    s = getSampleStyleSheet()
    s.add(ParagraphStyle(name="TitleMain", fontName="Helvetica-Bold", fontSize=14, textColor=INK, alignment=TA_CENTER, spaceAfter=2))
    s.add(ParagraphStyle(name="Sub", fontName="Helvetica", fontSize=9, textColor=MUTED, alignment=TA_CENTER, spaceAfter=10))
    s.add(ParagraphStyle(name="H", fontName="Helvetica-Bold", fontSize=10, textColor=INK, spaceBefore=9, spaceAfter=4))
    s.add(ParagraphStyle(name="Body", fontName="Helvetica", fontSize=8.5, textColor=INK, leading=11.5, spaceAfter=3))
    s.add(ParagraphStyle(name="CmdCode", fontName="Courier", fontSize=7.5, textColor=INK, leading=10, backColor=SOFT, leftIndent=4, spaceBefore=1, spaceAfter=1))
    s.add(ParagraphStyle(name="Cell", fontName="Helvetica", fontSize=7.5, textColor=INK, leading=10))
    s.add(ParagraphStyle(name="CellBold", fontName="Helvetica-Bold", fontSize=7.5, textColor=INK, leading=10))
    s.add(ParagraphStyle(name="Note", fontName="Helvetica", fontSize=7.5, textColor=MUTED, leading=10, spaceBefore=2))
    return s


def make_table(rows, col_widths):
    t = Table(rows, colWidths=col_widths, repeatRows=1)
    t.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, 0), INK),
                ("TEXTCOLOR", (0, 0), (-1, 0), WHITE),
                ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
                ("FONTNAME", (0, 1), (-1, -1), "Helvetica"),
                ("FONTSIZE", (0, 0), (-1, -1), 7.5),
                ("TEXTCOLOR", (0, 1), (-1, -1), INK),
                ("BACKGROUND", (0, 1), (-1, -1), SOFT),
                ("GRID", (0, 0), (-1, -1), 0.4, LINE),
                ("VALIGN", (0, 0), (-1, -1), "TOP"),
                ("LEFTPADDING", (0, 0), (-1, -1), 5),
                ("RIGHTPADDING", (0, 0), (-1, -1), 5),
                ("TOPPADDING", (0, 0), (-1, -1), 3.5),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 3.5),
            ]
        )
    )
    return t


def build():
    OUT.parent.mkdir(parents=True, exist_ok=True)
    doc = SimpleDocTemplate(
        str(OUT),
        pagesize=letter,
        leftMargin=0.6 * inch,
        rightMargin=0.6 * inch,
        topMargin=0.5 * inch,
        bottomMargin=0.5 * inch,
        title="SO-101 Pick-and-Place Recording Guide",
        author="so101-lerobot",
    )
    st = styles()
    story = []
    w = letter[0] - 1.2 * inch
    c = lambda t: Paragraph(t, st["Cell"])
    cb = lambda t: Paragraph(t, st["CellBold"])

    story.append(Paragraph("SO-101 Pick-and-Place Dataset Recording Guide", st["TitleMain"]))
    story.append(
        Paragraph(
            "How to record teleoperated demonstrations · LeRobot v3.0 · Windows",
            st["Sub"],
        )
    )

    # 1 Purpose
    story.append(Paragraph("1. Purpose", st["H"]))
    story.append(
        Paragraph(
            "This guide explains how to record pick-and-place demonstrations on the SO-101 "
            "(leader + follower arms, dual cameras). The operator moves the <b>leader</b>; the "
            "<b>follower</b> mirrors. Each demo picks a cube from a numbered sheet position "
            "(1–50) and places it in a <b>fixed box</b>. Position and demo count are entered "
            "interactively. Spoken audio cues announce each phase.",
            st["Body"],
        )
    )
    story.append(Paragraph("High-level flow", st["Note"]))
    story.append(
        FlowBox(
            ["Start script", "Enter position", "Enter # demos", "Teleop", "Right arrow saves"],
            w,
            40,
        )
    )

    # 2 Hardware defaults
    story.append(Paragraph("2. Hardware defaults", st["H"]))
    story.append(
        make_table(
            [
                [cb("Item"), cb("Default")],
                [c("Follower (robot)"), c("COM3 · id my_so_arm · calibration/robots/so_follower")],
                [c("Leader (teleop)"), c("COM4 · id my_so_arm · calibration/teleoperators/so_leader")],
                [c("Cameras"), c("gripper_cam index 0 · top_cam index 1 · 640×480 @ 30 fps")],
                [c("Drop target"), c("Fixed box (no position id)")],
                [c("Pickup id"), c("Sheet positions 1–50 (entered per recording block)")],
            ],
            [w * 0.28, w * 0.72],
        )
    )

    # 3 Storage layout
    story.append(Paragraph("3. Where files are stored", st["H"]))
    story.append(
        make_table(
            [
                [cb("Content"), cb("Path")],
                [c("Dataset root"), c("<font face='Courier'>hf_data/so101-pick-place-positions/</font>")],
                [c("Dataset id"), c("<font face='Courier'>aakashv100/so101-pick-place-positions</font>")],
                [c("Totals / schema"), c("<font face='Courier'>…/meta/info.json</font>")],
                [c("Episode metadata"), c("<font face='Courier'>…/meta/episodes/</font>")],
                [c("State &amp; actions"), c("<font face='Courier'>…/data/</font>")],
                [
                    c("Videos"),
                    c(
                        "<font face='Courier'>…/videos/observation.images.gripper_cam/</font><br/>"
                        "<font face='Courier'>…/videos/observation.images.top_cam/</font>"
                    ),
                ],
                [c("Position ↔ episode log"), c("<font face='Courier'>…/pickup_positions.jsonl</font>")],
                [c("Readable summary"), c("<font face='Courier'>…/pickup_positions_summary.json</font>")],
                [
                    c("Scripts"),
                    c(
                        "<font face='Courier'>scripts/record_pickplace.ps1</font> (launcher)<br/>"
                        "<font face='Courier'>scripts/record_pickplace.py</font> (recorder)"
                    ),
                ],
            ],
            [w * 0.3, w * 0.7],
        )
    )
    story.append(
        Paragraph(
            "If the dataset folder already exists with saved episodes, running the script again "
            "<b>appends</b> new demos. It does not overwrite previous data.",
            st["Note"],
        )
    )

    # 4 What is recorded
    story.append(Paragraph("4. What each episode records", st["H"]))
    story.append(
        make_table(
            [
                [cb("Modality"), cb("Details")],
                [c("Joints"), c("6-DoF state + actions @ 30 fps")],
                [c("Cameras"), c("gripper_cam + top_cam video; live side-by-side preview during recording")],
                [
                    c("Task label"),
                    c("Includes pickup position, e.g. “Pick up the cube from sheet position N and place it in the box”"),
                ],
                [c("Sidecar log"), c("JSONL maps episode_index → pickup_position for later analysis")],
            ],
            [w * 0.26, w * 0.74],
        )
    )

    # 5 Start
    story.append(Paragraph("5. How to start a recording session", st["H"]))
    story.append(
        make_table(
            [
                [cb("Step"), cb("Action")],
                [c("1"), c("Power both arms; connect USB; place cube on the sheet mark; box fixed at drop site")],
                [c("2"), c("Confirm calibrations and camera indices (see §2)")],
                [c("3"), c("Open PowerShell in the repo root")],
                [c("4"), c("Activate the environment, then launch the recorder (commands below)")],
            ],
            [w * 0.1, w * 0.9],
        )
    )
    story.append(Spacer(1, 3))
    story.append(Paragraph(r"cd <path-to>\so101-lerobot", st["CmdCode"]))
    story.append(Paragraph(r".\.venv\Scripts\Activate.ps1", st["CmdCode"]))
    story.append(Paragraph(r".\scripts\record_pickplace.ps1", st["CmdCode"]))
    story.append(
        Paragraph(
            r"Optional flags: -Fresh (fail if dataset already exists) · -PushToHub (upload when session ends) · "
            r"-RobotPort / -TeleopPort / -GripperCamIndex / -TopCamIndex to override defaults.",
            st["Note"],
        )
    )

    # 6 Interactive loop
    story.append(Paragraph("6. Interactive recording steps", st["H"]))
    story.append(
        make_table(
            [
                [cb("#"), cb("Operator action"), cb("System behavior")],
                [c("1"), c("Run the launcher"), c("Connects hardware; opens camera preview")],
                [c("2"), c("Press Enter when preview looks correct"), c("Shows text prompts")],
                [c("3"), c("Enter <b>pickup position</b> (1–50)"), c("Tags this block with that position id")],
                [c("4"), c("Enter <b>number of iterations</b>"), c("Plans that many demos for this position")],
                [c("5"), c("Teleop one pick → place"), c("Records joints + both cameras; may speak an audio cue")],
                [c("6"), c("Press <b>Right arrow</b> when done"), c("Saves the episode and updates the position log")],
                [c("7"), c("Reset cube on the same mark"), c("Audio: “Reset the environment”")],
                [c("8"), c("Press <b>Right arrow</b> to end reset"), c("Starts next iteration until the count is done")],
                [c("9"), c("Enter next position, or <b>q</b> to quit"), c("New block, or finalize and exit")],
            ],
            [w * 0.06, w * 0.42, w * 0.52],
        )
    )

    # 7 Audio + keys
    story.append(Paragraph("7. Audio cues and controls", st["H"]))
    story.append(
        Paragraph(
            "Spoken prompts use Windows text-to-speech (LeRobot <font face='Courier'>log_say</font>) "
            "so the operator can watch the robot instead of the terminal. Pass "
            "<font face='Courier'>--no-play-sounds</font> to disable.",
            st["Body"],
        )
    )
    story.append(
        make_table(
            [
                [cb("Audio cue"), cb("Meaning")],
                [c("“Position N, iteration i of M”"), c("Next demo in the current block is starting")],
                [c("“Reset the environment”"), c("Put the cube back; then Right arrow")],
                [c("“Re-record episode”"), c("Last attempt discarded (after Left arrow)")],
                [c("“Stop recording” / “Exiting”"), c("Session is ending")],
            ],
            [w * 0.42, w * 0.58],
        )
    )
    story.append(Spacer(1, 4))
    story.append(
        make_table(
            [
                [cb("Control"), cb("Action")],
                [c("Terminal: position"), c("Sheet pickup id for this block")],
                [c("Terminal: iterations"), c("How many demos to record now")],
                [c("Terminal: Enter (blank)"), c("Reuse last position or iteration count when offered")],
                [c("Terminal: q"), c("Quit at a prompt")],
                [c("<b>Right arrow</b>"), c("Save episode / end reset")],
                [c("<b>Left arrow</b>"), c("Discard current episode and re-record")],
                [c("<b>Esc</b>"), c("Stop the entire session")],
            ],
            [w * 0.32, w * 0.68],
        )
    )

    # 8 Append
    story.append(Paragraph("8. Continuing in a later session", st["H"]))
    story.append(
        make_table(
            [
                [cb("Goal"), cb("How")],
                [c("Add new sheet positions"), c(r"Run <font face='Courier'>.\scripts\record_pickplace.ps1</font> again; enter new position + iteration count")],
                [c("Add more demos for an existing position"), c("Same command; enter that position and how many more")],
                [
                    c("Start a brand-new dataset"),
                    c(
                        r"Delete <font face='Courier'>hf_data/so101-pick-place-positions</font>, then run the launcher again"
                    ),
                ],
            ],
            [w * 0.34, w * 0.66],
        )
    )

    # 9 Training note
    story.append(Paragraph("9. After recording (training)", st["H"]))
    story.append(
        Paragraph(
            "The dataset is LeRobot-compatible (state, action, dual-camera video). "
            "Point ACT training at the local root, for example:",
            st["Body"],
        )
    )
    story.append(
        Paragraph(
            r".\scripts\train_act.ps1 -RepoId aakashv100/so101-pick-place-positions "
            r"-DatasetRoot hf_data/so101-pick-place-positions -JobName act_so101_pick_place_positions",
            st["CmdCode"],
        )
    )

    doc.build(story)
    print(f"Wrote {OUT.resolve()}")


if __name__ == "__main__":
    build()
