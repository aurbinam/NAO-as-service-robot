"""
dissertation_figures.py
========================
Generates all Python-producible dissertation figures at 300 dpi.

INSTALL ONCE (run in your terminal first):
    pip install matplotlib networkx numpy

USAGE:
    python dissertation_figures.py

All PNGs are saved in the same folder as this script.
Edit any data / labels in the clearly-marked sections below.

FIGURES PRODUCED:
    fig1_1_aging_trend.png          — Ch.1  Ageing population line chart
    fig2_1_robot_comparison.png     — Ch.2  PARO/Pepper/Care-O-bot/HSR table
    fig3_1_system_architecture.png  — Ch.3  4-process system architecture
    fig3_2_hybrid_ai_split.png      — Ch.3  LLM vs deterministic layer split
    fig3_4_room_graph.png           — Ch.3  Semantic room graph (Dijkstra input)
    fig4_1_navigation_stack.png     — Ch.4  3-layer navigation stack
    fig4_3_robotbrain_fsm.png       — Ch.4  RobotBrain 5-stage state machine
    fig4_4_conversation_pipeline.png— Ch.4  3-level conversation pipeline
    fig4_5_voice_pipeline.png       — Ch.4  Voice capture sequence diagram
    fig4_6_door_crossing.png        — Ch.4  Door-crossing action sequence
    fig4_7_escort_fsm.png           — Ch.4  Assistive escort FSM
    fig5_1_objectives_table.png     — Ch.5  Objectives vs achievement matrix

NOT generated here (need Webots screenshots):
    fig3_3  — Webots floor plan screenshot
    fig4_2  — Annotated floor plan with route overlay
    fig5_2  — Side-by-side escort / doorway screenshots
"""

import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch
import networkx as nx
import numpy as np

# ── Output folder: same directory as this script ──────────────────────────────
OUT = os.path.dirname(os.path.abspath(__file__)) + os.sep

# ── Shared colour palette ──────────────────────────────────────────────────────
BLUE_DARK   = '#0C447C'
BLUE_MID    = '#185FA5'
BLUE_LIGHT  = '#E6F1FB'
BLUE_MED    = '#B5D4F4'
BLUE_DEEP   = '#042C53'
GREY_DARK   = '#2C2C2A'
GREY_MID    = '#5F5E5A'
GREY_LIGHT  = '#F1EFE8'
GREY_BORDER = '#D3D1C7'
GREY_LINE   = '#888780'
GREEN_DARK  = '#27500A'
GREEN_MID   = '#3B6D11'
GREEN_LIGHT = '#EAF3DE'
AMBER_DARK  = '#633806'
AMBER_MID   = '#854F0B'
AMBER_LIGHT = '#FAEEDA'
RED_DARK    = '#791F1F'
RED_MID     = '#A32D2D'
RED_LIGHT   = '#FCEBEB'
BORDER      = '#d8d4c8'
INK         = '#1a1a18'

# ── Shared helper functions ────────────────────────────────────────────────────

def rounded_box(ax, x, y, w, h, label, sublabel='',
                bg=BLUE_LIGHT, fg=BLUE_DARK, ec=BLUE_MID,
                fontsize=10, lw=1.2, radius=0.3, bold=True):
    """Draw a rounded rectangle with a title and optional italic sublabel."""
    r = FancyBboxPatch(
        (x, y), w, h,
        boxstyle=f'round,pad=0,rounding_size={radius}',
        linewidth=lw, edgecolor=ec, facecolor=bg, zorder=3)
    ax.add_patch(r)
    ty = y + h * 0.62 if sublabel else y + h / 2
    ax.text(x + w / 2, ty, label,
            ha='center', va='center',
            fontsize=fontsize, fontweight='bold' if bold else 'normal',
            color=fg, zorder=4, multialignment='center')
    if sublabel:
        ax.text(x + w / 2, y + h * 0.28, sublabel,
                ha='center', va='center',
                fontsize=fontsize - 1.5, color=fg, zorder=4,
                multialignment='center', style='italic')


def arrow(ax, x1, y1, x2, y2, label='', color=BLUE_MID, lw=1.5, rad=0.0):
    """Draw an annotated arrow between two points."""
    ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
        arrowprops=dict(arrowstyle='->', color=color, lw=lw,
                        connectionstyle=f'arc3,rad={rad}'), zorder=5)
    if label:
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2
        ax.text(mx + 0.05, my + 0.12, label,
                ha='center', va='bottom', fontsize=7.8, color=color,
                bbox=dict(fc='white', ec='none', pad=1))


def source_note(ax, text, y=0.02):
    """Add a small grey source attribution at the bottom of the axes."""
    ax.text(0.5, y, text, transform=ax.transAxes,
            fontsize=7.5, color=GREY_LINE, ha='center')


def save_fig(fig, filename):
    """Save and close a figure."""
    plt.tight_layout(pad=0.4)
    plt.savefig(OUT + filename, dpi=300, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f"  Saved  {filename}")


# ==============================================================================
# FIG 1.1 — Global ageing population trend (line chart)
# ==============================================================================
# SOURCE DATA:
#   UNFPA (2024). Ageing. https://www.unfpa.org/ageing
#   WHO (2025). Population Ageing Q&A. https://www.who.int/news-room/questions-and-answers/item/population-ageing
#   UN (2024). World Population Prospects 2024. https://www.un.org/en/global-issues/ageing
# ------------------------------------------------------------------------------
def fig1_1_aging_trend():
    # ── Edit these data points if you want different years ───────────────────
    years = [1974, 1990, 2000, 2010, 2024, 2050, 2074]
    pct   = [5.5,  6.2,  6.9,  7.6,  10.3, 16.0, 20.7]
    # ─────────────────────────────────────────────────────────────────────────

    SPLIT = 2024
    hist_x = [y for y in years if y <= SPLIT]
    hist_y = [p for y, p in zip(years, pct) if y <= SPLIT]
    proj_x = [y for y in years if y >= SPLIT]
    proj_y = [p for y, p in zip(years, pct) if y >= SPLIT]

    fig, ax = plt.subplots(figsize=(9, 5))
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')

    # Lines
    ax.plot(hist_x, hist_y, color=BLUE_MID, linewidth=2.5,
            marker='o', markersize=6, zorder=3)
    ax.plot(proj_x, proj_y, color=BLUE_MID, linewidth=2.5,
            marker='o', markersize=6, linestyle='--', zorder=3)

    # Shading
    ax.fill_between(hist_x, hist_y, alpha=0.12, color=BLUE_MID)
    ax.fill_between(proj_x, proj_y, alpha=0.07, color=BLUE_MID)

    # "Today" divider
    ax.axvline(x=SPLIT, color=GREY_LINE, linewidth=1.2, linestyle=':', zorder=2)
    ax.text(SPLIT + 1.5, 18.5, 'Today\n(2024)',
            fontsize=9, color=GREY_MID, va='top', ha='left')

    # Data labels
    for x, y in zip(years, pct):
        ax.annotate(f'{y}%', (x, y),
                    textcoords='offset points', xytext=(0, 9),
                    ha='center', fontsize=8.5, color=INK)

    ax.set_xlim(1968, 2082)
    ax.set_ylim(0, 24)
    ax.set_xlabel('Year', fontsize=11, color='#4a4a44')
    ax.set_ylabel('Population aged 65+ (%)', fontsize=11, color='#4a4a44')
    ax.tick_params(colors=GREY_LINE, labelsize=9)
    for spine in ax.spines.values():
        spine.set_edgecolor(BORDER)

    # Legend
    hist_patch = mpatches.Patch(color=BLUE_MID, label='Historical data')
    proj_patch = mpatches.Patch(color=BLUE_MID, alpha=0.5, label='Projected')
    ax.legend(handles=[hist_patch, proj_patch], fontsize=9,
              framealpha=0.8, edgecolor=BORDER, loc='upper left')

    source_note(ax,
        'Source: UNFPA (2024) Ageing; WHO (2025) Population Ageing Q&A; '
        'UN World Population Prospects (2024)', y=-0.11)

    save_fig(fig, 'fig1_1_aging_trend.png')


# ==============================================================================
# FIG 2.1 — Robot comparison table image
# ==============================================================================
# SOURCES:
#   Kachouie et al. (2014). Socially Assistive Robots in Elderly Care.
#       Int. J. Human-Computer Interaction 30, 369-393.
#   ACM (2022). Employing Socially Assistive Robots in Elderly Care.
#       arXiv:2304.14944
#   MDPI Applied Sciences (2024). Socially Assistive Robots in Smart Environments
#       to Attend Elderly People — A Survey. DOI:10.3390/app14125287
#   Manufacturer specifications (PARO, SoftBank Robotics, Fraunhofer, Toyota).
# ------------------------------------------------------------------------------
def fig2_1_robot_table():
    columns = ['Robot', 'Conversation', 'Physical\nhelp',
               'Navigation', 'Approx.\ncost', 'Target use']

    # ── Edit robot rows here ──────────────────────────────────────────────────
    rows = [
        ['PARO',
         'Touch / sound\nonly (scripted)',
         'None',
         'None',
         '~£4,000',
         'Dementia /\nemotional support'],
        ['Pepper',
         'Voice + screen\n(scripted dialogues)',
         'Minimal\n(no manipulation)',
         'Limited\nautonomous',
         '~£15,000',
         'Social\ninteraction'],
        ['Care-O-bot',
         'Limited\ncommands',
         'Object fetch\n(arm)',
         'Yes\n(lab only)',
         '~£100,000+',
         'Clinical /\nlab settings'],
        ['Toyota HSR',
         'Voice + tablet\nPC interface',
         'Arm + fetch\n(mobile)',
         'Yes\n(autonomous)',
         'Research\nonly',
         'Mobility-impaired\nat home'],
    ]
    # ─────────────────────────────────────────────────────────────────────────

    HEADER_BG = BLUE_MID
    ROW_BG    = ['#f5f8fc', 'white', '#f5f8fc', 'white']
    col_w     = [0.10, 0.16, 0.14, 0.16, 0.12, 0.20]

    # Build cumulative x positions
    xs = [0.01]
    for w in col_w[:-1]:
        xs.append(xs[-1] + w)

    fig, ax = plt.subplots(figsize=(12, 4.2))
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')
    ax.axis('off')

    n_rows = len(rows) + 1
    row_h  = 0.82 / n_rows

    def cell(x, y, w, h, text, bg, fg, bold=False):
        rect = mpatches.FancyBboxPatch(
            (x, y), w, h,
            boxstyle='square,pad=0', linewidth=0.5,
            edgecolor=BORDER, facecolor=bg,
            transform=ax.transAxes, clip_on=False)
        ax.add_patch(rect)
        ax.text(x + w / 2, y + h / 2, text,
                transform=ax.transAxes,
                ha='center', va='center', fontsize=9.2,
                color=fg, fontweight='bold' if bold else 'normal',
                multialignment='center', linespacing=1.35)

    # Header row
    ys_header = 1 - row_h
    for col, x, w in zip(columns, xs, col_w):
        cell(x, ys_header, w, row_h, col, HEADER_BG, 'white', bold=True)

    # Data rows
    for i, row in enumerate(rows):
        bg  = ROW_BG[i % len(ROW_BG)]
        y   = 1 - (i + 2) * row_h
        for val, x, w in zip(row, xs, col_w):
            cell(x, y, w, row_h, val, bg, INK)

    ax.text(0.5, -0.04,
            'Sources: Kachouie et al. (2014) Int. J. HCI 30; '
            'ACM (2022) arXiv:2304.14944; '
            'MDPI Appl. Sci. (2024) DOI:10.3390/app14125287; '
            'manufacturer specifications.',
            transform=ax.transAxes, fontsize=7.5, color=GREY_LINE, ha='center')

    save_fig(fig, 'fig2_1_robot_comparison.png')


# ==============================================================================
# FIG 3.1 — System architecture (4 processes, 2 communication channels)
# ==============================================================================
# SOURCES / INSPIRATION:
#   Fuertes et al. (2019). Architecture for Integration of Robots and Sensors
#       for Care of the Elderly. Robotics 8(3). DOI:10.3390/robotics8030076
#   Hendrich et al. (2015). Architecture and Software Design for a Service
#       Robot in an Elderly-Care Scenario. Engineering 1(1).
#       DOI:10.15302/J-ENG-2015007
# ------------------------------------------------------------------------------
def fig3_1_system_architecture():
    fig, ax = plt.subplots(figsize=(12, 7))
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 7)
    ax.axis('off')

    # ── Webots simulation boundary (dashed rectangle) ─────────────────────────
    webots = FancyBboxPatch(
        (0.5, 1.2), 7.8, 4.8,
        boxstyle='round,pad=0,rounding_size=0.3',
        linewidth=1.5, edgecolor=BLUE_MID, facecolor='#f0f6fd',
        linestyle='--', zorder=1)
    ax.add_patch(webots)
    ax.text(1.0, 5.75, 'Webots simulation environment',
            fontsize=9, color=BLUE_MID, style='italic', zorder=2)

    # ── Process boxes ─────────────────────────────────────────────────────────
    # nao_assist_controller  (main process, inside Webots)
    rounded_box(ax, 1.0, 2.0, 4.0, 3.2,
                'nao_assist_controller.py',
                'RobotBrain · CompanionBrain\nAssistive FSM · TCP listener\n'
                'Speech output · Motion control',
                bg=BLUE_LIGHT, fg=BLUE_DARK, ec=BLUE_MID, fontsize=10)

    # door_manager  (inside Webots)
    rounded_box(ax, 5.8, 2.8, 2.2, 1.8,
                'door_manager.py',
                'Hinge motor\ncontrol',
                bg=GREY_LIGHT, fg=GREY_DARK, ec=GREY_LINE,
                fontsize=9.5, radius=0.2)

    # voice_listener  (external process, above Webots boundary)
    rounded_box(ax, 0.8, 6.0, 3.5, 0.75,
                'voice_listener.py   (external process)',
                bg=GREY_LIGHT, fg=GREY_DARK, ec=GREY_LINE,
                fontsize=9, radius=0.2)

    # Claude API  (cloud / external)
    rounded_box(ax, 9.0, 3.2, 2.7, 1.6,
                'Claude API\n(remote)',
                'Intent classification\nConversation',
                bg=GREEN_LIGHT, fg=GREEN_DARK, ec=GREEN_MID,
                fontsize=9.5, radius=0.4)

    # ── Arrows ────────────────────────────────────────────────────────────────
    # voice_listener → controller  (TCP)
    arrow(ax, 2.55, 6.0, 2.55, 5.2,
          'TCP port 5005\n(text commands)', color=BLUE_MID, lw=1.8)

    # controller → door_manager  (Webots emitter)
    arrow(ax, 5.0, 3.6, 5.8, 3.6,
          'Webots emitter ch.1', color=GREY_MID, lw=1.5)

    # door_manager → controller  (door state back)
    ax.annotate('', xy=(5.0, 3.2), xytext=(5.8, 3.2),
        arrowprops=dict(arrowstyle='->', color=GREY_LINE, lw=1.2), zorder=5)
    ax.text(5.4, 2.95, 'door state', ha='center', fontsize=7.5, color=GREY_LINE)

    # controller ↔ Claude API  (HTTPS)
    arrow(ax, 5.0, 4.2, 9.0, 4.2,
          'HTTPS (JSON intent request)', color=GREEN_MID, lw=1.5)
    ax.annotate('', xy=(5.0, 3.8), xytext=(9.0, 3.8),
        arrowprops=dict(arrowstyle='->', color=GREEN_MID, lw=1.2), zorder=5)
    ax.text(7.0, 3.55, 'JSON response', ha='center',
            fontsize=7.5, color=GREEN_MID)

    ax.text(0.3, 0.55,
            'Solid border = internal (Webots)   |   '
            'Dashed border = Webots simulation boundary   |   '
            'Green = external cloud API',
            fontsize=8, color=GREY_LINE)
    source_note(ax,
        'Inspired by: Fuertes et al. (2019) Robotics 8(3) DOI:10.3390/robotics8030076; '
        'Hendrich et al. (2015) Engineering 1(1) DOI:10.15302/J-ENG-2015007', y=0.06)

    save_fig(fig, 'fig3_1_system_architecture.png')


# ==============================================================================
# FIG 3.2 — Hybrid AI split (probabilistic LLM over deterministic layer)
# ==============================================================================
# SOURCE:
#   Tagliabue et al. (2023). Towards a Hybrid LLM/Model-Based Architecture
#       for Robot Coaching. CEUR Workshop Proceedings Vol-4101, paper6.
#       https://ceur-ws.org/Vol-4101/paper6.pdf
# ------------------------------------------------------------------------------
def fig3_2_hybrid_ai_split():
    fig, ax = plt.subplots(figsize=(10, 6.5))
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 6.5)
    ax.axis('off')

    # ── Top band: probabilistic / LLM ────────────────────────────────────────
    ax.add_patch(FancyBboxPatch(
        (0.3, 3.5), 9.4, 2.7,
        boxstyle='round,pad=0,rounding_size=0.3',
        lw=1.5, edgecolor=BLUE_MID, facecolor=BLUE_LIGHT, zorder=1))
    ax.text(0.7, 6.0, 'Probabilistic layer  —  LLM (Claude)',
            fontsize=11, fontweight='bold', color=BLUE_DARK)
    ax.text(0.7, 5.65,
            'Natural language understanding  ·  '
            'intent classification  ·  open-ended conversation',
            fontsize=8.5, color=BLUE_MID, style='italic')

    rounded_box(ax, 1.0, 3.75, 3.2, 1.5,
                'GenAIInterpreter',
                'Parses user speech\n→ JSON intent',
                bg=BLUE_MED, fg=BLUE_DEEP, ec=BLUE_MID, fontsize=9.5)

    rounded_box(ax, 5.7, 3.75, 3.2, 1.5,
                'Companion Brain',
                'Conversation · memory\nproactive check-ins',
                bg=BLUE_MED, fg=BLUE_DEEP, ec=BLUE_MID, fontsize=9.5)

    # ── Bottom band: deterministic ────────────────────────────────────────────
    ax.add_patch(FancyBboxPatch(
        (0.3, 0.4), 9.4, 2.8,
        boxstyle='round,pad=0,rounding_size=0.3',
        lw=1.5, edgecolor=GREY_MID, facecolor=GREY_LIGHT, zorder=1))
    ax.text(0.7, 3.0, 'Deterministic layer',
            fontsize=11, fontweight='bold', color=GREY_DARK)
    ax.text(0.7, 2.68,
            'Verified plans  ·  state machines  ·  safe physical actions only',
            fontsize=8.5, color=GREY_MID, style='italic')

    # ── Edit these module boxes if your code differs ──────────────────────────
    det_boxes = [
        (0.6,  0.65, 2.0, 1.6, 'TaskPlanner',     'Room-graph\nDijkstra routing'),
        (2.9,  0.65, 2.0, 1.6, 'Executor',         'Action-by-action\nexecution'),
        (5.2,  0.65, 2.2, 1.6, 'ExecutionMonitor', 'Stall / deviation\ndetection'),
        (7.7,  0.65, 1.8, 1.6, 'Assistive\nFSM',   'Escort\nstate machine'),
    ]
    for bx, by, bw, bh, bl, bs in det_boxes:
        rounded_box(ax, bx, by, bw, bh, bl, bs,
                    bg=GREY_BORDER, fg=GREY_DARK, ec=GREY_LINE,
                    fontsize=9, radius=0.2)
    # ─────────────────────────────────────────────────────────────────────────

    # ── Boundary arrows ───────────────────────────────────────────────────────
    ax.annotate('', xy=(3.0, 3.5), xytext=(2.6, 3.25),
        arrowprops=dict(arrowstyle='->', color=BLUE_MID, lw=2.2), zorder=6)
    ax.text(3.35, 3.37,
            'JSON intent\n(navigate / multi_step /\ndoor / unknown)',
            fontsize=7.8, color=BLUE_MID, va='center')

    ax.annotate('', xy=(2.6, 3.5), xytext=(3.0, 3.25),
        arrowprops=dict(arrowstyle='->', color=GREY_LINE, lw=1.2), zorder=6)
    ax.text(1.45, 3.38, 'ExecutionResult\n(completed / failed)',
            fontsize=7.5, color=GREY_LINE, va='center')

    # ── Safety boundary note ──────────────────────────────────────────────────
    ax.add_patch(FancyBboxPatch(
        (6.5, 4.1), 2.9, 1.7,
        boxstyle='round,pad=0,rounding_size=0.2',
        lw=1, edgecolor=RED_MID, facecolor=RED_LIGHT, zorder=4))
    ax.text(7.95, 5.35, 'Safety boundary',
            fontsize=8.5, fontweight='bold', color=RED_MID, ha='center')
    ax.text(7.95, 4.75,
            'LLM cannot directly\ntrigger movement.\nAll actions verified\nby deterministic layer.',
            fontsize=7.8, color=RED_DARK, ha='center', va='center',
            multialignment='center')

    source_note(ax,
        'Adapted from: Tagliabue et al. (2023) CEUR Vol-4101 paper6  '
        'https://ceur-ws.org/Vol-4101/paper6.pdf')
    save_fig(fig, 'fig3_2_hybrid_ai_split.png')


# ==============================================================================
# FIG 3.4 — Semantic room graph (Dijkstra input)
# ==============================================================================
# SOURCES:
#   Fadzli et al. (2015). Robotic Indoor Path Planning Using Dijkstra's
#       Algorithm with Multi-Layer Dictionaries. IEEE ICIAS.
#       DOI:10.1109/ICIAS.2015.7371031
#   Boniardi et al. (2019). A Door-to-Door Path-Finding Approach for Indoor
#       Navigation. ResearchGate. DOI:10.5194/isprs-annals-IV-2-W5-45-2019
# ------------------------------------------------------------------------------
def fig3_4_room_graph():
    # ── Edit rooms and connections to match YOUR Webots floor plan ────────────
    rooms = ['Living\nRoom', 'Corridor', 'Kitchen', 'Bedroom', 'Bathroom']
    edges = [
        ('Living\nRoom', 'Corridor',  1.0),
        ('Corridor',     'Kitchen',   1.2),
        ('Corridor',     'Bedroom',   1.1),
        ('Corridor',     'Bathroom',  1.0),
        ('Kitchen',      'Bedroom',   2.5),   # high-cost indirect path
    ]
    # Positions: (x, y) in axes fraction — adjust to match your floor plan layout
    pos = {
        'Living\nRoom': (0.15, 0.50),
        'Corridor':     (0.45, 0.50),
        'Kitchen':      (0.75, 0.50),
        'Bedroom':      (0.55, 0.82),
        'Bathroom':     (0.35, 0.82),
    }
    # Example highlighted Dijkstra route
    route_edges = [('Living\nRoom', 'Corridor'), ('Corridor', 'Kitchen')]
    # ─────────────────────────────────────────────────────────────────────────

    G = nx.Graph()
    G.add_nodes_from(rooms)
    for u, v, w in edges:
        G.add_edge(u, v, weight=w)

    other_edges = [(u, v) for u, v, _ in edges
                   if (u, v) not in route_edges and (v, u) not in route_edges]

    fig, ax = plt.subplots(figsize=(9, 6))
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')
    ax.axis('off')

    nx.draw_networkx_nodes(G, pos, ax=ax, node_color=BLUE_MED,
                           node_size=2800, edgecolors=BLUE_MID, linewidths=1.5)
    nx.draw_networkx_labels(G, pos, ax=ax, font_size=9,
                            font_color=BLUE_DEEP, font_weight='bold')
    nx.draw_networkx_edges(G, pos, ax=ax, edgelist=route_edges,
                           edge_color=BLUE_MID, width=3.0)
    nx.draw_networkx_edges(G, pos, ax=ax, edgelist=other_edges,
                           edge_color=GREY_LINE, width=1.5,
                           style='dashed', alpha=0.7)

    edge_labels = {(u, v): f'w={w}' for u, v, w in edges}
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, ax=ax,
                                 font_size=8, font_color='#4a4a44',
                                 bbox=dict(fc='white', ec='none', pad=1.5))

    ax.plot([], [], color=BLUE_MID,  lw=3,   label='Dijkstra route (Living Room → Kitchen)')
    ax.plot([], [], color=GREY_LINE, lw=1.5, ls='--', label='Other connections')
    ax.legend(fontsize=8.5, loc='lower right', framealpha=0.9, edgecolor=BORDER)

    save_fig(fig, 'fig3_4_room_graph.png')


# ==============================================================================
# FIG 4.1 — Navigation stack (3 layers)
# ==============================================================================
# SOURCES:
#   Liu et al. (2021). Path Planning for Smart Car Based on Dijkstra Algorithm
#       and Dynamic Window Approach. Wireless Comms & Mobile Computing.
#       DOI:10.1155/2021/8881684
#   PMC (2025). ROS-Based Navigation and Obstacle Avoidance: A Study of
#       Architectures, Methods, and Trends. PMC12300016.
# ------------------------------------------------------------------------------
def fig4_1_navigation_stack():
    fig, ax = plt.subplots(figsize=(9, 6.5))
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')
    ax.set_xlim(0, 9)
    ax.set_ylim(0, 6.5)
    ax.axis('off')

    # ── Edit layer descriptions if your modules are named differently ─────────
    layers = [
        (0.5, 4.5, 8.0, 1.6,
         'Layer 1 — Dijkstra room routing  (global planner)',
         'Graph search over room nodes  →  ordered room sequence\n'
         'Module: TaskPlanner',
         BLUE_LIGHT, BLUE_DARK, BLUE_MID),

        (0.5, 2.6, 8.0, 1.6,
         'Layer 2 — Waypoint planner',
         'Room centres + door approach/crossing points  →  waypoint list\n'
         'Module: TaskPlanner (action sequence)',
         GREY_LIGHT, GREY_DARK, GREY_LINE),

        (0.5, 0.7, 8.0, 1.6,
         'Layer 3 — Reactive walking + sonar  (local controller)',
         'Walk to waypoint  ·  sonar obstacle check  ·  '
         'stability guard  ·  gait control\nModule: Executor',
         GREEN_LIGHT, GREEN_DARK, GREEN_MID),
    ]
    # ─────────────────────────────────────────────────────────────────────────

    for x, y, w, h, title, desc, bg, fg, ec in layers:
        rounded_box(ax, x, y, w, h, title, desc,
                    bg=bg, fg=fg, ec=ec, fontsize=10, radius=0.25)

    arrow(ax, 4.5, 4.5, 4.5, 4.2, color=BLUE_MID, lw=2.2)
    ax.text(4.9, 4.32, 'room sequence', fontsize=8, color=BLUE_MID)

    arrow(ax, 4.5, 2.6, 4.5, 2.3, color=GREY_LINE, lw=2.2)
    ax.text(4.9, 2.42, 'waypoint list', fontsize=8, color=GREY_LINE)

    source_note(ax,
        'Adapted from: Liu et al. (2021) Wireless Comms & Mobile Computing '
        'DOI:10.1155/2021/8881684; PMC (2025) ROS Navigation Stack review PMC12300016.')
    save_fig(fig, 'fig4_1_navigation_stack.png')


# ==============================================================================
# FIG 4.3 — RobotBrain state machine
# ==============================================================================
# SOURCES:
#   Chen et al. (2017). A Robot Architecture of Hierarchical Finite State
#       Machine for Autonomous Mobile Manipulator.
#       ResearchGate DOI:10.1109/IROS.2017.xxx
#   Colledanchise & Ögren (2024). Comparison between Behavior Trees and
#       Finite State Machines. arXiv:2405.16137.
# ------------------------------------------------------------------------------
def fig4_3_robotbrain_fsm():
    fig, ax = plt.subplots(figsize=(14, 7))
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 7)
    ax.axis('off')

    # ── Edit stage names / module names to match your code exactly ────────────
    states = [
        (0.4,  3.2, 2.0, 1.6, 'UNDERSTANDING', 'GenAIInterpreter'),
        (3.0,  3.2, 2.0, 1.6, 'PLANNING',       'TaskPlanner'),
        (5.6,  3.2, 2.0, 1.6, 'EXECUTING',      'Executor'),
        (8.2,  3.2, 2.0, 1.6, 'MONITORING',     'ExecutionMonitor'),
    ]
    # ─────────────────────────────────────────────────────────────────────────

    for x, y, w, h, s, mod in states:
        rounded_box(ax, x, y, w, h, s, mod,
                    bg=BLUE_LIGHT, fg=BLUE_DARK, ec=BLUE_MID,
                    fontsize=9.5, radius=0.25)

    # Terminal states
    rounded_box(ax, 10.8, 4.6, 2.2, 1.2, 'COMPLETED', '',
                bg=GREEN_LIGHT, fg=GREEN_DARK, ec=GREEN_MID,
                fontsize=10, radius=0.25)
    rounded_box(ax, 10.8, 2.5, 2.2, 1.2, 'REPLANNING', 'retry ≤ 3×',
                bg=AMBER_LIGHT, fg=AMBER_DARK, ec=AMBER_MID,
                fontsize=10, radius=0.25)
    rounded_box(ax, 10.8, 1.0, 2.2, 1.2, 'FAILED', '',
                bg=RED_LIGHT, fg=RED_DARK, ec=RED_MID,
                fontsize=10, radius=0.25)

    # Entry arrow
    ax.annotate('', xy=(0.4, 4.0), xytext=(-0.3, 4.0),
        arrowprops=dict(arrowstyle='->', color=BLUE_MID, lw=1.8), zorder=5)
    ax.text(-0.25, 4.25, 'user command\n(voice / text)',
            fontsize=8, color=BLUE_MID, ha='left')

    # Forward arrows between main states
    for x1, x2 in [(2.4, 3.0), (5.0, 5.6), (7.6, 8.2)]:
        arrow(ax, x1, 4.0, x2, 4.0, color=BLUE_MID, lw=1.8)

    # MONITORING → COMPLETED / REPLANNING
    arrow(ax, 10.2, 4.5, 10.8, 5.2, 'success',         color=GREEN_MID, lw=1.5)
    arrow(ax, 10.2, 3.5, 10.8, 3.1, 'stall / deviation',color=AMBER_MID, lw=1.5)

    # REPLANNING → PLANNING (loop back)
    ax.annotate('', xy=(4.0, 3.2), xytext=(11.0, 2.5),
        arrowprops=dict(arrowstyle='->', color=AMBER_MID, lw=1.5,
                        connectionstyle='arc3,rad=-0.35'), zorder=5)
    ax.text(7.5, 1.85, 'retry (new plan)', fontsize=8,
            color=AMBER_MID, ha='center')

    # REPLANNING → FAILED
    arrow(ax, 11.9, 2.5, 11.9, 2.2,
          'max retries\nexceeded', color=RED_MID, lw=1.5)

    source_note(ax,
        'Adapted from: Chen et al. (2017) HFSM for Autonomous Mobile Manipulator; '
        'Colledanchise & Ögren (2024) arXiv:2405.16137.')
    save_fig(fig, 'fig4_3_robotbrain_fsm.png')


# ==============================================================================
# FIG 4.4 — Conversation pipeline (3 levels + deterministic filters)
# ==============================================================================
# SOURCES:
#   Xue et al. (2025). Integrating Large Language Models for Intuitive Robot
#       Navigation. Frontiers in Robotics and AI.
#       DOI:10.3389/frobt.2025.1627937
#   Tagliabue et al. (2023). Towards a Hybrid LLM/Model-Based Architecture
#       for Robot Coaching. CEUR Vol-4101.
#       https://ceur-ws.org/Vol-4101/paper6.pdf
# ------------------------------------------------------------------------------
def fig4_4_conversation_pipeline():
    fig, ax = plt.subplots(figsize=(11, 7))
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')
    ax.set_xlim(0, 11)
    ax.set_ylim(0, 7)
    ax.axis('off')

    # Entry box
    rounded_box(ax, 3.5, 6.1, 4.0, 0.7, 'User input  (voice / text)',
                bg=GREY_BORDER, fg=GREY_DARK, ec=GREY_LINE,
                fontsize=10, radius=0.2)

    # ── Edit level descriptions to match your implementation ─────────────────
    rounded_box(ax, 2.5, 4.7, 6.0, 1.1,
                'Level 1 — Regex / keyword parser',
                'Strips filler words  ·  matches known patterns  ·  expands abbreviations',
                bg=GREY_LIGHT, fg=GREY_DARK, ec=GREY_LINE, fontsize=10, radius=0.2)

    rounded_box(ax, 2.5, 3.2, 6.0, 1.2,
                'Level 2 — GenAIInterpreter  (Claude API)',
                'Classifies intent as: navigate | multi_step | door | unknown\n'
                'Returns structured JSON',
                bg=BLUE_LIGHT, fg=BLUE_DARK, ec=BLUE_MID, fontsize=10, radius=0.2)

    rounded_box(ax, 2.5, 1.7, 6.0, 1.2,
                'Level 3 — Companion Brain',
                'Conversation  ·  proactive check-ins  ·  medication reminders  ·  memory',
                bg=GREEN_LIGHT, fg=GREEN_DARK, ec=GREEN_MID, fontsize=10, radius=0.2)
    # ─────────────────────────────────────────────────────────────────────────

    # Deterministic filter
    rounded_box(ax, 8.0, 2.9, 2.7, 1.8,
                'Deterministic\nfilters',
                'NO: medication advice\nNO: emergency info\n'
                'NO: private data\nNO: medical opinions',
                bg=RED_LIGHT, fg=RED_DARK, ec=RED_MID, fontsize=8.5, radius=0.2)

    # Vertical arrows
    arrow(ax, 5.5, 6.1, 5.5, 5.8, color=GREY_LINE, lw=1.6)
    arrow(ax, 5.5, 4.7, 5.5, 4.4, 'unknown input', color=GREY_LINE, lw=1.5)
    arrow(ax, 5.5, 3.2, 5.5, 2.9, 'social / unknown', color=GREY_LINE, lw=1.5)

    # Side exit: known command skips LLM
    ax.annotate('', xy=(9.5, 4.8), xytext=(8.5, 5.25),
        arrowprops=dict(arrowstyle='->', color=BLUE_MID, lw=1.5,
                        connectionstyle='arc3,rad=-0.2'), zorder=5)
    ax.text(9.55, 4.85, '→ RobotBrain\n(known command\nskips LLM)',
            fontsize=8, color=BLUE_MID, va='center')

    # Dashed link: filter ↔ Level 2
    ax.annotate('', xy=(8.5, 3.8), xytext=(8.0, 3.8),
        arrowprops=dict(arrowstyle='<->', color=RED_MID, lw=1.2,
                        linestyle='dashed'), zorder=5)

    source_note(ax,
        'Adapted from: Xue et al. (2025) Frontiers Robotics AI DOI:10.3389/frobt.2025.1627937; '
        'Tagliabue et al. (2023) CEUR Vol-4101.')
    save_fig(fig, 'fig4_4_conversation_pipeline.png')





# ==============================================================================
# FIG 4.6 — Door-crossing action sequence
# ==============================================================================
# SOURCE:
#   Executor module — original dissertation implementation.
#   Reference for FSM-based door handling:
#   Colledanchise & Ögren (2024). arXiv:2405.16137.
# ------------------------------------------------------------------------------
def fig4_6_door_crossing():
    fig, ax = plt.subplots(figsize=(13, 5))
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')
    ax.set_xlim(0, 13)
    ax.set_ylim(0, 5)
    ax.axis('off')

    # ── Edit action names to match your Executor function names ───────────────
    steps = [
        (0.4,  'navigate\ndoor_approach'),
        (2.5,  'verify\nsafe_approach'),
        (4.6,  'open\ndoor'),
        (6.7,  'cross\ndoorway'),        # highlighted as highest-risk step
        (8.8,  'close\ndoor'),
        (10.9, 'continue to\ndestination'),
    ]
    # ─────────────────────────────────────────────────────────────────────────

    for i, (x, label) in enumerate(steps):
        # Highlight doorway crossing as highest-risk
        if 'cross' in label:
            bg, ec, fg = RED_LIGHT, RED_MID, RED_DARK
        else:
            bg, ec, fg = BLUE_LIGHT, BLUE_MID, BLUE_DARK
        rounded_box(ax, x, 2.4, 1.8, 1.4, label,
                    bg=bg, fg=fg, ec=ec, fontsize=9, radius=0.25, bold=True)
        if i < len(steps) - 1:
            arrow(ax, x + 1.8, 3.1, x + 2.5, 3.1, color=BLUE_MID, lw=1.8)

    # Failure branch from verify_safe_approach
    ax.annotate('', xy=(3.4, 2.4), xytext=(3.4, 1.6),
        arrowprops=dict(arrowstyle='->', color=AMBER_MID, lw=1.3), zorder=5)
    rounded_box(ax, 1.8, 0.6, 2.6, 0.85,
                'adjust position → retry', '',
                bg=AMBER_LIGHT, fg=AMBER_DARK, ec=AMBER_MID,
                fontsize=8.5, radius=0.2, bold=False)
    ax.annotate('', xy=(3.4, 2.4), xytext=(3.4, 1.45),
        arrowprops=dict(arrowstyle='<-', color=AMBER_MID, lw=1.2,
                        connectionstyle='arc3,rad=-0.5'), zorder=4)
    ax.text(0.8, 1.95, 'clearance\n< threshold',
            fontsize=8, color=AMBER_MID)

    ax.text(7.6, 2.1, 'highest\nfailure risk',
            fontsize=8, color=RED_MID, ha='center', style='italic')

    source_note(ax,
        'Executor module action sequence. '
        'Door actions verified by ExecutionMonitor (max 3 retries). '
        'FSM pattern from Colledanchise & Ögren (2024) arXiv:2405.16137.')
    save_fig(fig, 'fig4_6_door_crossing.png')


# ==============================================================================
# FIG 4.7 — Assistive escort state machine
# ==============================================================================
# SOURCE:
#   Original dissertation implementation (Assistive State Machine).
#   FSM pattern reference:
#   Colledanchise & Ögren (2024). arXiv:2405.16137.
#   Chen et al. (2017). ResearchGate DOI:10.1109/IROS.2017.xxx
# ------------------------------------------------------------------------------
def fig4_7_escort_fsm():
    fig, ax = plt.subplots(figsize=(11, 6))
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')
    ax.set_xlim(0, 11)
    ax.set_ylim(0, 6)
    ax.axis('off')

    # ── Edit state names / transitions if your FSM differs ────────────────────
    rounded_box(ax, 0.5, 2.3, 2.2, 1.4, 'IDLE', '',
                bg=GREY_LIGHT, fg=GREY_DARK, ec=GREY_LINE,
                fontsize=11, radius=0.35)
    rounded_box(ax, 4.0, 2.3, 2.8, 1.4, 'AWAITING\nLOCATION', '',
                bg=AMBER_LIGHT, fg=AMBER_DARK, ec=AMBER_MID,
                fontsize=11, radius=0.3)
    rounded_box(ax, 8.0, 2.3, 2.5, 1.4, 'ESCORTING', '',
                bg=BLUE_LIGHT, fg=BLUE_DARK, ec=BLUE_MID,
                fontsize=11, radius=0.3)

    # Transitions
    arrow(ax, 2.7, 3.2, 4.0, 3.2,
          'escort request\nreceived', color=BLUE_MID, lw=1.8)
    arrow(ax, 6.8, 3.2, 8.0, 3.2,
          'valid room\nname confirmed', color=GREEN_MID, lw=1.8)

    # Timeout: AWAITING → IDLE
    ax.annotate('', xy=(1.6, 2.3), xytext=(4.4, 2.3),
        arrowprops=dict(arrowstyle='->', color=RED_MID, lw=1.5,
                        connectionstyle='arc3,rad=0.35'), zorder=5)
    ax.text(2.8, 1.3, '30s timeout\n(outputs encouraging message)',
            fontsize=8, color=RED_MID, ha='center')

    # Success: ESCORTING → IDLE
    ax.annotate('', xy=(1.6, 3.7), xytext=(8.25, 3.7),
        arrowprops=dict(arrowstyle='->', color=GREEN_MID, lw=1.5,
                        connectionstyle='arc3,rad=-0.3'), zorder=5)
    ax.text(5.0, 4.7, 'destination reached',
            fontsize=8, color=GREEN_MID, ha='center')

    # Entry to IDLE
    ax.annotate('', xy=(0.5, 3.0), xytext=(-0.4, 3.0),
        arrowprops=dict(arrowstyle='->', color=GREY_LINE, lw=1.5), zorder=5)
    ax.text(-0.35, 3.2, 'start', fontsize=8.5, color=GREY_LINE)
    # ─────────────────────────────────────────────────────────────────────────

    source_note(ax,
        'Original Assistive State Machine implementation. '
        'Runs independently from RobotBrain; suppressed during active navigation. '
        'FSM pattern: Chen et al. (2017); Colledanchise & Ögren (2024) arXiv:2405.16137.')
    save_fig(fig, 'fig4_7_escort_fsm.png')


# ==============================================================================
# FIG 5.1 — Objectives vs achievement matrix
# ==============================================================================
# (Original evaluation — no external source; based on your own project spec)
# ------------------------------------------------------------------------------
def fig5_1_objectives_table():
    # ── Edit rows to match your actual objectives and evaluation ──────────────
    objectives = [
        ('Identify and remember the user',
         'Met',
         'User name stored; used in all greetings and conversation.',
         'Single user only; no face recognition.'),
        ('Natural language conversation',
         'Met',
         'Claude model used for open-ended dialogue.',
         'Hallucination risk; latency from API.'),
        ('Understand spoken commands',
         'Met',
         'Google Speech API transcribes; parser + LLM classify.',
         'Requires internet; fixed recording window.'),
        ('Navigate room-to-room',
         'Met',
         'Dijkstra routes tested across all rooms.',
         'Layout defined in config; not self-mapping.'),
        ('Open doors and cross thresholds',
         'Partial',
         'Works in most rooms; success rate improved iteratively.',
         'Some doors remains fragile; geometry-sensitive.'),
        ('Escort user to destination',
         'Met',
         'Escort FSM verified; location confirmed before movement.',
         '30s timeout; depends on user stating room name.'),
        ('Medication reminders',
         'Met',
         'Scheduled checks against user medication data.',
         'Deterministic only; no confirmation of ingestion.'),
        ('Object retrieval',
         'Partial',
         'Basic pick-and-place implemented.',
         'Old code; causes simulation stops; not integrated with FSM.'),
        ('Operate without internet (fallback)',
         'Met',
         'Keyword parser + scripted companion activate on API failure.',
         'Reduced conversation quality in fallback mode.'),
    ]
    # ─────────────────────────────────────────────────────────────────────────

    STATUS_COLORS = {
        'Met':     (GREEN_LIGHT, GREEN_DARK, GREEN_MID),
        'Partial': (AMBER_LIGHT, AMBER_DARK, AMBER_MID),
        'Not met': (RED_LIGHT,   RED_DARK,   RED_MID),
    }

    col_labels = ['Objective', 'Status', 'Evidence', 'Limitation']
    col_widths  = [0.28, 0.08, 0.34, 0.28]

    xs = [0.01]
    for w in col_widths[:-1]:
        xs.append(xs[-1] + w)

    fig, ax = plt.subplots(figsize=(14, 8))
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')
    ax.axis('off')

    n_rows = len(objectives) + 1
    row_h  = 0.88 / n_rows

    def cell(x, y, w, h, text, bg, fg, bold=False, small=False):
        ax.add_patch(mpatches.FancyBboxPatch(
            (x, y), w, h,
            boxstyle='square,pad=0', linewidth=0.4,
            edgecolor=BORDER, facecolor=bg,
            transform=ax.transAxes, clip_on=False))
        ax.text(x + w / 2, y + h / 2, text,
                transform=ax.transAxes,
                ha='center', va='center',
                fontsize=8.0 if small else 9.2,
                color=fg,
                fontweight='bold' if bold else 'normal',
                multialignment='center', linespacing=1.3)

    # Header
    hdr_y = 1 - row_h
    for label, w, x in zip(col_labels, col_widths, xs):
        cell(x, hdr_y, w, row_h, label, BLUE_MID, 'white', bold=True)

    # Data rows
    for i, (obj, status, evidence, limit) in enumerate(objectives):
        bg_r = '#f9f9f7' if i % 2 == 0 else 'white'
        y    = 1 - (i + 2) * row_h
        sbg, sfg, _ = STATUS_COLORS[status]
        cell(xs[0], y, col_widths[0], row_h, obj,      bg_r, INK,  small=True)
        cell(xs[1], y, col_widths[1], row_h, status,   sbg,  sfg,  bold=True, small=True)
        cell(xs[2], y, col_widths[2], row_h, evidence, bg_r, INK,  small=True)
        cell(xs[3], y, col_widths[3], row_h, limit,    bg_r, GREY_MID, small=True)

    # Legend
    lx, ly = 0.01, 0.04
    for status, (bg, fg, ec) in STATUS_COLORS.items():
        ax.add_patch(mpatches.FancyBboxPatch(
            (lx, ly), 0.04, 0.025,
            boxstyle='square,pad=0', linewidth=0.5,
            edgecolor=ec, facecolor=bg, transform=ax.transAxes))
        ax.text(lx + 0.05, ly + 0.012, status,
                transform=ax.transAxes, fontsize=8.5, color=fg, va='center')
        lx += 0.13

    ax.text(0.5, 0.01,
            'Table 5.1: Evaluation of original project objectives '
            'against implementation outcomes.',
            transform=ax.transAxes, fontsize=8, color=GREY_LINE, ha='center')

    save_fig(fig, 'fig5_1_objectives_table.png')


# ==============================================================================
# MAIN — run all figures
# ==============================================================================
if __name__ == '__main__':
    print('\nGenerating dissertation figures...\n')
    fig1_1_aging_trend()
    fig2_1_robot_table()
    fig3_1_system_architecture()
    fig3_2_hybrid_ai_split()
    fig3_4_room_graph()
    fig4_1_navigation_stack()
    fig4_3_robotbrain_fsm()
    fig4_4_conversation_pipeline()
    fig4_5_voice_pipeline()
    fig4_6_door_crossing()
    fig4_7_escort_fsm()
    fig5_1_objectives_table()
    print(f'\nDone. All PNGs saved to:\n  {OUT}')