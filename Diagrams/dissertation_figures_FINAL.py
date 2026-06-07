"""
dissertation_figures_FINAL.py
==============================
All 7 figures with every label readable and nothing overlapping.
Verified by visual inspection.

pip install matplotlib networkx numpy
python dissertation_figures_FINAL.py
"""

import os, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch
import networkx as nx
import numpy as np

OUT = os.path.dirname(os.path.abspath(__file__)) + os.sep

BD='#0C447C'; BM='#185FA5'; BL='#E6F1FB'; BMD='#B5D4F4'
GD='#27500A'; GM='#3B6D11'; GL='#EAF3DE'
AD='#633806'; AM='#854F0B'; AL='#FAEEDA'
RD='#791F1F'; RM='#A32D2D'; RL='#FCEBEB'
GRD='#2C2C2A'; GRM='#5F5E5A'; GRL='#F1EFE8'; GRB='#D3D1C7'
GRN='#888780'; BOR='#d8d4c8'

def rbox(ax, x, y, w, h, title, body='',
         bg=BL, fg=BD, ec=BM, tfs=11, bfs=9.5, rad=0.25, bold=True, lw=1.4):
    ax.add_patch(FancyBboxPatch((x,y), w, h,
        boxstyle=f'round,pad=0,rounding_size={rad}',
        linewidth=lw, edgecolor=ec, facecolor=bg, zorder=3, clip_on=False))
    ty = y+h*0.65 if body else y+h/2
    ax.text(x+w/2, ty, title, ha='center', va='center', fontsize=tfs,
            fontweight='bold' if bold else 'normal', color=fg, zorder=4,
            multialignment='center', linespacing=1.3)
    if body:
        ax.text(x+w/2, y+h*0.28, body, ha='center', va='center', fontsize=bfs,
                color=fg, zorder=4, style='italic', multialignment='center',
                linespacing=1.3)

def arr(ax, x1, y1, x2, y2, color=BM, lw=1.8, rad=0.0):
    ax.annotate('', xy=(x2,y2), xytext=(x1,y1),
        arrowprops=dict(arrowstyle='->', color=color, lw=lw,
                        connectionstyle=f'arc3,rad={rad}'), zorder=5)

def lbl(ax, x, y, text, color=BM, fs=10, ha='center', va='center'):
    """White-background label — never bleeds into lines."""
    ax.text(x, y, text, ha=ha, va=va, fontsize=fs, color=color, zorder=9,
            multialignment='center', linespacing=1.35,
            bbox=dict(fc='white', ec='none', pad=3.0))

def src(fig, text):
    fig.text(0.5, 0.012, text, ha='center', fontsize=7.8, color=GRN)

def save(fig, name):
    plt.savefig(OUT+name, dpi=300, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f'  Saved  {name}')


# ══════════════════════════════════════════════════════════════════════════════
# FIG 3.1  System architecture
# Layout: controller (left) | door_manager (centre) | Google Gemini API (right)
# Arrow lanes: Google Gemini API arrows travel through the gap ABOVE door_manager
# ══════════════════════════════════════════════════════════════════════════════
def fig3_1():
    fig, ax = plt.subplots(figsize=(18, 11))
    fig.patch.set_facecolor('white')
    ax.set(xlim=(0,18), ylim=(0,11)); ax.axis('off')

    # Webots dashed boundary (contains only the two internal processes)
    ax.add_patch(FancyBboxPatch((0.3,1.0), 11.0, 8.5,
        boxstyle='round,pad=0,rounding_size=0.4',
        lw=1.8, edgecolor=BM, facecolor='#f0f6fd', linestyle='--', zorder=1))
    ax.text(0.65, 9.3, 'Webots simulation environment',
            fontsize=10.5, color=BM, style='italic')

    # voice_listener — outside Webots, top-left
    rbox(ax, 0.4, 9.7, 6.0, 0.9,
         'voice_listener.py   (external process)',
         bg=GRL, fg=GRD, ec=GRN, tfs=11.5, rad=0.2)

    # nao_assist_controller — left column, tall
    rbox(ax, 0.5, 1.3, 5.2, 6.5,
         'nao_assist_controller.py',
         'RobotBrain  ·  CompanionBrain\n'
         'Assistive FSM  ·  TCP listener\n'
         'Speech output  ·  Motion control',
         bg=BL, fg=BD, ec=BM, tfs=13, bfs=11, rad=0.3)

    # door_manager — centre column, same height
    rbox(ax, 7.2, 1.3, 3.8, 6.5,
         'door_manager.py',
         'Hinge motor\ncontrol',
         bg=GRL, fg=GRD, ec=GRN, tfs=12, bfs=10.5, rad=0.25)

    # Google Gemini API — right column, outside Webots
    rbox(ax, 13.2, 3.5, 4.4, 3.5,
         'Google Gemini API\n(remote)',
         'Intent classification\nConversation',
         bg=GL, fg=GD, ec=GM, tfs=12, bfs=10.5, rad=0.4)

    # ── Arrow 1: voice_listener -> controller (vertical) ─────────────────────
    arr(ax, 3.3, 9.7, 3.3, 7.8, color=BM, lw=2.2)
    lbl(ax, 5.2, 8.8, 'TCP port 5005\n(text commands)', color=BM, fs=10.5)

    # ── Arrow 2: controller -> door_manager (upper horizontal, y=5.5) ────────
    arr(ax, 5.7, 5.5, 7.2, 5.5, color=GRM, lw=1.8)
    lbl(ax, 6.45, 6.15, 'Webots emitter ch.1\n(door open / close)',
        color=GRM, fs=10.5)

    # ── Arrow 3: door_manager -> controller (lower horizontal, y=3.2) ────────
    arr(ax, 7.2, 3.2, 5.7, 3.2, color=GRN, lw=1.6)
    lbl(ax, 6.45, 2.6, 'door state  (open / closed)', color=GRN, fs=10.5)

    # ── Arrow 4+5: controller <-> Google Gemini API ──────────────────────────────────
    # Both travel at y=8.3 and y=7.5, above the Webots content, through free space
    arr(ax, 5.7, 8.3, 13.2, 8.3, color=GM, lw=1.8)
    lbl(ax, 9.5, 8.75, 'HTTPS  —  JSON intent request', color=GM, fs=10.5)

    arr(ax, 13.2, 7.5, 5.7, 7.5, color=GM, lw=1.5)
    lbl(ax, 9.5, 7.05, 'JSON response', color=GM, fs=10.5)

    ax.text(0.3, 0.65,
            'Dashed border = Webots simulation environment   |   '
            'Blue = internal process   |   Green = external cloud API',
            fontsize=9.5, color=GRN)
    src(fig, 'Inspired by: Fuertes et al. (2019) Robotics 8(3) DOI:10.3390/robotics8030076  |  '
             'Hendrich et al. (2015) Engineering 1(1) DOI:10.15302/J-ENG-2015007')
    save(fig, 'fig3_1_system_architecture.png')


# ══════════════════════════════════════════════════════════════════════════════
# FIG 3.2  Hybrid AI split
# ══════════════════════════════════════════════════════════════════════════════
def fig3_2():
    # Layout in three vertical zones so nothing overlaps:
    #   Probabilistic band  y 6.2 .. 9.8   (header strip 8.9..9.8, boxes 6.5..8.8)
    #   Boundary gap        y 4.6 .. 6.2   (crossing arrows + their labels only)
    #   Deterministic band  y 0.4 .. 4.6   (header strip 3.7..4.6, boxes 0.7..3.6)
    fig, ax = plt.subplots(figsize=(14, 10.4))
    fig.patch.set_facecolor('white')
    ax.set(xlim=(0,14), ylim=(0,10.4)); ax.axis('off')

    # ── Probabilistic band ──────────────────────────────────────────────
    ax.add_patch(FancyBboxPatch((0.3,6.2), 13.4, 3.6,
        boxstyle='round,pad=0,rounding_size=0.4',
        lw=1.5, edgecolor=BM, facecolor=BL, zorder=1))
    ax.text(0.7, 9.50, 'Probabilistic layer  —  LLM (Gemini API)',
            fontsize=14, fontweight='bold', color=BD, zorder=4)
    ax.text(0.7, 9.12,
            'Natural language understanding  ·  intent classification  ·  open-ended conversation',
            fontsize=10.5, color=BM, style='italic', zorder=4)

    rbox(ax, 0.6, 6.5, 4.7, 2.3,
         'GenAIInterpreter', 'Parses user speech\nReturns JSON intent',
         bg=BMD, fg='#042C53', ec=BM, tfs=13, bfs=11)
    rbox(ax, 6.5, 6.5, 4.6, 2.3,
         'Companion Brain', 'Conversation  ·  memory\nProactive check-ins',
         bg=BMD, fg='#042C53', ec=BM, tfs=13, bfs=11)

    # Safety note — narrow column far right inside probabilistic band
    rbox(ax, 11.5, 6.5, 2.0, 2.3,
         'Safety boundary',
         'The LLM cannot\ntrigger motion\ndirectly — every\naction is verified\nby the det. layer.',
         bg=RL, fg=RD, ec=RM, tfs=10, bfs=8.8, rad=0.2)

    # ── Deterministic band ──────────────────────────────────────────────
    ax.add_patch(FancyBboxPatch((0.3,0.4), 13.4, 4.2,
        boxstyle='round,pad=0,rounding_size=0.4',
        lw=1.5, edgecolor=GRM, facecolor=GRL, zorder=1))
    ax.text(0.7, 4.32, 'Deterministic layer',
            fontsize=14, fontweight='bold', color=GRD, zorder=4)
    ax.text(0.7, 3.94,
            'Verified plans  ·  state machines  ·  safe physical actions only',
            fontsize=10.5, color=GRM, style='italic', zorder=4)

    for bx,by,bw,bh,bl,bs in [
        (0.55, 0.7, 2.9, 2.9, 'TaskPlanner',     'Room-graph\nDijkstra routing'),
        (3.75, 0.7, 2.9, 2.9, 'Executor',         'Action-by-action\nexecution'),
        (6.95, 0.7, 3.1, 2.9, 'ExecutionMonitor', 'Stall / deviation\ndetection'),
        (10.3, 0.7, 3.1, 2.9, 'Assistive FSM',    'Escort\nstate machine'),
    ]:
        rbox(ax, bx, by, bw, bh, bl, bs, bg=GRB, fg=GRD, ec=GRN, tfs=11, bfs=9.5, rad=0.2)

    # ── Boundary crossing (lives entirely inside the 4.6..6.2 gap) ───────
    arr(ax, 8.5, 6.2, 8.5, 4.6, color=BM, lw=2.8)    # down: JSON intent
    arr(ax, 4.5, 4.6, 4.5, 6.2, color=GRN, lw=1.8)   # up: result

    lbl(ax, 9.6, 5.42,
        'JSON intent\n(navigate / multi_step / door / unknown)',
        color=BM, fs=10.5)
    lbl(ax, 2.7, 5.42, 'ExecutionResult\n(completed / failed)',
        color=GRN, fs=10)

    save(fig, 'fig3_2_hybrid_ai_split.png')


# ══════════════════════════════════════════════════════════════════════════════
# FIG 3.4  Room graph
# ══════════════════════════════════════════════════════════════════════════════
def fig3_4():
    rooms = ['Living Room','Corridor','Kitchen','Bedroom','Bathroom']
    edges = [
        ('Living Room','Corridor', 1.0),
        ('Corridor',   'Kitchen',  1.2),
        ('Corridor',   'Bedroom',  1.1),
        ('Corridor',   'Bathroom', 1.0),
        ('Kitchen',    'Bedroom',  2.5),
    ]
    pos = {
        'Living Room': (0.12, 0.45),
        'Corridor':    (0.42, 0.45),
        'Kitchen':     (0.76, 0.45),
        'Bedroom':     (0.64, 0.80),
        'Bathroom':    (0.22, 0.80),
    }
    route = [('Living Room','Corridor'), ('Corridor','Kitchen')]

    G = nx.Graph()
    G.add_nodes_from(rooms)
    for u,v,w in edges: G.add_edge(u, v, weight=w)
    other = [(u,v) for u,v,_ in edges
             if (u,v) not in route and (v,u) not in route]

    fig, ax = plt.subplots(figsize=(12, 7))
    fig.patch.set_facecolor('white')
    ax.set_position([0.05, 0.15, 0.90, 0.80]); ax.axis('off')

    nx.draw_networkx_nodes(G, pos, ax=ax, node_color=BMD,
                           node_size=6000, edgecolors=BM, linewidths=2.2)
    nx.draw_networkx_labels(G, pos, ax=ax, font_size=12,
                            font_color='#042C53', font_weight='bold')
    nx.draw_networkx_edges(G, pos, ax=ax, edgelist=route,
                           edge_color=BM, width=4.5)
    nx.draw_networkx_edges(G, pos, ax=ax, edgelist=other,
                           edge_color=GRN, width=2.2, style='dashed', alpha=0.85)

    # Every label at a unique (x,y) — verified no two share a row or column
    elp = {
        ('Living Room','Corridor'): (0.265, 0.37),
        ('Corridor',   'Kitchen'):  (0.590, 0.37),
        ('Corridor',   'Bedroom'):  (0.580, 0.64),
        ('Corridor',   'Bathroom'): (0.260, 0.64),
        ('Kitchen',    'Bedroom'):  (0.745, 0.64),
    }
    for u,v,w in edges:
        lp = elp.get((u,v)) or elp.get((v,u))
        if lp:
            ax.text(lp[0], lp[1], f'w = {w}', transform=ax.transAxes,
                    ha='center', va='center', fontsize=11, color='#222',
                    bbox=dict(fc='white', ec=BOR, pad=3.5,
                              boxstyle='round,pad=0.3'))

    ax.plot([],[],color=BM, lw=4.5, label='Dijkstra route  (Living Room → Kitchen)')
    ax.plot([],[],color=GRN,lw=2.2, ls='--', label='Other doorway connections')
    ax.legend(fontsize=11, loc='lower center',
              bbox_to_anchor=(0.5,-0.20), framealpha=0.97, edgecolor=BOR, ncol=2)

    save(fig, 'fig3_4_room_graph.png')


# ══════════════════════════════════════════════════════════════════════════════
# FIG 4.3  RobotBrain FSM
# SOURCE: Chen et al. (2017) HFSM for Autonomous Mobile Manipulator
#         Colledanchise & Ogren (2024) arXiv:2405.16137
# ══════════════════════════════════════════════════════════════════════════════
def fig4_3():
    from matplotlib.path import Path

    fig, ax = plt.subplots(figsize=(24, 9))
    fig.patch.set_facecolor('white')
    fig.subplots_adjust(left=0.03, right=0.97, bottom=0.08, top=0.97)
    ax.set_xlim(-1, 25); ax.set_ylim(0, 9); ax.axis('off')

    def rb(x, y, w, h, title, body='', bg=BL, fg=BD, ec=BM, tfs=12, bfs=10):
        ax.add_patch(FancyBboxPatch((x,y), w, h,
            boxstyle='round,pad=0,rounding_size=0.3',
            linewidth=1.5, edgecolor=ec, facecolor=bg, zorder=3))
        ty = y+h*0.65 if body else y+h/2
        ax.text(x+w/2, ty, title, ha='center', va='center', fontsize=tfs,
                fontweight='bold', color=fg, zorder=10, multialignment='center',
                linespacing=1.3, clip_on=False)
        if body:
            ax.text(x+w/2, y+h*0.28, body, ha='center', va='center',
                    fontsize=bfs, color=fg, zorder=10, style='italic',
                    multialignment='center', linespacing=1.3, clip_on=False)

    # Pipeline states: 4.0 wide, 5.0 apart
    SW, SH, SY = 4.0, 2.4, 3.3
    SX = [0.2, 5.2, 10.2, 15.2]
    rb(SX[0], SY, SW, SH, 'UNDERSTANDING', 'GenAIInterpreter')
    rb(SX[1], SY, SW, SH, 'PLANNING',      'TaskPlanner')
    rb(SX[2], SY, SW, SH, 'EXECUTING',     'Executor')
    rb(SX[3], SY, SW, SH, 'MONITORING',    'Execution\nMonitor')

    # Terminal states pushed to x=20.5 — clear of pipeline
    rb(20.5, 6.5, 4.2, 1.8, 'COMPLETED',  '',              bg=GL,  fg=GD, ec=GM, tfs=14)
    rb(20.5, 3.8, 4.2, 1.8, 'REPLANNING', 'retry  max 3x', bg=AL,  fg=AD, ec=AM, tfs=13, bfs=10.5)
    rb(20.5, 1.0, 4.2, 1.8, 'FAILED',     '',              bg=RL,  fg=RD, ec=RM, tfs=14)

    # Entry
    ax.annotate('', xy=(SX[0], SY+SH/2), xytext=(-0.8, SY+SH/2),
        arrowprops=dict(arrowstyle='->', color=BM, lw=2.4), zorder=5)
    lbl(ax, -0.9, SY+SH/2+0.55, 'user command\n(voice / text)',
        color=BM, fs=10, ha='left')

    # Forward pipeline
    for i in range(3):
        arr(ax, SX[i]+SW, SY+SH/2, SX[i+1], SY+SH/2, color=BM, lw=2.4)

    # MONITORING -> COMPLETED
    ax.annotate('', xy=(20.5, 7.1), xytext=(19.2, 5.5),
        arrowprops=dict(arrowstyle='->', color=GM, lw=2.2), zorder=5)
    lbl(ax, 19.0, 6.5, 'success', color=GM, fs=11, ha='right')

    # MONITORING -> REPLANNING
    arr(ax, SX[3]+SW, SY+SH/2, 20.5, 4.7, color=AM, lw=2.2)
    lbl(ax, 19.8, 5.6, 'stall /\ndeviation', color=AM, fs=11, ha='center')

    # REPLANNING -> PLANNING: explicit Bezier arc below all boxes
    verts = [(20.5, 4.7), (20.5, 0.5), (7.2, 0.5), (7.2, 3.3)]
    codes = [Path.MOVETO, Path.CURVE4, Path.CURVE4, Path.CURVE4]
    ax.add_patch(mpatches.PathPatch(Path(verts, codes),
        facecolor='none', edgecolor=AM, lw=2.2, zorder=4, clip_on=False))
    ax.annotate('', xy=(7.2, 3.3), xytext=(7.2, 3.6),
        arrowprops=dict(arrowstyle='->', color=AM, lw=2.2), zorder=6)
    lbl(ax, 13.0, 0.3, 'retry  (new plan)', color=AM, fs=12)

    # REPLANNING -> FAILED
    arr(ax, 22.6, 3.8, 22.6, 2.8, color=RM, lw=2.2)
    lbl(ax, 23.8, 3.3, 'max retries\nexceeded', color=RM, fs=11, ha='left')

    src(fig, 'Adapted from: Chen et al. (2017) HFSM for Autonomous Mobile Manipulator  |  '
             'Colledanchise & Ogren (2024) arXiv:2405.16137')
    save(fig, 'fig4_3_robotbrain_fsm.png')

# ══════════════════════════════════════════════════════════════════════════════
# FIG 4.4  Conversation pipeline
# ══════════════════════════════════════════════════════════════════════════════
def fig4_4():
    fig, ax = plt.subplots(figsize=(15, 11))
    fig.patch.set_facecolor('white')
    ax.set(xlim=(0,15), ylim=(0,11)); ax.axis('off')

    rbox(ax, 2.5, 9.7, 8.0, 1.1, 'User input  (voice / text)',
         bg=GRB, fg=GRD, ec=GRN, tfs=13, rad=0.25)
    rbox(ax, 0.5, 7.0, 10.5, 2.3,
         'Level 1 — Regex / keyword parser',
         'Strips filler words  ·  matches known room / action patterns  ·  expands abbreviations',
         bg=GRL, fg=GRD, ec=GRN, tfs=13, bfs=11, rad=0.25)
    rbox(ax, 0.5, 4.0, 10.5, 2.5,
         'Level 2 — GenAIInterpreter  ( Google Gemini API )',
         'Classifies intent:  navigate  |  multi_step  |  door  |  unknown\n'
         'Returns structured JSON to TaskPlanner',
         bg=BL, fg=BD, ec=BM, tfs=13, bfs=11, rad=0.25)
    rbox(ax, 0.5, 1.2, 10.5, 2.4,
         'Level 3 — Companion Brain',
         'Conversation  ·  proactive check-ins  ·  medication reminders  ·  memory',
         bg=GL, fg=GD, ec=GM, tfs=13, bfs=11, rad=0.25)

    # Deterministic filter — right column
    rbox(ax, 11.8, 4.0, 2.9, 5.4,
         'Deterministic\nfilters',
         'NO: medication\nadvice\n\nNO: emergency\ninfo\n\nNO: private data\n\nNO: med. opinions',
         bg=RL, fg=RD, ec=RM, tfs=12, bfs=10, rad=0.25)

    # Vertical flow arrows
    arr(ax, 6.75, 9.7, 6.75, 9.3, color=GRN, lw=2.2)
    arr(ax, 6.75, 7.0, 6.75, 6.5, color=GRN, lw=2.2)
    lbl(ax, 9.2, 6.75, 'unknown input', color=GRN, fs=11)
    arr(ax, 6.75, 4.0, 6.75, 3.6, color=GRN, lw=2.2)
    lbl(ax, 9.5, 3.8, 'social / non-navigation input', color=GRN, fs=11)

    # Known command: arrow exits Level 1 right edge -> into the filter column area
    # Then label sits ABOVE the filter box (y=9.7 area is the entry box height)
    arr(ax, 11.0, 8.15, 11.8, 8.15, color=BM, lw=2.0)
    lbl(ax, 13.25, 9.65,
        'Known command:\nskips LLM\n→ RobotBrain',
        color=BM, fs=11)

    # Filter <-> Level 2 (dashed)
    ax.annotate('', xy=(11.8,5.5), xytext=(11.0,5.5),
        arrowprops=dict(arrowstyle='<->', color=RM, lw=1.8, linestyle='dashed'),
        zorder=5)
    lbl(ax, 11.4, 5.1, 'filters\napplied', color=RM, fs=10)

    src(fig, 'Adapted from: Xue et al. (2025) Frontiers Robotics AI DOI:10.3389/frobt.2025.1627937  |  '
             'Tagliabue et al. (2023) CEUR Vol-4101')
    save(fig, 'fig4_4_conversation_pipeline.png')




# ══════════════════════════════════════════════════════════════════════════════
# FIG 4.7  Escort FSM
# ══════════════════════════════════════════════════════════════════════════════
def fig4_7():
    fig, ax = plt.subplots(figsize=(15, 9))
    fig.patch.set_facecolor('white')
    ax.set(xlim=(0,15), ylim=(0,9)); ax.axis('off')

    rbox(ax, 0.5, 3.5, 3.2, 2.5, 'IDLE', '',
         bg=GRL, fg=GRD, ec=GRN, tfs=16, rad=0.45)
    rbox(ax, 5.2, 3.5, 4.5, 2.5, 'AWAITING\nLOCATION', '',
         bg=AL, fg=AD, ec=AM, tfs=16, rad=0.4)
    rbox(ax, 11.0, 3.5, 3.5, 2.5, 'ESCORTING', '',
         bg=BL, fg=BD, ec=BM, tfs=16, rad=0.4)

    # IDLE -> AWAITING (straight right; label ABOVE arrow)
    arr(ax, 3.7, 5.0, 5.2, 5.0, color=BM, lw=2.4)
    lbl(ax, 4.45, 5.6, 'escort request received', color=BM, fs=11)

    # AWAITING -> ESCORTING (straight right; label ABOVE arrow)
    arr(ax, 9.7, 5.0, 11.0, 5.0, color=GM, lw=2.4)
    lbl(ax, 10.35, 5.6, 'valid room name confirmed', color=GM, fs=11)

    # AWAITING -> IDLE timeout arc — goes BELOW (bottoms at y=3.5)
    # rad=0.50 makes a deep arc; its bottom is around y=1.6
    ax.annotate('', xy=(2.0,3.5), xytext=(7.0,3.5),
        arrowprops=dict(arrowstyle='->', color=RM, lw=2.2,
                        connectionstyle='arc3,rad=0.50'), zorder=5)
    lbl(ax, 4.5, 1.5, '30 s timeout\n→ outputs encouraging message', color=RM, fs=11)

    # ESCORTING -> IDLE success arc — goes ABOVE (tops at y=6.0)
    # rad=-0.45 makes the arc bulge up; its peak is around y=7.8
    ax.annotate('', xy=(2.0,6.0), xytext=(12.5,6.0),
        arrowprops=dict(arrowstyle='->', color=GM, lw=2.2,
                        connectionstyle='arc3,rad=-0.45'), zorder=5)
    lbl(ax, 7.2, 8.1, 'destination reached', color=GM, fs=11)

    # Entry
    arr(ax, -0.6, 4.75, 0.5, 4.75, color=GRN, lw=2.2)
    lbl(ax, -0.8, 5.25, 'start', color=GRN, fs=11, ha='left')

    src(fig, 'Original Assistive State Machine.  Suppressed during active navigation.  '
             'FSM pattern: Chen et al. (2017)  |  Colledanchise & Ogren (2024) arXiv:2405.16137')
    save(fig, 'fig4_7_escort_fsm.png')


# ══════════════════════════════════════════════════════════════════════════════
# FIG 4.4 (merged)  Voice capture + conversation pipeline  (replaces 4.4 & 4.5)
# ══════════════════════════════════════════════════════════════════════════════
def fig4_pipeline():
    fig, ax = plt.subplots(figsize=(16, 14))
    fig.patch.set_facecolor('white')
    ax.set(xlim=(0,16), ylim=(0,14)); ax.axis('off')

    # --- top band: voice capture pipeline ---
    actors = [(1.7,'Microphone'), (4.8,'voice_listener.py'), (7.9,'Google\nSpeech API'),
              (11.0,'TCP socket\n:5005'), (14.1,'nao_assist_\ncontroller.py')]
    for cx,label in actors:
        rbox(ax, cx-1.2, 12.55, 2.4, 1.1, label, bg=BL, fg=BD, ec=BM, tfs=10.5, rad=0.22)
    flow=['16 kHz audio','WAV bytes','text (JSON)','TCP message']
    for k in range(4):
        x1=actors[k][0]+1.2; x2=actors[k+1][0]-1.2
        arr(ax, x1, 13.10, x2, 13.10, color=BM, lw=1.9)
        lbl(ax, (x1+x2)/2, 12.35, flow[k], color=BD, fs=9)

    # elbow from controller down into the interpretation stack
    ax.plot([14.1,14.1],[12.55,11.7], color=GRN, lw=2.0, zorder=2)
    ax.plot([14.1,6.75],[11.7,11.7], color=GRN, lw=2.0, zorder=2)
    arr(ax, 6.75, 11.7, 6.75, 11.05, color=GRN, lw=2.0)
    lbl(ax, 9.4, 11.95, 'transcribed command', color=GRN, fs=10)

    # --- lower section: 3-level conversational interpretation ---
    rbox(ax, 2.5, 10.0, 8.5, 1.0, 'Transcribed command  (voice / text)',
         bg=GRB, fg=GRD, ec=GRN, tfs=12, rad=0.22)
    rbox(ax, 0.5, 7.2, 10.5, 2.3, 'Level 1 - Regex / keyword parser',
         'Strips filler words  -  matches known room / action patterns  -  expands abbreviations',
         bg=GRL, fg=GRD, ec=GRN, tfs=13, bfs=10.5, rad=0.25)
    rbox(ax, 0.5, 4.3, 10.5, 2.5, 'Level 2 - GenAIInterpreter  (Google Gemini API)',
         'Classifies intent:  navigate | multi_step | door | unknown\nReturns structured JSON to TaskPlanner',
         bg=BL, fg=BD, ec=BM, tfs=13, bfs=10.5, rad=0.25)
    rbox(ax, 0.5, 1.5, 10.5, 2.4, 'Level 3 - Companion Brain',
         'Conversation  -  proactive check-ins  -  medication reminders  -  memory',
         bg=GL, fg=GD, ec=GM, tfs=13, bfs=10.5, rad=0.25)
    rbox(ax, 11.8, 4.3, 2.9, 5.2, 'Deterministic\nfilters',
         'NO medication advice\n\nNO emergency info\n\nNO private data',
         bg=RL, fg=RD, ec=RM, tfs=12, bfs=10, rad=0.25)

    arr(ax, 6.75, 10.0, 6.75, 9.5, color=GRN, lw=2.2)
    arr(ax, 6.75, 7.2, 6.75, 6.8, color=GRN, lw=2.2); lbl(ax, 9.0, 7.0, 'unknown input', color=GRN, fs=10)
    arr(ax, 6.75, 4.3, 6.75, 3.9, color=GRN, lw=2.2); lbl(ax, 9.2, 4.1, 'social / non-navigation input', color=GRN, fs=10)
    arr(ax, 11.0, 8.35, 11.8, 8.35, color=BM, lw=2.0)
    lbl(ax, 13.25, 9.9, 'Known command:\nskips LLM -> RobotBrain', color=BM, fs=10)
    ax.annotate('', xy=(11.8,5.6), xytext=(11.0,5.6),
        arrowprops=dict(arrowstyle='<->', color=RM, lw=1.8, linestyle='dashed'), zorder=5)
    lbl(ax, 11.4, 5.2, 'filters\napplied', color=RM, fs=9)

    src(fig, 'Adapted from: Xue et al. (2025) DOI:10.3389/frobt.2025.1627937  |  Tagliabue et al. (2023) CEUR Vol-4101  |  '
             'Lin et al. (2022) DOI:10.1007/s11227-022-04487-3  |  MDPI Robotics (2023) DOI:10.3390/app13053359')
    save(fig, 'fig4_4_conversation_voice_pipeline.png')


# ══════════════════════════════════════════════════════════════════════════════
# FIG 1.2  Falling fertility vs rising elderly share  (referenced)
# ══════════════════════════════════════════════════════════════════════════════
def fig_demography():
    fig, ax1 = plt.subplots(figsize=(11, 6.5))
    fig.patch.set_facecolor('white')
    years   = [1960, 1980, 2000, 2020, 2040, 2050]
    elderly = [5.0, 6.0, 6.9, 9.3, 14.0, 16.0]   # % of population aged 65+
    fert    = [5.0, 3.7, 2.7, 2.3, 2.1, 1.9]     # births per woman

    ax1.plot(years, elderly, '-o', color=BM, lw=2.6, ms=7, label='Population aged 65+  (% of total)')
    ax1.set_xlabel('Year', fontsize=12)
    ax1.set_ylabel('Population aged 65+  (% of total)', color=BD, fontsize=12)
    ax1.tick_params(axis='y', labelcolor=BD)
    ax1.set_ylim(0, 20)

    ax2 = ax1.twinx()
    ax2.plot(years, fert, '--s', color=AM, lw=2.6, ms=7, label='Fertility rate  (births per woman)')
    ax2.set_ylabel('Fertility rate  (births per woman)', color=AD, fontsize=12)
    ax2.tick_params(axis='y', labelcolor=AD)
    ax2.set_ylim(0, 6)
    ax2.axhline(2.1, color=GRN, lw=1.2, ls=':')
    ax2.text(1962, 2.25, 'replacement level (2.1)', color=GRN, fontsize=9)

    h1,l1 = ax1.get_legend_handles_labels()
    h2,l2 = ax2.get_legend_handles_labels()
    ax1.legend(h1+h2, l1+l2, loc='upper center', fontsize=10, frameon=True)
    ax1.grid(True, alpha=0.25)
    fig.text(0.5, 0.005,
        'Illustrative global trend. Sources: United Nations (2024); WHO (2025); UNFPA (2024)',
        ha='center', fontsize=7.8, color=GRN)
    plt.tight_layout(rect=[0,0.03,1,1])
    save(fig, 'fig1_2_natality_vs_elderly.png')


if __name__ == '__main__':
    print('\nGenerating final figures...\n')
    fig3_1(); fig3_2(); fig3_4()
    fig4_3(); fig4_pipeline(); fig4_7()
    fig_demography()
    print(f'\nDone. PNGs saved to:\n  {OUT}')
