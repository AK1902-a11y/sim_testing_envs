#!/usr/bin/env python3
"""ESDF test bench: pick a testing_env, press Run, press Execute path.

  ros2 run esdf_sim_bringup sim_gui.py
  python3 ~/ros2_ws/src/esdf_sim_bringup/scripts/sim_gui.py

* The settings fields are filled from config/nav2_params.yaml and refresh when you edit that file.
  Only the fields you change in the GUI are applied (to a per-run copy of the YAML); your YAML is never modified.
* Results go to  ~/esdf_local_costmap_benchmarking/test_results_sim/<run folder name>/  - you only type the name.
  The folder gets the two stats CSVs (file names as in the YAML), nav2_params.yaml (the exact copy used),
  run_info.json, launch.log and route_status.json.
* "Execute path" calls the route_runner node (started by navigation.launch.py), which drives the env's goal list.
"""
import datetime
import json
import os
import queue
import re
import shlex
import signal
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import messagebox, ttk

import yaml

HOME = os.path.expanduser('~')
SETUP_FILES = os.environ.get('ESDF_GUI_SETUP', ':'.join([
    '/opt/ros/humble/setup.bash', f'{HOME}/nav2_ws/install/setup.bash', f'{HOME}/ros2_ws/install/setup.bash'])).split(':')
RESULTS_ROOT = f'{HOME}/esdf_local_costmap_benchmarking/test_results_sim'
SRC_BRINGUP = f'{HOME}/ros2_ws/src/esdf_sim_bringup'

MODES = {   # label -> (local costmap plugins, FH compares against ours)
    'Both: ours + FH (correctness + paired timing)': (['obstacle_layer', 'esdf_layer', 'fh_layer'], True),
    'Ours only (ESDF layer)': (['obstacle_layer', 'esdf_layer'], False),
    'FH only (baseline)': (['obstacle_layer', 'fh_layer'], False),
    'Neither (plain Nav2 baseline)': (['obstacle_layer'], False),
}
CUSTOM = 'As in the YAML (custom plugin list)'
STACKS = {  # label -> (launch file, takes map, takes nav_params + route)
    'Full navigation (sim + AMCL + Nav2 + route runner)': ('navigation.launch.py', True, True),
    'Localization only (sim + AMCL)': ('localization.launch.py', True, False),
    'Simulation only (Gazebo + robot)': ('sim.launch.py', False, False),
}

# ---- light theme
BG, CARD, BORDER = '#f3f5f9', '#ffffff', '#e1e6ef'
TEXT, MUTED = '#1f2a44', '#6b7690'
ACCENT, ACCENT_D, ACCENT_L = '#3b6cf4', '#2f57c9', '#eaf0ff'
GREEN, RED, AMBER = '#1f9d62', '#d64545', '#c47f00'
FONT = ('DejaVu Sans', 10)
FONT_B = ('DejaVu Sans', 10, 'bold')
FONT_S = ('DejaVu Sans', 9)
FONT_H = ('DejaVu Sans', 16, 'bold')


def bringup_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory('esdf_sim_bringup')
    except Exception:
        return SRC_BRINGUP


def load_registry():
    for base in (bringup_dir(), SRC_BRINGUP):
        p = os.path.join(base, 'config', 'testing_envs.yaml')
        if os.path.exists(p):
            return base, yaml.safe_load(open(p))['envs']
    raise SystemExit('testing_envs.yaml not found - run generate_world.py --all and rebuild esdf_sim_bringup')


def lc_params(cfg):
    return cfg['local_costmap']['local_costmap']['ros__parameters']


def read_yaml_settings(path):
    """The values the GUI fields show, taken from the current nav2_params.yaml."""
    c = yaml.safe_load(open(path))
    lc = lc_params(c)
    mode = CUSTOM
    for label, (plugins, _) in MODES.items():
        if list(lc['plugins']) == plugins:
            mode = label
    scan = lc['obstacle_layer']['scan']
    return dict(
        mode=mode, window=lc['width'], dmax=lc['esdf_layer']['dmax'], check=lc['esdf_layer'].get('self_check_every', 0),
        obs=scan['obstacle_max_range'], ray=scan['raytrace_max_range'], res=lc.get('resolution', 0.05),
        freq=lc.get('update_frequency', 5.0),
        speed=c.get('controller_server', {}).get('ros__parameters', {}).get('FollowPath', {}).get('desired_linear_vel', 0.4),
        esdf_csv=os.path.basename(lc['esdf_layer'].get('stats_file') or 'esdf_local_costmap_stats.csv'),
        fh_csv=os.path.basename(lc['fh_layer'].get('stats_file') or 'fh_local_costmap_stats.csv'))


def fmt(v):
    return f'{v:g}' if isinstance(v, (int, float)) else str(v)


def derive_params(base_params, out_path, overrides, stats_dir):
    """Copy of the YAML: only `overrides` (fields the user changed) are applied, plus the stats file locations."""
    cfg = yaml.safe_load(open(base_params))
    lc = lc_params(cfg)
    e, f, scan = lc['esdf_layer'], lc['fh_layer'], lc['obstacle_layer']['scan']
    if 'mode' in overrides and overrides['mode'] in MODES:
        plugins, compare = MODES[overrides['mode']]
        lc['plugins'] = plugins
        f['compare_with'] = 'esdf_layer' if compare else ''
    if 'window' in overrides:
        # Nav2 declares width/height as INTEGER parameters: a float (5.0) makes controller_server abort
        lc['width'] = lc['height'] = int(round(float(overrides['window'])))
    if 'res' in overrides:
        lc['resolution'] = float(overrides['res'])
    if 'freq' in overrides:
        lc['update_frequency'] = float(overrides['freq'])   # double parameter in Nav2
    if 'dmax' in overrides:
        e['dmax'] = f['dmax'] = float(overrides['dmax'])
    if 'check' in overrides:
        e['self_check_every'] = int(overrides['check'])
    if 'obs' in overrides:
        scan['obstacle_max_range'] = float(overrides['obs'])
    if 'ray' in overrides:
        scan['raytrace_max_range'] = float(overrides['ray'])
    e['stats_file'] = os.path.join(stats_dir, os.path.basename(e.get('stats_file') or 'esdf_local_costmap_stats.csv'))
    f['stats_file'] = os.path.join(stats_dir, os.path.basename(f.get('stats_file') or 'fh_local_costmap_stats.csv'))
    with open(out_path, 'w') as fh:
        yaml.safe_dump(cfg, fh, sort_keys=False, default_flow_style=False)


def next_run_name(window, res):
    nums = [int(m.group(1)) for d in (os.listdir(RESULTS_ROOT) if os.path.isdir(RESULTS_ROOT) else [])
            if (m := re.match(r'test(\d+)', d))]
    try:
        cells = int(round(float(window) / float(res)))
    except (ValueError, ZeroDivisionError):
        cells = 0
    return f'test{max(nums, default=0) + 1}_{cells}m'


# ======================================================================================== GUI
class App(tk.Tk):
    FIELDS = ('window', 'res', 'freq', 'dmax', 'check', 'obs', 'ray')

    def __init__(self):
        super().__init__()
        self.title('ESDF Test Bench')
        self.geometry('1280x900')
        self.minsize(1100, 760)
        self.configure(bg=BG)
        self.base, self.envs = load_registry()
        self.yaml_path = os.path.join(self.base, 'config', 'nav2_params.yaml')
        self.proc = None
        self.run_dir = None
        self.log_q = queue.Queue()
        self.logfile = None
        self.photo = None
        self.status = {}
        self.name_auto = True
        self.env_cards = {}

        self.v_env = tk.StringVar(value=next(iter(self.envs)))
        self.v_stack = tk.StringVar(value=list(STACKS)[0])
        self.v = {k: tk.StringVar() for k in ('mode',) + self.FIELDS}
        self.v_noise = tk.StringVar(value='0.0')
        self.v_rviz = tk.BooleanVar(value=True)
        self.v_headless = tk.BooleanVar(value=False)
        self.v_name = tk.StringVar()
        self.v_laps = tk.StringVar(value='1')
        self.v_auto = tk.BooleanVar(value=False)
        self.v_status = tk.StringVar(value='Stopped')
        self.v_route = tk.StringVar(value='Start the stack to enable the route.')
        self.v_note = tk.StringVar(value='')
        self.widgets = {}

        self.loaded = {}
        self.lidar_hz = self._read_lidar_hz()
        self._style()
        self._build_ui()
        self._yaml_mtime = 0
        self._reload_yaml(force=True)
        self._select_env(self.v_env.get())
        self.after(500, self._poll)
        self.after(100, self._pump_log)
        self.protocol('WM_DELETE_WINDOW', self._on_close)

    @staticmethod
    def _read_lidar_hz():
        try:
            from ament_index_python.packages import get_package_share_directory
            p = os.path.join(get_package_share_directory('esdf_sim_description'), 'config', 'robot_params.yaml')
            return float(yaml.safe_load(open(p))['esdf_bot']['lidar']['rate'])
        except Exception:
            return 10.0

    # ------------------------------------------------------------------ styling
    def _style(self):
        s = ttk.Style(self)
        s.theme_use('clam')
        s.configure('.', background=BG, foreground=TEXT, font=FONT)
        s.configure('TFrame', background=BG)
        s.configure('Card.TFrame', background=CARD)
        s.configure('TLabel', background=CARD, foreground=TEXT)
        s.configure('Muted.TLabel', background=CARD, foreground=MUTED, font=FONT_S)
        s.configure('TCheckbutton', background=CARD, foreground=TEXT)
        s.map('TCheckbutton', background=[('active', CARD)])
        for name, fieldbg, fg in (('TEntry', '#ffffff', TEXT), ('Dirty.TEntry', ACCENT_L, ACCENT_D)):
            s.configure(name, fieldbackground=fieldbg, foreground=fg, bordercolor=BORDER, lightcolor=BORDER,
                        darkcolor=BORDER, padding=4)
        s.map('TEntry', bordercolor=[('focus', ACCENT)], lightcolor=[('focus', ACCENT)], darkcolor=[('focus', ACCENT)])
        s.map('Dirty.TEntry', bordercolor=[('focus', ACCENT)], lightcolor=[('focus', ACCENT)], darkcolor=[('focus', ACCENT)])
        for name, fieldbg, fg in (('TCombobox', '#ffffff', TEXT), ('Dirty.TCombobox', ACCENT_L, ACCENT_D)):
            s.configure(name, fieldbackground=fieldbg, foreground=fg, bordercolor=BORDER, lightcolor=BORDER,
                        darkcolor=BORDER, arrowcolor=MUTED, padding=4, background=CARD)
            s.map(name, fieldbackground=[('readonly', fieldbg)], foreground=[('readonly', fg)],
                  selectbackground=[('readonly', fieldbg)], selectforeground=[('readonly', fg)])
        self.option_add('*TCombobox*Listbox.background', '#ffffff')
        self.option_add('*TCombobox*Listbox.foreground', TEXT)
        self.option_add('*TCombobox*Listbox.selectBackground', ACCENT)
        for name, bg, fg, active in (('Accent.TButton', ACCENT, '#ffffff', ACCENT_D), ('Green.TButton', GREEN, '#ffffff', '#188050'),
                                     ('Danger.TButton', RED, '#ffffff', '#b93636'), ('Ghost.TButton', CARD, TEXT, '#eef1f7')):
            s.configure(name, background=bg, foreground=fg, bordercolor=BORDER if name == 'Ghost.TButton' else bg,
                        focusthickness=0, padding=(14, 7), font=FONT_B, relief='flat')
            s.map(name, background=[('disabled', '#e8ebf2'), ('active', active)], foreground=[('disabled', '#9aa3b8')],
                  bordercolor=[('disabled', '#e8ebf2')])
        s.configure('Treeview', background='#ffffff', fieldbackground='#ffffff', foreground=TEXT, rowheight=24,
                    bordercolor=BORDER, font=FONT_S)
        s.configure('Treeview.Heading', background='#eef1f7', foreground=MUTED, font=FONT_B, relief='flat')
        s.map('Treeview', background=[('selected', ACCENT_L)], foreground=[('selected', TEXT)])
        s.configure('Vertical.TScrollbar', background='#d9deea', troughcolor=BG, bordercolor=BG, arrowcolor=MUTED)

    def card(self, parent, title=None, **grid):
        outer = tk.Frame(parent, bg=CARD, highlightbackground=BORDER, highlightthickness=1)
        if grid:
            outer.grid(**grid)
        if title:
            tk.Label(outer, text=title, bg=CARD, fg=MUTED, font=FONT_B, anchor='w').pack(fill='x', padx=14, pady=(10, 2))
        body = ttk.Frame(outer, style='Card.TFrame', padding=(14, 4, 14, 12))
        body.pack(fill='both', expand=True)
        return outer, body

    # ------------------------------------------------------------------ layout
    def _build_ui(self):
        head = tk.Frame(self, bg=CARD, highlightbackground=BORDER, highlightthickness=1)
        head.pack(fill='x')
        tk.Label(head, text='ESDF Test Bench', bg=CARD, fg=TEXT, font=FONT_H).pack(side='left', padx=(18, 8), pady=12)
        tk.Label(head, text='rolling-window ESDF vs FH in Nav2', bg=CARD, fg=MUTED, font=FONT).pack(side='left')
        self.pill = tk.Label(head, textvariable=self.v_status, bg='#e8ebf2', fg=MUTED, font=FONT_B, padx=14, pady=5)
        self.pill.pack(side='right', padx=18)

        body = ttk.Frame(self, padding=12)
        body.pack(fill='both', expand=True)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        # --- environments (left)
        left = ttk.Frame(body)
        left.grid(row=0, column=0, sticky='ns', padx=(0, 12))
        tk.Label(left, text='TESTING ENVIRONMENTS', bg=BG, fg=MUTED, font=FONT_S).pack(anchor='w', pady=(0, 4))
        for name, e in self.envs.items():
            self._env_card(left, name, e)
        _, ib = self.card(left, 'Selected environment')
        self.info = tk.Label(ib, text='', bg=CARD, fg=TEXT, font=FONT_S, justify='left', anchor='nw', wraplength=250)
        self.info.pack(fill='x')
        ib.master.pack_configure(pady=(8, 0), fill='x')

        # --- map + route (middle)
        mid = ttk.Frame(body)
        mid.grid(row=0, column=1, sticky='nsew')
        mid.columnconfigure(0, weight=1)
        _, mb = self.card(mid, 'Map  -  start (green), goal route (red)', row=0, column=0, sticky='ew')
        self.canvas = tk.Canvas(mb, width=380, height=300, bg='#f0f2f7', highlightthickness=0)
        self.canvas.pack()
        _, rb = self.card(mid, 'Route', row=1, column=0, sticky='nsew', pady=(10, 0))
        mid.rowconfigure(1, weight=1)
        self._route_card(rb)

        # --- settings (right)
        _, sb = self.card(body, 'Run settings', row=0, column=2, sticky='ns', padx=(12, 0))
        self._settings_card(sb)

        # --- action bar + log
        bottom = ttk.Frame(self, padding=(12, 0, 12, 12))
        bottom.pack(fill='both', expand=False)
        actions = ttk.Frame(bottom)
        actions.pack(fill='x', pady=(0, 8))
        self.b_run = ttk.Button(actions, text='▶   Run stack', style='Accent.TButton', command=self.start)
        self.b_run.pack(side='left')
        self.b_stop = ttk.Button(actions, text='■   Stop', style='Danger.TButton', command=self.stop, state='disabled')
        self.b_stop.pack(side='left', padx=8)
        ttk.Button(actions, text='Open results folder', style='Ghost.TButton', command=self.open_results).pack(side='left')
        ttk.Button(actions, text='Copy analysis command', style='Ghost.TButton', command=self.copy_analysis).pack(side='left', padx=8)
        tk.Label(actions, textvariable=self.v_note, bg=BG, fg=MUTED, font=FONT_S).pack(side='right')
        lo, lb = self.card(bottom, 'Launch output')
        lo.pack(fill='both', expand=True)
        self.log = tk.Text(lb, height=11, wrap='none', font=('DejaVu Sans Mono', 9), bg='#fafbfd', fg=TEXT, bd=0,
                           highlightthickness=0, state='disabled', padx=6, pady=4)
        sc = ttk.Scrollbar(lb, command=self.log.yview)
        self.log['yscrollcommand'] = sc.set
        sc.pack(side='right', fill='y')
        self.log.pack(fill='both', expand=True)
        self.log.tag_config('err', foreground=RED)
        self.log.tag_config('warn', foreground=AMBER)
        self.log.tag_config('cmd', foreground=ACCENT_D)

    def _env_card(self, parent, name, e):
        f = tk.Frame(parent, bg=CARD, highlightbackground=BORDER, highlightthickness=2, cursor='hand2')
        f.pack(fill='x', pady=4)
        t = tk.Label(f, text=name.replace('testing_', 'testing_'), bg=CARD, fg=TEXT, font=FONT_B, anchor='w')
        t.pack(fill='x', padx=12, pady=(8, 0))
        d = tk.Label(f, text=e['title'], bg=CARD, fg=ACCENT_D, font=FONT_S, anchor='w')
        d.pack(fill='x', padx=12)
        st = e.get('stats', {})
        x = tk.Label(f, text=f"{e['obstacles']} shapes  ·  {st.get('edge_cells_10m_window_mean', '?')} edge cells / 10 m window",
                     bg=CARD, fg=MUTED, font=FONT_S, anchor='w')
        x.pack(fill='x', padx=12, pady=(0, 8))
        self.env_cards[name] = (f, (t, d, x))
        for w in (f, t, d, x):
            w.bind('<Button-1>', lambda ev, n=name: self._select_env(n))

    def _field(self, parent, r, label, key, hint=''):
        ttk.Label(parent, text=label).grid(row=r, column=0, sticky='w', pady=4)
        e = ttk.Entry(parent, textvariable=self.v[key], width=10)
        e.grid(row=r, column=1, sticky='w', padx=(10, 0), pady=4)
        if hint:
            ttk.Label(parent, text=hint, style='Muted.TLabel').grid(row=r, column=2, sticky='w', padx=8)
        self.widgets[key] = e
        self.v[key].trace_add('write', lambda *a, k=key: (self._mark(k), self._on_dims() if k in ('window', 'res', 'freq') else None))

    def _settings_card(self, p):
        r = 0
        ttk.Label(p, text='Launch').grid(row=r, column=0, sticky='w', pady=4)
        ttk.Combobox(p, textvariable=self.v_stack, values=list(STACKS), state='readonly', width=44).grid(
            row=r, column=1, columnspan=2, sticky='w', padx=(10, 0)); r += 1
        ttk.Label(p, text='Costmap mode').grid(row=r, column=0, sticky='w', pady=4)
        cb = ttk.Combobox(p, textvariable=self.v['mode'], values=list(MODES) + [CUSTOM], state='readonly', width=44)
        cb.grid(row=r, column=1, columnspan=2, sticky='w', padx=(10, 0)); r += 1
        self.widgets['mode'] = cb
        self.v['mode'].trace_add('write', lambda *a: self._mark('mode'))
        ttk.Separator(p).grid(row=r, column=0, columnspan=3, sticky='ew', pady=6); r += 1
        ttk.Label(p, text='Values from nav2_params.yaml  (blue = changed here)', style='Muted.TLabel').grid(
            row=r, column=0, columnspan=3, sticky='w'); r += 1
        for key, label, hint in (('window', 'Local window [m]', 'width = height'),
                                 ('res', 'Resolution [m/cell]', 'local costmap'),
                                 ('freq', 'Update frequency [Hz]', 'local costmap'),
                                 ('dmax', 'dmax [m]', 'distance cap'),
                                 ('check', 'Self-check every k', '0 = off'), ('obs', 'Obstacle max range [m]', 'marking'),
                                 ('ray', 'Raytrace max range [m]', 'clearing')):
            self._field(p, r, label, key, hint); r += 1
        self.v_grid = tk.StringVar(value='')
        ttk.Label(p, textvariable=self.v_grid, style='Muted.TLabel', wraplength=420, justify='left').grid(
            row=r, column=0, columnspan=3, sticky='w', pady=(4, 0)); r += 1
        ttk.Button(p, text='↻  Reload from YAML', style='Ghost.TButton', command=lambda: self._reload_yaml(force=True)).grid(
            row=r, column=0, columnspan=2, sticky='w', pady=(6, 2)); r += 1
        ttk.Separator(p).grid(row=r, column=0, columnspan=3, sticky='ew', pady=6); r += 1
        ttk.Label(p, text='Lidar noise stddev [m]').grid(row=r, column=0, sticky='w', pady=4)
        ttk.Entry(p, textvariable=self.v_noise, width=10).grid(row=r, column=1, sticky='w', padx=(10, 0)); r += 1
        c = ttk.Frame(p, style='Card.TFrame'); c.grid(row=r, column=0, columnspan=3, sticky='w', pady=4); r += 1
        ttk.Checkbutton(c, text='RViz', variable=self.v_rviz).pack(side='left')
        ttk.Checkbutton(c, text='Headless Gazebo (no GUI)', variable=self.v_headless).pack(side='left', padx=14)
        ttk.Separator(p).grid(row=r, column=0, columnspan=3, sticky='ew', pady=6); r += 1
        ttk.Label(p, text='Results folder name').grid(row=r, column=0, sticky='w', pady=(2, 0)); r += 1
        ent = ttk.Entry(p, textvariable=self.v_name, width=30)
        ent.grid(row=r, column=0, columnspan=3, sticky='ew', pady=4); r += 1
        ent.bind('<KeyRelease>', lambda e: setattr(self, 'name_auto', False))
        ttk.Label(p, text=f'saved in  {RESULTS_ROOT.replace(HOME, "~")}/<name>/', style='Muted.TLabel',
                  wraplength=400).grid(row=r, column=0, columnspan=3, sticky='w')

    def _route_card(self, p):
        top = ttk.Frame(p, style='Card.TFrame'); top.pack(fill='x')
        ttk.Label(top, text='Laps').pack(side='left')
        ttk.Entry(top, textvariable=self.v_laps, width=4).pack(side='left', padx=(6, 14))
        ttk.Checkbutton(top, text='start automatically when Nav2 is ready', variable=self.v_auto).pack(side='left')
        btns = ttk.Frame(p, style='Card.TFrame'); btns.pack(fill='x', pady=(8, 6))
        self.b_exec = ttk.Button(btns, text='▶   Execute path', style='Green.TButton', command=self.exec_path, state='disabled')
        self.b_exec.pack(side='left')
        self.b_cancel = ttk.Button(btns, text='■   Cancel path', style='Ghost.TButton', command=self.cancel_path, state='disabled')
        self.b_cancel.pack(side='left', padx=8)
        tk.Label(btns, textvariable=self.v_route, bg=CARD, fg=TEXT, font=FONT_S, anchor='w', wraplength=520,
                 justify='left').pack(side='left', padx=10)
        cols = ('lap', 'goal', 'x', 'y', 'result', 'time')
        self.tree = ttk.Treeview(p, columns=cols, show='headings', height=5)
        for c, w in zip(cols, (50, 50, 70, 70, 110, 80)):
            self.tree.heading(c, text=c)
            self.tree.column(c, width=w, anchor='center')
        self.tree.pack(fill='both', expand=True)
        self.tree.tag_configure('ok', foreground=GREEN)
        self.tree.tag_configure('bad', foreground=RED)

    # ------------------------------------------------------------------ environment + YAML values
    def _select_env(self, name):
        self.v_env.set(name)
        for n, (f, labels) in self.env_cards.items():
            sel = n == name
            f.config(highlightbackground=ACCENT if sel else BORDER, bg=ACCENT_L if sel else CARD)
            for w in labels:
                w.config(bg=ACCENT_L if sel else CARD)
        e = self.envs[name]
        s = e.get('stats', {})
        self.info.config(text=(
            f"{e['description']}\n\nroom {e['room'][1] - e['room'][0]:.1f} x {e['room'][3] - e['room'][2]:.1f} m\n"
            f"occupied {100 * s.get('occupied_fraction', 0):.1f}% of the room\n"
            f"lidar-visible edge cells in a 10 m window:\n   mean {s.get('edge_cells_10m_window_mean', '?')}, "
            f"max {s.get('edge_cells_10m_window_max', '?')}\nstart {e['start']}   goals {len(e['goals'])}"))
        self._draw_map(e)

    def _draw_map(self, e):
        self.canvas.delete('all')
        try:
            my = yaml.safe_load(open(os.path.join(self.base, 'maps', f"{e['map']}.yaml")))
            img = tk.PhotoImage(file=os.path.join(self.base, 'maps', f"{e['map']}.pgm"))
            w, h = img.width(), img.height()
            k = 1
            while max(w, h) / k > 300:
                k += 1
            if k > 1:
                img = img.subsample(k)
            self.photo = img
            self.canvas.config(width=img.width(), height=img.height())
            self.canvas.create_image(0, 0, image=img, anchor='nw')
            ox, oy, res = my['origin'][0], my['origin'][1], my['resolution'] * k
            px = lambda x, y: ((x - ox) / res, h / k - (y - oy) / res)
            pts = [px(*g) for g in e['goals']]
            sx, sy = px(e['start'][0], e['start'][1])
            chain = [(sx, sy)] + pts
            for a, b in zip(chain, chain[1:]):
                self.canvas.create_line(a[0], a[1], b[0], b[1], fill=RED, dash=(4, 3), width=2)
            for i, (x, y) in enumerate(pts, 1):
                self.canvas.create_oval(x - 5, y - 5, x + 5, y + 5, fill=RED, outline='white', width=1)
                self.canvas.create_text(x + 11, y - 9, text=str(i), fill=RED, font=FONT_B)
            self.canvas.create_oval(sx - 6, sy - 6, sx + 6, sy + 6, fill=GREEN, outline='white', width=1)
        except Exception as ex:
            self.canvas.create_text(10, 10, anchor='nw', text=f'(no preview: {ex})', fill=MUTED)

    def _reload_yaml(self, force=False):
        """Fill the fields from the YAML. Unchanged fields follow the file; fields you edited keep your value."""
        try:
            mt = os.path.getmtime(self.yaml_path)
            new = read_yaml_settings(self.yaml_path)
        except Exception as ex:
            self.v_note.set(f'could not read YAML: {ex}'); return
        changed_file = mt != self._yaml_mtime
        self._yaml_mtime = mt
        for k in ('mode',) + self.FIELDS:
            nv = fmt(new[k])
            if force or self.v[k].get() == self.loaded.get(k, None) or k not in self.loaded:
                self.v[k].set(nv)
            self.loaded[k] = nv
        self.loaded['_new'] = new
        for k in ('mode',) + self.FIELDS:
            self._mark(k)
        if self.name_auto:
            self.v_name.set(next_run_name(new['window'], new['res']))
        if changed_file and not force:
            self.v_note.set('nav2_params.yaml changed - values refreshed')

    def _on_dims(self):
        """Live hint: grid size, cells moved per update at the controller's speed, lidar vs update rate."""
        try:
            win, res, freq = (float(self.v[k].get()) for k in ('window', 'res', 'freq'))
            speed = float(self.loaded.get('_new', {}).get('speed', 0.4))
            cells = win / res
            moved = speed / (freq * res)
            txt = f'grid {cells:.0f} x {cells:.0f} cells  ·  about {moved:.1f} cells moved per update at {speed:g} m/s'
            if abs(cells - round(cells)) > 1e-6:
                txt += '  ·  NOT a whole number of cells - Run will refuse'
            if freq > self.lidar_hz:
                txt += f'  ·  note: the lidar publishes at {self.lidar_hz:g} Hz, so some updates see no new scan'
            self.v_grid.set(txt)
            if self.name_auto:
                self.v_name.set(next_run_name(win, res))
        except (ValueError, ZeroDivisionError):
            self.v_grid.set('')

    def _mark(self, k):
        w = self.widgets.get(k)
        if w is None or k not in self.loaded:
            return
        dirty = self.v[k].get().strip() != self.loaded[k]
        base = 'TCombobox' if k == 'mode' else 'TEntry'
        w.configure(style=('Dirty.' + base) if dirty else base)

    def _overrides(self):
        ov = {}
        for k in ('mode',) + self.FIELDS:
            val = self.v[k].get().strip()
            if val != self.loaded.get(k):
                ov[k] = val
        return ov

    # ------------------------------------------------------------------ run control
    def start(self):
        if self.proc is not None:
            return
        self._reload_yaml()
        ov = self._overrides()
        try:
            for k in ('window', 'res', 'freq', 'dmax', 'obs', 'ray'):
                if k in ov and float(ov[k]) <= 0:
                    raise ValueError(f'{k} must be > 0')
            win, res = float(self.v['window'].get()), float(self.v['res'].get())
            if abs(win - round(win)) > 1e-9:
                raise ValueError('Local window must be a whole number of metres (Nav2 declares width/height as integers)')
            cells = win / res
            if abs(cells - round(cells)) > 1e-6:
                raise ValueError(f'window / resolution = {cells:.3f} cells; it must be a whole number (e.g. 10 m at 0.05 or 0.025)')
            if cells > 1500:
                raise ValueError(f'{cells:.0f} x {cells:.0f} cells is too large')
            if 'check' in ov and int(ov['check']) < 0:
                raise ValueError('self-check must be >= 0')
            noise = float(self.v_noise.get()); laps = int(self.v_laps.get())
            if noise < 0 or laps < 1:
                raise ValueError('noise must be >= 0 and laps >= 1')
        except ValueError as ex:
            messagebox.showerror('Invalid setting', str(ex)); return
        name = self.v_name.get().strip()
        if not name or '/' in name or name in ('.', '..'):
            messagebox.showerror('Run folder', 'Give a folder name without slashes, e.g. test4_200m'); return
        self.run_dir = os.path.join(RESULTS_ROOT, name)
        if os.path.isdir(self.run_dir) and os.listdir(self.run_dir):
            if not messagebox.askyesno('Folder exists', f'{self.run_dir}\nalready has files. Overwrite the CSVs in it?'):
                return
        os.makedirs(self.run_dir, exist_ok=True)

        env_name, e = self.v_env.get(), self.envs[self.v_env.get()]
        launch, takes_map, takes_nav = STACKS[self.v_stack.get()]
        args = [f'world:={e["world"]}', f'headless:={str(self.v_headless.get()).lower()}',
                f'rviz:={str(self.v_rviz.get()).lower()}', f'lidar_noise:={noise}',
                f'x:={e["start"][0]}', f'y:={e["start"][1]}', f'yaw:={e["start"][2]}']
        if takes_map:
            args.append(f'map:={e["map"]}')
        if takes_nav:
            params_path = os.path.join(self.run_dir, 'nav2_params.yaml')
            try:
                derive_params(self.yaml_path, params_path, ov, self.run_dir)
            except Exception as ex:
                messagebox.showerror('Could not prepare parameters', str(ex)); return
            args += [f'nav_params:={params_path}', f'env:={env_name}', f'route_laps:={laps}',
                     f'route_auto_start:={str(self.v_auto.get()).lower()}',
                     f'route_status_file:={os.path.join(self.run_dir, "route_status.json")}']
        cmd = ['ros2', 'launch', 'esdf_sim_bringup', launch] + args
        sources = ' && '.join(f'source {shlex.quote(f)}' for f in SETUP_FILES if os.path.exists(f))
        self.script_prefix = sources
        script = f'{sources} && exec {" ".join(shlex.quote(a) for a in cmd)}'

        json.dump(dict(env=env_name, stack=self.v_stack.get(), launch=launch, overrides=ov, yaml_values=self.loaded['_new'],
                       lidar_noise=noise, laps=laps, started=datetime.datetime.now().isoformat(), command=cmd,
                       env_info=e), open(os.path.join(self.run_dir, 'run_info.json'), 'w'), indent=2)
        self.logfile = open(os.path.join(self.run_dir, 'launch.log'), 'w')
        env = dict(os.environ, PYTHONUNBUFFERED='1', RCUTILS_COLORIZED_OUTPUT='0')
        try:
            self.proc = subprocess.Popen(['bash', '-c', script], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                         text=True, bufsize=1, preexec_fn=os.setsid, env=env)
        except Exception as ex:
            messagebox.showerror('Could not start', str(ex)); self.logfile.close(); self.logfile = None; return
        self.takes_route = takes_nav
        threading.Thread(target=self._reader, args=(self.proc,), daemon=True).start()
        self._append(f'$ {" ".join(shlex.quote(a) for a in cmd)}\n', 'cmd')
        if ov:
            self._append(f'GUI overrides on top of the YAML: {ov}\n', 'cmd')
        self.status = {}
        self.tree.delete(*self.tree.get_children())
        self.v_route.set('Waiting for Nav2 ...' if takes_nav else 'This launch has no route runner.')
        self._set_state('running', f'Running  ·  {env_name}')
        self.v_note.set(f'results: {self.run_dir}')
        self.b_run.config(state='disabled'); self.b_stop.config(state='normal')

    def _set_state(self, kind, text):
        bg, fg = {'running': ('#dff5e9', GREEN), 'stopping': ('#fdf0d5', AMBER), 'stopped': ('#e8ebf2', MUTED)}[kind]
        self.pill.config(bg=bg, fg=fg); self.v_status.set(text)

    def _reader(self, proc):
        for line in proc.stdout:
            self.log_q.put(line)
        proc.wait()
        self.log_q.put(None)

    def _pump_log(self):
        try:
            while True:
                line = self.log_q.get_nowait()
                if line is None:
                    self._finished()
                else:
                    self._append(line)
        except queue.Empty:
            pass
        self.after(100, self._pump_log)

    def _append(self, text, tag=None):
        if 'process has died' in text and 'controller_server' in text and self.proc is not None:
            self._set_state('stopping', 'controller_server crashed - see the log')
        if self.logfile:
            self.logfile.write(text); self.logfile.flush()
        if tag is None:
            tag = 'err' if '[ERROR]' in text or 'Traceback' in text else ('warn' if '[WARN]' in text else None)
        self.log.config(state='normal')
        self.log.insert('end', text, tag or ())
        n = int(self.log.index('end-1c').split('.')[0])
        if n > 4000:
            self.log.delete('1.0', f'{n - 3000}.0')
        self.log.see('end'); self.log.config(state='disabled')

    def _finished(self):
        code = self.proc.returncode if self.proc else None
        self.proc = None
        if self.logfile:
            self.logfile.close(); self.logfile = None
        self._set_state('stopped', f'Stopped  (exit {code})')
        self.b_run.config(state='normal'); self.b_stop.config(state='disabled')
        self.b_exec.config(state='disabled'); self.b_cancel.config(state='disabled')

    def stop(self):
        if self.proc is None:
            return
        self.b_stop.config(state='disabled'); self._set_state('stopping', 'Stopping ...')
        threading.Thread(target=self._stop_group, args=(self.proc,), daemon=True).start()

    @staticmethod
    def _stop_group(proc):
        try:
            pgid = os.getpgid(proc.pid)
        except ProcessLookupError:
            return
        for sig, wait in ((signal.SIGINT, 15), (signal.SIGTERM, 6), (signal.SIGKILL, 3)):
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                return
            try:
                proc.wait(timeout=wait)
                break
            except subprocess.TimeoutExpired:
                continue
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass

    # ------------------------------------------------------------------ route control + status
    def _service(self, name, ok_text):
        if self.proc is None:
            return
        script = f'{self.script_prefix} && ros2 service call /route_runner/{name} std_srvs/srv/Trigger "{{}}"'

        def work():
            try:
                out = subprocess.run(['bash', '-c', script], capture_output=True, text=True, timeout=30).stdout
                m = re.search(r"success=(True|False), message='([^']*)'", out)
                msg = f'{ok_text}' if (m and m.group(1) == 'True') else (m.group(2) if m else 'no answer from route_runner')
            except Exception as ex:
                msg = f'service call failed: {ex}'
            self.after(0, lambda: self.v_route.set(msg))
        self.v_route.set('sending ...')
        threading.Thread(target=work, daemon=True).start()

    def exec_path(self):
        self._service('start', 'route started')

    def cancel_path(self):
        self._service('cancel', 'cancelling ...')

    def _poll(self):
        # YAML changed on disk?
        try:
            if os.path.getmtime(self.yaml_path) != self._yaml_mtime:
                self._reload_yaml()
        except OSError:
            pass
        # route status written by route_runner
        if self.proc is not None and self.run_dir and getattr(self, 'takes_route', False):
            try:
                st = json.load(open(os.path.join(self.run_dir, 'route_status.json')))
            except (OSError, ValueError):
                st = None
            if st:
                self._show_status(st)
        self.after(500, self._poll)

    def _show_status(self, st):
        self.status = st
        state = st.get('state')
        ready = st.get('nav2_ready')
        if state == 'running':
            txt = (f"running  ·  lap {st['lap']}/{st['laps']}  ·  goal {st['goal']}/{st['goals_total']} "
                   f"{tuple(st['goal_xy'])}  ·  {st['elapsed_s']:.0f} s")
        elif state in ('finished', 'cancelled', 'failed'):
            ok = sum(1 for r in st['results'] if r['status'] == 'succeeded')
            txt = f"{state}  ·  {ok}/{len(st['results'])} goals reached  ·  {st['message']}"
        elif state == 'no_route':
            txt = 'no route loaded for this env'
        else:
            txt = 'Nav2 ready - press Execute path' if ready else 'Waiting for Nav2 ...'
        if not self.v_route.get().startswith(('sending', 'route started', 'cancelling')) or state != 'idle':
            self.v_route.set(txt)
        self.b_exec.config(state='normal' if state in ('idle', 'finished', 'cancelled', 'failed') and ready else 'disabled')
        self.b_cancel.config(state='normal' if state == 'running' else 'disabled')
        rows = [(r['lap'], r['goal'], f"{r['x']:.1f}", f"{r['y']:.1f}", r['status'], f"{r['time_s']:.0f} s") for r in st['results']]
        if rows != getattr(self, '_rows', None):
            self._rows = rows
            self.tree.delete(*self.tree.get_children())
            for row in rows:
                self.tree.insert('', 'end', values=row, tags=('ok' if row[4] == 'succeeded' else 'bad',))

    # ------------------------------------------------------------------ helpers
    def open_results(self):
        d = self.run_dir or RESULTS_ROOT
        os.makedirs(d, exist_ok=True)
        subprocess.Popen(['xdg-open', d], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def copy_analysis(self):
        if not self.run_dir:
            messagebox.showinfo('Nothing yet', 'Start a run first.'); return
        y = self.loaded.get('_new', {})
        cmd = ('python3 ~/nav2_ws/src/nav2_esdf_fh/scripts/compare_runs.py '
               f"{self.run_dir}/{y.get('esdf_csv', 'esdf_local_costmap_stats.csv')} "
               f"{self.run_dir}/{y.get('fh_csv', 'fh_local_costmap_stats.csv')}")
        self.clipboard_clear(); self.clipboard_append(cmd)
        self.v_note.set('analysis command copied to the clipboard')

    def _on_close(self):
        if self.proc is not None:
            if not messagebox.askyesno('Still running', 'A launch is running. Stop it and quit?'):
                return
            self._stop_group(self.proc)
        self.destroy()


if __name__ == '__main__':
    if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        sys.exit('No display available')
    App().mainloop()
